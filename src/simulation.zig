const std = @import("std");
const gs = @import("gs");
const ids = @import("ids");
const registry = @import("registry.zig");
const worldgen = @import("worldgen.zig");
const wapi = @import("worldgen_api.zig");
const constants = @import("constants");
const player = @import("player.zig");
const physics = @import("physics.zig");

pub const StorageMode = union(enum) {
    memory,
    save_path: []const u8,
};

pub const SimulationOptions = struct {
    allocator: std.mem.Allocator,
    registries: *registry.Registry,
    mode: StorageMode,
    world_key: []const u8 = "minecraft:overworld",
};

pub const Snapshot = struct {
    tick: u64,
    actual_tps: f64,
    rolling_tps: f64,
    dt_sec: f64,
    world_seed: u64,
    section_count_y: u16,
    chunk_count: usize,
    player_count: usize,
    dynamic_entity_count: usize,
};

pub const RegionPos = struct { x: i32, z: i32 };

pub const RegionState = struct {
    chunks: std.ArrayList(gs.Chunk),
    chunk_index: std.AutoHashMap(gs.ChunkPos, usize),
};

pub const WorldState = struct {
    key: []const u8,
    sections_below: u16,
    sections_above: u16,
    regions: std.AutoHashMap(RegionPos, RegionState),
    spawn_point: gs.BlockPos,
};

pub const Simulation = struct {
    allocator: std.mem.Allocator,
    reg: *registry.Registry,
    mode: StorageMode,
    section_count_y: u16,
    world_seed: u64,
    // save structure: worlds -> regions -> chunks
    worlds_mutex: std.Thread.Mutex = .{},
    player_world: std.AutoHashMap(player.PlayerId, []const u8), // references keys in reg.worlds
    worlds_state: std.StringHashMap(WorldState),
    // tick counter and thread control
    tick_counter: std.atomic.Value(u64) = std.atomic.Value(u64).init(0),
    running: std.atomic.Value(bool) = std.atomic.Value(bool).init(false),
    thread: ?std.Thread = null,
    // fixed-timestep timing owned by simulation
    target_tps: f64 = 20.0,
    last_tick_ns: i128 = 0,
    max_accum: f64 = 0.25,
    spin_margin_ns: i128 = 2_000_000, // 2 ms spin for precise pacing on macOS
    // metrics
    last_tick_dt_sec: f64 = 0.0,
    actual_tps: f64 = 0.0,
    // timing window for bias (last 20 ticks)
    window_ns: [20]i128 = [_]i128{0} ** 20,
    window_sum_ns: i128 = 0,
    window_count: usize = 0,
    window_index: usize = 0,
    target_bias_ns: i128 = 0,
    // double-buffered immutable snapshots for client render
    snapshots: [2]Snapshot = undefined,
    snapshot_index: std.atomic.Value(u8) = std.atomic.Value(u8).init(0),

    // player systems
    players: player.PlayerRegistry,
    next_eid: u64 = 1,
    dynamic_entities: std.ArrayList(gs.EntityRecord),
    entity_by_player: std.AutoHashMap(player.PlayerId, usize),
    connections_mutex: std.Thread.Mutex = .{},

    // worldgen config and threads
    worldgen_threads: u32 = 1,
    gen_radius_chunks: i32 = 8,
    wg_threads: std.ArrayList(std.Thread),
    // incremental proto-chunks advancing through staged generation
    proto_chunks: std.AutoHashMap(gs.ChunkPos, wapi.ProtoChunk),

    pub fn init(opts: SimulationOptions) !Simulation {
        var seed_bytes: [8]u8 = undefined;
        std.crypto.random.bytes(&seed_bytes);
        const seed = std.mem.readInt(u64, &seed_bytes, .little);

        var sim: Simulation = .{
            .allocator = opts.allocator,
            .reg = opts.registries,
            .mode = opts.mode,
            .section_count_y = 0,
            .world_seed = seed,
            .worlds_mutex = .{},
            .player_world = std.AutoHashMap(player.PlayerId, []const u8).init(opts.allocator),
            .worlds_state = std.StringHashMap(WorldState).init(opts.allocator),
            .tick_counter = std.atomic.Value(u64).init(0),
            .running = std.atomic.Value(bool).init(false),
            .thread = null,
            .target_tps = 20.0,
            .last_tick_ns = 0,
            .max_accum = 0.25,
            .last_tick_dt_sec = 0.0,
            .actual_tps = 0.0,
            .snapshots = undefined,
            .snapshot_index = std.atomic.Value(u8).init(0),
            .players = player.PlayerRegistry.init(opts.allocator),
            .next_eid = 1,
            .dynamic_entities = undefined,
            .entity_by_player = std.AutoHashMap(player.PlayerId, usize).init(opts.allocator),
            .wg_threads = undefined,
            .proto_chunks = undefined,
        };
        // initialize snapshot buffers
        sim.publishSnapshot();
        // initialize containers
        sim.proto_chunks = std.AutoHashMap(gs.ChunkPos, wapi.ProtoChunk).init(opts.allocator);
        // ensure selected world exists in registry
        _ = sim.reg.worlds.get(opts.world_key) orelse unreachable;

        // initialize lists
        sim.dynamic_entities = try std.ArrayList(gs.EntityRecord).initCapacity(opts.allocator, 0);
        sim.wg_threads = try std.ArrayList(std.Thread).initCapacity(opts.allocator, 0);

        return sim;
    }

    pub fn initMemory(allocator: std.mem.Allocator, reg: *registry.Registry, section_count_y: u16) !Simulation {
        _ = section_count_y; // section count chosen per-world via registry
        return init(.{ .allocator = allocator, .registries = reg, .mode = .memory });
    }

    pub fn deinit(self: *Simulation) void {
        // free all chunks in save structure
        var w_it = self.worlds_state.iterator();
        while (w_it.next()) |entry| {
            var regions_it = entry.value_ptr.regions.iterator();
            while (regions_it.next()) |rentry| {
                var rs = rentry.value_ptr.*;
                for (rs.chunks.items) |*c| worldgen.deinitChunk(self.allocator, c);
                rs.chunks.deinit(self.allocator);
                rs.chunk_index.deinit();
            }
            entry.value_ptr.regions.deinit();
            self.allocator.free(entry.value_ptr.key);
        }
        self.worlds_state.deinit();
        self.dynamic_entities.deinit(self.allocator);
        self.entity_by_player.deinit();
        self.player_world.deinit();
        self.players.deinit();
        // free any in-flight proto-chunks
        var pit = self.proto_chunks.valueIterator();
        while (pit.next()) |p| {
            var tmp = p.*;
            worldgen.protoDeinit(&tmp);
        }
        self.proto_chunks.deinit();
        self.wg_threads.deinit(self.allocator);
    }

    pub fn tick(self: *Simulation) void {
        // pacing + one simulation step
        const base_target_ns: i128 = @intFromFloat(1_000_000_000.0 / self.target_tps);
        const effective_target_ns_unclamped: i128 = base_target_ns + self.target_bias_ns;
        const effective_target_ns: i128 = if (effective_target_ns_unclamped > 1) effective_target_ns_unclamped else 1;
        const t0: i128 = std.time.nanoTimestamp();

        // 0. Intake and Setup
        self.phaseIntakeSetup();
        // 1. Apply Commands (stub)
        self.phaseApplyCommands();
        // 2. Neighbor/Structural Updates (stub)
        self.phaseNeighborUpdates();
        // 3. Scheduled Block Updates (stub)
        self.phaseScheduled();
        // 4. Circuits (stub)
        self.phaseCircuits();
        // 5. Fluids (stub)
        self.phaseFluids();
        // 6. Physics (stub)
        self.phasePhysics();
        // 7. Entity AI (stub)
        self.phaseAI();
        // 8. Block Entities (stub)
        self.phaseBlockEntities();
        // 9. Random Ticks (stub)
        self.phaseRandomTicks();
        // 10. Lighting Barrier (stub)
        self.phaseLightingBarrier();
        // 11. Processing/Inventories (stub)
        self.phaseProcessing();
        // 12. Health/Status and Cleanup (stub)
        self.phaseStatusCleanup();
        // 13. Time/Weather (stub)
        self.phaseTimeWeather();
        // 14. Persistence marks (stub)
        self.phasePersistence();
        // 15. Networking (stub)
        self.phaseNetworking();
        self.phaseTickEnd();

        // increment tick counter for this completed step
        _ = self.tick_counter.fetchAdd(1, .monotonic);

        // measure compute time
        var dt_ns: i128 = std.time.nanoTimestamp() - t0;
        if (dt_ns < 0) dt_ns = 0;
        // sleep to maintain target TPS if able (so total tick duration matches target)
        if (dt_ns < effective_target_ns) {
            const remain: i128 = effective_target_ns - dt_ns;
            const spin_ns: i128 = self.spin_margin_ns;
            if (remain > spin_ns) {
                std.Thread.sleep(@intCast(remain - spin_ns));
            }
            const deadline: i128 = t0 + effective_target_ns;
            while (std.time.nanoTimestamp() < deadline) {}
        }
        // total duration including sleep
        const t_end: i128 = std.time.nanoTimestamp();
        var total_ns: i128 = t_end - t0;
        if (total_ns < 1) total_ns = 1; // avoid div-by-zero
        self.last_tick_dt_sec = @as(f64, @floatFromInt(total_ns)) / 1_000_000_000.0;
        self.actual_tps = 1.0 / self.last_tick_dt_sec;

        // update rolling window and compute bias for next tick
        if (self.window_count < self.window_ns.len) {
            self.window_sum_ns += total_ns;
            self.window_ns[self.window_index] = total_ns;
            self.window_index = (self.window_index + 1) % self.window_ns.len;
            self.window_count += 1;
        } else {
            self.window_sum_ns -= self.window_ns[self.window_index];
            self.window_sum_ns += total_ns;
            self.window_ns[self.window_index] = total_ns;
            self.window_index = (self.window_index + 1) % self.window_ns.len;
        }
        const avg_ns: i128 = @intFromFloat(@as(f64, @floatFromInt(self.window_sum_ns)) / @as(f64, @floatFromInt(self.window_count)));
        // desired bias to close average error
        const error_ns: i128 = base_target_ns - avg_ns;
        // limit bias change per tick to 10% of base_target_ns
        const max_step_ns: i128 = @divTrunc(base_target_ns, 10); // 10%
        var delta: i128 = error_ns - self.target_bias_ns;
        if (delta > max_step_ns) delta = max_step_ns else if (delta < -max_step_ns) delta = -max_step_ns;
        self.target_bias_ns += delta;

        // 16. Snapshot Publish (after timing so metrics reflect achieved TPS)
        self.publishSnapshot();
    }

    fn tickThreadMain(self_ptr: *Simulation) void {
        while (self_ptr.running.load(.acquire)) {
            self_ptr.tick();
        }
    }

    fn blockLookupCall(ctx: ?*anyopaque, name: []const u8) ?ids.BlockStateId {
        if (ctx) |p| {
            const regp: *registry.Registry = @ptrCast(@alignCast(p));
            for (regp.blocks.items, 0..) |b, i| {
                if (std.mem.eql(u8, b.name, name)) return @intCast(i);
            }
        }
        return null;
    }

    fn worldgenThreadMain(self_ptr: *Simulation) void {
        const alloc = self_ptr.allocator;
        var desired: std.AutoHashMap(gs.ChunkPos, void) = std.AutoHashMap(gs.ChunkPos, void).init(alloc);
        defer desired.deinit();
        while (self_ptr.running.load(.acquire)) {
            desired.clearRetainingCapacity();
            // Snapshot connected players and desired chunk positions
            self_ptr.connections_mutex.lock();
            {
                var it = self_ptr.players.by_id.valueIterator();
                while (it.next()) |pd| {
                    if (!pd.connected) continue;
                    if (self_ptr.entity_by_player.get(pd.id)) |idx| {
                        if (idx < self_ptr.dynamic_entities.items.len) {
                            const e = self_ptr.dynamic_entities.items[idx];
                            const cx: i32 = @divFloor(@as(i32, @intFromFloat(@floor(e.pos[0]))), @as(i32, @intCast(constants.chunk_size_x)));
                            const cz: i32 = @divFloor(@as(i32, @intFromFloat(@floor(e.pos[2]))), @as(i32, @intCast(constants.chunk_size_z)));
                            const r = self_ptr.gen_radius_chunks;
                            var dz: i32 = -r;
                            while (dz <= r) : (dz += 1) {
                                var dx: i32 = -r;
                                while (dx <= r) : (dx += 1) {
                                    const pos = gs.ChunkPos{ .x = cx + dx, .z = cz + dz };
                                    _ = desired.put(pos, {}) catch {};
                                }
                            }
                        }
                    }
                }
            }
            self_ptr.connections_mutex.unlock();

            // Generate up to a few proto-chunk phase advances per pass
            var advanced: u32 = 0;
            var it2 = desired.keyIterator();
            while (it2.next()) |pos_ptr| {
                const pos = pos_ptr.*;
                // compute the player's world (assume default for now)
                const world_key = "minecraft:overworld";

                // get world def for section count
                const wd = self_ptr.reg.worlds.get(world_key) orelse continue;

                // locate region
                const rpos = RegionPos{ .x = @divFloor(pos.x, 32), .z = @divFloor(pos.z, 32) };

                // check if chunk already finalized
                var missing = false;
                self_ptr.worlds_mutex.lock();
                var ws_ptr = self_ptr.ensureWorldState(world_key) catch null;
                if (ws_ptr) |ws| {
                    const rs_ptr = self_ptr.ensureRegionState(ws, rpos) catch null;
                    if (rs_ptr) |rs| {
                        missing = (rs.chunk_index.get(pos) == null);
                    }
                }
                self_ptr.worlds_mutex.unlock();
                if (!missing) continue;

                // pick worldgen based on world def
                const wg_def = self_ptr.reg.worldgen.get(wd.gen_key) orelse continue;
                const params: registry.WorldGen.Params = .{ .blocks = wd.gen_blocks, .biomes = wd.gen_biomes };
                const lookup: registry.WorldGen.BlockLookup = .{ .ctx = self_ptr.reg, .call = blockLookupCall };

                // Ensure proto-chunk exists
                var proto_ptr = self_ptr.proto_chunks.getPtr(pos);
                if (proto_ptr == null) {
                    var proto = worldgen.protoInit(alloc, wd.sections_below, wd.sections_above, pos) catch continue;
                    _ = self_ptr.proto_chunks.put(pos, proto) catch {
                        worldgen.protoDeinit(&proto);
                        continue;
                    };
                    proto_ptr = self_ptr.proto_chunks.getPtr(pos);
                }
                if (proto_ptr == null) continue;

                // Advance exactly one phase for this chunk
                const result = worldgen.advanceOnePhase(proto_ptr.?, self_ptr.world_seed, wg_def, params, lookup, 0) catch null;
                if (result) |chunk| {
                    // fold into simulation and drop proto-chunk
                    self_ptr.worlds_mutex.lock();
                    ws_ptr = self_ptr.ensureWorldState(world_key) catch null;
                    if (ws_ptr) |ws2| {
                        const rs2 = self_ptr.ensureRegionState(ws2, rpos) catch null;
                        if (rs2) |rsok| {
                            if (rsok.chunk_index.get(pos) == null) {
                                rsok.chunks.append(self_ptr.allocator, chunk) catch unreachable;
                                const idx = rsok.chunks.items.len - 1;
                                _ = rsok.chunk_index.put(pos, idx) catch {};
                            } else {
                                worldgen.deinitChunk(alloc, &chunk);
                            }
                        } else {
                            worldgen.deinitChunk(alloc, &chunk);
                        }
                    } else {
                        worldgen.deinitChunk(alloc, &chunk);
                    }
                    self_ptr.worlds_mutex.unlock();

                    // remove proto
                    if (self_ptr.proto_chunks.fetchRemove(pos)) |entry| {
                        var to_free = entry.value;
                        worldgen.protoDeinit(&to_free);
                    }
                }

                advanced += 1;
                if (advanced >= 8) break; // small per-pass budget across many chunks
            }

            // small sleep to avoid busy-loop
            std.Thread.sleep(5_000_000); // 5ms
        }
    }

    fn publishSnapshot(self: *Simulation) void {
        const cur = self.snapshot_index.load(.acquire);
        const next: u8 = cur ^ 1;
        var rolling_tps_val: f64 = self.actual_tps;
        if (self.window_count > 0 and self.window_sum_ns > 0) {
            const avg_ns_f: f64 = @as(f64, @floatFromInt(self.window_sum_ns)) / @as(f64, @floatFromInt(self.window_count));
            if (avg_ns_f > 0) rolling_tps_val = 1_000_000_000.0 / avg_ns_f;
        }
        // compute total chunk_count across all worlds/regions
        var total_chunks: usize = 0;
        var w_it = self.worlds_state.iterator();
        while (w_it.next()) |entry| {
            var r_it = entry.value_ptr.regions.iterator();
            while (r_it.next()) |rentry| {
                total_chunks += rentry.value_ptr.chunks.items.len;
            }
        }
        const snap = Snapshot{
            .tick = self.tick_counter.load(.monotonic),
            .actual_tps = self.actual_tps,
            .rolling_tps = rolling_tps_val,
            .dt_sec = self.last_tick_dt_sec,
            .world_seed = self.world_seed,
            .section_count_y = 0,
            .chunk_count = total_chunks,
            .player_count = self.players.by_id.count(),
            .dynamic_entity_count = self.dynamic_entities.items.len,
        };
        self.snapshots[next] = snap;
        self.snapshot_index.store(next, .release);
    }

    pub fn getSnapshot(self: *const Simulation) Snapshot {
        const idx = self.snapshot_index.load(.acquire);
        return self.snapshots[idx];
    }

    // Phase stubs (to be implemented with real logic)
    fn phaseIntakeSetup(self: *Simulation) void {
        _ = self;
    }
    fn phaseApplyCommands(self: *Simulation) void {
        _ = self;
    }
    fn phaseNeighborUpdates(self: *Simulation) void {
        _ = self;
    }
    fn phaseScheduled(self: *Simulation) void {
        _ = self;
    }
    fn phaseCircuits(self: *Simulation) void {
        _ = self;
    }
    fn phaseFluids(self: *Simulation) void {
        _ = self;
    }
    fn phasePhysics(self: *Simulation) void {
        // Physics: gravity and AABB collisions against solid voxels.
        const dt: f32 = @floatCast(1.0 / self.target_tps);
        const cfg: physics.PhysicsConfig = .{ .gravity = 9.80665 };

        // Solid sampler that checks palette/flags; unloaded/out-of-range treated as non-solid
        const solidSampler = struct {
            fn call(ctx: *anyopaque, ix: i32, iy: i32, iz: i32) bool {
                const sim: *Simulation = @ptrCast(@alignCast(ctx));
                const world_key = "minecraft:overworld";
                sim.worlds_mutex.lock();
                defer sim.worlds_mutex.unlock();
                const ws = sim.worlds_state.getPtr(world_key) orelse return false;
                // convert to chunk/section/local coords
                const cx: i32 = @divFloor(ix, @as(i32, @intCast(constants.chunk_size_x)));
                const cz: i32 = @divFloor(iz, @as(i32, @intCast(constants.chunk_size_z)));
                const lx_i: i32 = @mod(ix, @as(i32, @intCast(constants.chunk_size_x)));
                const lz_i: i32 = @mod(iz, @as(i32, @intCast(constants.chunk_size_z)));
                const rp = RegionPos{ .x = @divFloor(cx, 32), .z = @divFloor(cz, 32) };
                const rs = ws.regions.getPtr(rp) orelse return false;
                const idx_opt = rs.chunk_index.get(.{ .x = cx, .z = cz });
                if (idx_opt == null) return false;
                const ch = rs.chunks.items[idx_opt.?];
                const total_y: i32 = @intCast(ch.sections.len * constants.section_height);
                const ny0: i32 = iy + @as(i32, @intCast(ws.sections_below)) * @as(i32, @intCast(constants.section_height));
                if (ny0 < 0 or ny0 >= total_y) return false;
                const sy: usize = @intCast(@divTrunc(ny0, @as(i32, @intCast(constants.section_height))));
                const ly: usize = @intCast(@mod(ny0, @as(i32, @intCast(constants.section_height))));
                if (sy >= ch.sections.len) return false;
                const s = ch.sections[sy];
                // local helper to unpack bitpacked indices (little-endian within u32 words)
                const unpackBitsGetLocal = struct {
                    fn call(bits: []const u32, index: usize, bits_per_index: u6) u32 {
                        const bpi_usize: usize = @intCast(bits_per_index);
                        const bit_index: usize = index * bpi_usize;
                        const word_index: usize = bit_index / 32;
                        const bit_offset_u: usize = bit_index % 32;
                        const bit_offset: u5 = @intCast(bit_offset_u);
                        const mask: u32 = if (bpi_usize >= 32) 0xFFFF_FFFF else ((@as(u32, 1) << @intCast(bits_per_index)) - 1);
                        var value: u32 = bits[word_index] >> bit_offset;
                        const bits_in_word: usize = 32 - bit_offset_u;
                        if (bits_in_word < bpi_usize) {
                            value |= bits[word_index + 1] << @intCast(bits_in_word);
                            return value & mask;
                        } else {
                            return value & mask;
                        }
                    }
                }.call;
                // compute bits-per-index for this palette length (at least 1 bit)
                const pal_len: usize = s.palette.len;
                var tmp_bpi: u6 = 1;
                if (pal_len > 1) {
                    var v: usize = pal_len - 1;
                    var count: u6 = 0;
                    while (v != 0) : (v >>= 1) count += 1;
                    if (count == 0) count = 1;
                    tmp_bpi = count;
                }
                const bpi: u6 = tmp_bpi;
                const lx: usize = @intCast(lx_i);
                const lz: usize = @intCast(lz_i);
                if (lx >= constants.chunk_size_x or lz >= constants.chunk_size_z) return false;
                const idx3d: usize = lz * (constants.chunk_size_x * constants.section_height) + lx * constants.section_height + ly;
                const pidx: usize = @intCast(unpackBitsGetLocal(s.blocks_indices_bits, idx3d, bpi));
                if (pidx >= s.palette.len) return false;
                const bid: u32 = s.palette[pidx];
                if (bid == 0) return false;
                // respect collision flag from registry if known
                if (bid < sim.reg.blocks.items.len) {
                    return sim.reg.blocks.items[@intCast(bid)].full_block_collision;
                }
                // unknown: treat as solid conservatively
                return true;
            }
        }.call;

        // Helper: check that the chunk at (x,z) is present with at least one section
        const chunkLoadedAt = struct {
            fn call(sim: *Simulation, x: f32, z: f32) bool {
                const xi: i32 = @intFromFloat(@floor(x));
                const zi: i32 = @intFromFloat(@floor(z));
                const cx: i32 = @divFloor(xi, @as(i32, @intCast(constants.chunk_size_x)));
                const cz: i32 = @divFloor(zi, @as(i32, @intCast(constants.chunk_size_z)));
                const world_key = "minecraft:overworld";
                sim.worlds_mutex.lock();
                defer sim.worlds_mutex.unlock();
                const ws = sim.worlds_state.getPtr(world_key) orelse return false;
                const rp = RegionPos{ .x = @divFloor(cx, 32), .z = @divFloor(cz, 32) };
                const rs = ws.regions.getPtr(rp) orelse return false;
                const idx_opt = rs.chunk_index.get(.{ .x = cx, .z = cz });
                if (idx_opt == null) return false;
                const ch = rs.chunks.items[idx_opt.?];
                return ch.status == .full;
            }
        }.call;

        var i: usize = 0;
        while (i < self.dynamic_entities.items.len) : (i += 1) {
            var e = &self.dynamic_entities.items[i];
            // Wait for player chunk to be loaded before ticking physics to avoid falling through void
            if (!chunkLoadedAt(self, e.pos[0], e.pos[2])) {
                continue;
            }
            var on_ground_local: bool = e.flags.on_ground;
            var kin: physics.EntityKinematics = .{
                .pos = e.pos,
                .vel = e.vel,
                .half_extents = e.aabb_half_extents,
                .on_ground = &on_ground_local,
            };
            physics.integrateStep(cfg, &kin, dt, solidSampler, self);
            e.pos = kin.pos;
            e.vel = kin.vel;
            e.flags.on_ground = on_ground_local;
        }
    }
    fn phaseAI(self: *Simulation) void {
        _ = self;
    }
    fn phaseBlockEntities(self: *Simulation) void {
        _ = self;
    }
    fn phaseRandomTicks(self: *Simulation) void {
        _ = self;
    }
    fn phaseLightingBarrier(self: *Simulation) void {
        _ = self;
    }
    fn phaseProcessing(self: *Simulation) void {
        _ = self;
    }
    fn phaseStatusCleanup(self: *Simulation) void {
        _ = self;
    }
    fn phaseTimeWeather(self: *Simulation) void {
        _ = self;
    }
    fn phasePersistence(self: *Simulation) void {
        _ = self;
    }
    fn phaseNetworking(self: *Simulation) void {
        _ = self;
    }
    fn phaseTickEnd(self: *Simulation) void {
        _ = self;
    }

    // Player connection and entity management
    pub fn connectOrGetPlayer(self: *Simulation, account_name: []const u8, display_name: []const u8) !player.PlayerId {
        return try self.players.getOrCreate(account_name, display_name);
    }

    // Client-server API (temporary direct call): connect by PlayerId and account_name
    // Arguments must be serializable: PlayerId bytes and UTF-8 account_name
    pub fn connectPlayer(self: *Simulation, pid: player.PlayerId, account_name: []const u8) !void {
        self.connections_mutex.lock();
        defer self.connections_mutex.unlock();
        // trust the provided identity for now; display_name := account_name
        try self.players.connectWithId(pid, account_name, account_name);
        // assign player to default world for now
        if (self.player_world.get(pid) == null) {
            const wk = self.reg.worlds.get("minecraft:overworld") orelse return;
            _ = self.player_world.put(pid, wk.key) catch {};
        }
        _ = try self.ensurePlayerEntityLocked(pid);
    }

    fn ensurePlayerEntityLocked(self: *Simulation, pid: player.PlayerId) !usize {
        if (self.entity_by_player.get(pid)) |idx| return idx;
        // Determine player's world and spawn at world spawn (adjusted to positive Y)
        const wk = self.player_world.get(pid) orelse "minecraft:overworld";
        const ws = try self.ensureWorldState(wk);
        var sp = ws.spawn_point;
        if (sp.y < 1) sp.y = 1; // ensure positive Y to avoid caves
        const center_x: f32 = @as(f32, @floatFromInt(sp.x)) + 0.5;
        const center_z: f32 = @as(f32, @floatFromInt(sp.z)) + 0.5;
        const half_height: f32 = 0.9;
        const center_y: f32 = @as(f32, @floatFromInt(sp.y)) + half_height + 0.1; // keep feet above ground

        const eid = self.next_eid;
        self.next_eid += 1;
        const rec: gs.EntityRecord = .{
            .eid = eid,
            .kind = 0, // 0 reserved for player entity kind for now
            .pos = .{ center_x, center_y, center_z },
            .vel = .{ 0, 0, 0 },
            .yaw_pitch_roll = .{ 0, 0, 0 },
            .look_dir = .{ 1, 0, 0 },
            .facing_dir_xz = .{ 1, 0 },
            // Collider: 0.75w x 1.75h x 0.4l (half-extents 0.375, 0.875, 0.2)
            .aabb_half_extents = .{ 0.375, 0.875, 0.2 },
            .flags = .{},
        };
        try self.dynamic_entities.append(self.allocator, rec);
        const new_idx = self.dynamic_entities.items.len - 1;
        try self.entity_by_player.put(pid, new_idx);
        return new_idx;
    }

    pub fn ensurePlayerEntity(self: *Simulation, pid: player.PlayerId) !usize {
        self.connections_mutex.lock();
        defer self.connections_mutex.unlock();
        return self.ensurePlayerEntityLocked(pid);
    }

    fn ensureWorldState(self: *Simulation, world_key: []const u8) !*WorldState {
        if (self.worlds_state.getPtr(world_key)) |ws| return ws;
        const wd = self.reg.worlds.get(world_key) orelse return error.WorldNotRegistered;
        const key_copy = try self.allocator.dupe(u8, world_key);
        const spawn = self.computeSpawnAtColumn(wd, 0, 0);
        const ws = WorldState{
            .key = key_copy,
            .sections_below = wd.sections_below,
            .sections_above = wd.sections_above,
            .regions = std.AutoHashMap(RegionPos, RegionState).init(self.allocator),
            .spawn_point = spawn,
        };
        try self.worlds_state.put(key_copy, ws);
        return self.worlds_state.getPtr(key_copy).?;
    }

    fn computeSpawnAtColumn(self: *Simulation, wd: registry.World.Def, x: i32, z: i32) gs.BlockPos {
        // Generate the containing chunk and read its heightmap for (x,z)
        const wg_def = self.reg.worldgen.get(wd.gen_key) orelse return .{ .x = x, .y = 0, .z = z };
        const params: registry.WorldGen.Params = .{ .blocks = wd.gen_blocks, .biomes = wd.gen_biomes };
        const lookup: registry.WorldGen.BlockLookup = .{ .ctx = self.reg, .call = blockLookupCall };

        const cx: i32 = @divFloor(x, @as(i32, @intCast(constants.chunk_size_x)));
        const cz: i32 = @divFloor(z, @as(i32, @intCast(constants.chunk_size_z)));
        const chunk_pos = gs.ChunkPos{ .x = cx, .z = cz };
        const total_sections: u16 = wd.sections_below + wd.sections_above;
        const ch = worldgen.generateChunk(self.allocator, total_sections, chunk_pos, self.world_seed, wg_def, params, lookup) catch return .{ .x = x, .y = 0, .z = z };
        defer worldgen.deinitChunk(self.allocator, &ch);
        // Local indices within chunk
        const lx_i: i32 = @mod(x, @as(i32, @intCast(constants.chunk_size_x)));
        const lz_i: i32 = @mod(z, @as(i32, @intCast(constants.chunk_size_z)));
        const lx: usize = @intCast(@as(u32, @bitCast(lx_i)) % @as(u32, @intCast(constants.chunk_size_x)));
        const lz: usize = @intCast(@as(u32, @bitCast(lz_i)) % @as(u32, @intCast(constants.chunk_size_z)));
        const col_idx: usize = lz * constants.chunk_size_x + lx;
        const top: i32 = ch.heightmaps.world_surface[col_idx];
        const spawn_y: i32 = if (top < 0) 1 else top + 1;
        return .{ .x = x, .y = spawn_y, .z = z };
    }

    fn ensureRegionState(self: *Simulation, ws: *WorldState, rp: RegionPos) !*RegionState {
        if (ws.regions.getPtr(rp)) |rs| return rs;
        const rs = RegionState{
            .chunks = try std.ArrayList(gs.Chunk).initCapacity(self.allocator, 0),
            .chunk_index = std.AutoHashMap(gs.ChunkPos, usize).init(self.allocator),
        };
        try ws.regions.put(rp, rs);
        return ws.regions.getPtr(rp).?;
    }

    /// Check if a specific chunk is present in the in-memory world state
    pub fn isChunkLoadedAt(self: *Simulation, world_key: []const u8, pos: gs.ChunkPos) bool {
        self.worlds_mutex.lock();
        defer self.worlds_mutex.unlock();
        const ws = self.worlds_state.getPtr(world_key) orelse return false;
        const rpos = RegionPos{ .x = @divFloor(pos.x, 32), .z = @divFloor(pos.z, 32) };
        const rs = ws.regions.getPtr(rpos) orelse return false;
        return rs.chunk_index.get(pos) != null;
    }

    pub fn start(self: *Simulation) !void {
        if (self.running.swap(true, .acquire)) return; // already running
        self.last_tick_ns = std.time.nanoTimestamp();
        self.thread = try std.Thread.spawn(.{}, tickThreadMain, .{self});
        // start worldgen threads
        const count: usize = if (self.worldgen_threads == 0) 1 else @intCast(self.worldgen_threads);
        try self.wg_threads.ensureTotalCapacity(self.allocator, count);
        var i: usize = 0;
        while (i < count) : (i += 1) {
            const th = try std.Thread.spawn(.{}, worldgenThreadMain, .{self});
            self.wg_threads.appendAssumeCapacity(th);
        }
    }

    pub fn stop(self: *Simulation) void {
        if (!self.running.swap(false, .release)) return;
        if (self.thread) |t| {
            t.join();
            self.thread = null;
        }
        // join worldgen threads
        var i: usize = 0;
        while (i < self.wg_threads.items.len) : (i += 1) {
            self.wg_threads.items[i].join();
        }
        self.wg_threads.clearRetainingCapacity();
    }
};
