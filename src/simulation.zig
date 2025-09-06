const std = @import("std");
const gs = @import("gs.zig");
const ids = @import("ids.zig");
const registry = @import("registry.zig");
const worldgen = @import("worldgen.zig");
const constants = @import("constants.zig");
const player = @import("player.zig");

pub const StorageMode = union(enum) {
    memory,
    save_path: []const u8,
};

pub const SimulationOptions = struct {
    allocator: std.mem.Allocator,
    registries: *registry.Registry,
    mode: StorageMode,
    world_key: []const u8 = "vox:overworld",
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
    section_count_y: u16,
    regions: std.AutoHashMap(RegionPos, RegionState),
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
        };
        // initialize snapshot buffers
        sim.publishSnapshot();
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
                            const cx: i32 = @divTrunc(@as(i32, @intFromFloat(e.pos[0])), @as(i32, constants.chunk_size_x));
                            const cz: i32 = @divTrunc(@as(i32, @intFromFloat(e.pos[2])), @as(i32, constants.chunk_size_z));
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

            // Generate up to a few missing chunks per pass
            var generated: u32 = 0;
            var it2 = desired.keyIterator();
            while (it2.next()) |pos_ptr| {
                const pos = pos_ptr.*;
                // compute the player's world (assume default for now)
                const world_key = "vox:overworld";

                // get world def for section count
                const wd = self_ptr.reg.worlds.get(world_key) orelse continue;

                // locate region
                const rpos = RegionPos{ .x = @divTrunc(pos.x, 32), .z = @divTrunc(pos.z, 32) };

                // check if chunk exists
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

                // generate void chunk for now
                const maybe_void = self_ptr.reg.findBiome("core:void");
                const void_biome_id: ids.BiomeId = maybe_void orelse self_ptr.reg.addBiome("core:void") catch 0;
                const air_id: ids.BlockStateId = 0;
                const chunk = worldgen.generateVoidChunk(alloc, wd.section_count_y, pos, air_id, void_biome_id) catch continue;

                // fold into simulation
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

                generated += 1;
                if (generated >= 4) break; // limit per pass
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
        _ = self;
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
            const wk = self.reg.worlds.get("vox:overworld") orelse return;
            _ = self.player_world.put(pid, wk.key) catch {};
        }
        _ = try self.ensurePlayerEntityLocked(pid);
    }

    fn ensurePlayerEntityLocked(self: *Simulation, pid: player.PlayerId) !usize {
        if (self.entity_by_player.get(pid)) |idx| return idx;
        // spawn at origin for now
        const eid = self.next_eid;
        self.next_eid += 1;
        const rec: gs.EntityRecord = .{
            .eid = eid,
            .kind = 0, // 0 reserved for player entity kind for now
            .pos = .{ 0, 0, 0 },
            .vel = .{ 0, 0, 0 },
            .yaw_pitch_roll = .{ 0, 0, 0 },
            .aabb_half_extents = .{ 0.5, 0.9, 0.5 },
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
        const ws = WorldState{
            .key = key_copy,
            .section_count_y = wd.section_count_y,
            .regions = std.AutoHashMap(RegionPos, RegionState).init(self.allocator),
        };
        try self.worlds_state.put(key_copy, ws);
        return self.worlds_state.getPtr(key_copy).?;
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
