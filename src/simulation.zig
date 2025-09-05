const std = @import("std");
const gs = @import("gs.zig");
const ids = @import("ids.zig");
const registry = @import("registry.zig");
const worldgen = @import("worldgen.zig");
const constants = @import("constants.zig");

pub const StorageMode = union(enum) {
    memory,
    save_path: []const u8,
};

pub const SimulationOptions = struct {
    allocator: std.mem.Allocator,
    registries: *registry.Registry,
    mode: StorageMode,
    section_count_y: u16,
};

pub const Snapshot = struct {
    tick: u64,
    actual_tps: f64,
    rolling_tps: f64,
    dt_sec: f64,
    world_seed: u64,
    section_count_y: u16,
    chunk_count: usize,
};

pub const Simulation = struct {
    allocator: std.mem.Allocator,
    reg: *registry.Registry,
    mode: StorageMode,
    section_count_y: u16,
    world_seed: u64,
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

    chunks: std.ArrayList(gs.Chunk),

    pub fn init(opts: SimulationOptions) !Simulation {
        var seed_bytes: [8]u8 = undefined;
        std.crypto.random.bytes(&seed_bytes);
        const seed = std.mem.readInt(u64, &seed_bytes, .little);

        var sim: Simulation = .{
            .allocator = opts.allocator,
            .reg = opts.registries,
            .mode = opts.mode,
            .section_count_y = opts.section_count_y,
            .world_seed = seed,
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
            .chunks = undefined,
        };
        // initialize snapshot buffers
        sim.publishSnapshot();

        // Bootstrap vox:default: only core:void biome. Generate an 8x8 around origin.
        // For symmetry around origin, generate ranges [-4..3] inclusive on both axes.
        const half: i32 = 4; // 8-wide dimension
        const total: usize = @intCast((half * 2) * (half * 2));
        sim.chunks = try std.ArrayList(gs.Chunk).initCapacity(opts.allocator, total);

        // Ensure prereqs exist in registry: air and core:void
        try sim.reg.ensureAir();
        const maybe_void = sim.reg.findBiome("core:void");
        const void_biome_id: ids.BiomeId = maybe_void orelse try sim.reg.addBiome("core:void");
        const air_id: ids.BlockStateId = 0; // by convention from ensureAir()

        var z: i32 = -half;
        while (z < half) : (z += 1) {
            var x: i32 = -half;
            while (x < half) : (x += 1) {
                const pos = gs.ChunkPos{ .x = x, .z = z };
                const chunk = try worldgen.generateVoidChunk(opts.allocator, sim.section_count_y, pos, air_id, void_biome_id);
                sim.chunks.appendAssumeCapacity(chunk);
            }
        }

        return sim;
    }

    pub fn initMemory(allocator: std.mem.Allocator, reg: *registry.Registry, section_count_y: u16) !Simulation {
        return init(.{ .allocator = allocator, .registries = reg, .mode = .memory, .section_count_y = section_count_y });
    }

    pub fn deinit(self: *Simulation) void {
        for (self.chunks.items) |*c| worldgen.deinitChunk(self.allocator, c);
        self.chunks.deinit(self.allocator);
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

    fn publishSnapshot(self: *Simulation) void {
        const cur = self.snapshot_index.load(.acquire);
        const next: u8 = cur ^ 1;
        var rolling_tps_val: f64 = self.actual_tps;
        if (self.window_count > 0 and self.window_sum_ns > 0) {
            const avg_ns_f: f64 = @as(f64, @floatFromInt(self.window_sum_ns)) / @as(f64, @floatFromInt(self.window_count));
            if (avg_ns_f > 0) rolling_tps_val = 1_000_000_000.0 / avg_ns_f;
        }
        const snap = Snapshot{
            .tick = self.tick_counter.load(.monotonic),
            .actual_tps = self.actual_tps,
            .rolling_tps = rolling_tps_val,
            .dt_sec = self.last_tick_dt_sec,
            .world_seed = self.world_seed,
            .section_count_y = self.section_count_y,
            .chunk_count = self.chunks.items.len,
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

    pub fn start(self: *Simulation) !void {
        if (self.running.swap(true, .acquire)) return; // already running
        self.last_tick_ns = std.time.nanoTimestamp();
        self.thread = try std.Thread.spawn(.{}, tickThreadMain, .{self});
    }

    pub fn stop(self: *Simulation) void {
        if (!self.running.swap(false, .release)) return;
        if (self.thread) |t| {
            t.join();
            self.thread = null;
        }
    }
};
