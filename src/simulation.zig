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
    // metrics
    last_tick_dt_sec: f64 = 0.0,
    actual_tps: f64 = 0.0,
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
            .chunks = undefined,
        };

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
        const target_ns: i128 = @intFromFloat(1_000_000_000.0 / self.target_tps);
        const t0: i128 = std.time.nanoTimestamp();

        // TODO: real simulation step phases go here (terrain, entities, etc.)
        // increment tick counter
        _ = self.tick_counter.fetchAdd(1, .monotonic);

        const t1: i128 = std.time.nanoTimestamp();
        var dt_ns: i128 = t1 - t0;
        if (dt_ns < 0) dt_ns = 0;
        self.last_tick_dt_sec = @as(f64, @floatFromInt(dt_ns)) / 1_000_000_000.0;
        if (self.last_tick_dt_sec > 0) {
            self.actual_tps = 1.0 / self.last_tick_dt_sec;
        }
        // sleep to maintain target TPS if able
        if (dt_ns < target_ns) {
            const remain: i128 = target_ns - dt_ns;
            std.Thread.sleep(@intCast(remain));
        }
    }

    fn tickThreadMain(self_ptr: *Simulation) void {
        while (self_ptr.running.load(.acquire)) {
            self_ptr.tick();
        }
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
