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
    tick_counter: u64 = 0,
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
        // Placeholder: follow docs/wiki/architecture-simulation-update-order.md in future
        self.tick_counter += 1;
    }
};
