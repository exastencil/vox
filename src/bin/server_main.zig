const std = @import("std");
const gs = @import("gs");
const ids = @import("ids");
const vox = @import("vox");
const registry = vox.registry;
const simulation = vox.simulation;
const worldgen = vox.worldgen;

pub fn main() !void {
    var gpa: std.heap.GeneralPurposeAllocator(.{}) = .{};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    // Ensure core modules compile
    _ = gs.GameState;
    _ = worldgen.generateChunk;

    // Initialize registry (headless)
    var reg = try registry.Registry.init(allocator);
    defer reg.deinit();
    try reg.ensureAir();
    const plains_biome = reg.addBiome("vox:plains") catch 0;
    _ = reg.addBlock("core:stone") catch {};
    const grass_id = reg.addBlock("vox:grass") catch 0;
    const dirt_id = reg.addBlock("vox:dirt") catch 0;
    reg.worlds.addWorldWithGen("vox:overworld", "The Overworld", 4, .{ .key = "core:superflat", .blocks = &[_]ids.BlockStateId{ grass_id, dirt_id }, .biomes = &[_]ids.BiomeId{plains_biome} }) catch {};

    // Start authoritative simulation thread
    var sim = try simulation.Simulation.initMemory(allocator, &reg, 4);
    defer {
        sim.stop();
        sim.deinit();
    }
    try sim.start();

    // Networking TODO (Zig 0.15): upgrade to std.net.tcpListen API
    // For now, just print that the server would listen and keep the simulation running.
    std.debug.print("vox-server (headless) started; networking not yet wired for Zig 0.15 API.\n", .{});
    // Keep process alive to let the simulation run; sleep in a loop
    while (true) {
        std.Thread.sleep(250_000_000); // 250ms
    }
}
