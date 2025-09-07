const std = @import("std");
const gs = @import("gs");
const ids = @import("ids");
const vox = @import("vox");
const registry = vox.registry;
const simulation = vox.simulation;
const worldgen = vox.worldgen;
const vox_modules = vox.vox_modules;

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

    // Load all modules and register their worldgen and content
    inline for (vox_modules.modules) |m| {
        reg.modules.add(m.key, m.display_name, m.version, m.requires, m.worldgen) catch {};
        var j: usize = 0;
        while (j < m.worldgen.len) : (j += 1) {
            const wg = m.worldgen[j];
            reg.worldgen.add(wg.key, wg.display_name, wg) catch {};
        }
        if (m.init) |f| {
            const ctx: *anyopaque = @ptrCast(&reg);
            f(ctx) catch {};
        }
    }

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
