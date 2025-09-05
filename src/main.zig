const std = @import("std");
const sokol = @import("sokol");
const slog = sokol.log;
const sg = sokol.gfx;
const sapp = sokol.app;
const sglue = sokol.glue;
const gs = @import("gs.zig");
const worldgen = @import("worldgen.zig");
const registry = @import("registry.zig");

const State = struct {
    pass_action: sg.PassAction = .{},
};

var state: State = .{};
var gpa: std.heap.GeneralPurposeAllocator(.{}) = .{};
var reg: ?registry.Registry = null;

export fn init() void {
    // touch GS types to ensure they compile
    _ = gs.GameState;
    _ = worldgen.generateChunk; // ensure worldgen compiles

    // initialize registry with Air and a default biome
    const allocator = gpa.allocator();
    var r = registry.Registry.init(allocator) catch return;
    r.ensureAir() catch return;
    // ensure there's at least one biome
    _ = r.addBiome("core:default") catch return;
    reg = r;

    sg.setup(.{
        .environment = sglue.environment(),
        .logger = .{ .func = slog.func },
    });
    // Clear to solid black (default is zero-initialized, but set explicitly for clarity)
    state.pass_action.colors[0] = .{
        .load_action = .CLEAR,
        .clear_value = .{ .r = 0, .g = 0, .b = 0, .a = 1 },
    };
}

export fn frame() void {
    sg.beginPass(.{ .action = state.pass_action, .swapchain = sglue.swapchain() });
    sg.endPass();
    sg.commit();
}

export fn cleanup() void {
    sg.shutdown();
    if (reg) |*r| {
        r.deinit();
        reg = null;
    }
    _ = gpa.deinit();
}

pub fn main() void {
    sapp.run(.{
        .init_cb = init,
        .frame_cb = frame,
        .cleanup_cb = cleanup,
        .width = 1920,
        .height = 1080,
        .icon = .{ .sokol_default = true },
        .window_title = "Vox Aetatum",
        .logger = .{ .func = slog.func },
        .win32_console_attach = true,
    });
}
