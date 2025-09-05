const std = @import("std");
const sokol = @import("sokol");
const slog = sokol.log;
const sg = sokol.gfx;
const sapp = sokol.app;
const sglue = sokol.glue;
const sdtx = sokol.debugtext;
const gs = @import("gs.zig");
const worldgen = @import("worldgen.zig");
const registry = @import("registry.zig");
const ids = @import("ids.zig");
const simulation = @import("simulation.zig");
const client = @import("client.zig");

const State = struct {
    pass_action: sg.PassAction = .{},
    sim: ?simulation.Simulation = null,
    cl: ?client.Client = null,
    ui_scale: f32 = 1.0,
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
    // If any of the following steps fail, make sure to deinit the partially-initialized registry
    r.ensureAir() catch {
        r.deinit();
        return;
    };
    // register default void biome (id unused here; Simulation will look it up)
    _ = r.addBiome("core:void") catch {
        r.deinit();
        return;
    };
    reg = r;

    // graphics setup
    sg.setup(.{
        .environment = sglue.environment(),
        .logger = .{ .func = slog.func },
    });
    // debug text setup with builtin fonts
    sdtx.setup(.{ .fonts = .{
        sdtx.fontKc853(),
        sdtx.fontKc854(),
        sdtx.fontZ1013(),
        sdtx.fontCpc(),
        sdtx.fontC64(),
        sdtx.fontOric(),
        .{},
        .{},
    } });
    // Use C64 font by default (index 4 in the array above)
    sdtx.font(4);

    // UI scale (higher -> larger text). On hi-dpi monitors, 2.0 is a good start.
    state.ui_scale = 2.0;
    // Clear to solid black (default is zero-initialized, but set explicitly for clarity)
    state.pass_action.colors[0] = .{
        .load_action = .CLEAR,
        .clear_value = .{ .r = 0, .g = 0, .b = 0, .a = 1 },
    };

    // Create in-memory simulation with a default height of 4 sections (4*32 = 128)
    const sim = simulation.Simulation.initMemory(allocator, &reg.?, 4) catch {
        return;
    };
    state.sim = sim;

    // initialize client with fixed-timestep simulation at 20 TPS
    const cl_local = client.Client.init(&state.sim.?, 20.0);
    state.cl = cl_local;
}

export fn frame() void {
    // advance simulation at fixed 20 TPS using client accumulator
    if (state.cl) |*c| {
        c.update();
    }

    sg.beginPass(.{ .action = state.pass_action, .swapchain = sglue.swapchain() });

    // draw tick counter in upper-right only
    // Scale the virtual canvas to enlarge text: canvas = window_size / ui_scale
    const scale: f32 = state.ui_scale;
    const w_px: f32 = @floatFromInt(sapp.width());
    const h_px: f32 = @floatFromInt(sapp.height());
    sdtx.canvas(w_px / scale, h_px / scale);
    sdtx.origin(0, 0);
    // ensure C64 font in case other code changes it later
    sdtx.font(4);

    // Right-aligned tick text (row 1)
    if (state.sim) |s| {
        var buf: [64:0]u8 = undefined;
        const text_slice = std.fmt.bufPrint(buf[0 .. buf.len - 1], "tick: {d}", .{s.tick_counter}) catch "tick: ?";
        buf[text_slice.len] = 0; // ensure 0-terminated for sdtx
        const text: [:0]const u8 = buf[0..text_slice.len :0];
        const cols_f: f32 = (w_px / scale) / 8.0;
        const text_cols_f: f32 = @floatFromInt(text.len);
        const col_start_f: f32 = @max(0.0, cols_f - text_cols_f - 1.0);
        sdtx.pos(col_start_f, 1.0);
        sdtx.color3b(255, 255, 255);
        sdtx.puts(text);
    }

    sdtx.draw();

    sg.endPass();
    sg.commit();
}

export fn cleanup() void {
    // drop client first
    if (state.cl) |_| {
        state.cl = null;
    }

    // deinit simulation if present
    if (state.sim) |*s| {
        s.deinit();
        state.sim = null;
    }

    // shutdown debug text first
    sdtx.shutdown();

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
