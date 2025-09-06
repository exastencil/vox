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
const player = @import("player.zig");
const builtin = @import("builtin");
const mc = @import("assets/mc_textures_1_18.zig");

const State = struct {
    pass_action: sg.PassAction = .{},
    sim: ?simulation.Simulation = null,
    ui_scale: f32 = 1.0,
    // optional local test client id
    local_player_id: ?player.PlayerId = null,
    // debug overlay toggle
    show_debug: bool = builtin.mode == .Debug,
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
    // ensure common biomes/blocks used by builtin gens
    const plains_biome = r.addBiome("vox:plains") catch 0;
    _ = r.addBlock("core:stone") catch {};
    const grass_id = r.addBlock("vox:grass") catch 0;
    const dirt_id = r.addBlock("vox:dirt") catch 0;

    // register default world with superflat worldgen
    r.worlds.addWorldWithGen("vox:overworld", "The Overworld", 4, .{
        .key = "core:superflat",
        .blocks = &[_]ids.BlockStateId{ grass_id, dirt_id },
        .biomes = &[_]ids.BiomeId{plains_biome},
    }) catch {
        r.deinit();
        return;
    };
    // register built-in worldgen types
    r.worldgen.add("core:void", "Void", registry.WorldGen.selectVoid, registry.WorldGen.selectBlockVoid) catch {
        r.deinit();
        return;
    };
    r.worldgen.add("core:superflat", "Superflat", registry.WorldGen.selectSuperflat, registry.WorldGen.selectBlockSuperflat) catch {
        r.deinit();
        return;
    };
    // ensure common biomes/blocks used by builtin gens
    _ = r.addBiome("vox:plains") catch {};
    _ = r.addBlock("core:stone") catch {};

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

    // Build textures from embedded Minecraft 1.18 assets (testing-only, not for distribution)
    if (reg) |*rp| {
        const dirt_tex = sg.makeImage(.{
            .width = @intCast(mc.dirt_width),
            .height = @intCast(mc.dirt_height),
            .pixel_format = .RGBA8,
            .data = .{ .subimage = .{
                .{ sg.asRange(mc.dirt_pixels[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
            } },
        });
        const grass_top_tex = sg.makeImage(.{
            .width = @intCast(mc.grass_top_width),
            .height = @intCast(mc.grass_top_height),
            .pixel_format = .RGBA8,
            .data = .{ .subimage = .{
                .{ sg.asRange(mc.grass_top_pixels[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
            } },
        });
        const grass_side_tex = sg.makeImage(.{
            .width = @intCast(mc.grass_side_width),
            .height = @intCast(mc.grass_side_height),
            .pixel_format = .RGBA8,
            .data = .{ .subimage = .{
                .{ sg.asRange(mc.grass_side_pixels[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
            } },
        });
        rp.resources.setUniform("vox:dirt", dirt_tex) catch {};
        rp.resources.setFacing("vox:grass", grass_top_tex, grass_side_tex) catch {};
    }

    // Create in-memory simulation with a default height of 4 sections (4*32 = 128)
    const sim = simulation.Simulation.initMemory(allocator, &reg.?, 4) catch {
        return;
    };
    state.sim = sim;

    // start simulation thread (decoupled from rendering)
    state.sim.?.start() catch return;

    // TEMP: instantiate a local test client and connect to the SIM
    const pid = player.genUuidV4();
    state.local_player_id = pid;
    state.sim.?.connectPlayer(pid, "Exa Stencil") catch return;
}

export fn frame() void {
    // rendering only; simulation ticks on its own thread

    sg.beginPass(.{ .action = state.pass_action, .swapchain = sglue.swapchain() });

    if (state.show_debug) {
        // draw tick counter in upper-right
        // Scale the virtual canvas to enlarge text: canvas = window_size / ui_scale
        const scale: f32 = state.ui_scale;
        const w_px: f32 = @floatFromInt(sapp.width());
        const h_px: f32 = @floatFromInt(sapp.height());
        sdtx.canvas(w_px / scale, h_px / scale);
        sdtx.origin(0, 0);
        // ensure C64 font in case other code changes it later
        sdtx.font(4);

        if (state.sim) |s| {
            const snap = s.getSnapshot();
            var buf: [96:0]u8 = undefined;
            const text_slice = std.fmt.bufPrint(buf[0 .. buf.len - 1], "tick: {d}  tps: {d:.2}  players: {d}", .{ snap.tick, snap.rolling_tps, snap.player_count }) catch "tick: ?";
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
    }

    sg.endPass();
    sg.commit();
}

export fn cleanup() void {
    // stop and deinit simulation if present
    if (state.sim) |*s| {
        s.stop();
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

export fn event(e: [*c]const sapp.Event) void {
    const ev: sapp.Event = e.*;
    switch (ev.type) {
        .KEY_UP => {
            if (ev.key_code == .F3) {
                state.show_debug = !state.show_debug;
            }
        },
        else => {},
    }
}

pub fn main() void {
    sapp.run(.{
        .init_cb = init,
        .frame_cb = frame,
        .cleanup_cb = cleanup,
        .event_cb = event,
        .width = 1920,
        .height = 1080,
        .icon = .{ .sokol_default = true },
        .window_title = "Vox Aetatum",
        .logger = .{ .func = slog.func },
        .win32_console_attach = true,
    });
}
