const std = @import("std");
const sokol = @import("sokol");
const slog = sokol.log;
const sg = sokol.gfx;
const sapp = sokol.app;
const sglue = sokol.glue;
const sdtx = sokol.debugtext;

fn loadOrFallback(allocator: std.mem.Allocator, path: []const u8) sg.Image {
    const png = @import("png.zig");
    const loaded = png.loadFileRGBA8(allocator, path) catch null;
    if (loaded) |img| {
        defer allocator.free(img.pixels);
        return sg.makeImage(.{
            .width = @intCast(img.width),
            .height = @intCast(img.height),
            .pixel_format = .RGBA8,
            .data = .{ .subimage = .{
                .{ sg.asRange(img.pixels[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
            } },
        });
    } else {
        // Generate a magenta/black checkerboard as a visible missing-texture fallback
        const w: usize = 128;
        const h: usize = 128;
        const tile: usize = 16; // 8x8 tiles across
        var pixels = allocator.alloc(u8, w * h * 4) catch {
            // As a last resort, create a 1x1 magenta pixel
            var one = [_]u8{ 255, 0, 255, 255 };
            return sg.makeImage(.{
                .width = 1,
                .height = 1,
                .pixel_format = .RGBA8,
                .data = .{ .subimage = .{
                    .{ sg.asRange(one[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                    .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                    .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                    .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                    .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                    .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                } },
            });
        };
        defer allocator.free(pixels);
        var y: usize = 0;
        while (y < h) : (y += 1) {
            var x: usize = 0;
            while (x < w) : (x += 1) {
                const cx = (x / tile) & 1;
                const cy = (y / tile) & 1;
                const is_magenta = (cx ^ cy) == 0;
                const idx = (y * w + x) * 4;
                if (is_magenta) {
                    pixels[idx + 0] = 255;
                    pixels[idx + 1] = 0;
                    pixels[idx + 2] = 255;
                    pixels[idx + 3] = 255;
                } else {
                    pixels[idx + 0] = 0;
                    pixels[idx + 1] = 0;
                    pixels[idx + 2] = 0;
                    pixels[idx + 3] = 255;
                }
            }
        }
        return sg.makeImage(.{
            .width = @intCast(w),
            .height = @intCast(h),
            .pixel_format = .RGBA8,
            .data = .{ .subimage = .{
                .{ sg.asRange(pixels[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
            } },
        });
    }
}
const gs = @import("gs");
const worldgen = @import("worldgen.zig");
const registry = @import("registry.zig");
const ids = @import("ids");
const simulation = @import("simulation.zig");
const player = @import("player.zig");
const client_mod = @import("client.zig");
const builtin = @import("builtin");
const vox_modules = @import("generated/vox_modules.zig");

const State = struct {
    pass_action: sg.PassAction = .{},
    sim: ?simulation.Simulation = null,
    ui_scale: f32 = 1.0,
    // local test client and player id
    local_player_id: ?player.PlayerId = null,
    client: ?client_mod.Client = null,
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
    // register default void biome (id unused here; Simulation may look it up)
    _ = r.addBiome("core:void") catch {
        r.deinit();
        return;
    };

    // Load all modules discovered at build-time and register any worldgen they expose
    inline for (vox_modules.modules) |m| {
        r.modules.add(m.key, m.display_name, m.version, m.requires, m.worldgen) catch {
            r.deinit();
            return;
        };
        var j: usize = 0;
        while (j < m.worldgen.len) : (j += 1) {
            const wg = m.worldgen[j];
            r.worldgen.add(wg.key, wg.display_name, wg) catch {
                r.deinit();
                return;
            };
        }
        if (m.init) |f| {
            const ctx: *anyopaque = @ptrCast(&r);
            f(ctx) catch {
                r.deinit();
                return;
            };
        }
    }

    reg = r;

    // graphics setup
    sg.setup(.{
        .environment = sglue.environment(),
        .logger = .{ .func = slog.func },
    });

    // Create in-memory simulation with a default height of 4 sections (4*32 = 128)
    const sim = simulation.Simulation.initMemory(allocator, &reg.?, 4) catch {
        return;
    };
    state.sim = sim;

    // start simulation thread (decoupled from rendering)
    state.sim.?.start() catch return;

    // Create a local client and connect it to the simulation
    const pid = player.genUuidV4();
    state.local_player_id = pid;
    var cl = client_mod.Client.init(allocator, &state.sim.?, pid, "Exa Stencil") catch return;
    cl.connect() catch return;
    cl.initGfx();
    state.client = cl;
}

export fn frame() void {
    // rendering only; simulation ticks on its own thread

    if (state.client) |*cl| {
        cl.frame();
    }
}

export fn cleanup() void {
    // stop and deinit simulation if present
    if (state.sim) |*s| {
        s.stop();
        s.deinit();
        state.sim = null;
    }
    // deinit client
    if (state.client) |*c| {
        c.deinit();
        state.client = null;
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
    if (state.client) |*cl| {
        cl.event(ev);
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
