const std = @import("std");
const sokol = @import("sokol");
const vox = @import("vox");
const slog = sokol.log;
const sg = sokol.gfx;
const sapp = sokol.app;
const sglue = sokol.glue;
const sdtx = sokol.debugtext;

fn loadOrFallback(allocator: std.mem.Allocator, path: []const u8) sg.Image {
    const png = vox.png;
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
        const tile: usize = 16;
        var pixels = allocator.alloc(u8, w * h * 4) catch {
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
const ids = @import("ids");
const registry = vox.registry;
const simulation = vox.simulation;
const player = vox.player;
const client_mod = vox.client;
const builtin = @import("builtin");
const vox_modules = vox.vox_modules;

const State = struct {
    pass_action: sg.PassAction = .{},
    sim: ?simulation.Simulation = null,
    ui_scale: f32 = 1.0,
    local_player_id: ?player.PlayerId = null,
    client: ?client_mod.Client = null,
    show_debug: bool = builtin.mode == .Debug,
};

var state: State = .{};
var gpa: std.heap.GeneralPurposeAllocator(.{}) = .{};
var reg: ?registry.Registry = null;

fn netDialThreadMain() void {
    // Minimal network dial to prove client build connects to a server
    const address = std.net.Address.parseIp4("127.0.0.1", 7777) catch |e| {
        std.debug.print("client: failed to parse default address: {s}\n", .{@errorName(e)});
        return;
    };
    var conn = std.net.tcpConnectToAddress(address) catch |e| {
        std.debug.print("client: failed to connect to server: {s}\n", .{@errorName(e)});
        return;
    };
    defer conn.close();
    std.debug.print("client: connected to server at 127.0.0.1:7777\n", .{});
}

export fn init() void {
    _ = gs.GameState;

    const allocator = gpa.allocator();

    // Initialize registry similar to full build
    var r = registry.Registry.init(allocator) catch return;
    r.ensureAir() catch {
        r.deinit();
        return;
    };
    // Load all modules and register their worldgen and content
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
    // (modules loaded above)

    reg = r;

    // graphics setup (UI origin at top-left per project rule)
    sg.setup(.{ .environment = sglue.environment(), .logger = .{ .func = slog.func } });

    // Client-only build owns a Simulation instance for client-side state/prediction, but does not run it.
    const sim = simulation.Simulation.initMemory(allocator, &reg.?, 4) catch {
        return;
    };
    state.sim = sim;

    // Create a local client connected to the local (non-authoritative) state holder.
    const pid = player.genUuidV4();
    state.local_player_id = pid;
    var cl = client_mod.Client.init(allocator, &state.sim.?, pid, "Exa Stencil") catch return;
    // NOTE: Do NOT call cl.connect() here; in client build we do not attach to an in-process authoritative SIM.
    cl.initGfx();
    state.client = cl;

    // Attempt a background network connection to the default server endpoint.
    _ = std.Thread.spawn(.{}, netDialThreadMain, .{}) catch {};
}

export fn frame() void {
    if (state.client) |*cl| {
        cl.frame();
    }
}

export fn cleanup() void {
    if (state.client) |*c| {
        c.deinit();
        state.client = null;
    }
    if (state.sim) |*s| {
        // No thread to stop in client build; just deinit storage.
        s.deinit();
        state.sim = null;
    }
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
        .window_title = "Vox Aetatum (Client)",
        .logger = .{ .func = slog.func },
        .win32_console_attach = true,
    });
}
