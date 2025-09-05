const std = @import("std");
const sokol = @import("sokol");
const slog = sokol.log;
const sg = sokol.gfx;
const sapp = sokol.app;
const sglue = sokol.glue;
const gs = @import("gs.zig");
const worldgen = @import("worldgen.zig");
const registry = @import("registry.zig");
const ids = @import("ids.zig");

const State = struct {
    pass_action: sg.PassAction = .{},
    chunks: std.ArrayList(gs.Chunk),
    world_seed: u64 = 0,
    section_count_y: u16 = 0,
};

var state: State = .{ .chunks = undefined };
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
    // register default void biome
    const void_biome = r.addBiome("core:void") catch {
        r.deinit();
        return;
    };
    reg = r;

    // graphics setup
    sg.setup(.{
        .environment = sglue.environment(),
        .logger = .{ .func = slog.func },
    });
    // Clear to solid black (default is zero-initialized, but set explicitly for clarity)
    state.pass_action.colors[0] = .{
        .load_action = .CLEAR,
        .clear_value = .{ .r = 0, .g = 0, .b = 0, .a = 1 },
    };

    // worldgen: create void world seed and generate chunks in a 3-chunk radius around (0,0)
    state.section_count_y = 4; // 4*32 = 128 world height for now
    // random seed
    var seed_bytes: [8]u8 = undefined;
    std.crypto.random.bytes(&seed_bytes);
    state.world_seed = std.mem.readInt(u64, &seed_bytes, .little);

    const radius: i32 = 3;
    const expected: usize = @intCast((radius * 2 + 1) * (radius * 2 + 1));
    state.chunks = std.ArrayList(gs.Chunk).initCapacity(allocator, expected) catch {
        return;
    };

    // use registered ids
    const air_id: ids.BlockStateId = 0; // ensured by ensureAir
    const void_biome_id: ids.BiomeId = void_biome;

    var z: i32 = -radius;
    while (z <= radius) : (z += 1) {
        var x: i32 = -radius;
        while (x <= radius) : (x += 1) {
            const pos = gs.ChunkPos{ .x = x, .z = z };
            const chunk = worldgen.generateVoidChunk(allocator, state.section_count_y, pos, air_id, void_biome_id) catch {
                // if any generation fails, free previously generated chunks and abort init
                for (state.chunks.items) |*c| worldgen.deinitChunk(allocator, c);
                state.chunks.deinit(allocator);
                return;
            };
            state.chunks.appendAssumeCapacity(chunk);
        }
    }
}

export fn frame() void {
    sg.beginPass(.{ .action = state.pass_action, .swapchain = sglue.swapchain() });
    sg.endPass();
    sg.commit();
}

export fn cleanup() void {
    // free generated chunks
    const allocator = gpa.allocator();
    if (@hasField(State, "chunks")) {
        if (@TypeOf(state.chunks) == std.ArrayList(gs.Chunk)) {
            for (state.chunks.items) |*c| worldgen.deinitChunk(allocator, c);
            state.chunks.deinit(allocator);
        }
    }

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
