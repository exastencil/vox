const std = @import("std");
const simulation = @import("simulation.zig");
const player = @import("player.zig");
const sokol = @import("sokol");
const sg = sokol.gfx;
const sapp = sokol.app;
const sglue = sokol.glue;
const sdtx = sokol.debugtext;
const shd_mod = @import("shaders/chunk_shd.zig");

fn loadOrFallback(allocator: std.mem.Allocator, path: []const u8) sg.Image {
    const png = @import("png.zig");
    const loaded: ?png.Image = blk: {
        const res = png.loadFileRGBA8(allocator, path) catch |e| {
            std.log.warn("texture: failed to load '{s}': {s}", .{ path, @errorName(e) });
            break :blk null;
        };
        break :blk res;
    };
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
        // Generate a magenta/black checkerboard fallback with exactly 4 squares (2x2) per face
        const w: usize = 16;
        const h: usize = 16;
        const tile: usize = 8; // 2 squares across each axis, 8x8 each
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

const Camera = struct {
    pos: [3]f32 = .{ 0, 0, 0 },
    yaw: f32 = 0.0,
    pitch: f32 = 0.0,
    roll: f32 = 0.0,
    // Derived forward direction from yaw/pitch
    dir: [3]f32 = .{ 0, 0, -1 },

    fn computeDir(yaw: f32, pitch: f32) [3]f32 {
        // Assuming yaw around +Y, pitch around +X, radians
        const cy = @cos(yaw);
        const sy = @sin(yaw);
        const cp = @cos(pitch);
        const sp = @sin(pitch);
        // Forward in right-handed Y-up: x = cp*cy, y = sp, z = cp*sy
        return .{ cp * cy, sp, cp * sy };
    }

    fn setYawPitch(self: *Camera, yaw: f32, pitch: f32) void {
        self.yaw = yaw;
        self.pitch = pitch;
        self.dir = computeDir(yaw, pitch);
    }

    fn setDir(self: *Camera, dir_in: [3]f32) void {
        // normalize and update yaw/pitch to match
        const dx = dir_in[0];
        const dy = dir_in[1];
        const dz = dir_in[2];
        const len: f32 = @sqrt(dx * dx + dy * dy + dz * dz);
        if (len > 0.000001) {
            const nx = dx / len;
            const ny = dy / len;
            const nz = dz / len;
            self.dir = .{ nx, ny, nz };
            self.pitch = std.math.asin(std.math.clamp(ny, -1.0, 1.0));
            self.yaw = std.math.atan2(nz, nx);
        }
    }
};

const Vertex = struct { pos: [3]f32, uv: [2]f32 };

fn buildQuadVerts(out: []Vertex, x0: f32, y0: f32, size: f32) usize {
    if (out.len < 6) return 0;
    const x1: f32 = x0 + size;
    const y1: f32 = y0 + size;
    out[0] = .{ .pos = .{ x0, y0 }, .uv = .{ 0, 0 } };
    out[1] = .{ .pos = .{ x1, y0 }, .uv = .{ 1, 0 } };
    out[2] = .{ .pos = .{ x1, y1 }, .uv = .{ 1, 1 } };
    out[3] = .{ .pos = .{ x0, y0 }, .uv = .{ 0, 0 } };
    out[4] = .{ .pos = .{ x1, y1 }, .uv = .{ 1, 1 } };
    out[5] = .{ .pos = .{ x0, y1 }, .uv = .{ 0, 1 } };
    return 6;
}

const UvRect = struct { u0: f32, v0: f32, u1: f32, v1: f32, w: u32, h: u32 };

fn makeFallbackTile(allocator: std.mem.Allocator, size: usize) !struct { w: u32, h: u32, pixels: []u8 } {
    const w: usize = size;
    const h: usize = size;
    const tile: usize = @max(1, size / 2); // exactly 2x2 squares per face
    var pixels = try allocator.alloc(u8, w * h * 4);
    errdefer allocator.free(pixels);
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
    return .{ .w = @intCast(w), .h = @intCast(h), .pixels = pixels };
}

fn buildAtlasFromPaths(allocator: std.mem.Allocator, paths: []const []const u8) !struct {
    atlas_pixels: []u8,
    atlas_w: u32,
    atlas_h: u32,
    rects: []UvRect,
} {
    const png = @import("png.zig");
    // Load all images (or fallback tiles) and record dims
    const N = paths.len;
    var loaded = try allocator.alloc(struct { w: u32, h: u32, pixels: []u8 }, N);
    errdefer {
        // free any allocated pixels
        var j: usize = 0;
        while (j < loaded.len) : (j += 1) {
            if (loaded[j].pixels.len > 0) allocator.free(loaded[j].pixels);
        }
        allocator.free(loaded);
    }

    var max_w: u32 = 0;
    var max_h: u32 = 0;
    var i: usize = 0;
    while (i < N) : (i += 1) {
        const img: ?png.Image = blk: {
            const res = png.loadFileRGBA8(allocator, paths[i]) catch |e| {
                std.log.warn("texture: failed to load '{s}': {s}", .{ paths[i], @errorName(e) });
                break :blk null;
            };
            break :blk res;
        };
        if (img) |im| {
            loaded[i].w = im.width;
            loaded[i].h = im.height;
            loaded[i].pixels = im.pixels; // take ownership
        } else {
            const fb = try makeFallbackTile(allocator, 16);
            loaded[i].w = fb.w;
            loaded[i].h = fb.h;
            loaded[i].pixels = fb.pixels;
        }
        if (loaded[i].w > max_w) max_w = loaded[i].w;
        if (loaded[i].h > max_h) max_h = loaded[i].h;
    }
    const tile_w: u32 = if (max_w == 0) 16 else max_w;
    const tile_h: u32 = if (max_h == 0) 16 else max_h;

    // Simple grid packing: smallest cols where cols*cols >= N
    var cols: u32 = 1;
    while (@as(usize, cols) * @as(usize, cols) < N) cols += 1;
    const rows: u32 = @intCast((N + cols - 1) / cols);
    const atlas_w: u32 = cols * tile_w;
    const atlas_h: u32 = rows * tile_h;
    var atlas_pixels = try allocator.alloc(u8, @as(usize, atlas_w) * @as(usize, atlas_h) * 4);
    @memset(atlas_pixels, 0);

    var rects = try allocator.alloc(UvRect, N);

    // Blit each tile
    i = 0;
    while (i < N) : (i += 1) {
        const c: u32 = @intCast(i % cols);
        const r: u32 = @intCast(i / cols);
        const dst_x: u32 = c * tile_w;
        const dst_y: u32 = r * tile_h;
        // Copy row by row
        var py: u32 = 0;
        while (py < loaded[i].h) : (py += 1) {
            const src_off: usize = @as(usize, py) * @as(usize, loaded[i].w) * 4;
            const dst_row_off: usize = (@as(usize, dst_y + py) * @as(usize, atlas_w) + @as(usize, dst_x)) * 4;
            const copy_bytes: usize = @as(usize, loaded[i].w) * 4;
            @memcpy(atlas_pixels[dst_row_off .. dst_row_off + copy_bytes], loaded[i].pixels[src_off .. src_off + copy_bytes]);
        }
        rects[i] = .{
            .u0 = @as(f32, @floatFromInt(dst_x)) / @as(f32, @floatFromInt(atlas_w)),
            .v0 = @as(f32, @floatFromInt(dst_y)) / @as(f32, @floatFromInt(atlas_h)),
            .u1 = @as(f32, @floatFromInt(dst_x + loaded[i].w)) / @as(f32, @floatFromInt(atlas_w)),
            .v1 = @as(f32, @floatFromInt(dst_y + loaded[i].h)) / @as(f32, @floatFromInt(atlas_h)),
            .w = loaded[i].w,
            .h = loaded[i].h,
        };
    }

    // Free individual tiles; keep atlas_pixels and rects
    i = 0;
    while (i < N) : (i += 1) allocator.free(loaded[i].pixels);
    allocator.free(loaded);

    return .{ .atlas_pixels = atlas_pixels, .atlas_w = atlas_w, .atlas_h = atlas_h, .rects = rects };
}

fn addUniquePath(allocator: std.mem.Allocator, list: *std.ArrayList([]const u8), s: []const u8) !void {
    // Avoid duplicates by linear scan (small N)
    for (list.items) |p| if (std.mem.eql(u8, p, s)) return;
    try list.append(allocator, s);
}

fn findRectForPath(paths: []const []const u8, rects: []const UvRect, s: []const u8) ?UvRect {
    var i: usize = 0;
    while (i < paths.len) : (i += 1) if (std.mem.eql(u8, paths[i], s)) return rects[i];
    return null;
}

fn buildTextureAtlas(self: *Client) void {
    // Collect unique paths from resource registry
    var paths = std.ArrayList([]const u8).initCapacity(self.allocator, 0) catch {
        self.textures_ready = true;
        return;
    };
    defer paths.deinit(self.allocator);
    var it = self.sim.reg.resources.by_block.valueIterator();
    while (it.next()) |res| {
        switch (res.*) {
            .Uniform => |u| addUniquePath(self.allocator, &paths, u.all_path) catch {},
            .Facing => |f| {
                addUniquePath(self.allocator, &paths, f.face_path) catch {};
                addUniquePath(self.allocator, &paths, f.other_path) catch {};
            },
            .Void => {},
        }
    }
    if (paths.items.len == 0) {
        // Nothing to do: create a 1x1 fallback atlas
        var one = [_]u8{ 255, 0, 255, 255 };
        self.grass_img = sg.makeImage(.{ .width = 1, .height = 1, .pixel_format = .RGBA8, .data = .{ .subimage = .{ .{ sg.asRange(one[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} } } } });
        self.atlas_w = 1;
        self.atlas_h = 1;
        self.atlas_uv_scale = .{ 1, 1 };
        self.atlas_uv_offset = .{ 0, 0 };
        // Set per-face defaults to match the selected default region
        self.atlas_uv_top_scale = self.atlas_uv_scale;
        self.atlas_uv_top_offset = self.atlas_uv_offset;
        self.atlas_uv_side_scale = self.atlas_uv_scale;
        self.atlas_uv_side_offset = self.atlas_uv_offset;
        self.textures_ready = true;
        return;
    }

    const built = buildAtlasFromPaths(self.allocator, paths.items) catch {
        // Fallback as above on failure
        var one = [_]u8{ 255, 0, 255, 255 };
        self.grass_img = sg.makeImage(.{ .width = 1, .height = 1, .pixel_format = .RGBA8, .data = .{ .subimage = .{ .{ sg.asRange(one[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} } } } });
        self.atlas_w = 1;
        self.atlas_h = 1;
        self.atlas_uv_scale = .{ 1, 1 };
        self.atlas_uv_offset = .{ 0, 0 };
        self.atlas_uv_top_scale = self.atlas_uv_scale;
        self.atlas_uv_top_offset = self.atlas_uv_offset;
        self.atlas_uv_side_scale = self.atlas_uv_scale;
        self.atlas_uv_side_offset = self.atlas_uv_offset;
        self.textures_ready = true;
        return;
    };
    defer self.allocator.free(built.atlas_pixels);
    defer self.allocator.free(built.rects);

    // Upload atlas image to GPU
    self.grass_img = sg.makeImage(.{
        .width = @intCast(built.atlas_w),
        .height = @intCast(built.atlas_h),
        .pixel_format = .RGBA8,
        .data = .{ .subimage = .{
            .{ sg.asRange(built.atlas_pixels[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
            .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
            .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
            .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
            .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
            .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
        } },
    });
    self.atlas_w = built.atlas_w;
    self.atlas_h = built.atlas_h;

    // Choose a default atlas region to use for the current single-material demo mesh.
    // Prefer minecraft:dirt (uniform), otherwise fall back to grass_block face, then stone, then first tile.
    var chosen: bool = false;
    if (!chosen) {
        if (self.sim.reg.resources.get("minecraft:dirt")) |r_dirt| switch (r_dirt) {
            .Uniform => |u| if (findRectForPath(paths.items, built.rects, u.all_path)) |rc| {
                self.atlas_uv_offset = .{ rc.u0, rc.v0 };
                self.atlas_uv_scale = .{ rc.u1 - rc.u0, rc.v1 - rc.v0 };
                chosen = true;
            },
            else => {},
        };
    }
    if (!chosen) {
        if (self.sim.reg.resources.get("minecraft:grass")) |res| switch (res) {
            .Facing => |f| if (findRectForPath(paths.items, built.rects, f.face_path)) |rc| {
                self.atlas_uv_offset = .{ rc.u0, rc.v0 };
                self.atlas_uv_scale = .{ rc.u1 - rc.u0, rc.v1 - rc.v0 };
                chosen = true;
            },
            .Uniform => |u| if (findRectForPath(paths.items, built.rects, u.all_path)) |rc| {
                self.atlas_uv_offset = .{ rc.u0, rc.v0 };
                self.atlas_uv_scale = .{ rc.u1 - rc.u0, rc.v1 - rc.v0 };
                chosen = true;
            },
            .Void => {},
        };
    }
    if (!chosen) {
        if (self.sim.reg.resources.get("minecraft:stone")) |r_stone| switch (r_stone) {
            .Uniform => |u| if (findRectForPath(paths.items, built.rects, u.all_path)) |rc| {
                self.atlas_uv_offset = .{ rc.u0, rc.v0 };
                self.atlas_uv_scale = .{ rc.u1 - rc.u0, rc.v1 - rc.v0 };
                chosen = true;
            },
            else => {},
        };
    }
    if (!chosen and built.rects.len > 0) {
        const rc = built.rects[0];
        self.atlas_uv_offset = .{ rc.u0, rc.v0 };
        self.atlas_uv_scale = .{ rc.u1 - rc.u0, rc.v1 - rc.v0 };
    }

    // Compute per-face transforms
    // Top: prefer grass_block face, fallback to dirt, then stone, then default
    var top_set = false;
    if (!top_set) {
        if (self.sim.reg.resources.get("minecraft:grass_block")) |res| switch (res) {
            .Facing => |f| if (findRectForPath(paths.items, built.rects, f.face_path)) |rc| {
                self.atlas_uv_top_offset = .{ rc.u0, rc.v0 };
                self.atlas_uv_top_scale = .{ rc.u1 - rc.u0, rc.v1 - rc.v0 };
                top_set = true;
            },
            else => {},
        };
    }
    if (!top_set) {
        if (self.sim.reg.resources.get("minecraft:dirt")) |r_dirt2| switch (r_dirt2) {
            .Uniform => |u| if (findRectForPath(paths.items, built.rects, u.all_path)) |rc| {
                self.atlas_uv_top_offset = .{ rc.u0, rc.v0 };
                self.atlas_uv_top_scale = .{ rc.u1 - rc.u0, rc.v1 - rc.v0 };
                top_set = true;
            },
            else => {},
        };
    }
    if (!top_set) {
        self.atlas_uv_top_offset = self.atlas_uv_offset;
        self.atlas_uv_top_scale = self.atlas_uv_scale;
    }

    // Side: prefer grass_block other (dirt), fallback to dirt, then stone, then default
    var side_set = false;
    if (!side_set) {
        if (self.sim.reg.resources.get("minecraft:grass_block")) |res2| switch (res2) {
            .Facing => |f| if (findRectForPath(paths.items, built.rects, f.other_path)) |rc| {
                self.atlas_uv_side_offset = .{ rc.u0, rc.v0 };
                self.atlas_uv_side_scale = .{ rc.u1 - rc.u0, rc.v1 - rc.v0 };
                side_set = true;
            },
            else => {},
        };
    }
    if (!side_set) {
        if (self.sim.reg.resources.get("minecraft:dirt")) |r_dirt3| switch (r_dirt3) {
            .Uniform => |u| if (findRectForPath(paths.items, built.rects, u.all_path)) |rc| {
                self.atlas_uv_side_offset = .{ rc.u0, rc.v0 };
                self.atlas_uv_side_scale = .{ rc.u1 - rc.u0, rc.v1 - rc.v0 };
                side_set = true;
            },
            else => {},
        };
    }
    if (!side_set) {
        self.atlas_uv_side_offset = self.atlas_uv_offset;
        self.atlas_uv_side_scale = self.atlas_uv_scale;
    }

    self.textures_ready = true;
}

pub const Client = struct {
    allocator: std.mem.Allocator,
    sim: *simulation.Simulation,
    player_id: player.PlayerId,
    account_name: []const u8,
    connected: bool = false,
    // Connection/ready state
    ready: bool = false,

    // Texture/atlas preloading state
    textures_ready: bool = false,
    // UV transform defaults for the current demo material
    atlas_uv_scale: [2]f32 = .{ 1, 1 },
    atlas_uv_offset: [2]f32 = .{ 0, 0 },
    // Per-face transforms for the simple terrain surface (top uses grass top; sides use dirt)
    atlas_uv_top_scale: [2]f32 = .{ 1, 1 },
    atlas_uv_top_offset: [2]f32 = .{ 0, 0 },
    atlas_uv_side_scale: [2]f32 = .{ 1, 1 },
    atlas_uv_side_offset: [2]f32 = .{ 0, 0 },

    // UI / rendering state
    pass_action: sg.PassAction = .{},
    ui_scale: f32 = 2.0,
    show_debug: bool = true,
    show_atlas_overlay: bool = false,

    // Minimal chunk renderer state
    shd: sg.Shader = .{},
    pip: sg.Pipeline = .{},
    quad_pip: sg.Pipeline = .{},
    vbuf: sg.Buffer = .{},
    ui_vbuf: sg.Buffer = .{},
    bind: sg.Bindings = .{},
    // We reuse grass_* for the atlas image/view binding for now
    grass_img: sg.Image = .{},
    grass_view: sg.View = .{},
    sampler: sg.Sampler = .{},
    atlas_w: u32 = 1,
    atlas_h: u32 = 1,

    // GPU buffer capacity tracking
    vbuf_capacity_bytes: usize = 0,
    ui_vbuf_capacity_bytes: usize = 0,

    // Camera mirrored from player entity (position only; orientation is client-local)
    camera: Camera = .{},

    // Local view state (client-predicted, decoupled from simulation)
    local_yaw: f32 = 0.0,
    local_pitch: f32 = 0.0,
    local_dir: [3]f32 = .{ 1, 0, 0 },
    local_orient_initialized: bool = false,
    last_look_tick_sent: u64 = 0,

    // Debug: track last observed position for delta reporting
    dbg_last_pos: [3]f32 = .{ 0, 0, 0 },

    // Input state
    move_forward: bool = false,
    move_back: bool = false,
    move_left: bool = false,
    move_right: bool = false,
    jump_pressed: bool = false, // edge-triggered

    // Snapshot monitoring
    latest_snapshot: simulation.Snapshot = .{
        .tick = 0,
        .actual_tps = 0,
        .rolling_tps = 0,
        .dt_sec = 0,
        .world_seed = 0,
        .section_count_y = 0,
        .chunk_count = 0,
        .player_count = 0,
        .dynamic_entity_count = 0,
    },
    last_seen_tick: u64 = 0,

    pub fn init(allocator: std.mem.Allocator, sim: *simulation.Simulation, player_id: player.PlayerId, account_name: []const u8) !Client {
        const acc = try allocator.dupe(u8, account_name);
        return .{ .allocator = allocator, .sim = sim, .player_id = player_id, .account_name = acc };
    }

    // Enable/disable raw mouse input if supported by the sokol app wrapper
    fn setRawMouse(on: bool) void {
        if (@hasDecl(sapp, "rawMouseSupported") and @hasDecl(sapp, "enableRawMouse")) {
            if (sapp.rawMouseSupported()) {
                sapp.enableRawMouse(on);
            }
        } else {}
    }

    pub fn deinit(self: *Client) void {
        self.allocator.free(self.account_name);
    }

    pub fn connect(self: *Client) !void {
        if (self.connected) return;
        try self.sim.connectPlayer(self.player_id, self.account_name);
        self.connected = true;
        self.ready = false;
        // Ensure server creates/associates an entity for this player
        _ = self.sim.ensurePlayerEntity(self.player_id) catch {};
        // Do not block here; frame() will show "Connecting…" and poll readiness until the
        // player's chunk is loaded.
    }

    pub fn initGfx(self: *Client) void {
        // setup debug text and default pass action
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
        sdtx.font(4);
        self.ui_scale = 2.0;
        self.pass_action.colors[0] = .{ .load_action = .CLEAR, .clear_value = .{ .r = 0, .g = 0, .b = 0, .a = 1 } };
        self.show_debug = true;

        // shader: use shdc-generated cross-platform shader
        const backend = sg.queryBackend();
        if (@hasDecl(shd_mod, "shaderDesc")) {
            self.shd = sg.makeShader(shd_mod.shaderDesc(backend));
        } else if (@hasDecl(shd_mod, "shader_desc")) {
            self.shd = sg.makeShader(shd_mod.shader_desc(backend));
        } else if (@hasDecl(shd_mod, "chunkShaderDesc")) {
            self.shd = sg.makeShader(shd_mod.chunkShaderDesc(backend));
        } else if (@hasDecl(shd_mod, "chunk_shader_desc")) {
            self.shd = sg.makeShader(shd_mod.chunk_shader_desc(backend));
        } else {
            @compileError("shdc-generated shader module does not expose a known descriptor function");
        }

        var pdesc: sg.PipelineDesc = .{};
        pdesc.label = "chunk-pipeline";
        pdesc.shader = self.shd;
        pdesc.layout.attrs[0].format = .FLOAT3;
        pdesc.layout.attrs[1].format = .FLOAT2;
        pdesc.primitive_type = .TRIANGLES;
        pdesc.cull_mode = .BACK;
        // Front-face winding should match our emitted CCW triangles
        pdesc.face_winding = .CCW;
        pdesc.depth = .{ .compare = .LESS_EQUAL, .write_enabled = true };
        // ensure pipeline color/depth target matches swapchain format
        const desc = sg.queryDesc();
        pdesc.color_count = 1;
        pdesc.colors[0].pixel_format = desc.environment.defaults.color_format;
        self.pip = sg.makePipeline(pdesc);
        // dedicated 2D quad pipeline for overlay (no depth test, no culling)
        var qdesc: sg.PipelineDesc = .{};
        qdesc.label = "quad-pipeline";
        qdesc.shader = self.shd;
        qdesc.layout.attrs[0].format = .FLOAT3;
        qdesc.layout.attrs[1].format = .FLOAT2;
        qdesc.primitive_type = .TRIANGLES;
        qdesc.cull_mode = .NONE;
        qdesc.face_winding = .CCW;
        qdesc.depth = .{ .compare = .ALWAYS, .write_enabled = false };
        qdesc.color_count = 1;
        qdesc.colors[0].pixel_format = desc.environment.defaults.color_format;
        self.quad_pip = sg.makePipeline(qdesc);

        // dynamic vertex buffer; start with a generous capacity (e.g., 16384 verts)
        const initial_vcount: usize = 16 * 16 * 16; // plenty for top + sides on flat worlds
        const initial_bytes: usize = initial_vcount * @sizeOf(Vertex);
        self.vbuf = sg.makeBuffer(.{ .usage = .{ .vertex_buffer = true, .stream_update = true }, .size = @intCast(initial_bytes) });
        self.vbuf_capacity_bytes = initial_bytes;
        self.bind.vertex_buffers[0] = self.vbuf;
        // separate small dynamic vertex buffer for UI/overlay (e.g., 6 verts for a quad)
        const ui_initial_vcount: usize = 6;
        const ui_initial_bytes: usize = ui_initial_vcount * @sizeOf(Vertex);
        self.ui_vbuf = sg.makeBuffer(.{ .usage = .{ .vertex_buffer = true, .stream_update = true }, .size = @intCast(ui_initial_bytes) });
        self.ui_vbuf_capacity_bytes = ui_initial_bytes;
        self.pass_action.depth = .{ .load_action = .CLEAR, .clear_value = 1.0 };

        // Build texture atlas from registry resource paths
        buildTextureAtlas(self);

        // create a view and sampler for the atlas (reusing grass_* fields)
        self.grass_view = sg.makeView(.{ .texture = .{ .image = self.grass_img } });
        self.sampler = sg.makeSampler(.{ .min_filter = .NEAREST, .mag_filter = .NEAREST, .mipmap_filter = .NEAREST, .wrap_u = .CLAMP_TO_EDGE, .wrap_v = .CLAMP_TO_EDGE });
        // shader expects VIEW_tex_texture at slot 1 and SMP_tex_sampler at slot 2
        self.bind.views[1] = self.grass_view;
        self.bind.samplers[2] = self.sampler;

        // Initialize camera from player entity if available
        self.updateCameraFromPlayer();
        // Lock and enable raw mouse input on startup
        sapp.lockMouse(true);
        setRawMouse(true);
    }

    fn buildChunkSurfaceMesh(self: *Client, out: *std.ArrayList(Vertex)) usize {
        const uv_scale_top = self.atlas_uv_top_scale;
        const uv_offset_top = self.atlas_uv_top_offset;
        const uv_scale_side = self.atlas_uv_side_scale;
        const uv_offset_side = self.atlas_uv_side_offset;
        // Pad in atlas units to avoid cross-tile sampling (use full 1.0 texel)
        const pad_u: f32 = if (self.atlas_w > 0) (1.0 / @as(f32, @floatFromInt(self.atlas_w))) else 0.0;
        const pad_v: f32 = if (self.atlas_h > 0) (1.0 / @as(f32, @floatFromInt(self.atlas_h))) else 0.0;
        // Render all loaded chunks in the overworld
        const wk = "minecraft:overworld";
        self.sim.worlds_mutex.lock();
        defer self.sim.worlds_mutex.unlock();
        const ws = self.sim.worlds_state.getPtr(wk) orelse return 0;

        // Helpers to append a quad (two triangles) with CCW winding
        const addQuad = struct {
            fn wrap01(v: f32) f32 {
                const f = v - std.math.floor(v);
                // keep upper edge at 1.0 instead of wrapping to 0.0 when v is an exact integer > 0
                return if (f == 0.0 and v > 0.0) 1.0 else f;
            }
            fn call(list: *std.ArrayList(Vertex), v0: [3]f32, v1: [3]f32, v2: [3]f32, v3: [3]f32, uv0: [2]f32, uv1: [2]f32, uv2: [2]f32, uv3: [2]f32, uv_scale_p: [2]f32, uv_offset_p: [2]f32, pad_u_in: f32, pad_v_in: f32) void {
                // Wrap UVs into [0,1] (inclusive upper edge) per tile
                const w0x: f32 = wrap01(uv0[0]);
                const w0y: f32 = wrap01(uv0[1]);
                const w1x: f32 = wrap01(uv1[0]);
                const w1y: f32 = wrap01(uv1[1]);
                const w2x: f32 = wrap01(uv2[0]);
                const w2y: f32 = wrap01(uv2[1]);
                const w3x: f32 = wrap01(uv3[0]);
                const w3y: f32 = wrap01(uv3[1]);
                // inset by full texel in atlas space to avoid bleeding, and clamp scale accordingly
                const min_u = uv_offset_p[0] + pad_u_in;
                const min_v = uv_offset_p[1] + pad_v_in;
                const range_u = @max(0.0, uv_scale_p[0] - 2.0 * pad_u_in);
                const range_v = @max(0.0, uv_scale_p[1] - 2.0 * pad_v_in);
                const t0 = .{ min_u + w0x * range_u, min_v + w0y * range_v };
                const t1 = .{ min_u + w1x * range_u, min_v + w1y * range_v };
                const t2 = .{ min_u + w2x * range_u, min_v + w2y * range_v };
                const t3 = .{ min_u + w3x * range_u, min_v + w3y * range_v };
                // Emit triangles with standard CCW winding (v0->v1->v2, v0->v2->v3)
                list.appendAssumeCapacity(.{ .pos = v0, .uv = t0 });
                list.appendAssumeCapacity(.{ .pos = v1, .uv = t1 });
                list.appendAssumeCapacity(.{ .pos = v2, .uv = t2 });
                list.appendAssumeCapacity(.{ .pos = v0, .uv = t0 });
                list.appendAssumeCapacity(.{ .pos = v2, .uv = t2 });
                list.appendAssumeCapacity(.{ .pos = v3, .uv = t3 });
            }
        }.call;

        // convenience to get heightmap value with bounds check (outside => -1)
        const getH = struct {
            fn call(hm: *const [16 * 16]i32, x: i32, z: i32) i32 {
                if (x < 0 or x >= 16 or z < 0 or z >= 16) return -1;
                return hm[@as(usize, @intCast(z)) * 16 + @as(usize, @intCast(x))];
            }
        }.call;

        const verts_before: usize = out.items.len;

        // Iterate all regions and their chunks
        var reg_it = ws.regions.iterator();
        while (reg_it.next()) |rentry| {
            const rs_val = rentry.value_ptr.*; // copy for read-only access
            for (rs_val.chunks.items) |ch| {
                const base_x: f32 = @floatFromInt(ch.pos.x * 16);
                const base_z: f32 = @floatFromInt(ch.pos.z * 16);

                // Upper bound capacity per chunk (36 verts per column)
                const cur_len: usize = out.items.len;
                out.ensureTotalCapacity(self.allocator, cur_len + 16 * 16 * 6 * 6) catch {};

                // Build top faces and simple side faces from heightmap
                const hm = ch.heightmaps.world_surface;
                for (0..16) |z| {
                    for (0..16) |x| {
                        const xi: i32 = @intCast(x);
                        const zi: i32 = @intCast(z);
                        const h = hm[z * 16 + x];
                        if (h < 0) continue; // empty column
                        const y_top: f32 = @floatFromInt(h + 1);
                        const x0: f32 = base_x + @as(f32, @floatFromInt(x));
                        const x1: f32 = x0 + 1.0;
                        const z0: f32 = base_z + @as(f32, @floatFromInt(z));
                        const z1: f32 = z0 + 1.0;
                        // Top face (upward normal), CCW seen from +Y
                        addQuad(
                            out,
                            .{ x0, y_top, z0 },
                            .{ x0, y_top, z1 },
                            .{ x1, y_top, z1 },
                            .{ x1, y_top, z0 },
                            .{ 0, 0 },
                            .{ 0, 1 },
                            .{ 1, 1 },
                            .{ 1, 0 },
                            uv_scale_top,
                            uv_offset_top,
                            pad_u,
                            pad_v,
                        );
                        // Sides if neighbor lower (within the same chunk). For inter-chunk gaps, we'll
                        // handle neighbors later — this renders a simple "wall" at chunk borders for now.
                        const h_w = getH(&hm, xi - 1, zi);
                        if (h_w < h) {
                            var yb: i32 = h_w + 1;
                            const yt: i32 = h;
                            while (yb <= yt) : (yb += 1) {
                                const y0s: f32 = @floatFromInt(yb);
                                const y1s: f32 = y0s + 1.0;
                                addQuad(
                                    out,
                                    .{ x0, y0s, z1 },
                                    .{ x0, y1s, z1 },
                                    .{ x0, y1s, z0 },
                                    .{ x0, y0s, z0 }, // -X face (one-block tall)
                                    .{ 0, 0 },
                                    .{ 0, 1 },
                                    .{ 1, 1 },
                                    .{ 1, 0 },
                                    uv_scale_side,
                                    uv_offset_side,
                                    pad_u,
                                    pad_v,
                                );
                            }
                        }
                        const h_e = getH(&hm, xi + 1, zi);
                        if (h_e < h) {
                            var yb: i32 = h_e + 1;
                            const yt: i32 = h;
                            while (yb <= yt) : (yb += 1) {
                                const y0s: f32 = @floatFromInt(yb);
                                const y1s: f32 = y0s + 1.0;
                                addQuad(
                                    out,
                                    .{ x1, y0s, z0 },
                                    .{ x1, y1s, z0 },
                                    .{ x1, y1s, z1 },
                                    .{ x1, y0s, z1 }, // +X face (one-block tall)
                                    .{ 0, 0 },
                                    .{ 0, 1 },
                                    .{ 1, 1 },
                                    .{ 1, 0 },
                                    uv_scale_side,
                                    uv_offset_side,
                                    pad_u,
                                    pad_v,
                                );
                            }
                        }
                        const h_n = getH(&hm, xi, zi - 1);
                        if (h_n < h) {
                            var yb: i32 = h_n + 1;
                            const yt: i32 = h;
                            while (yb <= yt) : (yb += 1) {
                                const y0s: f32 = @floatFromInt(yb);
                                const y1s: f32 = y0s + 1.0;
                                addQuad(
                                    out,
                                    .{ x0, y0s, z0 },
                                    .{ x0, y1s, z0 },
                                    .{ x1, y1s, z0 },
                                    .{ x1, y0s, z0 }, // -Z face (one-block tall)
                                    .{ 0, 0 },
                                    .{ 0, 1 },
                                    .{ 1, 1 },
                                    .{ 1, 0 },
                                    uv_scale_side,
                                    uv_offset_side,
                                    pad_u,
                                    pad_v,
                                );
                            }
                        }
                        const h_s = getH(&hm, xi, zi + 1);
                        if (h_s < h) {
                            var yb: i32 = h_s + 1;
                            const yt: i32 = h;
                            while (yb <= yt) : (yb += 1) {
                                const y0s: f32 = @floatFromInt(yb);
                                const y1s: f32 = y0s + 1.0;
                                addQuad(
                                    out,
                                    .{ x1, y0s, z1 },
                                    .{ x1, y1s, z1 },
                                    .{ x0, y1s, z1 },
                                    .{ x0, y0s, z1 }, // +Z face (one-block tall)
                                    .{ 0, 0 },
                                    .{ 0, 1 },
                                    .{ 1, 1 },
                                    .{ 1, 0 },
                                    uv_scale_side,
                                    uv_offset_side,
                                    pad_u,
                                    pad_v,
                                );
                            }
                        }
                    }
                }
            }
        }
        return out.items.len - verts_before;
    }

    fn checkReady(self: *Client) void {
        if (self.ready) return;
        // Determine the player's current chunk and see if it's loaded
        // For now, assume the single-world prototype key
        const wk = "minecraft:overworld";
        // Read player position under the connections mutex
        self.sim.connections_mutex.lock();
        var has_pos = false;
        var cx: i32 = 0;
        var cz: i32 = 0;
        const idx_opt = self.sim.entity_by_player.get(self.player_id);
        if (idx_opt) |idx| {
            if (idx < self.sim.dynamic_entities.items.len) {
                const e = self.sim.dynamic_entities.items[idx];
                cx = @divTrunc(@as(i32, @intFromFloat(e.pos[0])), 16);
                cz = @divTrunc(@as(i32, @intFromFloat(e.pos[2])), 16);
                has_pos = true;
            }
        }
        self.sim.connections_mutex.unlock();
        if (!has_pos) return;
        if (self.sim.isChunkLoadedAt(wk, .{ .x = cx, .z = cz })) {
            self.ready = true;
            // Initialize camera now that the world around the player exists
            self.updateCameraFromPlayer();
        }
    }

    pub fn frame(self: *Client) void {
        // Keep camera mirrored to player each frame for now
        self.updateCameraFromPlayer();

        sg.beginPass(.{ .action = self.pass_action, .swapchain = sglue.swapchain() });

        // If textures not ready, draw a Texturing overlay and return early
        if (!self.textures_ready) {
            const scale: f32 = self.ui_scale;
            const w_px: f32 = @floatFromInt(sapp.width());
            const h_px: f32 = @floatFromInt(sapp.height());
            sdtx.canvas(w_px / scale, h_px / scale);
            sdtx.origin(0, 0);
            sdtx.font(4);
            const msg: [:0]const u8 = "Texturing...";
            const cols: f32 = (w_px / scale) / 8.0;
            const rows: f32 = (h_px / scale) / 8.0;
            const col_start: f32 = @max(0.0, (cols - @as(f32, @floatFromInt(msg.len))) / 2.0);
            const row_start: f32 = @max(0.0, rows / 2.0);
            sdtx.pos(col_start, row_start);
            sdtx.color3b(255, 255, 255);
            sdtx.puts(msg);
            sdtx.draw();
            sg.endPass();
            sg.commit();
            return;
        }

        // If not ready, draw a simple Connecting overlay and return early
        if (!self.ready) {
            const scale: f32 = self.ui_scale;
            const w_px: f32 = @floatFromInt(sapp.width());
            const h_px: f32 = @floatFromInt(sapp.height());
            sdtx.canvas(w_px / scale, h_px / scale);
            sdtx.origin(0, 0);
            sdtx.font(4);
            // Center-ish text: compute columns and rows (8x8 font)
            const msg: [:0]const u8 = "Connecting...";
            const cols: f32 = (w_px / scale) / 8.0;
            const rows: f32 = (h_px / scale) / 8.0;
            const col_start: f32 = @max(0.0, (cols - @as(f32, @floatFromInt(msg.len))) / 2.0);
            const row_start: f32 = @max(0.0, rows / 2.0);
            sdtx.pos(col_start, row_start);
            sdtx.color3b(255, 255, 255);
            sdtx.puts(msg);
            sdtx.draw();
            sg.endPass();
            sg.commit();
            // Poll readiness for next frame
            self.checkReady();
            return;
        }

        // Apply movement inputs to player entity (client-side write into sim)
        self.applyMovementInputs();
        // Send local look to simulation at most once per tick
        self.maybeSendLookToSim();

        // Build chunk surface mesh into a dynamic list
        var vert_list = std.ArrayList(Vertex).initCapacity(self.allocator, 0) catch return;
        defer vert_list.deinit(self.allocator);
        _ = self.buildChunkSurfaceMesh(&vert_list);

        // Prepare MVP
        const w_px: f32 = @floatFromInt(sapp.width());
        const h_px: f32 = @floatFromInt(sapp.height());
        const aspect_wh: f32 = if (h_px > 0) (w_px / h_px) else 1.0;
        const proj = makePerspective(60.0 * std.math.pi / 180.0, aspect_wh, 0.1, 1000.0);
        // Horizon lock (no-roll) view: strictly horizontal right and world +Y up
        const view = self.makeViewNoRoll(self.camera.pos, self.camera.yaw, self.camera.pitch);
        const mvp = matMul(proj, view);
        var vs_params: shd_mod.VsParams = .{ .mvp = mvp };

        // Optional continuous probe logging

        if (vert_list.items.len > 0 and self.grass_img.id != 0) {
            sg.applyPipeline(self.pip);
            sg.applyBindings(self.bind);
            sg.applyUniforms(0, sg.asRange(&vs_params));

            // Ensure vertex buffer capacity
            const needed_bytes: usize = vert_list.items.len * @sizeOf(Vertex);
            if (self.vbuf_capacity_bytes < needed_bytes) {
                if (self.vbuf.id != 0) sg.destroyBuffer(self.vbuf);
                self.vbuf = sg.makeBuffer(.{ .usage = .{ .vertex_buffer = true, .stream_update = true }, .size = @intCast(needed_bytes) });
                self.vbuf_capacity_bytes = needed_bytes;
                self.bind.vertex_buffers[0] = self.vbuf;
            }

            // Upload and draw
            sg.updateBuffer(self.vbuf, sg.asRange(vert_list.items));
            sg.draw(0, @intCast(vert_list.items.len), 1);
        }

        // Optional atlas overlay (top-left UI origin)
        if (self.show_atlas_overlay and self.grass_img.id != 0) {
            self.drawAtlasOverlay();
        }

        if (self.show_debug) {
            const scale: f32 = self.ui_scale;
            const w_px_dbg: f32 = @floatFromInt(sapp.width());
            const h_px_dbg: f32 = @floatFromInt(sapp.height());
            sdtx.canvas(w_px_dbg / scale, h_px_dbg / scale);
            sdtx.origin(0, 0);
            sdtx.font(4);

            const snap = self.pollSnapshot().*;
            var buf: [96:0]u8 = undefined;
            const text_slice = std.fmt.bufPrint(buf[0 .. buf.len - 1], "Tick: {d}  TPS: {d:.2}", .{ snap.tick, snap.rolling_tps }) catch "Ticks: ?";
            buf[text_slice.len] = 0;
            const text: [:0]const u8 = buf[0..text_slice.len :0];
            const cols_f: f32 = (w_px / scale) / 8.0;
            const text_cols_f: f32 = @floatFromInt(text.len);
            const col_start_f: f32 = @max(0.0, cols_f - text_cols_f - 1.0);
            sdtx.pos(col_start_f, 1.0);
            sdtx.color3b(255, 255, 255);
            sdtx.puts(text);

            // Left-side debug: player's position and look vector
            // Gather current player entity safely
            self.sim.connections_mutex.lock();
            var pos: [3]f32 = .{ 0, 0, 0 };
            var look: [3]f32 = .{ 0, 0, 0 };
            const idx_opt_dbg = self.sim.entity_by_player.get(self.player_id);
            if (idx_opt_dbg) |idx_dbg| {
                if (idx_dbg < self.sim.dynamic_entities.items.len) {
                    const e_dbg = self.sim.dynamic_entities.items[idx_dbg];
                    pos = e_dbg.pos;
                    look = e_dbg.look_dir;
                }
            }
            self.sim.connections_mutex.unlock();

            var pbuf: [96:0]u8 = undefined;
            const ptxt_slice = std.fmt.bufPrint(pbuf[0 .. pbuf.len - 1], "Position: {d:.1} {d:.1} {d:.1}", .{ pos[0], pos[1], pos[2] }) catch "Position: ?";
            pbuf[ptxt_slice.len] = 0;
            const ptxt: [:0]const u8 = pbuf[0..ptxt_slice.len :0];
            sdtx.pos(1.0, 1.0);
            sdtx.color3b(255, 255, 255);
            sdtx.puts(ptxt);

            var lbuf: [96:0]u8 = undefined;
            const ltxt_slice = std.fmt.bufPrint(lbuf[0 .. lbuf.len - 1], "Look: {d:.2} {d:.2} {d:.2}", .{ look[0], look[1], look[2] }) catch "Look: ?";
            lbuf[ltxt_slice.len] = 0;
            const ltxt: [:0]const u8 = lbuf[0..ltxt_slice.len :0];
            sdtx.pos(1.0, 2.0);
            sdtx.color3b(255, 255, 255);
            sdtx.puts(ltxt);

            sdtx.draw();
        }

        sg.endPass();
        sg.commit();
    }

    pub fn event(self: *Client, ev: sapp.Event) void {
        switch (ev.type) {
            .KEY_UP => {
                if (ev.key_code == .F3) self.show_debug = !self.show_debug;
                if (ev.key_code == .F5) self.show_atlas_overlay = !self.show_atlas_overlay;
                if (ev.key_code == .ESCAPE) {
                    // Release mouse when ESC is pressed
                    sapp.lockMouse(false);
                    setRawMouse(false);
                    self.clearMovementInputs();
                }
                switch (ev.key_code) {
                    .W => self.move_forward = false,
                    .S => self.move_back = false,
                    .A => self.move_left = false,
                    .D => self.move_right = false,
                    else => {},
                }
            },
            .FOCUSED => {
                // Lock the mouse when the window gains focus
                sapp.lockMouse(true);
                // Enable raw mouse input if supported
                setRawMouse(true);
            },
            .UNFOCUSED => {
                // Release the mouse when the window loses focus
                sapp.lockMouse(false);
                setRawMouse(false);
                self.clearMovementInputs();
            },
            .KEY_DOWN => {
                switch (ev.key_code) {
                    .W => self.move_forward = true,
                    .S => self.move_back = true,
                    .A => self.move_left = true,
                    .D => self.move_right = true,
                    .SPACE => self.jump_pressed = true,
                    else => {},
                }
            },
            .MOUSE_DOWN => {
                // If the mouse is unlocked (e.g., after ESC), clicking the window should re-lock it
                if (!sapp.mouseLocked()) {
                    sapp.lockMouse(true);
                    setRawMouse(true);
                }
            },
            .MOUSE_MOVE => {
                if (sapp.mouseLocked()) {
                    const sens: f32 = 0.002; // radians per pixel
                    const dx = ev.mouse_dx * sens;
                    _ = ev.mouse_dy; // ignore vertical mouse input (no pitch)

                    // Rotate look around +Y for horizontal mouse motion
                    var new_dir = self.local_dir;
                    if (dx != 0) {
                        const c = @cos(-dx); // right drag turns right
                        const s = @sin(-dx);
                        const x = new_dir[0];
                        const z = new_dir[2];
                        new_dir[0] = c * x - s * z;
                        new_dir[2] = s * x + c * z;
                    }
                    // normalize
                    const len = @sqrt(new_dir[0] * new_dir[0] + new_dir[1] * new_dir[1] + new_dir[2] * new_dir[2]);
                    if (len > 0.000001) {
                        new_dir[0] /= len;
                        new_dir[1] /= len;
                        new_dir[2] /= len;
                    }
                    self.local_dir = new_dir;
                    // update camera from dir (also updates yaw/pitch)
                    self.camera.setDir(self.local_dir);
                    self.local_yaw = self.camera.yaw;
                    self.local_pitch = self.camera.pitch;

                    // If not moving, constrain facing so look stays within 90° of facing (XZ)
                    if (!(self.move_forward or self.move_back or self.move_left or self.move_right)) {
                        // Use camera forward
                        const lpx = self.local_dir[0];
                        const lpz = self.local_dir[2];
                        const lplen = @sqrt(lpx * lpx + lpz * lpz);
                        if (lplen > 0.0001) {
                            const nlx = lpx / lplen;
                            const nlz = lpz / lplen;
                            // Read/adjust facing under mutex
                            self.sim.connections_mutex.lock();
                            const idx_opt_f = self.sim.entity_by_player.get(self.player_id);
                            if (idx_opt_f) |idx_f| {
                                if (idx_f < self.sim.dynamic_entities.items.len) {
                                    var e_f = &self.sim.dynamic_entities.items[idx_f];
                                    const fvx = e_f.facing_dir_xz[0];
                                    const fvz = e_f.facing_dir_xz[1];
                                    const dot = fvx * nlx + fvz * nlz;
                                    if (dot < 0) { // angle > 90°
                                        const facing_yaw = std.math.atan2(fvz, fvx);
                                        const look_yaw = std.math.atan2(nlz, nlx);
                                        var diff = look_yaw - facing_yaw;
                                        // wrap to [-pi, pi]
                                        if (diff > std.math.pi) diff -= 2 * std.math.pi;
                                        if (diff < -std.math.pi) diff += 2 * std.math.pi;
                                        const sign: f32 = if (diff >= 0) 1.0 else -1.0;
                                        const delta = @abs(diff) - (std.math.pi / 2.0);
                                        const new_yaw = facing_yaw + sign * delta;
                                        e_f.facing_dir_xz = .{ @cos(new_yaw), @sin(new_yaw) };
                                    }
                                }
                            }
                            self.sim.connections_mutex.unlock();
                        }
                    }
                }
            },
            else => {},
        }
    }

    pub fn shutdownGfx(self: *Client) void {
        _ = self; // currently nothing to do except sdtx, which main will shut down
    }

    // Pull the most recent snapshot from the simulation. Returns a pointer to the
    // internal latest_snapshot for convenience.
    pub fn pollSnapshot(self: *Client) *const simulation.Snapshot {
        const snap = self.sim.getSnapshot();
        if (snap.tick != self.last_seen_tick) {
            self.latest_snapshot = snap;
            self.last_seen_tick = snap.tick;
        }
        return &self.latest_snapshot;
    }

    // Update camera to mirror the player's entity position; keep orientation client-local
    fn maybeSendLookToSim(self: *Client) void {
        const snap = self.pollSnapshot().*;
        if (snap.tick == self.last_look_tick_sent) return;
        self.last_look_tick_sent = snap.tick;
        self.sim.connections_mutex.lock();
        const idx_opt = self.sim.entity_by_player.get(self.player_id);
        if (idx_opt) |idx| {
            if (idx < self.sim.dynamic_entities.items.len) {
                var e = &self.sim.dynamic_entities.items[idx];
                e.look_dir = self.local_dir;
            }
        }
        self.sim.connections_mutex.unlock();
    }

    // Update camera to mirror the player's entity position; keep orientation client-local
    fn updateCameraFromPlayer(self: *Client) void {
        self.sim.connections_mutex.lock();
        defer self.sim.connections_mutex.unlock();
        const idx_opt = self.sim.entity_by_player.get(self.player_id);
        if (idx_opt) |idx| {
            if (idx < self.sim.dynamic_entities.items.len) {
                const e = self.sim.dynamic_entities.items[idx];
                const eye_y: f32 = e.pos[1] + e.aabb_half_extents[1];
                self.camera.pos = .{ e.pos[0], eye_y, e.pos[2] };
                if (!self.local_orient_initialized) {
                    // Seed local orientation from sim once
                    const dx = e.look_dir[0];
                    const dy = e.look_dir[1];
                    const dz = e.look_dir[2];
                    const len: f32 = @sqrt(dx * dx + dy * dy + dz * dz);
                    if (len > 0.000001) {
                        const nx = dx / len;
                        const ny = dy / len;
                        const nz = dz / len;
                        self.local_pitch = std.math.asin(std.math.clamp(ny, -1.0, 1.0));
                        self.local_yaw = std.math.atan2(nz, nx);
                        const cp = @cos(self.local_pitch);
                        const sp = @sin(self.local_pitch);
                        const cy = @cos(self.local_yaw);
                        const sy = @sin(self.local_yaw);
                        self.local_dir = .{ cp * cy, sp, cp * sy };
                        self.local_orient_initialized = true;
                    }
                }
                // Always orient camera from local yaw/pitch, zero roll
                self.camera.setYawPitch(self.local_yaw, self.local_pitch);
                self.camera.roll = 0;
            }
        }
    }

    // Apply currently-pressed movement keys to the player entity
    fn applyMovementInputs(self: *Client) void {
        // Compute desired horizontal velocity from inputs relative to local yaw
        self.sim.connections_mutex.lock();
        defer self.sim.connections_mutex.unlock();
        const idx_opt = self.sim.entity_by_player.get(self.player_id);
        if (idx_opt) |idx| {
            if (idx < self.sim.dynamic_entities.items.len) {
                var e = &self.sim.dynamic_entities.items[idx];
                // While moving, keep facing synced to the horizontal component of the look vector
                if (self.move_forward or self.move_back or self.move_left or self.move_right) {
                    // Sync facing to camera forward projected onto XZ
                    const lx = self.local_dir[0];
                    const lz = self.local_dir[2];
                    const llen: f32 = @sqrt(lx * lx + lz * lz);
                    if (llen > 0.0001) {
                        e.facing_dir_xz = .{ lx / llen, lz / llen };
                    }
                }
                // Derive forward from yaw only (horizontal heading), independent of pitch
                const fwd_x: f32 = -@cos(self.local_yaw);
                const fwd_z: f32 = @sin(self.local_yaw);
                // Screen-right (world right) is cross(forward, up) on XZ: (-fwd_z, fwd_x)
                const right_x: f32 = -fwd_z;
                const right_z: f32 = fwd_x;

                var vx: f32 = 0;
                var vz: f32 = 0;
                if (self.move_forward) {
                    vx += fwd_x;
                    vz += fwd_z;
                }
                if (self.move_back) {
                    vx -= fwd_x;
                    vz -= fwd_z;
                }
                if (self.move_right) {
                    vx += right_x;
                    vz += right_z;
                }
                if (self.move_left) {
                    vx -= right_x;
                    vz -= right_z;
                }
                // Normalize if any input
                const mag: f32 = @sqrt(vx * vx + vz * vz);
                if (mag > 0.0001) {
                    vx /= mag;
                    vz /= mag;
                }
                const speed: f32 = 4.5; // m/s walking speed
                e.vel[0] = vx * speed;
                e.vel[2] = vz * speed;
                // While moving, ensure entity yaw aligns to facing on XZ (optional; keeps model heading)
                if (self.move_forward or self.move_back or self.move_left or self.move_right) {
                    const facing_yaw: f32 = std.math.atan2(e.facing_dir_xz[1], e.facing_dir_xz[0]);
                    e.yaw_pitch_roll = .{ facing_yaw, 0, 0 };
                }
                // Jump impulse on edge press if on ground
                if (self.jump_pressed and e.flags.on_ground) {
                    const g: f32 = 9.80665;
                    const target_h: f32 = 1.1; // slightly over 1 block
                    const v0: f32 = @sqrt(2.0 * g * target_h);
                    e.vel[1] = v0;
                    e.flags.on_ground = false;
                }
            }
        }
        // consume jump edge
        self.jump_pressed = false;
    }

    fn clearMovementInputs(self: *Client) void {
        self.move_forward = false;
        self.move_back = false;
        self.move_left = false;
        self.move_right = false;
        // zero horizontal velocity
        self.sim.connections_mutex.lock();
        const idx_opt = self.sim.entity_by_player.get(self.player_id);
        if (idx_opt) |idx| {
            if (idx < self.sim.dynamic_entities.items.len) {
                var e = &self.sim.dynamic_entities.items[idx];
                e.vel[0] = 0;
                e.vel[2] = 0;
            }
        }
        self.sim.connections_mutex.unlock();
    }

    // Basic 4x4 column-major matrix helpers
    fn matMul(a: [16]f32, b: [16]f32) [16]f32 {
        var r: [16]f32 = undefined;
        var i: usize = 0;
        while (i < 4) : (i += 1) {
            var j: usize = 0;
            while (j < 4) : (j += 1) {
                // column-major: r[col*4+row]
                r[i * 4 + j] =
                    a[0 * 4 + j] * b[i * 4 + 0] +
                    a[1 * 4 + j] * b[i * 4 + 1] +
                    a[2 * 4 + j] * b[i * 4 + 2] +
                    a[3 * 4 + j] * b[i * 4 + 3];
            }
        }
        return r;
    }

    fn makePerspective(fovy_radians: f32, aspect: f32, znear: f32, zfar: f32) [16]f32 {
        const f: f32 = 1.0 / @tan(fovy_radians / 2.0);
        var m: [16]f32 = [_]f32{0} ** 16;
        m[0] = f / aspect;
        m[5] = f;
        m[10] = (zfar + znear) / (znear - zfar);
        m[11] = -1.0;
        m[14] = (2.0 * zfar * znear) / (znear - zfar);
        return m;
    }

    fn makeOrthoTopLeft(width_px: f32, height_px: f32) [16]f32 {
        var m: [16]f32 = [_]f32{0} ** 16;
        // map x: [0..W] -> [-1..+1]
        m[0] = if (width_px != 0) (2.0 / width_px) else 0.0;
        // map y: [0..H] (top-down) -> [+1..-1]
        m[5] = if (height_px != 0) (-2.0 / height_px) else 0.0;
        m[10] = 1.0;
        m[15] = 1.0;
        // translation to top-left origin
        m[12] = -1.0;
        m[13] = 1.0;
        return m;
    }

    fn drawAtlasOverlay(self: *Client) void {
        const w_px: f32 = @floatFromInt(sapp.width());
        const h_px: f32 = @floatFromInt(sapp.height());
        const margin: f32 = 8.0;
        const aw: f32 = @floatFromInt(self.atlas_w);
        const ah: f32 = @floatFromInt(self.atlas_h);
        if (!(aw > 0 and ah > 0)) return;
        const max_w: f32 = w_px * 0.5;
        const max_h: f32 = h_px * 0.5;
        var scale: f32 = 1.0;
        const sx = if (aw != 0) (max_w / aw) else 1.0;
        const sy = if (ah != 0) (max_h / ah) else 1.0;
        scale = @min(sx, sy);
        if (scale > 1.0) scale = 1.0;
        const dw: f32 = aw * scale;
        const dh: f32 = ah * scale;
        const x0: f32 = margin;
        const y0: f32 = margin;
        const x1: f32 = x0 + dw;
        const y1: f32 = y0 + dh;
        var quad: [6]Vertex = .{
            .{ .pos = .{ x0, y0, 0 }, .uv = .{ 0, 0 } },
            .{ .pos = .{ x1, y0, 0 }, .uv = .{ 1, 0 } },
            .{ .pos = .{ x1, y1, 0 }, .uv = .{ 1, 1 } },
            .{ .pos = .{ x0, y0, 0 }, .uv = .{ 0, 0 } },
            .{ .pos = .{ x1, y1, 0 }, .uv = .{ 1, 1 } },
            .{ .pos = .{ x0, y1, 0 }, .uv = .{ 0, 1 } },
        };
        const ortho = makeOrthoTopLeft(w_px, h_px);
        var vs_params: shd_mod.VsParams = .{ .mvp = ortho };
        sg.applyPipeline(self.quad_pip);
        // use a separate UI vertex buffer to avoid multiple updates to the same buffer in one frame
        var bind_ui = self.bind;
        bind_ui.vertex_buffers[0] = self.ui_vbuf;
        sg.applyBindings(bind_ui);
        sg.applyUniforms(0, sg.asRange(&vs_params));
        const needed_bytes: usize = quad.len * @sizeOf(Vertex);
        if (self.ui_vbuf_capacity_bytes < needed_bytes) {
            if (self.ui_vbuf.id != 0) sg.destroyBuffer(self.ui_vbuf);
            self.ui_vbuf = sg.makeBuffer(.{ .usage = .{ .vertex_buffer = true, .stream_update = true }, .size = @intCast(needed_bytes) });
            self.ui_vbuf_capacity_bytes = needed_bytes;
        }
        sg.updateBuffer(self.ui_vbuf, sg.asRange(&quad));
        sg.draw(0, @intCast(quad.len), 1);
    }

    fn makeViewConv(eye: [3]f32, forward: [3]f32, up_in: [3]f32, conventional: bool) [16]f32 {
        // normalize forward
        const f0 = forward[0];
        const f1 = forward[1];
        const f2 = forward[2];
        const flen: f32 = @sqrt(f0 * f0 + f1 * f1 + f2 * f2);
        const fx = if (flen > 0.000001) (f0 / flen) else 1.0;
        const fy = if (flen > 0.000001) (f1 / flen) else 0.0;
        const fz = if (flen > 0.000001) (f2 / flen) else 0.0;
        // s = normalize(cross(f, up)) (conventional) or cross(up, f)
        var sx0: f32 = undefined;
        var sy0: f32 = undefined;
        var sz0: f32 = undefined;
        if (conventional) {
            // cross(f, up)
            sx0 = fy * up_in[2] - fz * up_in[1];
            sy0 = fz * up_in[0] - fx * up_in[2];
            sz0 = fx * up_in[1] - fy * up_in[0];
        } else {
            // cross(up, f)
            sx0 = up_in[1] * fz - up_in[2] * fy;
            sy0 = up_in[2] * fx - up_in[0] * fz;
            sz0 = up_in[0] * fy - up_in[1] * fx;
        }
        const slen: f32 = @sqrt(sx0 * sx0 + sy0 * sy0 + sz0 * sz0);
        const sx = sx0 / slen;
        const sy = sy0 / slen;
        const sz = sz0 / slen;
        // u = cross(s, f)
        const ux = sy * fz - sz * fy;
        const uy = sz * fx - sx * fz;
        const uz = sx * fy - sy * fx;
        return makeViewFromBasis(eye, .{ sx, sy, sz }, .{ ux, uy, uz }, .{ fx, fy, fz });
    }

    fn makeViewFromBasis(eye: [3]f32, s: [3]f32, u: [3]f32, f: [3]f32) [16]f32 {
        var m: [16]f32 = undefined;
        m[0] = s[0];
        m[4] = u[0];
        // Use OpenGL-style view with -f in the third column (camera looks down -Z in view space)
        m[8] = -f[0];
        m[12] = 0;
        m[1] = s[1];
        m[5] = u[1];
        m[9] = -f[1];
        m[13] = 0;
        m[2] = s[2];
        m[6] = u[2];
        m[10] = -f[2];
        m[14] = 0;
        m[3] = 0;
        m[7] = 0;
        m[11] = 0;
        m[15] = 1;
        var t: [16]f32 = [_]f32{0} ** 16;
        t[0] = 1;
        t[5] = 1;
        t[10] = 1;
        t[15] = 1;
        t[12] = -eye[0];
        t[13] = -eye[1];
        t[14] = -eye[2];
        return matMul(m, t);
    }

    fn makeViewNoRoll(self: *Client, eye: [3]f32, yaw: f32, pitch: f32) [16]f32 {
        _ = self;
        const cy = @cos(yaw);
        const sy = @sin(yaw);
        const cp = @cos(pitch);
        const sp = @sin(pitch);
        // forward from yaw/pitch
        const f: [3]f32 = .{ cp * cy, sp, cp * sy };
        // force horizon: right is strictly horizontal, up is world +Y
        const s: [3]f32 = .{ -sy, 0, cy };
        const u: [3]f32 = .{ 0, 1, 0 };
        return makeViewFromBasis(eye, s, u, f);
    }
};
