const std = @import("std");
const simulation = @import("simulation.zig");
const player = @import("player.zig");
const constants = @import("constants");
const gs = @import("gs");
const sokol = @import("sokol");
const physics = @import("physics.zig");
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

const Vertex = struct { pos: [3]f32, uv: [2]f32, uv_off: [2]f32, uv_scale: [2]f32 };

inline fn wrap01m(v: f32) f32 {
    const f = v - std.math.floor(v);
    return if (f == 0.0 and v > 0.0) 1.0 else f;
}

inline fn emitQuad(allocator: std.mem.Allocator, list: *std.ArrayList(Vertex), v0: [3]f32, v1: [3]f32, v2: [3]f32, v3: [3]f32, uv0: [2]f32, uv1: [2]f32, uv2: [2]f32, uv3: [2]f32, uv_scale_p: [2]f32, uv_offset_p: [2]f32) void {
    // Store tile-local UVs (may be >1) and atlas transform; shader applies fract and transform
    const off = uv_offset_p;
    const sc = uv_scale_p;
    list.append(allocator, .{ .pos = v0, .uv = uv0, .uv_off = off, .uv_scale = sc }) catch return;
    list.append(allocator, .{ .pos = v1, .uv = uv1, .uv_off = off, .uv_scale = sc }) catch return;
    list.append(allocator, .{ .pos = v2, .uv = uv2, .uv_off = off, .uv_scale = sc }) catch return;
    list.append(allocator, .{ .pos = v0, .uv = uv0, .uv_off = off, .uv_scale = sc }) catch return;
    list.append(allocator, .{ .pos = v2, .uv = uv2, .uv_off = off, .uv_scale = sc }) catch return;
    list.append(allocator, .{ .pos = v3, .uv = uv3, .uv_off = off, .uv_scale = sc }) catch return;
}

// Cached mesh types (region-level aggregation)
const SectionMesh = struct { first: u32 = 0, count: u32 = 0 };
const SectionDraw = struct { chunk_pos: gs.ChunkPos, sy: u16, first: u32, count: u32 };
const RegionMesh = struct {
    vbuf: sg.Buffer = .{},
    draws: []SectionDraw,
    built_chunk_count: usize = 0,
    dirty: bool = false,
    inflight: bool = false,
    last_built_frame: u32 = 0,
};

const MeshingJob = struct { rpos: simulation.RegionPos };
const MeshingResult = struct { rpos: simulation.RegionPos, built_chunk_count: usize, verts: []Vertex, draws: []SectionDraw };

// Snapshot types for background meshing
const SectionSnapshot = struct {
    palette: []u32,
    blocks_indices_bits: []u32,
};

const ChunkSnapshot = struct {
    pos: gs.ChunkPos,
    sections: []SectionSnapshot,
};

const RegionSnapshot = struct {
    rpos: simulation.RegionPos,
    base_cx: i32,
    base_cz: i32,
    section_count_y: u16,
    chunks: []ChunkSnapshot,
    // 32x32 grid mapping local (x,z) -> chunk index or -1
    grid_map: []i32,
};

// Temporary slice-ref structs for capturing under lock, then copying outside the lock
const SectionSliceRefs = struct { pal_ptr: [*]const u32, pal_len: usize, bits_ptr: [*]const u32, bits_len: usize };
const ChunkSliceRefs = struct { pos: gs.ChunkPos, sections: []SectionSliceRefs };
const RegionSliceRefs = struct { rpos: simulation.RegionPos, section_count_y: u16, base_cx: i32, base_cz: i32, chunks: []ChunkSliceRefs };

const REGION_MESH_BUDGET: usize = 128; // generous, regions are fewer than chunks

const AtlasTransform = struct {
    offset: [2]f32,
    scale: [2]f32,
};

fn bitsFor(n: usize) u6 {
    if (n <= 1) return 1;
    var v: usize = n - 1;
    var b: u6 = 0;
    while (v > 0) : (v >>= 1) b += 1;
    return if (b == 0) 1 else b;
}

fn unpackBitsGet(src: []const u32, idx: usize, bits_per_index: u6) u32 {
    // read logical element at index idx where each element has bits_per_index bits
    const bit_off: usize = idx * @as(usize, bits_per_index);
    const word_idx: usize = bit_off >> 5; // /32
    const bit_idx: u5 = @intCast(bit_off & 31);
    if (bits_per_index == 32) return src[word_idx];
    const mask: u32 = (@as(u32, 1) << @intCast(bits_per_index)) - 1;
    const rem: u6 = @as(u6, 32) - @as(u6, bit_idx);
    if (bits_per_index <= rem) {
        return (src[word_idx] >> bit_idx) & mask;
    } else {
        const lo_bits: u6 = rem;
        const hi_bits: u6 = @intCast(bits_per_index - lo_bits);
        const lo_mask: u32 = (@as(u32, 1) << @as(u5, @intCast(lo_bits))) - 1;
        const lo_val: u32 = (src[word_idx] >> bit_idx) & lo_mask;
        const hi_val: u32 = (src[word_idx + 1] & ((@as(u32, 1) << @as(u5, @intCast(hi_bits))) - 1)) << @as(u5, @intCast(lo_bits));
        return lo_val | hi_val;
    }
}

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

    // Clear any previous atlas map entries
    self.atlas_uv_by_path.clearRetainingCapacity();

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

    // Populate atlas UV map for each source path
    var i_paths: usize = 0;
    while (i_paths < paths.items.len) : (i_paths += 1) {
        const rc = built.rects[i_paths];
        const tx: AtlasTransform = .{ .offset = .{ rc.u0, rc.v0 }, .scale = .{ rc.u1 - rc.u0, rc.v1 - rc.v0 } };
        // Note: keys are owned by registry; we do not dup or free them here
        _ = self.atlas_uv_by_path.put(paths.items[i_paths], tx) catch {};
    }

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
    // Atlas UV transforms per texture path
    atlas_uv_by_path: std.StringHashMap(AtlasTransform) = undefined,

    // Cached region meshes keyed by region position
    region_mesh_cache: std.AutoHashMap(simulation.RegionPos, RegionMesh) = undefined,

    // GPU buffer capacity tracking
    vbuf_capacity_bytes: usize = 0,
    ui_vbuf_capacity_bytes: usize = 0,

    // View frustum (derived each frame from MVP) for culling
    frustum_planes: [6][4]f32 = [_][4]f32{.{ 0, 0, 0, 0 }} ** 6,
    frustum_valid: bool = false,
    prev_frustum_planes: [6][4]f32 = [_][4]f32{.{ 0, 0, 0, 0 }} ** 6,
    prev_frustum_valid: bool = false,
    // Last visible frame for chunks and regions
    last_visible_frame_chunk: std.AutoHashMap(gs.ChunkPos, u32) = undefined,
    last_visible_frame_region: std.AutoHashMap(simulation.RegionPos, u32) = undefined,
    frame_index: u32 = 0,

    // Meshing throttle
    rebuild_budget_per_frame: u32 = 1,
    rebuilds_issued_this_frame: u32 = 0,
    rebuild_cooldown_frames: u32 = 8,

    // Background mesher
    mesher_mutex: std.Thread.Mutex = .{},
    mesher_cv: std.Thread.Condition = .{},
    mesher_jobs: std.ArrayList(MeshingJob) = undefined,
    mesher_results: std.ArrayList(MeshingResult) = undefined,
    mesher_threads: std.ArrayList(std.Thread) = undefined,
    mesher_running: bool = false,
    mesher_started: bool = false,
    adopt_budget_per_frame: u32 = 1,

    // Debug counters
    dbg_last_scheduled: u32 = 0,
    dbg_last_adopted: u32 = 0,

    // Camera mirrored from player entity (position only; orientation is client-local)
    camera: Camera = .{},

    // Local view state (client-predicted, decoupled from simulation)
    local_yaw: f32 = 0.0,
    local_pitch: f32 = 0.0,
    local_dir: [3]f32 = .{ 1, 0, 0 },
    local_orient_initialized: bool = false,
    last_look_tick_sent: u64 = 0,

    // Local predicted player kinematics used for rendering
    local_pos: [3]f32 = .{ 0, 0, 0 },
    local_vel: [3]f32 = .{ 0, 0, 0 },
    local_aabb_half_extents: [3]f32 = .{ 0.375, 0.875, 0.2 }, // keep in sync with Simulation default
    local_on_ground: bool = false,
    last_move_tick_sent: u64 = 0,
    jump_pending: bool = false,

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

    // FPS measurement
    fps_last_ns: i128 = 0,
    fps_accum_ns: i128 = 0,
    fps_accum_frames: u32 = 0,
    fps_value: f32 = 0.0,

    pub fn init(allocator: std.mem.Allocator, sim: *simulation.Simulation, player_id: player.PlayerId, account_name: []const u8) !Client {
        const acc = try allocator.dupe(u8, account_name);
        return .{
            .allocator = allocator,
            .sim = sim,
            .player_id = player_id,
            .account_name = acc,
            .atlas_uv_by_path = std.StringHashMap(AtlasTransform).init(allocator),
            .region_mesh_cache = std.AutoHashMap(simulation.RegionPos, RegionMesh).init(allocator),
            .last_visible_frame_chunk = std.AutoHashMap(gs.ChunkPos, u32).init(allocator),
            .last_visible_frame_region = std.AutoHashMap(simulation.RegionPos, u32).init(allocator),
            .mesher_jobs = std.ArrayList(MeshingJob).initCapacity(allocator, 0) catch unreachable,
            .mesher_results = std.ArrayList(MeshingResult).initCapacity(allocator, 0) catch unreachable,
            .mesher_threads = std.ArrayList(std.Thread).initCapacity(allocator, 0) catch unreachable,
        };
    }

    fn mesherThreadMain(self: *Client) void {
        // mark started (debug)
        self.mesher_started = true;
        const alloc = std.heap.page_allocator;
        while (true) {
            // Wait for a job
            self.mesher_mutex.lock();
            while (self.mesher_running and self.mesher_jobs.items.len == 0) self.mesher_cv.wait(&self.mesher_mutex);
            if (!self.mesher_running) {
                self.mesher_mutex.unlock();
                return;
            }
            const job = self.mesher_jobs.items[self.mesher_jobs.items.len - 1];
            _ = self.mesher_jobs.pop();
            self.mesher_mutex.unlock();

            // Under world lock: gather slice refs only (no large allocations or copies)
            self.sim.worlds_mutex.lock();
            const ws_ptr = self.sim.worlds_state.getPtr("minecraft:overworld") orelse {
                self.sim.worlds_mutex.unlock();
                continue;
            };
            const rs_ptr = ws_ptr.regions.getPtr(job.rpos) orelse {
                self.sim.worlds_mutex.unlock();
                continue;
            };
            var refs = self.buildRegionRefsUnderLock(alloc, ws_ptr, rs_ptr, job.rpos);
            self.sim.worlds_mutex.unlock();

            if (refs.chunks.len == 0) {
                self.freeRegionRefs(alloc, &refs);
                continue;
            }

            // Outside the world lock: copy data into an owned snapshot and free refs
            var snap = self.copySnapshotFromRefs(alloc, &refs);
            self.freeRegionRefs(alloc, &refs);

            if (snap.chunks.len == 0) {
                self.freeRegionSnapshot(alloc, &snap);
                continue;
            }

            // Mesh from snapshot (no locks)
            const result = self.meshRegionFromSnapshot(alloc, &snap);
            self.freeRegionSnapshot(alloc, &snap);

            // Publish result
            self.mesher_mutex.lock();
            self.mesher_results.append(self.allocator, result) catch {
                if (result.verts.len > 0) alloc.free(result.verts);
                if (result.draws.len > 0) self.allocator.free(result.draws);
            };
            self.mesher_mutex.unlock();
        }
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
        // Release GPU resources if created
        if (self.vbuf.id != 0) sg.destroyBuffer(self.vbuf);
        if (self.ui_vbuf.id != 0) sg.destroyBuffer(self.ui_vbuf);
        if (self.grass_view.id != 0) sg.destroyView(self.grass_view);
        if (self.sampler.id != 0) sg.destroySampler(self.sampler);
        if (self.grass_img.id != 0) sg.destroyImage(self.grass_img);
        if (self.pip.id != 0) sg.destroyPipeline(self.pip);
        if (self.quad_pip.id != 0) sg.destroyPipeline(self.quad_pip);
        if (self.shd.id != 0) sg.destroyShader(self.shd);

        // Stop mesher threads
        if (self.mesher_running) {
            self.mesher_mutex.lock();
            self.mesher_running = false;
            self.mesher_cv.broadcast();
            self.mesher_mutex.unlock();
            var i: usize = 0;
            while (i < self.mesher_threads.items.len) : (i += 1) self.mesher_threads.items[i].join();
            self.mesher_threads.deinit(self.allocator);
            // Free any pending results
            var r: usize = 0;
            while (r < self.mesher_results.items.len) : (r += 1) {
                const mr = self.mesher_results.items[r];
                std.heap.page_allocator.free(mr.verts);
                self.allocator.free(mr.draws);
            }
            self.mesher_results.deinit(self.allocator);
            self.mesher_jobs.deinit(self.allocator);
        }

        // Deinitialize atlas map (values only; keys are borrowed from registry)
        // Destroy cached region meshes
        var it = self.region_mesh_cache.valueIterator();
        while (it.next()) |rm| {
            if (rm.vbuf.id != 0) sg.destroyBuffer(rm.vbuf);
            self.allocator.free(rm.draws);
        }
        self.region_mesh_cache.deinit();

        self.atlas_uv_by_path.deinit();
        self.last_visible_frame_chunk.deinit();
        self.last_visible_frame_region.deinit();

        // Owned string
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
        pdesc.layout.attrs[2].format = .FLOAT2;
        pdesc.layout.attrs[3].format = .FLOAT2;
        self.pip = sg.makePipeline(pdesc);
        // dedicated 2D quad pipeline for overlay (no depth test, no culling)
        var qdesc: sg.PipelineDesc = .{};
        qdesc.label = "quad-pipeline";
        qdesc.shader = self.shd;
        qdesc.layout.attrs[0].format = .FLOAT3;
        qdesc.layout.attrs[1].format = .FLOAT2;
        qdesc.layout.attrs[2].format = .FLOAT2;
        qdesc.layout.attrs[3].format = .FLOAT2;
        qdesc.primitive_type = .TRIANGLES;
        qdesc.cull_mode = .NONE;
        qdesc.face_winding = .CCW;
        qdesc.depth = .{ .compare = .ALWAYS, .write_enabled = false };
        qdesc.color_count = 1;
        qdesc.colors[0].pixel_format = desc.environment.defaults.color_format;
        self.quad_pip = sg.makePipeline(qdesc);

        // Defer starting mesher threads until first frame when this Client is stored in state
        self.mesher_running = false;
        self.mesher_started = false;
        // honor user's request: adopt 1 result per frame
        self.adopt_budget_per_frame = 1;

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

        // Build texture atlas from registry resource paths (map already initialized in init)
        buildTextureAtlas(self);

        // create a view and sampler for the atlas (reusing grass_* fields)
        self.grass_view = sg.makeView(.{ .texture = .{ .image = self.grass_img } });
        self.sampler = sg.makeSampler(.{ .min_filter = .NEAREST, .mag_filter = .NEAREST, .mipmap_filter = .NEAREST, .wrap_u = .CLAMP_TO_EDGE, .wrap_v = .CLAMP_TO_EDGE });
        // shader expects VIEW_tex_texture at slot 1 and SMP_tex_sampler at slot 2
        self.bind.views[1] = self.grass_view;
        self.bind.samplers[2] = self.sampler;

        // Initialize local state and camera from player entity if available
        self.initLocalFromSim();
        self.updateCameraFromLocal();
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
                cx = @divFloor(@as(i32, @intFromFloat(@floor(e.pos[0]))), 16);
                cz = @divFloor(@as(i32, @intFromFloat(@floor(e.pos[2]))), 16);
                has_pos = true;
            }
        }
        self.sim.connections_mutex.unlock();
        if (!has_pos) return;
        if (self.sim.isChunkLoadedAt(wk, .{ .x = cx, .z = cz })) {
            self.ready = true;
            // Initialize local player and camera now that the world around the player exists
            self.initLocalFromSim();
            self.updateCameraFromLocal();
        }
    }

    fn startMesherIfNeeded(self: *Client) void {
        if (!self.mesher_started) {
            self.mesher_running = true;
            const thread_count: usize = 1;
            self.mesher_threads.ensureTotalCapacity(self.allocator, thread_count) catch return;
            var ti: usize = 0;
            while (ti < thread_count) : (ti += 1) {
                const th = std.Thread.spawn(.{}, mesherThreadMain, .{self}) catch break;
                self.mesher_threads.appendAssumeCapacity(th);
            }
            self.mesher_started = true;
        }
    }

    fn pumpMesherScheduleAndAdopt(self: *Client) void {
        // Snapshot regions and chunk counts, schedule jobs, and adopt results without drawing
        const wk = "minecraft:overworld";
        var regions = std.ArrayList(struct { rpos: simulation.RegionPos, chunk_count: usize }).initCapacity(self.allocator, 0) catch return;
        defer regions.deinit(self.allocator);
        self.sim.worlds_mutex.lock();
        const ws = self.sim.worlds_state.getPtr(wk);
        if (ws) |ws_ptr| {
            var reg_it = ws_ptr.regions.iterator();
            while (reg_it.next()) |rentry| {
                regions.append(self.allocator, .{ .rpos = rentry.key_ptr.*, .chunk_count = rentry.value_ptr.chunks.items.len }) catch {};
            }
        }
        self.sim.worlds_mutex.unlock();
        // reset issuance counter for this prepass
        self.rebuilds_issued_this_frame = 0;
        var i: usize = 0;
        while (i < regions.items.len) : (i += 1) self.ensureRegionMesh(regions.items[i].rpos, regions.items[i].chunk_count);
        self.dbg_last_scheduled = self.rebuilds_issued_this_frame;
        // adopt after scheduling
        self.adoptMesherResults();
    }

    pub fn frame(self: *Client) void {
        // Bump frame index (used for culling hysteresis)
        self.frame_index +%= 1;

        // Frame timing and FPS
        const t_now: i128 = std.time.nanoTimestamp();
        var dt_s: f32 = 0.0;
        if (self.fps_last_ns != 0) {
            const dt_ns: i128 = t_now - self.fps_last_ns;
            if (dt_ns > 0) {
                dt_s = @as(f32, @floatFromInt(dt_ns)) / 1_000_000_000.0;
                self.fps_accum_ns += dt_ns;
                self.fps_accum_frames += 1;
                if (self.fps_accum_ns >= 500_000_000) { // 0.5s window
                    const seconds: f32 = @as(f32, @floatFromInt(self.fps_accum_ns)) / 1_000_000_000.0;
                    self.fps_value = @as(f32, @floatFromInt(self.fps_accum_frames)) / seconds;
                    self.fps_accum_ns = 0;
                    self.fps_accum_frames = 0;
                }
            }
        }
        self.fps_last_ns = t_now;

        sg.beginPass(.{ .action = self.pass_action, .swapchain = sglue.swapchain() });

        // Ensure mesher thread(s) started after this Client is fully stored
        self.startMesherIfNeeded();
        // Even if not ready, keep background mesher pumping so meshes are ready when world appears
        self.pumpMesherScheduleAndAdopt();

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

        // If not ready, keep mesher pumping and draw a simple Connecting overlay, then return early
        if (!self.ready) {
            // one more pump (in case new regions appeared this frame)
            self.pumpMesherScheduleAndAdopt();
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

        // Apply movement inputs locally and predict physics every render frame
        self.applyMovementInputsLocal();
        const dt_clamped: f32 = std.math.clamp(dt_s, 0.0, 0.05);
        if (dt_clamped > 0.0) self.localPhysicsStep(dt_clamped);
        self.updateCameraFromLocal();
        // Notify simulation no more than once per tick
        self.maybeSendLookToSim();
        self.maybeSendMoveToSim();

        // Prepare MVP and frustum before building the mesh (used for culling)
        const w_px: f32 = @floatFromInt(sapp.width());
        const h_px: f32 = @floatFromInt(sapp.height());
        const aspect_wh: f32 = if (h_px > 0) (w_px / h_px) else 1.0;
        const proj = makePerspective(60.0 * std.math.pi / 180.0, aspect_wh, 0.1, 1000.0);
        const view = self.makeViewNoRoll(self.camera.pos, self.camera.yaw, self.camera.pitch);
        const mvp = matMul(proj, view);
        self.updateFrustum(mvp);
        var vs_params: shd_mod.VsParams = .{ .mvp = mvp, .atlas_pad = .{ 0, 0 } };

        // Render cached meshes (build on demand)
        if (self.grass_img.id != 0) {
            sg.applyPipeline(self.pip);
            sg.applyUniforms(0, sg.asRange(&vs_params));
            self.renderWorldCached(&vs_params);
        }

        // Optional continuous probe logging

        // Cached rendering handled above

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
            const cols_f: f32 = (w_px / scale) / 8.0;
            // Right-aligned, one datum per line
            var tbuf: [64:0]u8 = undefined;
            var slice = std.fmt.bufPrint(tbuf[0 .. tbuf.len - 1], "Tick: {d}", .{snap.tick}) catch "Tick: ?";
            tbuf[slice.len] = 0;
            var line: [:0]const u8 = tbuf[0..slice.len :0];
            var col_start_f: f32 = @max(0.0, cols_f - @as(f32, @floatFromInt(line.len)) - 1.0);
            sdtx.pos(col_start_f, 1.0);
            sdtx.color3b(255, 255, 255);
            sdtx.puts(line);

            slice = std.fmt.bufPrint(tbuf[0 .. tbuf.len - 1], "TPS: {d:.2}", .{snap.rolling_tps}) catch "TPS: ?";
            tbuf[slice.len] = 0;
            line = tbuf[0..slice.len :0];
            col_start_f = @max(0.0, cols_f - @as(f32, @floatFromInt(line.len)) - 1.0);
            sdtx.pos(col_start_f, 2.0);
            sdtx.color3b(255, 255, 255);
            sdtx.puts(line);

            slice = std.fmt.bufPrint(tbuf[0 .. tbuf.len - 1], "FPS: {d:.1}", .{self.fps_value}) catch "FPS: ?";
            tbuf[slice.len] = 0;
            line = tbuf[0..slice.len :0];
            col_start_f = @max(0.0, cols_f - @as(f32, @floatFromInt(line.len)) - 1.0);
            sdtx.pos(col_start_f, 3.0);
            sdtx.color3b(255, 255, 255);
            sdtx.puts(line);

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

            // World and mesher stats
            // Count regions and chunks under lock
            var regions_count: usize = 0;
            var chunks_count: usize = 0;
            self.sim.worlds_mutex.lock();
            const ws_dbg = self.sim.worlds_state.getPtr("minecraft:overworld");
            if (ws_dbg) |wsp| {
                var it = wsp.regions.iterator();
                while (it.next()) |entry| {
                    regions_count += 1;
                    chunks_count += entry.value_ptr.chunks.items.len;
                }
            }
            self.sim.worlds_mutex.unlock();

            // Mesher queues and cache counts
            var jobs_len: usize = 0;
            var results_len: usize = 0;
            self.mesher_mutex.lock();
            jobs_len = self.mesher_jobs.items.len;
            results_len = self.mesher_results.items.len;
            self.mesher_mutex.unlock();
            var rcache_count: usize = 0;
            var inflight_count: usize = 0;
            var it_rm = self.region_mesh_cache.valueIterator();
            while (it_rm.next()) |rmc| {
                rcache_count += 1;
                if (rmc.inflight) inflight_count += 1;
            }

            // World group header
            const world_hdr: [:0]const u8 = "World";
            sdtx.pos(1.0, 3.0);
            sdtx.color3b(180, 220, 255);
            sdtx.puts(world_hdr);

            // World stats under header
            var wbuf: [64:0]u8 = undefined;
            var wslice = std.fmt.bufPrint(wbuf[0 .. wbuf.len - 1], "regions: {d}", .{regions_count}) catch "regions: ?";
            wbuf[wslice.len] = 0;
            var wline: [:0]const u8 = wbuf[0..wslice.len :0];
            sdtx.pos(1.0, 4.0);
            sdtx.color3b(180, 220, 255);
            sdtx.puts(wline);

            wslice = std.fmt.bufPrint(wbuf[0 .. wbuf.len - 1], "chunks: {d}", .{chunks_count}) catch "chunks: ?";
            wbuf[wslice.len] = 0;
            wline = wbuf[0..wslice.len :0];
            sdtx.pos(1.0, 5.0);
            sdtx.color3b(180, 220, 255);
            sdtx.puts(wline);

            wslice = std.fmt.bufPrint(wbuf[0 .. wbuf.len - 1], "ready: {s}", .{if (self.ready) "true" else "false"}) catch "ready: ?";
            wbuf[wslice.len] = 0;
            wline = wbuf[0..wslice.len :0];
            sdtx.pos(1.0, 6.0);
            sdtx.color3b(180, 220, 255);
            sdtx.puts(wline);

            // Mesher group header
            const mesher_hdr: [:0]const u8 = "Mesher";
            sdtx.pos(1.0, 7.0);
            sdtx.color3b(180, 255, 180);
            sdtx.puts(mesher_hdr);

            // Mesher stats under header
            var mbuf: [72:0]u8 = undefined;
            var mslice = std.fmt.bufPrint(mbuf[0 .. mbuf.len - 1], "jobs: {d}", .{jobs_len}) catch "jobs: ?";
            mbuf[mslice.len] = 0;
            var mline: [:0]const u8 = mbuf[0..mslice.len :0];
            sdtx.pos(1.0, 8.0);
            sdtx.color3b(180, 255, 180);
            sdtx.puts(mline);

            mslice = std.fmt.bufPrint(mbuf[0 .. mbuf.len - 1], "results: {d}", .{results_len}) catch "results: ?";
            mbuf[mslice.len] = 0;
            mline = mbuf[0..mslice.len :0];
            sdtx.pos(1.0, 9.0);
            sdtx.color3b(180, 255, 180);
            sdtx.puts(mline);

            mslice = std.fmt.bufPrint(mbuf[0 .. mbuf.len - 1], "inflight: {d}", .{inflight_count}) catch "inflight: ?";
            mbuf[mslice.len] = 0;
            mline = mbuf[0..mslice.len :0];
            sdtx.pos(1.0, 10.0);
            sdtx.color3b(180, 255, 180);
            sdtx.puts(mline);

            mslice = std.fmt.bufPrint(mbuf[0 .. mbuf.len - 1], "rcache: {d}", .{rcache_count}) catch "rcache: ?";
            mbuf[mslice.len] = 0;
            mline = mbuf[0..mslice.len :0];
            sdtx.pos(1.0, 11.0);
            sdtx.color3b(180, 255, 180);
            sdtx.puts(mline);

            mslice = std.fmt.bufPrint(mbuf[0 .. mbuf.len - 1], "scheduled: {d}", .{self.dbg_last_scheduled}) catch "scheduled: ?";
            mbuf[mslice.len] = 0;
            mline = mbuf[0..mslice.len :0];
            sdtx.pos(1.0, 12.0);
            sdtx.color3b(180, 255, 180);
            sdtx.puts(mline);

            mslice = std.fmt.bufPrint(mbuf[0 .. mbuf.len - 1], "adopted: {d}", .{self.dbg_last_adopted}) catch "adopted: ?";
            mbuf[mslice.len] = 0;
            mline = mbuf[0..mslice.len :0];
            sdtx.pos(1.0, 13.0);
            sdtx.color3b(180, 255, 180);
            sdtx.puts(mline);

            mslice = std.fmt.bufPrint(mbuf[0 .. mbuf.len - 1], "threads: {d}", .{self.mesher_threads.items.len}) catch "threads: ?";
            mbuf[mslice.len] = 0;
            mline = mbuf[0..mslice.len :0];
            sdtx.pos(1.0, 14.0);
            sdtx.color3b(180, 255, 180);
            sdtx.puts(mline);

            mslice = std.fmt.bufPrint(mbuf[0 .. mbuf.len - 1], "running: {s}", .{if (self.mesher_running) "true" else "false"}) catch "running: ?";
            mbuf[mslice.len] = 0;
            mline = mbuf[0..mslice.len :0];
            sdtx.pos(1.0, 15.0);
            sdtx.color3b(180, 255, 180);
            sdtx.puts(mline);

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

    // Initialize local predicted player state from the authoritative simulation
    fn initLocalFromSim(self: *Client) void {
        self.sim.connections_mutex.lock();
        defer self.sim.connections_mutex.unlock();
        const idx_opt = self.sim.entity_by_player.get(self.player_id);
        if (idx_opt) |idx| {
            if (idx < self.sim.dynamic_entities.items.len) {
                const e = self.sim.dynamic_entities.items[idx];
                self.local_pos = e.pos;
                self.local_vel = e.vel;
                self.local_aabb_half_extents = e.aabb_half_extents;
                self.local_on_ground = e.flags.on_ground;
                if (!self.local_orient_initialized) {
                    // Seed local orientation from sim
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
            }
        }
    }

    // Update camera to mirror the client-local predicted player position; keep orientation client-local
    fn updateCameraFromLocal(self: *Client) void {
        const eye_y: f32 = self.local_pos[1] + self.local_aabb_half_extents[1];
        self.camera.pos = .{ self.local_pos[0], eye_y, self.local_pos[2] };
        self.camera.setYawPitch(self.local_yaw, self.local_pitch);
        self.camera.roll = 0;
    }

    // Apply currently-pressed movement keys to the client-local predicted player
    fn applyMovementInputsLocal(self: *Client) void {
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
        self.local_vel[0] = vx * speed;
        self.local_vel[2] = vz * speed;

        // If moving, keep facing synced to the horizontal component of the look vector
        if (self.move_forward or self.move_back or self.move_left or self.move_right) {
            // Note: facing_dir_xz is only written back to SIM in maybeSendMoveToSim()
        }

        // Jump impulse on edge press if on ground (client-local)
        if (self.jump_pressed and self.local_on_ground) {
            const g: f32 = 9.80665;
            const target_h: f32 = 1.1; // slightly over 1 block
            const v0: f32 = @sqrt(2.0 * g * target_h);
            self.local_vel[1] = v0;
            self.local_on_ground = false;
            self.jump_pending = true; // request SIM jump on next tick
        }
        // consume jump edge
        self.jump_pressed = false;
    }

    fn isSolidAt(self: *Client, ws: *const simulation.WorldState, start_chunk: *const gs.Chunk, cpos_x_in: i32, cpos_z_in: i32, lx_in: i32, abs_y_in: i32, lz_in: i32) bool {
        var nx = lx_in;
        const ny = abs_y_in;
        var nz = lz_in;
        var cpos_x = cpos_x_in;
        var cpos_z = cpos_z_in;
        var chn: *const gs.Chunk = start_chunk;

        // Cross-chunk in X
        if (nx < 0) {
            cpos_x -= 1;
            const rp_w = simulation.RegionPos{ .x = @divFloor(cpos_x, 32), .z = @divFloor(cpos_z, 32) };
            const rsn = ws.regions.getPtr(rp_w) orelse return false;
            const idxn = rsn.chunk_index.get(.{ .x = cpos_x, .z = cpos_z }) orelse return false;
            chn = &rsn.chunks.items[idxn];
            nx += @as(i32, @intCast(constants.chunk_size_x));
        } else if (nx >= @as(i32, @intCast(constants.chunk_size_x))) {
            cpos_x += 1;
            const rp_e = simulation.RegionPos{ .x = @divFloor(cpos_x, 32), .z = @divFloor(cpos_z, 32) };
            const rsn2 = ws.regions.getPtr(rp_e) orelse return false;
            const idxn2 = rsn2.chunk_index.get(.{ .x = cpos_x, .z = cpos_z }) orelse return false;
            chn = &rsn2.chunks.items[idxn2];
            nx -= @as(i32, @intCast(constants.chunk_size_x));
        }
        // Cross-chunk in Z
        if (nz < 0) {
            cpos_z -= 1;
            const rp_n = simulation.RegionPos{ .x = @divFloor(cpos_x, 32), .z = @divFloor(cpos_z, 32) };
            const rsn3 = ws.regions.getPtr(rp_n) orelse return false;
            const idxn3 = rsn3.chunk_index.get(.{ .x = cpos_x, .z = cpos_z }) orelse return false;
            chn = &rsn3.chunks.items[idxn3];
            nz += @as(i32, @intCast(constants.chunk_size_z));
        } else if (nz >= @as(i32, @intCast(constants.chunk_size_z))) {
            cpos_z += 1;
            const rp_s = simulation.RegionPos{ .x = @divFloor(cpos_x, 32), .z = @divFloor(cpos_z, 32) };
            const rsn4 = ws.regions.getPtr(rp_s) orelse return false;
            const idxn4 = rsn4.chunk_index.get(.{ .x = cpos_x, .z = cpos_z }) orelse return false;
            chn = &rsn4.chunks.items[idxn4];
            nz -= @as(i32, @intCast(constants.chunk_size_z));
        }

        // Y in-bounds
        const total_y: i32 = @intCast(chn.sections.len * constants.section_height);
        if (ny < 0 or ny >= total_y) return false;
        const n_sy_idx: usize = @intCast(@divTrunc(ny, @as(i32, @intCast(constants.section_height))));
        const n_ly: usize = @intCast(@mod(ny, @as(i32, @intCast(constants.section_height))));
        const s = chn.sections[n_sy_idx];
        const bpi: u6 = bitsFor(s.palette.len);
        const idx: usize = @as(usize, @intCast(nz)) * (constants.chunk_size_x * constants.section_height) + @as(usize, @intCast(nx)) * constants.section_height + n_ly;
        const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, idx, bpi));
        if (pidx >= s.palette.len) return false;
        const bid: u32 = s.palette[pidx];
        if (bid == 0) return false;
        if (bid < self.sim.reg.blocks.items.len) {
            return self.sim.reg.blocks.items[@intCast(bid)].full_block_collision;
        }
        return true;
    }

    fn buildSectionVertices(self: *Client, ws: *const simulation.WorldState, ch: *const gs.Chunk, sy: usize, base_x: f32, base_z: f32, out: *std.ArrayList(Vertex)) void {
        const s = ch.sections[sy];
        const pal_len = s.palette.len;
        const bpi: u6 = bitsFor(pal_len);
        // Build per-palette info (draw, solid, top/side transforms)
        const PalInfo = struct {
            draw: bool,
            solid: bool,
            top_tx: AtlasTransform,
            side_tx: AtlasTransform,
        };
        var pal_info = std.ArrayList(PalInfo).initCapacity(self.allocator, pal_len) catch {
            return;
        };
        defer pal_info.deinit(self.allocator);
        var pi: usize = 0;
        while (pi < pal_len) : (pi += 1) {
            const bid: u32 = s.palette[pi];
            var pinfo: PalInfo = .{
                .draw = true,
                .solid = true,
                .top_tx = .{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale },
                .side_tx = .{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale },
            };
            if (bid < self.sim.reg.blocks.items.len) {
                pinfo.solid = self.sim.reg.blocks.items[@intCast(bid)].full_block_collision;
            }
            if (bid == 0) {
                pinfo.draw = false;
                pinfo.solid = false;
            } else {
                const name = self.sim.reg.getBlockName(@intCast(bid));
                if (self.sim.reg.resources.get(name)) |res| switch (res) {
                    .Void => {
                        pinfo.draw = false;
                        pinfo.solid = false;
                    },
                    .Uniform => |u| {
                        const tx: AtlasTransform = self.atlas_uv_by_path.get(u.all_path) orelse AtlasTransform{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale };
                        pinfo.top_tx = tx;
                        pinfo.side_tx = tx;
                    },
                    .Facing => |f| {
                        const top_tx: AtlasTransform = self.atlas_uv_by_path.get(f.face_path) orelse AtlasTransform{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale };
                        const side_tx: AtlasTransform = self.atlas_uv_by_path.get(f.other_path) orelse AtlasTransform{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale };
                        pinfo.top_tx = top_tx;
                        pinfo.side_tx = side_tx;
                    },
                };
            }
            pal_info.appendAssumeCapacity(pinfo);
        }

        // Greedy meshing for +Y (top) faces per Y-slice to drastically reduce vertex count
        const FaceMat = struct { off: [2]f32, sc: [2]f32 };
        const Mat = struct {
            fn eq(a: FaceMat, b: FaceMat) bool {
                return a.off[0] == b.off[0] and a.off[1] == b.off[1] and a.sc[0] == b.sc[0] and a.sc[1] == b.sc[1];
            }
        };
        const MaskCell = struct { present: bool, m: FaceMat };
        const section_base_y: i32 = @intCast(sy * constants.section_height);
        var ly_top: usize = 0;
        while (ly_top < constants.section_height) : (ly_top += 1) {
            const abs_y: i32 = section_base_y + @as(i32, @intCast(ly_top));
            var mask: [constants.chunk_size_z * constants.chunk_size_x]MaskCell = undefined;
            // fill mask
            var mz: usize = 0;
            while (mz < constants.chunk_size_z) : (mz += 1) {
                var mx: usize = 0;
                while (mx < constants.chunk_size_x) : (mx += 1) {
                    const idx: usize = mz * constants.chunk_size_x + mx;
                    mask[idx].present = false;
                    // palette index
                    const src_idx: usize = mz * (constants.chunk_size_x * constants.section_height) + mx * constants.section_height + ly_top;
                    const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, src_idx, bpi));
                    if (pidx >= pal_info.items.len) continue;
                    const info = pal_info.items[pidx];
                    if (!info.draw) continue;
                    // top visible if neighbor above not solid
                    if (!self.isSolidAt(ws, ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(mx)), abs_y + 1, @as(i32, @intCast(mz)))) {
                        mask[idx] = .{ .present = true, .m = .{ .off = info.top_tx.offset, .sc = info.top_tx.scale } };
                    }
                }
            }
            // greedy rectangles over mask (x along width, z along height)
            mz = 0;
            while (mz < constants.chunk_size_z) : (mz += 1) {
                var mx: usize = 0;
                while (mx < constants.chunk_size_x) : (mx += 1) {
                    const base_idx: usize = mz * constants.chunk_size_x + mx;
                    if (!mask[base_idx].present) continue;
                    const mkey: FaceMat = mask[base_idx].m;
                    // determine width
                    var w: usize = 1;
                    while ((mx + w) < constants.chunk_size_x) : (w += 1) {
                        const idx2 = mz * constants.chunk_size_x + (mx + w);
                        if (!(mask[idx2].present and Mat.eq(mask[idx2].m, mkey))) break;
                    }
                    // determine height
                    var h: usize = 1;
                    outer: while ((mz + h) < constants.chunk_size_z) : (h += 1) {
                        var kx: usize = 0;
                        while (kx < w) : (kx += 1) {
                            const idx3 = (mz + h) * constants.chunk_size_x + (mx + kx);
                            if (!(mask[idx3].present and Mat.eq(mask[idx3].m, mkey))) break :outer;
                        }
                    }
                    // emit quad for rectangle (mx..mx+w, mz..mz+h) at y = abs_y + 1
                    const wx0: f32 = base_x + @as(f32, @floatFromInt(mx));
                    const wz0: f32 = base_z + @as(f32, @floatFromInt(mz));
                    const wx1: f32 = base_x + @as(f32, @floatFromInt(mx + w));
                    const wz1: f32 = base_z + @as(f32, @floatFromInt(mz + h));
                    const wy1: f32 = @as(f32, @floatFromInt(abs_y + 1));
                    // UVs repeat across the merged area (0..w, 0..h)
                    const uv00: [2]f32 = .{ 0, 0 };
                    const uv0h: [2]f32 = .{ 0, @as(f32, @floatFromInt(h)) };
                    const uvwh: [2]f32 = .{ @as(f32, @floatFromInt(w)), @as(f32, @floatFromInt(h)) };
                    const uvw0: [2]f32 = .{ @as(f32, @floatFromInt(w)), 0 };
                    emitQuad(self.allocator, out, .{ wx0, wy1, wz0 }, .{ wx0, wy1, wz1 }, .{ wx1, wy1, wz1 }, .{ wx1, wy1, wz0 }, uv00, uv0h, uvwh, uvw0, mkey.sc, mkey.off);
                    // clear mask for used cells
                    var zz: usize = 0;
                    while (zz < h) : (zz += 1) {
                        var xx: usize = 0;
                        while (xx < w) : (xx += 1) {
                            mask[(mz + zz) * constants.chunk_size_x + (mx + xx)].present = false;
                        }
                    }
                }
            }
        }

        // Now emit the remaining faces (bottom and sides) per-voxel
        for (0..constants.chunk_size_z) |lz| {
            for (0..constants.chunk_size_x) |lx| {
                for (0..constants.section_height) |ly| {
                    const idx: usize = lz * (constants.chunk_size_x * constants.section_height) + lx * constants.section_height + ly;
                    const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, idx, bpi));
                    if (pidx >= pal_info.items.len) continue;
                    const info = pal_info.items[pidx];
                    if (!info.draw) continue;

                    const wx0: f32 = base_x + @as(f32, @floatFromInt(lx));
                    const wy0: f32 = @as(f32, @floatFromInt(@as(i32, @intCast(sy * constants.section_height)) + @as(i32, @intCast(ly))));
                    const wz0: f32 = base_z + @as(f32, @floatFromInt(lz));
                    const wx1: f32 = wx0 + 1.0;
                    const wy1: f32 = wy0 + 1.0;
                    const wz1: f32 = wz0 + 1.0;

                    const abs_y_this: i32 = section_base_y + @as(i32, @intCast(ly));

                    const uv00: [2]f32 = .{ 0, 0 };
                    const uv01: [2]f32 = .{ 0, 1 };
                    const uv11: [2]f32 = .{ 1, 1 };
                    const uv10: [2]f32 = .{ 1, 0 };

                    if (false) {}
                    if (!self.isSolidAt(ws, ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y_this - 1, @as(i32, @intCast(lz)))) {
                        emitQuad(self.allocator, out, .{ wx0, wy0, wz0 }, .{ wx1, wy0, wz0 }, .{ wx1, wy0, wz1 }, .{ wx0, wy0, wz1 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset);
                    }
                    if (!self.isSolidAt(ws, ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)) - 1, abs_y_this, @as(i32, @intCast(lz)))) {
                        emitQuad(self.allocator, out, .{ wx0, wy0, wz1 }, .{ wx0, wy1, wz1 }, .{ wx0, wy1, wz0 }, .{ wx0, wy0, wz0 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset);
                    }
                    if (!self.isSolidAt(ws, ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)) + 1, abs_y_this, @as(i32, @intCast(lz)))) {
                        emitQuad(self.allocator, out, .{ wx1, wy0, wz0 }, .{ wx1, wy1, wz0 }, .{ wx1, wy1, wz1 }, .{ wx1, wy0, wz1 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset);
                    }
                    if (!self.isSolidAt(ws, ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y_this, @as(i32, @intCast(lz)) - 1)) {
                        emitQuad(self.allocator, out, .{ wx0, wy0, wz0 }, .{ wx0, wy1, wz0 }, .{ wx1, wy1, wz0 }, .{ wx1, wy0, wz0 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset);
                    }
                    if (!self.isSolidAt(ws, ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y_this, @as(i32, @intCast(lz)) + 1)) {
                        emitQuad(self.allocator, out, .{ wx1, wy0, wz1 }, .{ wx1, wy1, wz1 }, .{ wx0, wy1, wz1 }, .{ wx0, wy0, wz1 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset);
                    }
                }
            }
        }
    }

    // Build a single region mesh (one buffer) by concatenating all section vertices of all chunks in the region
    fn buildRegionMeshNow(self: *Client, ws: *const simulation.WorldState, rs: *const simulation.RegionState) RegionMesh {
        var draws = self.allocator.alloc(SectionDraw, rs.chunks.items.len * @as(usize, @intCast(ws.section_count_y))) catch return .{ .vbuf = .{}, .draws = &[_]SectionDraw{}, .built_chunk_count = 0, .dirty = false, .inflight = false, .last_built_frame = self.frame_index };
        var draws_len: usize = 0;
        var verts = std.ArrayList(Vertex).initCapacity(self.allocator, 0) catch {
            self.allocator.free(draws);
            return .{ .vbuf = .{}, .draws = &[_]SectionDraw{}, .built_chunk_count = 0, .dirty = false, .inflight = false, .last_built_frame = self.frame_index };
        };
        defer verts.deinit(self.allocator);
        for (rs.chunks.items) |ch| {
            const base_x: f32 = @floatFromInt(ch.pos.x * @as(i32, @intCast(constants.chunk_size_x)));
            const base_z: f32 = @floatFromInt(ch.pos.z * @as(i32, @intCast(constants.chunk_size_z)));
            var sy: usize = 0;
            while (sy < ch.sections.len) : (sy += 1) {
                const start: usize = verts.items.len;
                self.buildSectionVertices(ws, &ch, sy, base_x, base_z, &verts);
                const count: usize = verts.items.len - start;
                if (count > 0) {
                    draws[draws_len] = .{ .chunk_pos = ch.pos, .sy = @intCast(sy), .first = @intCast(start), .count = @intCast(count) };
                    draws_len += 1;
                }
            }
        }
        if (draws_len < draws.len) {
            const new_draws = self.allocator.alloc(SectionDraw, draws_len) catch {
                self.allocator.free(draws);
                return .{ .vbuf = .{}, .draws = &[_]SectionDraw{}, .built_chunk_count = 0, .dirty = false, .inflight = false, .last_built_frame = self.frame_index };
            };
            @memcpy(new_draws[0..draws_len], draws[0..draws_len]);
            self.allocator.free(draws);
            draws = new_draws;
        }
        var buf: sg.Buffer = .{};
        if (verts.items.len > 0) {
            buf = sg.makeBuffer(.{ .usage = .{ .vertex_buffer = true }, .data = sg.asRange(verts.items) });
        }
        return .{ .vbuf = buf, .draws = draws, .built_chunk_count = rs.chunks.items.len, .dirty = false, .inflight = false, .last_built_frame = self.frame_index };
    }

    // Under world lock: capture slice refs for a region (no heavy copies)
    fn buildRegionRefsUnderLock(self: *Client, alloc: std.mem.Allocator, ws: *const simulation.WorldState, rs: *const simulation.RegionState, rpos: simulation.RegionPos) RegionSliceRefs {
        _ = self;
        var refs: RegionSliceRefs = .{ .rpos = rpos, .section_count_y = ws.section_count_y, .base_cx = rpos.x * 32, .base_cz = rpos.z * 32, .chunks = &[_]ChunkSliceRefs{} };
        const count: usize = rs.chunks.items.len;
        if (count == 0) return refs;
        var chunks = alloc.alloc(ChunkSliceRefs, count) catch return refs;
        var i: usize = 0;
        while (i < count) : (i += 1) {
            const ch = rs.chunks.items[i];
            const sec_count = ch.sections.len;
            var secs: []SectionSliceRefs = &[_]SectionSliceRefs{};
            if (sec_count > 0) secs = alloc.alloc(SectionSliceRefs, sec_count) catch &[_]SectionSliceRefs{};
            var sy: usize = 0;
            while (sy < secs.len) : (sy += 1) {
                const s = ch.sections[sy];
                secs[sy] = .{ .pal_ptr = s.palette.ptr, .pal_len = s.palette.len, .bits_ptr = s.blocks_indices_bits.ptr, .bits_len = s.blocks_indices_bits.len };
            }
            chunks[i] = .{ .pos = ch.pos, .sections = secs };
        }
        refs.chunks = chunks;
        return refs;
    }

    fn freeRegionRefs(self: *Client, alloc: std.mem.Allocator, refs: *RegionSliceRefs) void {
        _ = self;
        var i: usize = 0;
        while (i < refs.chunks.len) : (i += 1) {
            const ch = refs.chunks[i];
            if (ch.sections.len > 0) alloc.free(ch.sections);
        }
        if (refs.chunks.len > 0) alloc.free(refs.chunks);
        refs.* = .{ .rpos = refs.rpos, .section_count_y = refs.section_count_y, .base_cx = refs.base_cx, .base_cz = refs.base_cz, .chunks = &[_]ChunkSliceRefs{} };
    }

    // Outside world lock: copy refs into an owned RegionSnapshot
    fn copySnapshotFromRefs(self: *Client, alloc: std.mem.Allocator, refs: *const RegionSliceRefs) RegionSnapshot {
        _ = self;
        var snap: RegionSnapshot = .{
            .rpos = refs.rpos,
            .base_cx = refs.base_cx,
            .base_cz = refs.base_cz,
            .section_count_y = refs.section_count_y,
            .chunks = &[_]ChunkSnapshot{},
            .grid_map = &[_]i32{},
        };
        const count: usize = refs.chunks.len;
        if (count == 0) return snap;
        var chunks = alloc.alloc(ChunkSnapshot, count) catch return snap;
        var grid = alloc.alloc(i32, 32 * 32) catch {
            alloc.free(chunks);
            return snap;
        };
        var gi: usize = 0;
        while (gi < grid.len) : (gi += 1) grid[gi] = -1;
        var i: usize = 0;
        while (i < count) : (i += 1) {
            const chref = refs.chunks[i];
            const pos = chref.pos;
            const lx: i32 = pos.x - snap.base_cx;
            const lz: i32 = pos.z - snap.base_cz;
            if (!(lx >= 0 and lx < 32 and lz >= 0 and lz < 32)) {
                chunks[i] = .{ .pos = pos, .sections = &[_]SectionSnapshot{} };
                continue;
            }
            const m_idx: usize = @intCast(@as(usize, @intCast(lz)) * 32 + @as(usize, @intCast(lx)));
            grid[m_idx] = @intCast(i);
            var secs: []SectionSnapshot = &[_]SectionSnapshot{};
            if (chref.sections.len > 0) secs = alloc.alloc(SectionSnapshot, chref.sections.len) catch &[_]SectionSnapshot{};
            var sy: usize = 0;
            while (sy < secs.len) : (sy += 1) {
                const sref = chref.sections[sy];
                var pal: []u32 = &[_]u32{};
                var bits: []u32 = &[_]u32{};
                if (sref.pal_len > 0) {
                    pal = alloc.alloc(u32, sref.pal_len) catch &[_]u32{};
                    if (pal.len == sref.pal_len) @memcpy(pal[0..pal.len], sref.pal_ptr[0..sref.pal_len]);
                }
                if (sref.bits_len > 0) {
                    bits = alloc.alloc(u32, sref.bits_len) catch &[_]u32{};
                    if (bits.len == sref.bits_len) @memcpy(bits[0..bits.len], sref.bits_ptr[0..sref.bits_len]);
                }
                secs[sy] = .{ .palette = pal, .blocks_indices_bits = bits };
            }
            chunks[i] = .{ .pos = pos, .sections = secs };
        }
        snap.chunks = chunks;
        snap.grid_map = grid;
        return snap;
    }

    fn freeRegionSnapshot(self: *Client, alloc: std.mem.Allocator, snap: *RegionSnapshot) void {
        _ = self;
        // free sections and their arrays
        var i: usize = 0;
        while (i < snap.chunks.len) : (i += 1) {
            const ch = snap.chunks[i];
            var sy: usize = 0;
            while (sy < ch.sections.len) : (sy += 1) {
                const s = ch.sections[sy];
                if (s.palette.len > 0) alloc.free(s.palette);
                if (s.blocks_indices_bits.len > 0) alloc.free(s.blocks_indices_bits);
            }
            if (ch.sections.len > 0) alloc.free(ch.sections);
        }
        if (snap.chunks.len > 0) alloc.free(snap.chunks);
        if (snap.grid_map.len > 0) alloc.free(snap.grid_map);
        // reset
        snap.* = .{ .rpos = snap.rpos, .base_cx = snap.base_cx, .base_cz = snap.base_cz, .section_count_y = snap.section_count_y, .chunks = &[_]ChunkSnapshot{}, .grid_map = &[_]i32{} };
    }

    fn snapGetChunkIndex(snap: *const RegionSnapshot, cx: i32, cz: i32) ?usize {
        const lx: i32 = cx - snap.base_cx;
        const lz: i32 = cz - snap.base_cz;
        if (!(lx >= 0 and lx < 32 and lz >= 0 and lz < 32)) return null;
        const idx: usize = @intCast(@as(usize, @intCast(lz)) * 32 + @as(usize, @intCast(lx)));
        const val: i32 = snap.grid_map[idx];
        if (val < 0) return null;
        return @intCast(val);
    }

    fn isSolidAtSnapshot(self: *Client, snap: *const RegionSnapshot, start_cx: i32, start_cz: i32, lx_in: i32, abs_y_in: i32, lz_in: i32) bool {
        var lx = lx_in;
        var lz = lz_in;
        var cx = start_cx;
        var cz = start_cz;
        // cross-chunk X
        if (lx < 0) {
            cx -= 1;
            lx += @as(i32, @intCast(constants.chunk_size_x));
        } else if (lx >= @as(i32, @intCast(constants.chunk_size_x))) {
            cx += 1;
            lx -= @as(i32, @intCast(constants.chunk_size_x));
        }
        // cross-chunk Z
        if (lz < 0) {
            cz -= 1;
            lz += @as(i32, @intCast(constants.chunk_size_z));
        } else if (lz >= @as(i32, @intCast(constants.chunk_size_z))) {
            cz += 1;
            lz -= @as(i32, @intCast(constants.chunk_size_z));
        }
        const idx_opt = snapGetChunkIndex(snap, cx, cz);
        if (idx_opt == null) return true; // outside snapshot => treat as solid to avoid seams
        const ch = snap.chunks[idx_opt.?];
        const total_y: i32 = @as(i32, @intCast(snap.section_count_y)) * @as(i32, @intCast(constants.section_height));
        const ny = abs_y_in;
        if (ny < 0 or ny >= total_y) return false;
        const sy: usize = @intCast(@divTrunc(ny, @as(i32, @intCast(constants.section_height))));
        const ly: usize = @intCast(@mod(ny, @as(i32, @intCast(constants.section_height))));
        if (sy >= ch.sections.len) return false;
        const s = ch.sections[sy];
        const bpi: u6 = bitsFor(s.palette.len);
        const idx3d: usize = @as(usize, @intCast(lz)) * (constants.chunk_size_x * constants.section_height) + @as(usize, @intCast(lx)) * constants.section_height + ly;
        const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, idx3d, bpi));
        if (pidx >= s.palette.len) return false;
        const bid: u32 = s.palette[pidx];
        if (bid == 0) return false;
        if (bid < self.sim.reg.blocks.items.len) return self.sim.reg.blocks.items[@intCast(bid)].full_block_collision;
        return true;
    }

    fn buildSectionVerticesFromSnapshot(self: *Client, snap: *const RegionSnapshot, ch: *const ChunkSnapshot, sy: usize, base_x: f32, base_z: f32, out: *std.ArrayList(Vertex), alloc: std.mem.Allocator) void {
        const s = ch.sections[sy];
        const pal_len = s.palette.len;
        const bpi: u6 = bitsFor(pal_len);
        const PalInfo = struct {
            draw: bool,
            solid: bool,
            top_tx: AtlasTransform,
            side_tx: AtlasTransform,
        };
        var pal_info = std.ArrayList(PalInfo).initCapacity(self.allocator, pal_len) catch {
            return;
        };
        defer pal_info.deinit(self.allocator);
        var pi: usize = 0;
        while (pi < pal_len) : (pi += 1) {
            const bid: u32 = s.palette[pi];
            var pinfo: PalInfo = .{
                .draw = true,
                .solid = true,
                .top_tx = .{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale },
                .side_tx = .{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale },
            };
            if (bid < self.sim.reg.blocks.items.len) {
                pinfo.solid = self.sim.reg.blocks.items[@intCast(bid)].full_block_collision;
            }
            if (bid == 0) {
                pinfo.draw = false;
                pinfo.solid = false;
            } else {
                const name = self.sim.reg.getBlockName(@intCast(bid));
                if (self.sim.reg.resources.get(name)) |res| switch (res) {
                    .Void => {
                        pinfo.draw = false;
                        pinfo.solid = false;
                    },
                    .Uniform => |u| {
                        const tx: AtlasTransform = self.atlas_uv_by_path.get(u.all_path) orelse AtlasTransform{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale };
                        pinfo.top_tx = tx;
                        pinfo.side_tx = tx;
                    },
                    .Facing => |f| {
                        const top_tx: AtlasTransform = self.atlas_uv_by_path.get(f.face_path) orelse AtlasTransform{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale };
                        const side_tx: AtlasTransform = self.atlas_uv_by_path.get(f.other_path) orelse AtlasTransform{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale };
                        pinfo.top_tx = top_tx;
                        pinfo.side_tx = side_tx;
                    },
                };
            }
            pal_info.appendAssumeCapacity(pinfo);
        }

        // Greedy top faces per Y-slice
        const FaceMat = struct { off: [2]f32, sc: [2]f32 };
        const Mat = struct {
            fn eq(a: FaceMat, b: FaceMat) bool {
                return a.off[0] == b.off[0] and a.off[1] == b.off[1] and a.sc[0] == b.sc[0] and a.sc[1] == b.sc[1];
            }
        };
        const section_base_y: i32 = @intCast(sy * constants.section_height);
        var ly_top: usize = 0;
        while (ly_top < constants.section_height) : (ly_top += 1) {
            const abs_y: i32 = section_base_y + @as(i32, @intCast(ly_top));
            var mask: [constants.chunk_size_z * constants.chunk_size_x]struct { present: bool, m: FaceMat } = undefined;
            // fill mask
            var mz: usize = 0;
            while (mz < constants.chunk_size_z) : (mz += 1) {
                var mx: usize = 0;
                while (mx < constants.chunk_size_x) : (mx += 1) {
                    const idxm: usize = mz * constants.chunk_size_x + mx;
                    mask[idxm].present = false;
                    const src_idx: usize = mz * (constants.chunk_size_x * constants.section_height) + mx * constants.section_height + ly_top;
                    const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, src_idx, bpi));
                    if (pidx >= pal_info.items.len) continue;
                    const info = pal_info.items[pidx];
                    if (!info.draw) continue;
                    if (!self.isSolidAtSnapshot(snap, ch.pos.x, ch.pos.z, @as(i32, @intCast(mx)), abs_y + 1, @as(i32, @intCast(mz)))) {
                        mask[idxm] = .{ .present = true, .m = .{ .off = info.top_tx.offset, .sc = info.top_tx.scale } };
                    }
                }
            }
            // rectangles
            mz = 0;
            while (mz < constants.chunk_size_z) : (mz += 1) {
                var mx: usize = 0;
                while (mx < constants.chunk_size_x) : (mx += 1) {
                    const base_idx: usize = mz * constants.chunk_size_x + mx;
                    if (!mask[base_idx].present) continue;
                    const mkey: FaceMat = mask[base_idx].m;
                    // width
                    var w: usize = 1;
                    while ((mx + w) < constants.chunk_size_x) : (w += 1) {
                        const idx2 = mz * constants.chunk_size_x + (mx + w);
                        if (!(mask[idx2].present and Mat.eq(mask[idx2].m, mkey))) break;
                    }
                    // height
                    var h: usize = 1;
                    outer: while ((mz + h) < constants.chunk_size_z) : (h += 1) {
                        var kx: usize = 0;
                        while (kx < w) : (kx += 1) {
                            const idx3 = (mz + h) * constants.chunk_size_x + (mx + kx);
                            if (!(mask[idx3].present and Mat.eq(mask[idx3].m, mkey))) break :outer;
                        }
                    }
                    // emit quad
                    const wx0: f32 = base_x + @as(f32, @floatFromInt(mx));
                    const wz0: f32 = base_z + @as(f32, @floatFromInt(mz));
                    const wx1: f32 = base_x + @as(f32, @floatFromInt(mx + w));
                    const wz1: f32 = base_z + @as(f32, @floatFromInt(mz + h));
                    const wy1: f32 = @as(f32, @floatFromInt(abs_y + 1));
                    const uv00: [2]f32 = .{ 0, 0 };
                    const uv0h: [2]f32 = .{ 0, @as(f32, @floatFromInt(h)) };
                    const uvwh: [2]f32 = .{ @as(f32, @floatFromInt(w)), @as(f32, @floatFromInt(h)) };
                    const uvw0: [2]f32 = .{ @as(f32, @floatFromInt(w)), 0 };
                    emitQuad(alloc, out, .{ wx0, wy1, wz0 }, .{ wx0, wy1, wz1 }, .{ wx1, wy1, wz1 }, .{ wx1, wy1, wz0 }, uv00, uv0h, uvwh, uvw0, mkey.sc, mkey.off);
                    // clear mask
                    var zz: usize = 0;
                    while (zz < h) : (zz += 1) {
                        var xx: usize = 0;
                        while (xx < w) : (xx += 1) mask[(mz + zz) * constants.chunk_size_x + (mx + xx)].present = false;
                    }
                }
            }
        }

        // Remaining faces per-voxel
        for (0..constants.chunk_size_z) |lz| {
            for (0..constants.chunk_size_x) |lx| {
                for (0..constants.section_height) |ly| {
                    const idx: usize = lz * (constants.chunk_size_x * constants.section_height) + lx * constants.section_height + ly;
                    const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, idx, bpi));
                    if (pidx >= pal_info.items.len) continue;
                    const info = pal_info.items[pidx];
                    if (!info.draw) continue;

                    const wx0: f32 = base_x + @as(f32, @floatFromInt(lx));
                    const wy0: f32 = @as(f32, @floatFromInt(@as(i32, @intCast(sy * constants.section_height)) + @as(i32, @intCast(ly))));
                    const wz0: f32 = base_z + @as(f32, @floatFromInt(lz));
                    const wx1: f32 = wx0 + 1.0;
                    const wy1: f32 = wy0 + 1.0;
                    const wz1: f32 = wz0 + 1.0;

                    const abs_y_this: i32 = section_base_y + @as(i32, @intCast(ly));
                    const uv00: [2]f32 = .{ 0, 0 };
                    const uv01: [2]f32 = .{ 0, 1 };
                    const uv11: [2]f32 = .{ 1, 1 };
                    const uv10: [2]f32 = .{ 1, 0 };

                    if (!self.isSolidAtSnapshot(snap, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y_this + 1, @as(i32, @intCast(lz)))) {
                        emitQuad(alloc, out, .{ wx0, wy1, wz0 }, .{ wx0, wy1, wz1 }, .{ wx1, wy1, wz1 }, .{ wx1, wy1, wz0 }, uv00, uv01, uv11, uv10, info.top_tx.scale, info.top_tx.offset);
                    }
                    if (!self.isSolidAtSnapshot(snap, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y_this - 1, @as(i32, @intCast(lz)))) {
                        emitQuad(alloc, out, .{ wx0, wy0, wz0 }, .{ wx1, wy0, wz0 }, .{ wx1, wy0, wz1 }, .{ wx0, wy0, wz1 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset);
                    }
                    if (!self.isSolidAtSnapshot(snap, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)) - 1, abs_y_this, @as(i32, @intCast(lz)))) {
                        emitQuad(alloc, out, .{ wx0, wy0, wz1 }, .{ wx0, wy1, wz1 }, .{ wx0, wy1, wz0 }, .{ wx0, wy0, wz0 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset);
                    }
                    if (!self.isSolidAtSnapshot(snap, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)) + 1, abs_y_this, @as(i32, @intCast(lz)))) {
                        emitQuad(alloc, out, .{ wx1, wy0, wz0 }, .{ wx1, wy1, wz0 }, .{ wx1, wy1, wz1 }, .{ wx1, wy0, wz1 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset);
                    }
                    if (!self.isSolidAtSnapshot(snap, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y_this, @as(i32, @intCast(lz)) - 1)) {
                        emitQuad(alloc, out, .{ wx0, wy0, wz0 }, .{ wx0, wy1, wz0 }, .{ wx1, wy1, wz0 }, .{ wx1, wy0, wz0 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset);
                    }
                    if (!self.isSolidAtSnapshot(snap, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y_this, @as(i32, @intCast(lz)) + 1)) {
                        emitQuad(alloc, out, .{ wx1, wy0, wz1 }, .{ wx1, wy1, wz1 }, .{ wx0, wy1, wz1 }, .{ wx0, wy0, wz1 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset);
                    }
                }
            }
        }
    }

    fn meshRegionFromSnapshot(self: *Client, alloc: std.mem.Allocator, snap: *const RegionSnapshot) MeshingResult {
        var draws = self.allocator.alloc(SectionDraw, snap.chunks.len * @as(usize, @intCast(snap.section_count_y))) catch return .{ .rpos = snap.rpos, .built_chunk_count = 0, .verts = &[_]Vertex{}, .draws = &[_]SectionDraw{} };
        var draws_len: usize = 0;
        var verts = std.ArrayList(Vertex).initCapacity(alloc, 0) catch {
            self.allocator.free(draws);
            return .{ .rpos = snap.rpos, .built_chunk_count = 0, .verts = &[_]Vertex{}, .draws = &[_]SectionDraw{} };
        };
        defer verts.deinit(alloc);
        for (snap.chunks) |ch| {
            const base_x: f32 = @floatFromInt(ch.pos.x * @as(i32, @intCast(constants.chunk_size_x)));
            const base_z: f32 = @floatFromInt(ch.pos.z * @as(i32, @intCast(constants.chunk_size_z)));
            var sy: usize = 0;
            while (sy < ch.sections.len) : (sy += 1) {
                const start: usize = verts.items.len;
                self.buildSectionVerticesFromSnapshot(snap, &ch, sy, base_x, base_z, &verts, alloc);
                const count: usize = verts.items.len - start;
                if (count > 0) {
                    draws[draws_len] = .{ .chunk_pos = ch.pos, .sy = @intCast(sy), .first = @intCast(start), .count = @intCast(count) };
                    draws_len += 1;
                }
            }
        }
        // shrink draws
        if (draws_len < draws.len) {
            const new_draws = self.allocator.alloc(SectionDraw, draws_len) catch {
                // keep existing draws oversized
                return .{ .rpos = snap.rpos, .built_chunk_count = snap.chunks.len, .verts = blk: {
                    if (verts.items.len == 0) break :blk &[_]Vertex{};
                    const owned = alloc.alloc(Vertex, verts.items.len) catch break :blk &[_]Vertex{};
                    @memcpy(owned[0..verts.items.len], verts.items[0..verts.items.len]);
                    break :blk owned;
                }, .draws = draws };
            };
            @memcpy(new_draws[0..draws_len], draws[0..draws_len]);
            self.allocator.free(draws);
            draws = new_draws;
        }
        // copy verts into owned slice
        var verts_owned: []Vertex = &[_]Vertex{};
        if (verts.items.len > 0) {
            verts_owned = alloc.alloc(Vertex, verts.items.len) catch &[_]Vertex{};
            if (verts_owned.len == verts.items.len) @memcpy(verts_owned[0..verts.items.len], verts.items[0..verts.items.len]);
        }
        return .{ .rpos = snap.rpos, .built_chunk_count = snap.chunks.len, .verts = verts_owned, .draws = draws };
    }

    fn adoptMesherResults(self: *Client) void {
        var processed: u32 = 0;
        while (processed < self.adopt_budget_per_frame) {
            // fetch one result
            self.mesher_mutex.lock();
            if (self.mesher_results.items.len == 0) {
                self.mesher_mutex.unlock();
                break;
            }
            const mr = self.mesher_results.items[self.mesher_results.items.len - 1];
            _ = self.mesher_results.pop();
            self.mesher_mutex.unlock();

            // Upload GPU buffer
            var buf: sg.Buffer = .{};
            if (mr.verts.len > 0) {
                buf = sg.makeBuffer(.{ .usage = .{ .vertex_buffer = true }, .data = sg.asRange(mr.verts) });
            }

            // Swap into cache
            if (self.region_mesh_cache.getPtr(mr.rpos)) |existing| {
                const old = existing.*;
                existing.* = .{ .vbuf = buf, .draws = mr.draws, .built_chunk_count = mr.built_chunk_count, .dirty = false, .inflight = false, .last_built_frame = self.frame_index };
                if (old.vbuf.id != 0) sg.destroyBuffer(old.vbuf);
                self.allocator.free(old.draws);
            } else {
                if (self.regionMeshCacheCount() < REGION_MESH_BUDGET) {
                    const rm: RegionMesh = .{ .vbuf = buf, .draws = mr.draws, .built_chunk_count = mr.built_chunk_count, .dirty = false, .inflight = false, .last_built_frame = self.frame_index };
                    _ = self.region_mesh_cache.put(mr.rpos, rm) catch {
                        if (rm.vbuf.id != 0) sg.destroyBuffer(rm.vbuf);
                        self.allocator.free(mr.draws);
                    };
                } else {
                    // over budget: drop result
                    if (buf.id != 0) sg.destroyBuffer(buf);
                    self.allocator.free(mr.draws);
                }
            }

            // free CPU verts
            if (mr.verts.len > 0) std.heap.page_allocator.free(mr.verts);
            processed += 1;
        }
        self.dbg_last_adopted = processed;
    }

    fn ensureRegionMesh(self: *Client, rpos: simulation.RegionPos, expected_chunk_count: usize) void {
        // If nothing to build yet, avoid scheduling and clear any stale inflight
        if (expected_chunk_count == 0) {
            if (self.region_mesh_cache.getPtr(rpos)) |existing0| {
                existing0.built_chunk_count = 0;
                existing0.inflight = false;
            }
            return;
        }
        // If cache entry exists and is up-to-date, nothing to do
        if (self.region_mesh_cache.getPtr(rpos)) |existing| {
            if (existing.built_chunk_count == expected_chunk_count and existing.vbuf.id != 0) return;
            // If a rebuild is already in-flight but appears stuck, allow reschedule after cooldown
            if (existing.inflight) {
                if ((self.frame_index - existing.last_built_frame) < self.rebuild_cooldown_frames) return;
                // Reset and fall through to schedule a fresh job
                existing.inflight = false;
            }
            // Throttle job issuance
            if (self.rebuilds_issued_this_frame >= self.rebuild_budget_per_frame) return;
            if ((self.frame_index - existing.last_built_frame) < self.rebuild_cooldown_frames) return;
            // Mark inflight and enqueue job
            existing.inflight = true;
            self.mesher_mutex.lock();
            self.mesher_jobs.append(self.allocator, .{ .rpos = rpos }) catch {};
            self.mesher_cv.broadcast();
            self.mesher_mutex.unlock();
            self.rebuilds_issued_this_frame += 1;
            return;
        }
        // No cache entry: if under budget, insert placeholder and schedule build
        if (self.rebuilds_issued_this_frame >= self.rebuild_budget_per_frame) return;
        if (self.regionMeshCacheCount() >= REGION_MESH_BUDGET) return;
        const placeholder: RegionMesh = .{ .vbuf = .{}, .draws = &[_]SectionDraw{}, .built_chunk_count = 0, .dirty = false, .inflight = true, .last_built_frame = self.frame_index };
        _ = self.region_mesh_cache.put(rpos, placeholder) catch return;
        self.mesher_mutex.lock();
        self.mesher_jobs.append(self.allocator, .{ .rpos = rpos }) catch {};
        self.mesher_cv.broadcast();
        self.mesher_mutex.unlock();
        self.rebuilds_issued_this_frame += 1;
    }

    fn renderWorldCached(self: *Client, vs_in: *const shd_mod.VsParams) void {
        // Adopt up to N meshing results per frame on the render thread
        self.adoptMesherResults();
        const wk = "minecraft:overworld";
        // Snapshot region positions and chunk counts under world lock
        var regions = std.ArrayList(struct { rpos: simulation.RegionPos, chunk_count: usize }).initCapacity(self.allocator, 0) catch return;
        defer regions.deinit(self.allocator);
        self.sim.worlds_mutex.lock();
        const ws = self.sim.worlds_state.getPtr(wk);
        if (ws) |ws_ptr| {
            var reg_it = ws_ptr.regions.iterator();
            while (reg_it.next()) |rentry| {
                const rpos = rentry.key_ptr.*;
                const rs_ptr = rentry.value_ptr;
                const count = rs_ptr.chunks.items.len;
                regions.append(self.allocator, .{ .rpos = rpos, .chunk_count = count }) catch {};
            }
        }
        self.sim.worlds_mutex.unlock();

        // constants for padding
        const pad_u: f32 = if (self.atlas_w > 0) (1.0 / @as(f32, @floatFromInt(self.atlas_w))) else 0.0;
        const pad_v: f32 = if (self.atlas_h > 0) (1.0 / @as(f32, @floatFromInt(self.atlas_h))) else 0.0;
        var vs_loc: shd_mod.VsParams = .{ .mvp = vs_in.mvp, .atlas_pad = .{ pad_u, pad_v } };

        // reset per-frame rebuild counter
        self.rebuilds_issued_this_frame = 0;

        // Iterate regions without holding world lock
        var i: usize = 0;
        while (i < regions.items.len) : (i += 1) {
            const rp = regions.items[i].rpos;
            const expected_chunks = regions.items[i].chunk_count;
            // Ensure mesh scheduled or up-to-date
            self.ensureRegionMesh(rp, expected_chunks);
            const rmesh = self.region_mesh_cache.getPtr(rp) orelse continue;
            if (rmesh.vbuf.id == 0 or rmesh.draws.len == 0) continue;

            // Bind region buffer once
            var b = self.bind;
            b.vertex_buffers[0] = rmesh.vbuf;
            sg.applyBindings(b);

            // Draw visible sections in this region
            var di: usize = 0;
            while (di < rmesh.draws.len) : (di += 1) {
                const d = rmesh.draws[di];
                // Section-level culling
                if (self.frustum_valid) {
                    const base_x: f32 = @floatFromInt(d.chunk_pos.x * @as(i32, @intCast(constants.chunk_size_x)));
                    const base_z: f32 = @floatFromInt(d.chunk_pos.z * @as(i32, @intCast(constants.chunk_size_z)));
                    const minp_s: [3]f32 = .{ base_x, @as(f32, @floatFromInt((@as(usize, d.sy) * constants.section_height))), base_z };
                    const maxp_s: [3]f32 = .{ base_x + 16.0, @as(f32, @floatFromInt(((@as(usize, d.sy) + 1) * constants.section_height))), base_z + 16.0 };
                    const cam = self.camera.pos;
                    const vis_sec = aabbContainsPoint(minp_s, maxp_s, cam) or aabbInFrustum(self.frustum_planes, minp_s, maxp_s, 0.5);
                    if (!vis_sec) continue;
                }
                sg.applyUniforms(0, sg.asRange(&vs_loc));
                sg.draw(d.first, d.count, 1);
                _ = self.last_visible_frame_chunk.put(d.chunk_pos, self.frame_index) catch {};
            }
            _ = self.last_visible_frame_region.put(rp, self.frame_index) catch {};
        }

        // Evict old or excess region meshes to bound pool usage
        self.evictRegionMeshes(REGION_MESH_BUDGET, 10);
    }

    fn buildChunkBlockMesh(self: *Client, out: *std.ArrayList(Vertex)) usize {
        // Helpers for UV padding and wrapping
        const pad_u: f32 = if (self.atlas_w > 0) (1.0 / @as(f32, @floatFromInt(self.atlas_w))) else 0.0;
        const pad_v: f32 = if (self.atlas_h > 0) (1.0 / @as(f32, @floatFromInt(self.atlas_h))) else 0.0;
        const addQuad = struct {
            fn wrap01(v: f32) f32 {
                const f = v - std.math.floor(v);
                return if (f == 0.0 and v > 0.0) 1.0 else f;
            }
            fn call(list: *std.ArrayList(Vertex), v0: [3]f32, v1: [3]f32, v2: [3]f32, v3: [3]f32, uv0: [2]f32, uv1: [2]f32, uv2: [2]f32, uv3: [2]f32, uv_scale_p: [2]f32, uv_offset_p: [2]f32, pad_u_in: f32, pad_v_in: f32) void {
                const w0x: f32 = wrap01(uv0[0]);
                const w0y: f32 = wrap01(uv0[1]);
                const w1x: f32 = wrap01(uv1[0]);
                const w1y: f32 = wrap01(uv1[1]);
                const w2x: f32 = wrap01(uv2[0]);
                const w2y: f32 = wrap01(uv2[1]);
                const w3x: f32 = wrap01(uv3[0]);
                const w3y: f32 = wrap01(uv3[1]);
                const min_u = uv_offset_p[0] + pad_u_in;
                const min_v = uv_offset_p[1] + pad_v_in;
                const range_u = @max(0.0, uv_scale_p[0] - 2.0 * pad_u_in);
                const range_v = @max(0.0, uv_scale_p[1] - 2.0 * pad_v_in);
                const t0 = .{ min_u + w0x * range_u, min_v + w0y * range_v };
                const t1 = .{ min_u + w1x * range_u, min_v + w1y * range_v };
                const t2 = .{ min_u + w2x * range_u, min_v + w2y * range_v };
                const t3 = .{ min_u + w3x * range_u, min_v + w3y * range_v };
                list.append(self.allocator, .{ .pos = v0, .uv = t0 }) catch return;
                list.append(self.allocator, .{ .pos = v1, .uv = t1 }) catch return;
                list.append(self.allocator, .{ .pos = v2, .uv = t2 }) catch return;
                list.append(self.allocator, .{ .pos = v0, .uv = t0 }) catch return;
                list.append(self.allocator, .{ .pos = v2, .uv = t2 }) catch return;
                list.append(self.allocator, .{ .pos = v3, .uv = t3 }) catch return;
            }
        }.call;

        // Mesh all loaded chunks in the single overworld for now
        const wk = "minecraft:overworld";
        self.sim.worlds_mutex.lock();
        defer self.sim.worlds_mutex.unlock();
        const ws = self.sim.worlds_state.getPtr(wk) orelse return 0;

        const verts_before: usize = out.items.len;

        var reg_it = ws.regions.iterator();
        while (reg_it.next()) |rentry| {
            const rs_ptr = rentry.value_ptr; // keep pointer to access chunk_index
            for (rs_ptr.chunks.items) |*ch_ptr| {
                const ch = ch_ptr.*; // copy for convenience
                const base_x: f32 = @floatFromInt(ch.pos.x * @as(i32, @intCast(constants.chunk_size_x)));
                const base_z: f32 = @floatFromInt(ch.pos.z * @as(i32, @intCast(constants.chunk_size_z)));

                // Frustum culling per-chunk AABB
                if (self.frustum_valid) {
                    const minp: [3]f32 = .{ base_x, 0.0, base_z };
                    const maxp: [3]f32 = .{
                        base_x + @as(f32, @floatFromInt(constants.chunk_size_x)),
                        @as(f32, @floatFromInt(ch.sections.len * constants.section_height)),
                        base_z + @as(f32, @floatFromInt(constants.chunk_size_z)),
                    };
                    const cull_margin: f32 = 0.5; // tighter margin; hysteresis handles stability
                    // Always keep the chunk containing the camera
                    const cam = self.camera.pos;
                    var visible = aabbContainsPoint(minp, maxp, cam);
                    if (!visible) {
                        visible = aabbInFrustum(self.frustum_planes, minp, maxp, cull_margin);
                        if (!visible) {
                            // Per-chunk linger: keep if seen within the last 2 frames
                            if (self.last_visible_frame.get(ch.pos)) |last_f| {
                                const delta = self.frame_index - last_f;
                                if (delta <= 2) visible = true;
                            }
                        }
                    }
                    if (!visible) continue;
                    // Update last visible frame index
                    _ = self.last_visible_frame.put(ch.pos, self.frame_index) catch {};
                }

                // Rough upper bound capacity: worst-case 6 faces per voxel (very high), but we reserve moderately
                const cur_len: usize = out.items.len;
                out.ensureTotalCapacity(self.allocator, cur_len + 16 * 16 * 32 * 6) catch {};

                // Iterate sections
                const sections_len: usize = ch.sections.len;
                var sy: usize = 0;
                while (sy < sections_len) : (sy += 1) {
                    const s = ch.sections[sy];
                    const pal_len = s.palette.len;
                    const bpi: u6 = bitsFor(pal_len);
                    // Build per-palette info (draw, solid, top/side transforms)
                    const PalInfo = struct {
                        draw: bool,
                        solid: bool,
                        top_tx: AtlasTransform,
                        side_tx: AtlasTransform,
                    };
                    var pal_info = std.ArrayList(PalInfo).initCapacity(self.allocator, pal_len) catch {
                        // If alloc fails, skip this section gracefully
                        continue;
                    };
                    defer pal_info.deinit(self.allocator);
                    var pi: usize = 0;
                    while (pi < pal_len) : (pi += 1) {
                        const bid: u32 = s.palette[pi];
                        var pinfo: PalInfo = .{
                            .draw = true,
                            .solid = true,
                            .top_tx = .{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale },
                            .side_tx = .{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale },
                        };
                        // Solid from registry collision flag
                        if (bid < self.sim.reg.blocks.items.len) {
                            pinfo.solid = self.sim.reg.blocks.items[@intCast(bid)].full_block_collision;
                        }
                        // Treat block 0 (air) as not drawn and not solid
                        if (bid == 0) {
                            pinfo.draw = false;
                            pinfo.solid = false;
                        } else {
                            // Resource lookup by block name
                            const name = self.sim.reg.getBlockName(@intCast(bid));
                            if (self.sim.reg.resources.get(name)) |res| switch (res) {
                                .Void => {
                                    pinfo.draw = false;
                                    pinfo.solid = false;
                                },
                                .Uniform => |u| {
                                    const tx: AtlasTransform = self.atlas_uv_by_path.get(u.all_path) orelse AtlasTransform{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale };
                                    pinfo.top_tx = tx;
                                    pinfo.side_tx = tx;
                                },
                                .Facing => |f| {
                                    const top_tx: AtlasTransform = self.atlas_uv_by_path.get(f.face_path) orelse AtlasTransform{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale };
                                    const side_tx: AtlasTransform = self.atlas_uv_by_path.get(f.other_path) orelse AtlasTransform{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale };
                                    pinfo.top_tx = top_tx;
                                    pinfo.side_tx = side_tx;
                                },
                            };
                        }
                        pal_info.appendAssumeCapacity(pinfo);
                    }

                    // Precompute section base Y to avoid capturing mutable 'sy'
                    const section_base_y: i32 = @intCast(sy * constants.section_height);

                    // Iterate voxels: order used in worldgen is (lz, lx, ly) with ly fastest
                    for (0..constants.chunk_size_z) |lz| {
                        for (0..constants.chunk_size_x) |lx| {
                            for (0..constants.section_height) |ly| {
                                const idx: usize = lz * (constants.chunk_size_x * constants.section_height) + lx * constants.section_height + ly;
                                const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, idx, bpi));
                                if (pidx >= pal_info.items.len) continue;
                                const info = pal_info.items[pidx];
                                if (!info.draw) continue;

                                // World coords for this block
                                const wx0: f32 = base_x + @as(f32, @floatFromInt(lx));
                                const wy0: f32 = @as(f32, @floatFromInt(@as(i32, @intCast(sy * constants.section_height)) + @as(i32, @intCast(ly))));
                                const wz0: f32 = base_z + @as(f32, @floatFromInt(lz));
                                const wx1: f32 = wx0 + 1.0;
                                const wy1: f32 = wy0 + 1.0;
                                const wz1: f32 = wz0 + 1.0;

                                // We'll use a helper that takes absolute Y and handles cross-chunk
                                const abs_y_this: i32 = section_base_y + @as(i32, @intCast(ly));

                                // Emit faces if neighbor is not solid
                                const uv00: [2]f32 = .{ 0, 0 };
                                const uv01: [2]f32 = .{ 0, 1 };
                                const uv11: [2]f32 = .{ 1, 1 };
                                const uv10: [2]f32 = .{ 1, 0 };

                                // +Y (top)
                                if (!self.isSolidAt(ws, &ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y_this + 1, @as(i32, @intCast(lz)))) {
                                    addQuad(out, .{ wx0, wy1, wz0 }, .{ wx0, wy1, wz1 }, .{ wx1, wy1, wz1 }, .{ wx1, wy1, wz0 }, uv00, uv01, uv11, uv10, info.top_tx.scale, info.top_tx.offset, pad_u, pad_v);
                                }
                                // -Y (bottom)
                                if (!self.isSolidAt(ws, &ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y_this - 1, @as(i32, @intCast(lz)))) {
                                    addQuad(out, .{ wx0, wy0, wz0 }, .{ wx1, wy0, wz0 }, .{ wx1, wy0, wz1 }, .{ wx0, wy0, wz1 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset, pad_u, pad_v);
                                }
                                // -X (west)
                                if (!self.isSolidAt(ws, &ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)) - 1, abs_y_this, @as(i32, @intCast(lz)))) {
                                    addQuad(out, .{ wx0, wy0, wz1 }, .{ wx0, wy1, wz1 }, .{ wx0, wy1, wz0 }, .{ wx0, wy0, wz0 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset, pad_u, pad_v);
                                }
                                // +X (east)
                                if (!self.isSolidAt(ws, &ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)) + 1, abs_y_this, @as(i32, @intCast(lz)))) {
                                    addQuad(out, .{ wx1, wy0, wz0 }, .{ wx1, wy1, wz0 }, .{ wx1, wy1, wz1 }, .{ wx1, wy0, wz1 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset, pad_u, pad_v);
                                }
                                // -Z (north)
                                if (!self.isSolidAt(ws, &ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y_this, @as(i32, @intCast(lz)) - 1)) {
                                    addQuad(out, .{ wx0, wy0, wz0 }, .{ wx0, wy1, wz0 }, .{ wx1, wy1, wz0 }, .{ wx1, wy0, wz0 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset, pad_u, pad_v);
                                }
                                // +Z (south)
                                if (!self.isSolidAt(ws, &ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y_this, @as(i32, @intCast(lz)) + 1)) {
                                    addQuad(out, .{ wx1, wy0, wz1 }, .{ wx1, wy1, wz1 }, .{ wx0, wy1, wz1 }, .{ wx0, wy0, wz1 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset, pad_u, pad_v);
                                }
                            }
                        }
                    }
                }
            }
        }
        return out.items.len - verts_before;
    }

    fn clearMovementInputs(self: *Client) void {
        self.move_forward = false;
        self.move_back = false;
        self.move_left = false;
        self.move_right = false;
        // zero horizontal velocity (local)
        self.local_vel[0] = 0;
        self.local_vel[2] = 0;
        // Optionally also zero on SIM at next tick
        self.jump_pending = false;
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

    // Extract 6 view-frustum planes from a column-major MVP matrix.
    // Plane order: left, right, bottom, top, near, far. Each plane is (a,b,c,d) with outward normal.
    fn extractFrustumPlanes(m: [16]f32) [6][4]f32 {
        // rows in column-major storage
        const r0 = [4]f32{ m[0], m[4], m[8], m[12] };
        const r1 = [4]f32{ m[1], m[5], m[9], m[13] };
        const r2 = [4]f32{ m[2], m[6], m[10], m[14] };
        const r3 = [4]f32{ m[3], m[7], m[11], m[15] };
        var planes: [6][4]f32 = undefined;
        // left = r3 + r0
        planes[0] = .{ r3[0] + r0[0], r3[1] + r0[1], r3[2] + r0[2], r3[3] + r0[3] };
        // right = r3 - r0
        planes[1] = .{ r3[0] - r0[0], r3[1] - r0[1], r3[2] - r0[2], r3[3] - r0[3] };
        // bottom = r3 + r1
        planes[2] = .{ r3[0] + r1[0], r3[1] + r1[1], r3[2] + r1[2], r3[3] + r1[3] };
        // top = r3 - r1
        planes[3] = .{ r3[0] - r1[0], r3[1] - r1[1], r3[2] - r1[2], r3[3] - r1[3] };
        // near = r3 + r2
        planes[4] = .{ r3[0] + r2[0], r3[1] + r2[1], r3[2] + r2[2], r3[3] + r2[3] };
        // far = r3 - r2
        planes[5] = .{ r3[0] - r2[0], r3[1] - r2[1], r3[2] - r2[2], r3[3] - r2[3] };
        // normalize planes
        var pi: usize = 0;
        while (pi < 6) : (pi += 1) {
            const a = planes[pi][0];
            const b = planes[pi][1];
            const c = planes[pi][2];
            const inv_len = 1.0 / @max(0.000001, @sqrt(a * a + b * b + c * c));
            planes[pi][0] = a * inv_len;
            planes[pi][1] = b * inv_len;
            planes[pi][2] = c * inv_len;
            planes[pi][3] = planes[pi][3] * inv_len;
        }
        return planes;
    }

    fn aabbInFrustum(planes: [6][4]f32, min: [3]f32, max: [3]f32, margin: f32) bool {
        var p: [3]f32 = undefined;
        var i: usize = 0;
        while (i < 6) : (i += 1) {
            const a = planes[i][0];
            const b = planes[i][1];
            const c = planes[i][2];
            const d = planes[i][3];
            p[0] = if (a >= 0) max[0] else min[0];
            p[1] = if (b >= 0) max[1] else min[1];
            p[2] = if (c >= 0) max[2] else min[2];
            const dist = a * p[0] + b * p[1] + c * p[2] + d;
            if (dist < -margin) return false;
        }
        return true;
    }

    fn updateFrustum(self: *Client, mvp: [16]f32) void {
        if (self.frustum_valid) {
            self.prev_frustum_planes = self.frustum_planes;
            self.prev_frustum_valid = true;
        }
        self.frustum_planes = extractFrustumPlanes(mvp);
        self.frustum_valid = true;
    }

    fn aabbContainsPoint(min: [3]f32, max: [3]f32, p: [3]f32) bool {
        return p[0] >= min[0] and p[0] <= max[0] and p[1] >= min[1] and p[1] <= max[1] and p[2] >= min[2] and p[2] <= max[2];
    }

    // Local physics step using shared routine
    fn localPhysicsStep(self: *Client, dt: f32) void {
        const cfg: physics.PhysicsConfig = .{ .gravity = 9.80665 };
        var kin: physics.EntityKinematics = .{
            .pos = self.local_pos,
            .vel = self.local_vel,
            .half_extents_y = self.local_aabb_half_extents[1],
            .on_ground = &self.local_on_ground,
        };
        const sampler = struct {
            fn call(ctx: *anyopaque, x: f32, z: f32) ?f32 {
                const cli: *Client = @ptrCast(@alignCast(ctx));
                const xi: i32 = @intFromFloat(@floor(x));
                const zi: i32 = @intFromFloat(@floor(z));
                const cx: i32 = @divFloor(xi, @as(i32, @intCast(constants.chunk_size_x)));
                const cz: i32 = @divFloor(zi, @as(i32, @intCast(constants.chunk_size_z)));
                const lx: i32 = @mod(xi, @as(i32, @intCast(constants.chunk_size_x)));
                const lz: i32 = @mod(zi, @as(i32, @intCast(constants.chunk_size_z)));
                const wk = "minecraft:overworld";
                cli.sim.worlds_mutex.lock();
                defer cli.sim.worlds_mutex.unlock();
                const ws = cli.sim.worlds_state.getPtr(wk) orelse return null;
                const rp = simulation.RegionPos{ .x = @divFloor(cx, 32), .z = @divFloor(cz, 32) };
                const rs = ws.regions.getPtr(rp) orelse return null;
                const idx_opt = rs.chunk_index.get(.{ .x = cx, .z = cz });
                if (idx_opt == null) return null;
                const ch = rs.chunks.items[idx_opt.?];
                const lx_u: usize = @intCast(lx);
                const lz_u: usize = @intCast(lz);
                if (lx_u >= 16 or lz_u >= 16) return null;
                const h = ch.heightmaps.world_surface[lz_u * 16 + lx_u];
                if (h < 0) return null;
                return @as(f32, @floatFromInt(h + 1));
            }
        }.call;
        physics.integrateStep(cfg, &kin, dt, sampler, self);
        self.local_pos = kin.pos;
        self.local_vel = kin.vel;
    }

    // Send movement state to SIM at most once per authoritative tick
    fn maybeSendMoveToSim(self: *Client) void {
        const snap = self.pollSnapshot().*;
        if (snap.tick == self.last_move_tick_sent) return;
        self.last_move_tick_sent = snap.tick;
        self.sim.connections_mutex.lock();
        const idx_opt = self.sim.entity_by_player.get(self.player_id);
        if (idx_opt) |idx| {
            if (idx < self.sim.dynamic_entities.items.len) {
                var e = &self.sim.dynamic_entities.items[idx];
                // Horizontal velocity from local prediction
                e.vel[0] = self.local_vel[0];
                e.vel[2] = self.local_vel[2];
                // Apply jump impulse if pending (server will clamp by on_ground)
                if (self.jump_pending) {
                    const g2: f32 = 9.80665;
                    const target_h: f32 = 1.1;
                    const v0: f32 = @sqrt(2.0 * g2 * target_h);
                    e.vel[1] = v0;
                    e.flags.on_ground = false;
                }
                // Keep facing aligned to camera forward projected onto XZ when moving
                if (self.move_forward or self.move_back or self.move_left or self.move_right) {
                    const lx = self.local_dir[0];
                    const lz = self.local_dir[2];
                    const llen: f32 = @sqrt(lx * lx + lz * lz);
                    if (llen > 0.0001) {
                        e.facing_dir_xz = .{ lx / llen, lz / llen };
                        const facing_yaw: f32 = std.math.atan2(e.facing_dir_xz[1], e.facing_dir_xz[0]);
                        e.yaw_pitch_roll = .{ facing_yaw, 0, 0 };
                    }
                }
            }
        }
        self.sim.connections_mutex.unlock();
        // clear one-shot jump after sending this tick
        self.jump_pending = false;
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
            .{ .pos = .{ x0, y0, 0 }, .uv = .{ 0, 0 }, .uv_off = .{ 0, 0 }, .uv_scale = .{ 1, 1 } },
            .{ .pos = .{ x1, y0, 0 }, .uv = .{ 1, 0 }, .uv_off = .{ 0, 0 }, .uv_scale = .{ 1, 1 } },
            .{ .pos = .{ x1, y1, 0 }, .uv = .{ 1, 1 }, .uv_off = .{ 0, 0 }, .uv_scale = .{ 1, 1 } },
            .{ .pos = .{ x0, y0, 0 }, .uv = .{ 0, 0 }, .uv_off = .{ 0, 0 }, .uv_scale = .{ 1, 1 } },
            .{ .pos = .{ x1, y1, 0 }, .uv = .{ 1, 1 }, .uv_off = .{ 0, 0 }, .uv_scale = .{ 1, 1 } },
            .{ .pos = .{ x0, y1, 0 }, .uv = .{ 0, 1 }, .uv_off = .{ 0, 0 }, .uv_scale = .{ 1, 1 } },
        };
        const ortho = makeOrthoTopLeft(w_px, h_px);
        var vs_params: shd_mod.VsParams = .{ .mvp = ortho, .atlas_pad = .{ 0, 0 } };
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

    fn regionMeshCacheCount(self: *Client) usize {
        var cnt: usize = 0;
        var it = self.region_mesh_cache.keyIterator();
        while (it.next()) |_| cnt += 1;
        return cnt;
    }

    fn evictRegionMeshes(self: *Client, max_count: usize, ttl_frames: u32) void {
        // Collect stale regions (older than ttl_frames)
        var to_remove = std.ArrayList(simulation.RegionPos).initCapacity(self.allocator, 0) catch {
            return;
        };
        defer to_remove.deinit(self.allocator);
        var it = self.region_mesh_cache.iterator();
        while (it.next()) |entry| {
            const pos = entry.key_ptr.*;
            const last_opt = self.last_visible_frame_region.get(pos);
            const last_seen: u32 = last_opt orelse 0;
            const age = self.frame_index - last_seen;
            if (age > ttl_frames) {
                to_remove.append(self.allocator, pos) catch {};
            }
        }
        // Remove stale
        var i: usize = 0;
        while (i < to_remove.items.len) : (i += 1) {
            const pos = to_remove.items[i];
            if (self.region_mesh_cache.get(pos)) |rm| {
                if (rm.vbuf.id != 0) sg.destroyBuffer(rm.vbuf);
                self.allocator.free(rm.draws);
            }
            _ = self.region_mesh_cache.remove(pos);
        }

        // If still over budget, remove oldest until under limit
        var count = self.regionMeshCacheCount();
        if (count <= max_count) return;

        var candidates = std.ArrayList(struct { pos: simulation.RegionPos, last: u32 }).initCapacity(self.allocator, 0) catch {
            return;
        };
        defer candidates.deinit(self.allocator);
        var it2 = self.region_mesh_cache.iterator();
        while (it2.next()) |entry| {
            const pos = entry.key_ptr.*;
            const last = self.last_visible_frame_region.get(pos) orelse 0;
            candidates.append(self.allocator, .{ .pos = pos, .last = last }) catch {};
        }
        while (count > max_count and candidates.items.len > 0) {
            var min_idx: usize = 0;
            var min_last: u32 = candidates.items[0].last;
            var j: usize = 1;
            while (j < candidates.items.len) : (j += 1) {
                if (candidates.items[j].last < min_last) {
                    min_last = candidates.items[j].last;
                    min_idx = j;
                }
            }
            const pos = candidates.items[min_idx].pos;
            if (self.region_mesh_cache.get(pos)) |rm| {
                if (rm.vbuf.id != 0) sg.destroyBuffer(rm.vbuf);
                self.allocator.free(rm.draws);
            }
            _ = self.region_mesh_cache.remove(pos);
            _ = candidates.swapRemove(min_idx);
            count -= 1;
        }
    }
};
