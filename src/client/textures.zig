// Client textures and basic texture-array helpers
const std = @import("std");
const sokol = @import("sokol");
const sg = sokol.gfx;

pub fn loadOrFallback(allocator: std.mem.Allocator, path: []const u8) sg.Image {
    const png = @import("../png.zig");
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

pub fn makeFallbackTile(allocator: std.mem.Allocator, size: usize) !struct { w: u32, h: u32, pixels: []u8 } {
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

pub fn buildTextureArrayFromPaths(allocator: std.mem.Allocator, paths: []const []const u8) !struct {
    pixels: []u8, // concatenated slices (N * w * h * 4)
    w: u32,
    h: u32,
} {
    const png = @import("../png.zig");
    // Load all images (or fallback tiles) and record dims
    const N = paths.len;
    var loaded = try allocator.alloc(struct { w: u32, h: u32, pixels: []u8 }, N);
    errdefer {
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

    // Concatenate slices (each slice is tightly packed w*h*4)
    var all_pixels = try allocator.alloc(u8, @as(usize, N) * @as(usize, tile_w) * @as(usize, tile_h) * 4);
    @memset(all_pixels, 0);
    i = 0;
    while (i < N) : (i += 1) {
        var py: u32 = 0;
        while (py < loaded[i].h) : (py += 1) {
            const src_off: usize = @as(usize, py) * @as(usize, loaded[i].w) * 4;
            const dst_slice_base: usize = i * (@as(usize, tile_w) * @as(usize, tile_h) * 4);
            const dst_row_off: usize = dst_slice_base + (@as(usize, py) * @as(usize, tile_w) * 4);
            const copy_bytes: usize = @as(usize, loaded[i].w) * 4;
            @memcpy(all_pixels[dst_row_off .. dst_row_off + copy_bytes], loaded[i].pixels[src_off .. src_off + copy_bytes]);
        }
    }

    // Free individual tiles; keep concatenated pixels
    i = 0;
    while (i < N) : (i += 1) allocator.free(loaded[i].pixels);
    allocator.free(loaded);

    return .{ .pixels = all_pixels, .w = tile_w, .h = tile_h };
}

pub fn addUniqueId(allocator: std.mem.Allocator, list: *std.ArrayList([]const u8), s: []const u8) !void {
    // Avoid duplicates by linear scan (small N)
    for (list.items) |p| if (std.mem.eql(u8, p, s)) return;
    try list.append(allocator, s);
}

pub fn findIndexForId(id_list: []const []const u8, s: []const u8) ?usize {
    var i: usize = 0;
    while (i < id_list.len) : (i += 1) if (std.mem.eql(u8, id_list[i], s)) return i;
    return null;
}

pub fn inferTileSide(len_rgba: usize) ?u32 {
    if (len_rgba % 4 != 0) return null;
    const pixels: usize = len_rgba / 4;
    var s: u32 = 1;
    while (@as(usize, s) * @as(usize, s) < pixels) : (s += 1) {}
    if (@as(usize, s) * @as(usize, s) != pixels) return null;
    if ((s & (s - 1)) != 0) return null;
    return s;
}

pub fn loadAllTxtrSlices(_: *anyopaque, allocator: std.mem.Allocator) !struct { pixels: []u8, w: u32, h: u32, ids: [][]const u8 } {
    const texbin = @import("../texture.zig");
    // Collect all .txtr files under resources/
    var id_list = std.ArrayList([]const u8).empty;
    var slices = std.ArrayList(struct { w: u32, h: u32, pixels: []u8 }).empty;
    errdefer {
        for (slices.items) |s| if (s.pixels.len > 0) allocator.free(s.pixels);
        slices.deinit(allocator);
        for (id_list.items) |id| allocator.free(id);
        id_list.deinit(allocator);
    }

    var stack = std.ArrayList([]const u8).empty;
    defer {
        for (stack.items) |p| allocator.free(p);
        stack.deinit(allocator);
    }
    try stack.append(allocator, try allocator.dupe(u8, "resources"));

    while (stack.items.len > 0) {
        const dir_path = stack.pop().?;
        var dir = std.fs.cwd().openDir(dir_path, .{ .iterate = true }) catch {
            allocator.free(dir_path);
            continue;
        };
        defer dir.close();
        var it = dir.iterate();
        while (try it.next()) |entry| {
            if (entry.kind == .directory) {
                if (std.mem.eql(u8, entry.name, ".") or std.mem.eql(u8, entry.name, "..")) continue;
                const child = try std.fmt.allocPrint(allocator, "{s}/{s}", .{ dir_path, entry.name });
                try stack.append(allocator, child);
                continue;
            }
            if (entry.kind == .file and std.mem.endsWith(u8, entry.name, ".txtr")) {
                const file_path = try std.fmt.allocPrint(allocator, "{s}/{s}", .{ dir_path, entry.name });
                defer allocator.free(file_path);
                // Read file
                var f = try std.fs.cwd().openFile(file_path, .{});
                defer f.close();
                const stat = try f.stat();
                const buf = try allocator.alloc(u8, stat.size);
                errdefer allocator.free(buf);
                _ = try f.readAll(buf);
                // Parse
                const tex = texbin.Binary.parse(buf) catch {
                    allocator.free(buf);
                    continue;
                };
                const side = inferTileSide(tex.rgba8.len) orelse {
                    allocator.free(buf);
                    continue;
                };
                // Own id slice and rgba copy (we own the buffer already; tex references it)
                const id_bytes = try allocator.dupe(u8, tex.idSpan());
                try id_list.append(allocator, id_bytes);
                // Move pixels ownership into slices; keep buf around until texture is appended
                try slices.append(allocator, .{ .w = side, .h = side, .pixels = buf });
            }
        }
        allocator.free(dir_path);
    }

    if (slices.items.len == 0) {
        return .{ .pixels = &[_]u8{}, .w = 1, .h = 1, .ids = &[_][]const u8{} };
    }

    // Determine max tile size
    var max_w: u32 = 0;
    var max_h: u32 = 0;
    for (slices.items) |s| {
        if (s.w > max_w) max_w = s.w;
        if (s.h > max_h) max_h = s.h;
    }
    const tile_w: u32 = if (max_w == 0) 16 else max_w;
    const tile_h: u32 = if (max_h == 0) 16 else max_h;

    // Concatenate slices (each slice is tightly packed w*h*4)
    const total_bytes: usize = @as(usize, slices.items.len) * @as(usize, tile_w) * @as(usize, tile_h) * 4;
    var all_pixels = try allocator.alloc(u8, total_bytes);
    @memset(all_pixels, 0);

    var i: usize = 0;
    while (i < slices.items.len) : (i += 1) {
        const s = slices.items[i];
        var py: u32 = 0;
        while (py < s.h) : (py += 1) {
            const src_off: usize = @as(usize, py) * @as(usize, s.w) * 4;
            const dst_slice_base: usize = i * (@as(usize, tile_w) * @as(usize, tile_h) * 4);
            const dst_row_off: usize = dst_slice_base + (@as(usize, py) * @as(usize, tile_w) * 4);
            const copy_bytes: usize = @as(usize, s.w) * 4;
            @memcpy(all_pixels[dst_row_off .. dst_row_off + copy_bytes], s.pixels[src_off .. src_off + copy_bytes]);
        }
        // free slice pixel buffer now that it's copied
        allocator.free(s.pixels);
    }
    // free ArrayList backing storage for slices
    slices.deinit(allocator);

    return .{ .pixels = all_pixels, .w = tile_w, .h = tile_h, .ids = try id_list.toOwnedSlice(allocator) };
}
