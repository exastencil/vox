const std = @import("std");

pub const Image = struct {
    width: u32,
    height: u32,
    pixels: []u8, // RGBA8
};

const PngHeader = struct {
    width: u32,
    height: u32,
    bit_depth: u8,
    color_type: u8,
    compression: u8,
    filter: u8,
    interlace: u8,
};

fn readBEu32(b: []const u8) u32 {
    return (@as(u32, b[0]) << 24) | (@as(u32, b[1]) << 16) | (@as(u32, b[2]) << 8) | @as(u32, b[3]);
}

fn absI32(x: i32) i32 {
    return if (x < 0) -x else x;
}

fn paeth(a: u8, b: u8, c: u8) u8 {
    const ai: i32 = @as(i32, a);
    const bi: i32 = @as(i32, b);
    const ci: i32 = @as(i32, c);
    const p = ai + bi - ci;
    const pa = absI32(p - ai);
    const pb = absI32(p - bi);
    const pc = absI32(p - ci);
    if (pa <= pb and pa <= pc) return a;
    if (pb <= pc) return b;
    return c;
}

fn unfilterScanlines(allocator: std.mem.Allocator, data: []const u8, width: usize, height: usize, bpp: usize) ![]u8 {
    const stride = width * bpp;
    const expected = (stride + 1) * height;
    if (data.len != expected) return error.BadDecompressedLength;

    var out = try allocator.alloc(u8, stride * height);
    errdefer allocator.free(out);

    var row: usize = 0;
    while (row < height) : (row += 1) {
        const src_row_off = row * (stride + 1);
        const filter = data[src_row_off];
        const src = data[src_row_off + 1 .. src_row_off + 1 + stride];
        const dst = out[row * stride .. row * stride + stride];
        switch (filter) {
            0 => { // None
                @memcpy(dst, src);
            },
            1 => { // Sub
                var i: usize = 0;
                while (i < stride) : (i += 1) {
                    const a: u8 = if (i >= bpp) dst[i - bpp] else 0;
                    dst[i] = src[i] +% a;
                }
            },
            2 => { // Up
                const prev = if (row > 0) out[(row - 1) * stride .. (row - 1) * stride + stride] else null;
                var i: usize = 0;
                while (i < stride) : (i += 1) {
                    const b: u8 = if (prev) |p| p[i] else 0;
                    dst[i] = src[i] +% b;
                }
            },
            3 => { // Average
                const prev = if (row > 0) out[(row - 1) * stride .. (row - 1) * stride + stride] else null;
                var i: usize = 0;
                while (i < stride) : (i += 1) {
                    const a: u8 = if (i >= bpp) dst[i - bpp] else 0;
                    const b: u8 = if (prev) |p| p[i] else 0;
                    dst[i] = src[i] +% @as(u8, @intCast((@as(u16, a) + @as(u16, b)) / 2));
                }
            },
            4 => { // Paeth
                const prev = if (row > 0) out[(row - 1) * stride .. (row - 1) * stride + stride] else null;
                var i: usize = 0;
                while (i < stride) : (i += 1) {
                    const a: u8 = if (i >= bpp) dst[i - bpp] else 0;
                    const b: u8 = if (prev) |p| p[i] else 0;
                    const c: u8 = if (prev) |p| (if (i >= bpp) p[i - bpp] else 0) else 0;
                    dst[i] = src[i] +% paeth(a, b, c);
                }
            },
            else => return error.UnsupportedFilter,
        }
    }
    return out;
}

pub fn decodeRGBA8(allocator: std.mem.Allocator, bytes: []const u8) !Image {
    if (bytes.len < 8) return error.InvalidPNG;
    const sig = bytes[0..8];
    const expected_sig = [_]u8{ 0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A };
    if (!std.mem.eql(u8, sig, &expected_sig)) return error.InvalidPNG;

    var idx: usize = 8;
    var ihdr: ?PngHeader = null;
    var idat_concat: std.ArrayListUnmanaged(u8) = .{};
    defer idat_concat.deinit(allocator);

    while (idx + 8 <= bytes.len) {
        const length = readBEu32(bytes[idx .. idx + 4]);
        const ctype = bytes[idx + 4 .. idx + 8];
        idx += 8;
        if (idx + length + 4 > bytes.len) return error.TruncatedPNG;
        const data = bytes[idx .. idx + length];
        const crc_bytes = bytes[idx + length .. idx + length + 4];
        _ = crc_bytes; // currently no CRC validation
        idx += length + 4;

        if (std.mem.eql(u8, ctype, "IHDR")) {
            if (length < 13) return error.InvalidIHDR;
            ihdr = .{
                .width = readBEu32(data[0..4]),
                .height = readBEu32(data[4..8]),
                .bit_depth = data[8],
                .color_type = data[9],
                .compression = data[10],
                .filter = data[11],
                .interlace = data[12],
            };
        } else if (std.mem.eql(u8, ctype, "IDAT")) {
            try idat_concat.appendSlice(allocator, data);
        } else if (std.mem.eql(u8, ctype, "IEND")) {
            break;
        } else {
            // ignore ancillary chunks
        }
    }

    const h = ihdr orelse return error.MissingIHDR;
    if (h.compression != 0 or h.filter != 0 or h.interlace != 0) return error.UnsupportedPNG;
    if (!(h.bit_depth == 8 and (h.color_type == 6 or h.color_type == 2))) return error.UnsupportedPNG;

    const bpp: usize = if (h.color_type == 6) 4 else 3;
    const out_bpp: usize = 4;

    // Decompress IDAT data
    const expected_decomp_len: usize = (@as(usize, h.width) * bpp + 1) * @as(usize, h.height);

    // Decompress IDAT using zlib container via flate.Decompress
    var reader: std.Io.Reader = .fixed(idat_concat.items);

    // The flate decompressor requires an internal buffer of at least
    // std.compress.flate.max_window_len (2 * history_len). Passing an empty
    // buffer causes underflow in std when rebasing the sliding window.
    const flate_buf = try allocator.alloc(u8, std.compress.flate.max_window_len);
    errdefer allocator.free(flate_buf);

    var decomp_state: std.compress.flate.Decompress = .init(&reader, .zlib, flate_buf);

    var decomp_buf = try allocator.alloc(u8, expected_decomp_len);
    errdefer allocator.free(decomp_buf);

    var written: usize = 0;
    while (written < expected_decomp_len) {
        const n = try decomp_state.reader.readSliceShort(decomp_buf[written..]);
        if (n == 0) break;
        written += n;
    }
    // decomp_state no longer needed; free the flate buffer backing its reader
    allocator.free(flate_buf);

    if (written != expected_decomp_len) return error.BadDecompressedLength;

    // Unfilter to raw scanlines
    const raw = try unfilterScanlines(allocator, decomp_buf, @intCast(h.width), @intCast(h.height), bpp);
    errdefer allocator.free(raw);
    allocator.free(decomp_buf);

    if (bpp == out_bpp) {
        return .{ .width = h.width, .height = h.height, .pixels = raw };
    }
    // Convert RGB to RGBA
    const out_len: usize = @as(usize, @intCast(h.width)) * @as(usize, @intCast(h.height)) * out_bpp;
    var out = try allocator.alloc(u8, out_len);
    var src_i: usize = 0;
    var dst_i: usize = 0;
    while (src_i < raw.len) : ({ src_i += 3; dst_i += 4; }) {
        out[dst_i + 0] = raw[src_i + 0];
        out[dst_i + 1] = raw[src_i + 1];
        out[dst_i + 2] = raw[src_i + 2];
        out[dst_i + 3] = 0xFF;
    }
    allocator.free(raw);
    return .{ .width = h.width, .height = h.height, .pixels = out };
}

pub fn loadFileRGBA8(allocator: std.mem.Allocator, path: []const u8) !Image {
    var file = try std.fs.cwd().openFile(path, .{});
    defer file.close();
    const bytes = try file.readToEndAlloc(allocator, 10 * 1024 * 1024); // 10MB cap
    defer allocator.free(bytes);
    return try decodeRGBA8(allocator, bytes);
}
