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

fn unfilterScanlines(
    allocator: std.mem.Allocator,
    data: []const u8,
    height: usize,
    stride_bytes: usize, // bytes per row in the decompressed data (excluding filter byte)
    filter_bpp: usize, // bytes-per-pixel for filter purposes (ceil(bits_per_pixel/8), at least 1)
) ![]u8 {
    const expected = (stride_bytes + 1) * height;
    if (data.len != expected) return error.BadDecompressedLength;

    var out = try allocator.alloc(u8, stride_bytes * height);
    errdefer allocator.free(out);

    var row: usize = 0;
    while (row < height) : (row += 1) {
        const src_row_off = row * (stride_bytes + 1);
        const filter = data[src_row_off];
        const src = data[src_row_off + 1 .. src_row_off + 1 + stride_bytes];
        const dst = out[row * stride_bytes .. row * stride_bytes + stride_bytes];
        switch (filter) {
            0 => { // None
                @memcpy(dst, src);
            },
            1 => { // Sub
                var i: usize = 0;
                while (i < stride_bytes) : (i += 1) {
                    const a: u8 = if (i >= filter_bpp) dst[i - filter_bpp] else 0;
                    dst[i] = src[i] +% a;
                }
            },
            2 => { // Up
                const prev = if (row > 0) out[(row - 1) * stride_bytes .. (row - 1) * stride_bytes + stride_bytes] else null;
                var i: usize = 0;
                while (i < stride_bytes) : (i += 1) {
                    const b: u8 = if (prev) |p| p[i] else 0;
                    dst[i] = src[i] +% b;
                }
            },
            3 => { // Average
                const prev = if (row > 0) out[(row - 1) * stride_bytes .. (row - 1) * stride_bytes + stride_bytes] else null;
                var i: usize = 0;
                while (i < stride_bytes) : (i += 1) {
                    const a: u8 = if (i >= filter_bpp) dst[i - filter_bpp] else 0;
                    const b: u8 = if (prev) |p| p[i] else 0;
                    dst[i] = src[i] +% @as(u8, @intCast((@as(u16, a) + @as(u16, b)) / 2));
                }
            },
            4 => { // Paeth
                const prev = if (row > 0) out[(row - 1) * stride_bytes .. (row - 1) * stride_bytes + stride_bytes] else null;
                var i: usize = 0;
                while (i < stride_bytes) : (i += 1) {
                    const a: u8 = if (i >= filter_bpp) dst[i - filter_bpp] else 0;
                    const b: u8 = if (prev) |p| p[i] else 0;
                    const c: u8 = if (prev) |p| (if (i >= filter_bpp) p[i - filter_bpp] else 0) else 0;
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
    var plte: ?[]const u8 = null; // palette RGB triplets
    var trns: ?[]const u8 = null; // transparency for palette entries

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
        } else if (std.mem.eql(u8, ctype, "PLTE")) {
            plte = data;
        } else if (std.mem.eql(u8, ctype, "tRNS")) {
            trns = data;
        } else if (std.mem.eql(u8, ctype, "IEND")) {
            break;
        } else {
            // ignore ancillary chunks
        }
    }

    const h = ihdr orelse return error.MissingIHDR;
    if (h.compression != 0 or h.filter != 0 or h.interlace != 0) return error.UnsupportedPNG;

    // Supported combinations: CT=6 (RGBA, 8-bit), CT=2 (RGB, 8-bit), CT=0 (Gray, 8-bit), CT=3 (Indexed, 1/2/4/8-bit)
    switch (h.color_type) {
        6, 2, 0, 3 => {},
        else => return error.UnsupportedPNG,
    }
    if (h.bit_depth != 8 and h.color_type != 3) return error.UnsupportedPNG; // allow 1/2/4/8 for palette, restrict others to 8
    if (h.color_type == 3 and !(h.bit_depth == 1 or h.bit_depth == 2 or h.bit_depth == 4 or h.bit_depth == 8)) return error.UnsupportedPNG;

    const samples_per_pixel: usize = switch (h.color_type) {
        0 => 1, // gray
        2 => 3, // rgb
        //3 => 1, // indexed
        3 => 1,
        4 => 2, // gray+alpha (not implemented yet)
        6 => 4, // rgba
        else => 0,
    };
    const bits_per_pixel: usize = samples_per_pixel * @as(usize, h.bit_depth);
    const filter_bpp_bytes: usize = @max(1, (bits_per_pixel + 7) / 8);
    const stride_bytes: usize = (@as(usize, @intCast(h.width)) * bits_per_pixel + 7) / 8;

    // Decompress IDAT data
    const expected_decomp_len: usize = (stride_bytes + 1) * @as(usize, @intCast(h.height));

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
    const raw = try unfilterScanlines(allocator, decomp_buf, @intCast(h.height), stride_bytes, filter_bpp_bytes);
    errdefer allocator.free(raw);
    allocator.free(decomp_buf);

    const out_bpp: usize = 4;

    switch (h.color_type) {
        6 => { // RGBA8 directly
            return .{ .width = h.width, .height = h.height, .pixels = raw };
        },
        2 => { // RGB8 -> RGBA8
            const out_len: usize = @as(usize, @intCast(h.width)) * @as(usize, @intCast(h.height)) * out_bpp;
            var out = try allocator.alloc(u8, out_len);
            var src_i: usize = 0;
            var dst_i: usize = 0;
            while (src_i < raw.len) : ({
                src_i += 3;
                dst_i += 4;
            }) {
                out[dst_i + 0] = raw[src_i + 0];
                out[dst_i + 1] = raw[src_i + 1];
                out[dst_i + 2] = raw[src_i + 2];
                out[dst_i + 3] = 0xFF;
            }
            allocator.free(raw);
            return .{ .width = h.width, .height = h.height, .pixels = out };
        },
        0 => { // Gray8 -> RGBA8
            const w: usize = @intCast(h.width);
            const hgt: usize = @intCast(h.height);
            const out_len: usize = w * hgt * out_bpp;
            var out = try allocator.alloc(u8, out_len);
            var i: usize = 0;
            var j: usize = 0;
            while (i < raw.len) : ({
                i += 1;
                j += 4;
            }) {
                const v = raw[i];
                out[j + 0] = v;
                out[j + 1] = v;
                out[j + 2] = v;
                out[j + 3] = 0xFF;
            }
            allocator.free(raw);
            return .{ .width = h.width, .height = h.height, .pixels = out };
        },
        3 => { // Indexed-color -> RGBA8
            const palette = plte orelse return error.UnsupportedPNG; // required for indexed-color
            if ((palette.len % 3) != 0) return error.InvalidPNG;
            const palette_len: usize = palette.len / 3;
            const trns_slice = trns; // may be null

            const w: usize = @intCast(h.width);
            const hgt: usize = @intCast(h.height);
            const out_len: usize = w * hgt * out_bpp;
            var out = try allocator.alloc(u8, out_len);

            if (h.bit_depth == 8) {
                var si: usize = 0;
                var di: usize = 0;
                while (si < raw.len) : ({
                    si += 1;
                    di += 4;
                }) {
                    const pidx = raw[si];
                    if (pidx >= palette_len) return error.InvalidPNG;
                    const p = pidx * 3;
                    out[di + 0] = palette[p + 0];
                    out[di + 1] = palette[p + 1];
                    out[di + 2] = palette[p + 2];
                    out[di + 3] = if (trns_slice) |t| (if (pidx < t.len) t[pidx] else 0xFF) else 0xFF;
                }
            } else if (h.bit_depth == 4) {
                // two pixels per byte, high nibble first
                var di: usize = 0;
                const row_stride = (w + 1) / 2; // ceil(w/2)
                var row: usize = 0;
                while (row < hgt) : (row += 1) {
                    const src_row = raw[row * row_stride .. row * row_stride + row_stride];
                    var col: usize = 0;
                    var bi: usize = 0;
                    while (col < w) : (col += 1) {
                        const byte = src_row[bi];
                        const pidx: usize = if ((col & 1) == 0) @intCast(byte >> 4) else @intCast(byte & 0x0F);
                        if ((col & 1) == 1) bi += 1;
                        if (pidx >= palette_len) return error.InvalidPNG;
                        const p = pidx * 3;
                        out[di + 0] = palette[p + 0];
                        out[di + 1] = palette[p + 1];
                        out[di + 2] = palette[p + 2];
                        out[di + 3] = if (trns_slice) |t| (if (pidx < t.len) t[pidx] else 0xFF) else 0xFF;
                        di += 4;
                    }
                }
            } else if (h.bit_depth == 2 or h.bit_depth == 1) {
                // Generic unpacker for <= 2 bpp, MSB first
                const pixels_per_byte: usize = 8 / @as(usize, h.bit_depth);
                const row_stride = (w + pixels_per_byte - 1) / pixels_per_byte;
                var di: usize = 0;
                var row: usize = 0;
                while (row < hgt) : (row += 1) {
                    const src_row = raw[row * row_stride .. row * row_stride + row_stride];
                    var col: usize = 0;
                    var bi: usize = 0;
                    while (col < w) : (col += 1) {
                        const shift: u3 = @intCast(8 - @as(usize, h.bit_depth) - ((col % pixels_per_byte) * @as(usize, h.bit_depth)));
                        const mask: u8 = (@as(u8, 0xFF) >> @intCast(8 - @as(usize, h.bit_depth))) << shift;
                        const byte = src_row[bi];
                        const pidx: usize = @intCast((byte & mask) >> shift);
                        if (((col + 1) % pixels_per_byte) == 0) bi += 1;
                        if (pidx >= palette_len) return error.InvalidPNG;
                        const p = pidx * 3;
                        out[di + 0] = palette[p + 0];
                        out[di + 1] = palette[p + 1];
                        out[di + 2] = palette[p + 2];
                        out[di + 3] = if (trns_slice) |t| (if (pidx < t.len) t[pidx] else 0xFF) else 0xFF;
                        di += 4;
                    }
                }
            } else {
                return error.UnsupportedPNG;
            }

            allocator.free(raw);
            return .{ .width = h.width, .height = h.height, .pixels = out };
        },
        else => return error.UnsupportedPNG,
    }
}

pub fn loadFileRGBA8(allocator: std.mem.Allocator, path: []const u8) !Image {
    var file = try std.fs.cwd().openFile(path, .{});
    defer file.close();
    const bytes = try file.readToEndAlloc(allocator, 10 * 1024 * 1024); // 10MB cap
    defer allocator.free(bytes);
    return try decodeRGBA8(allocator, bytes);
}
