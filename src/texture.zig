const std = @import("std");

/// Texture represents a texture asset with a string identifier and raw RGBA8 pixel data.
///
/// - id: null-terminated identifier used for lookup in registries/atlases.
/// - rgba8: raw pixel bytes in RGBA8 format (length must be a multiple of 4).
pub const Texture = struct {
    id: [:0]const u8,
    rgba8: []const u8,

    pub const Error = error{
        InvalidPixels,
        InvalidId,
    };

    /// Validate basic invariants.
    pub fn validate(self: *const Texture) Error!void {
        if (self.rgba8.len % 4 != 0) return error.InvalidPixels;
        // Ensure no interior nulls in the id (id excludes the terminating sentinel)
        if (self.id.len == 0) return error.InvalidId;
        if (std.mem.indexOfScalar(u8, self.id, 0) != null) return error.InvalidId;
    }

    /// Convenience: ID as a non-null-terminated slice (excludes the trailing null byte).
    pub fn idSpan(self: *const Texture) []const u8 {
        // id is a sentinel-terminated slice; len excludes the sentinel
        return self.id[0..self.id.len];
    }
};

/// Binary serialization format for Texture that supports compile-time parsing.
///
/// Layout (all integers little-endian):
/// - magic:   4 bytes = "TXTR"
/// - id_len:  u32     = number of bytes INCLUDING the trailing null terminator
/// - pix_len: u32     = number of bytes of RGBA8 data (must be multiple of 4)
/// - id:      id_len bytes (last byte must be 0)
/// - pixels:  pix_len bytes of raw RGBA8 data
///
/// This format is designed so it can be parsed at comptime from an @embedFile'd slice.
pub const Binary = struct {
    pub const magic: [4]u8 = .{ 'T', 'X', 'T', 'R' };

    pub const Error = error{
        InvalidMagic,
        Truncated,
        MissingNullTerminator,
        InvalidPixels,
        InvalidId,
        OutOfMemory,
    };

    fn readU32le(bytes: []const u8, off: usize) Error!u32 {
        if (off + 4 > bytes.len) return error.Truncated;
        return (@as(u32, bytes[off + 0])) |
            (@as(u32, bytes[off + 1]) << 8) |
            (@as(u32, bytes[off + 2]) << 16) |
            (@as(u32, bytes[off + 3]) << 24);
    }

    fn writeU32le(dst: []u8, off: usize, v: u32) void {
        dst[off + 0] = @as(u8, @truncate(v >> 0));
        dst[off + 1] = @as(u8, @truncate(v >> 8));
        dst[off + 2] = @as(u8, @truncate(v >> 16));
        dst[off + 3] = @as(u8, @truncate(v >> 24));
    }

    /// Parse a Texture view from bytes. Slices in the returned Texture reference the input memory.
    /// Safe for both runtime and comptime usage.
    pub fn parse(bytes: []const u8) Error!Texture {
        if (bytes.len < 12) return error.Truncated;
        if (!std.mem.eql(u8, bytes[0..4], &magic)) return error.InvalidMagic;

        const id_len = try readU32le(bytes, 4);
        const pix_len = try readU32le(bytes, 8);

        var cursor: usize = 12;

        if (id_len == 0) return error.MissingNullTerminator;
        if (cursor + id_len > bytes.len) return error.Truncated;

        const id_with_null = bytes[cursor .. cursor + id_len];
        if (id_with_null[id_with_null.len - 1] != 0) return error.MissingNullTerminator;
        // Create a sentinel-terminated slice view of the id+null region.
        // Slice to exclude the trailing null from the length; the sentinel will reside at index len
        const id_cstr: [:0]const u8 = id_with_null[0 .. id_with_null.len - 1 :0];
        cursor += id_len;

        if (cursor + pix_len > bytes.len) return error.Truncated;
        const rgba8 = bytes[cursor .. cursor + pix_len];
        if (rgba8.len % 4 != 0) return error.InvalidPixels;

        return Texture{ .id = id_cstr, .rgba8 = rgba8 };
    }

    /// Encode a Texture to bytes. The returned buffer is newly allocated and owned by the caller.
    /// The input `id` must NOT contain an interior null; a trailing null is added during encoding.
    pub fn encode(allocator: std.mem.Allocator, id: []const u8, rgba8: []const u8) Error![]u8 {
        if (std.mem.indexOfScalar(u8, id, 0) != null) return error.InvalidId;
        if (rgba8.len % 4 != 0) return error.InvalidPixels;

        const id_len_plus_null: u32 = @intCast(id.len + 1);
        const pix_len_u32: u32 = @intCast(rgba8.len);
        const total_len: usize = 4 + 4 + 4 + id_len_plus_null + pix_len_u32;

        var buf = try allocator.alloc(u8, total_len);
        // magic
        std.mem.copyForwards(u8, buf[0..4], &magic);
        // header
        writeU32le(buf, 4, id_len_plus_null);
        writeU32le(buf, 8, pix_len_u32);
        // id
        std.mem.copyForwards(u8, buf[12 .. 12 + id.len], id);
        buf[12 + id.len] = 0;
        // pixels
        const pix_off = 12 + id_len_plus_null;
        std.mem.copyForwards(u8, buf[pix_off .. pix_off + rgba8.len], rgba8);
        return buf;
    }
};

// -----------------
// Tests
// -----------------

const testing = std.testing;

test "binary roundtrip: encode then parse" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const alloc = gpa.allocator();

    const id = "grass";
    const pixels = [_]u8{ 0, 0, 0, 255, 255, 0, 0, 255 };

    const bin = try Binary.encode(alloc, id, &pixels);
    defer alloc.free(bin);

    const tex = try Binary.parse(bin);
    try testing.expectEqualStrings(id, tex.idSpan());
    try testing.expectEqual(@as(usize, pixels.len), tex.rgba8.len);
}

test "parse comptime literal via @embedFile-like bytes" {
    // id = "grass\x00" (6 bytes); pix_len = 8 bytes; pixels = { 0,0,0,255, 255,0,0,255 }
    const lit = [_]u8{
        'T', 'X', 'T', 'R',
        0x06, 0x00, 0x00, 0x00, // id_len = 6
        0x08, 0x00, 0x00, 0x00, // pix_len = 8
        'g', 'r', 'a', 's', 's', 0x00, // id + null
        0, 0, 0, 255, 255, 0, 0, 255, // pixels
    };

    const tex = comptime Binary.parse(&lit) catch unreachable;
    try testing.expectEqual(@as(usize, 8), tex.rgba8.len);
    try testing.expectEqual(@as(u8, 'g'), tex.id[0]);
}
