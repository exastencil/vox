# Texture binary format (comptime-parseable)

This document describes the binary format used for serializing textures so they can be embedded and parsed at compile time to build a texture atlas.

File extension suggestion: .txtr (but any name works).

Layout (all integers little-endian):
- magic: 4 bytes = "TXTR"
- id_len: u32 = number of bytes for the identifier INCLUDING the trailing null terminator
- pix_len: u32 = number of bytes of RGBA8 data (must be a multiple of 4)
- id: id_len bytes (last byte must be 0)
- pixels: pix_len bytes of raw RGBA8 data

Notes
- The identifier is a null-terminated string. Interior null bytes are not allowed.
- Pixel data is raw RGBA8 (R,G,B,A), with no width/height stored — the atlas builder determines placement/dimensions from context.

Example: encoding and parsing in Zig

```zig path=null start=null
const std = @import("std");
const texture = @import("/Users/exa/Code/github.com/exastencil/vox/src/texture.zig");

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const alloc = gpa.allocator();

    const id = "grass"; // no trailing null; encoder appends it
    const pixels = [_]u8{ 0,0,0,255, 255,0,0,255 };

    // Create a .txtr payload producers could write to disk
    const bin = try texture.Binary.encode(alloc, id, &pixels);
    defer alloc.free(bin);

    // Compile-time parse example (using an embedded file)
    const embedded_bytes = @embedFile("assets/textures/grass.txtr");
    const tex = comptime texture.Binary.parse(embedded_bytes) catch @compileError("Invalid texture file: grass.txtr");
    _ = tex; // tex.id is [:0]const u8, tex.rgba8 is []const u8
}
```

Atlas build at compile time
- Content creators can generate .txtr files offline.
- In code, list the files and parse them at comptime using @embedFile and Binary.parse. Use your preferred packing algorithm to build the atlas at compile time.
