const std = @import("std");

// UV mapping helpers for side faces (±X, ±Z) used by the mesher.
// Invariants:
// - All mappings are returned in the UV order corresponding to the geometry
//   vertex order (v0, v3, v2, v1) we emit for CCW outward winding with
//   cull BACK, except for +Z where client.zig purposely emits (v0, v1, v2, v3)
//   and remaps UVs accordingly at the call site.
// - Face indices: 0:-X, 1:+X, 4:-Z, 5:+Z
// - w is the tile count along the horizontal axis of the face, h along vertical.
//   We return UVs in "tile space" such that U ranges across w and V across h.
// - These mappings have been verified in-game against grass side textures for
//   orientation and tiling (no mirroring, no stretching).
pub inline fn sideTileUVs(face: u3, w: usize, h: usize) [4][2]f32 {
    const fw: f32 = @floatFromInt(w);
    const fh: f32 = @floatFromInt(h);
    switch (face) {
        // -X
        0 => return .{ .{ 0, fw }, .{ fh, fw }, .{ fh, 0 }, .{ 0, 0 } },
        // +X
        1 => return .{ .{ fh, fw }, .{ fh, 0 }, .{ 0, 0 }, .{ 0, fw } },
        // -Z (rotated 180° relative to +Z based on visual validation)
        4 => return .{ .{ fw, fh }, .{ fw, 0 }, .{ 0, 0 }, .{ 0, fh } },
        // +Z (vertically flipped so grass edge appears at the top, left edge left)
        5 => return .{ .{ 0, fh }, .{ 0, 0 }, .{ fw, 0 }, .{ fw, fh } },
        else => return .{ .{ 0, 0 }, .{ 0, 0 }, .{ 0, 0 }, .{ 0, 0 } },
    }
}

// Basic tests to freeze expected outputs. These run with `zig test src/uvmap.zig`.
test "sideTileUVs returns stable mappings for ±X, ±Z (w=2,h=3)" {
    const w: usize = 2;
    const h: usize = 3;
    const eql = std.testing.expectEqual;

    const nx = sideTileUVs(0, w, h);
    try eql(@as([2]f32, .{ 0, 2 }), nx[0]);
    try eql(@as([2]f32, .{ 3, 2 }), nx[1]);
    try eql(@as([2]f32, .{ 3, 0 }), nx[2]);
    try eql(@as([2]f32, .{ 0, 0 }), nx[3]);

    const px = sideTileUVs(1, w, h);
    try eql(@as([2]f32, .{ 3, 2 }), px[0]);
    try eql(@as([2]f32, .{ 3, 0 }), px[1]);
    try eql(@as([2]f32, .{ 0, 0 }), px[2]);
    try eql(@as([2]f32, .{ 0, 2 }), px[3]);

    const nz = sideTileUVs(4, w, h);
    try eql(@as([2]f32, .{ 2, 3 }), nz[0]);
    try eql(@as([2]f32, .{ 2, 0 }), nz[1]);
    try eql(@as([2]f32, .{ 0, 0 }), nz[2]);
    try eql(@as([2]f32, .{ 0, 3 }), nz[3]);

    const pz = sideTileUVs(5, w, h);
    try eql(@as([2]f32, .{ 0, 3 }), pz[0]);
    try eql(@as([2]f32, .{ 0, 0 }), pz[1]);
    try eql(@as([2]f32, .{ 2, 0 }), pz[2]);
    try eql(@as([2]f32, .{ 2, 3 }), pz[3]);
}
