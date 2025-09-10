const std = @import("std");

pub const Def = struct {
    name: []const u8, // namespace:name
    // Optional biome tint map: key -> RGB color in linear [0..1]
    // Mods can define arbitrary tint keys (e.g., "grass", "foliage", "water").
    tints: std.StringHashMap([3]f32),
    // TODO: climate params, colors, generation hints
};
