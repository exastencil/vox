const std = @import("std");

pub const Def = struct {
    name: []const u8, // namespace:name
    // Physics/collision properties
    full_block_collision: bool = true,
    // TODO: properties (geometry mask, light emission, tags, etc.)
};
