const std = @import("std");

pub const Def = struct {
    name: []const u8, // namespace:name
    // TODO: properties (geometry mask, light emission, tags, etc.)
};
