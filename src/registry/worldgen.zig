const std = @import("std");
const gs = @import("../gs.zig");

pub const SelectBiomeFn = *const fn (seed: u64, pos: gs.BlockPos) []const u8;

pub const Def = struct {
    key: []const u8,
    display_name: []const u8,
    select_biome: SelectBiomeFn,
};

pub const Registry = struct {
    allocator: std.mem.Allocator,
    by_key: std.StringHashMap(Def),

    pub fn init(allocator: std.mem.Allocator) Registry {
        return .{ .allocator = allocator, .by_key = std.StringHashMap(Def).init(allocator) };
    }

    pub fn deinit(self: *Registry) void {
        var it = self.by_key.valueIterator();
        while (it.next()) |d| {
            self.allocator.free(d.key);
            self.allocator.free(d.display_name);
        }
        self.by_key.deinit();
    }

    pub fn add(self: *Registry, key: []const u8, display_name: []const u8, select_biome: SelectBiomeFn) !void {
        if (self.by_key.get(key) != null) return;
        const k = try self.allocator.dupe(u8, key);
        const dn = try self.allocator.dupe(u8, display_name);
        try self.by_key.put(k, .{ .key = k, .display_name = dn, .select_biome = select_biome });
    }

    pub fn get(self: *const Registry, key: []const u8) ?Def {
        if (self.by_key.get(key)) |d| return d;
        return null;
    }
};

// Built-in selectors
pub fn selectVoid(seed: u64, pos: gs.BlockPos) []const u8 {
    _ = seed;
    _ = pos;
    return "core:void";
}

pub fn selectSuperflat(seed: u64, pos: gs.BlockPos) []const u8 {
    _ = seed;
    _ = pos;
    return "core:plains";
}
