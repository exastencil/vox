const std = @import("std");

pub const Def = struct {
    key: []const u8, // unique world key, e.g. "vox:overworld"
    display_name: []const u8, // localizable name, e.g. "The Overworld"
    section_count_y: u16, // number of vertical sections per chunk
};

pub const Registry = struct {
    allocator: std.mem.Allocator,
    by_key: std.StringHashMap(Def),

    pub fn init(allocator: std.mem.Allocator) Registry {
        return .{ .allocator = allocator, .by_key = std.StringHashMap(Def).init(allocator) };
    }

    pub fn deinit(self: *Registry) void {
        var it = self.by_key.valueIterator();
        while (it.next()) |wd| {
            self.allocator.free(wd.key);
            self.allocator.free(wd.display_name);
        }
        self.by_key.deinit();
    }

    pub fn addWorld(self: *Registry, key: []const u8, display_name: []const u8, section_count_y: u16) !void {
        if (self.by_key.get(key) != null) return; // ignore duplicates for now
        const k = try self.allocator.dupe(u8, key);
        const dn = try self.allocator.dupe(u8, display_name);
        try self.by_key.put(k, .{ .key = k, .display_name = dn, .section_count_y = section_count_y });
    }

    pub fn get(self: *const Registry, key: []const u8) ?Def {
        if (self.by_key.get(key)) |wd| return wd;
        return null;
    }
};
