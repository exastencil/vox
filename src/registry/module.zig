const std = @import("std");
const WorldGen = @import("worldgen.zig");

// Module definitions represent loadable content groups provided by the engine or mods.
// Keys use a namespaced identifier with a slash (e.g., "vox/blocks", "core/combat").
// The display_name is a human-friendly label (e.g., "Blocks for Vox Aetatum").
// Version is semantic (major.minor.patch). "requires" lists other module keys this module depends on.
pub const Version = struct {
    major: u16 = 0,
    minor: u16 = 1,
    patch: u16 = 0,
};

pub const Def = struct {
    key: []const u8, // namespace/name, e.g. "vox/blocks"
    display_name: []const u8, // e.g. "Blocks for Vox Aetatum"
    version: Version, // semantic version for compatibility checks
    requires: []const []const u8, // required module keys

    // Worldgen definitions provided by this module (static references)
    worldgen: []const WorldGen.Def = &[_]WorldGen.Def{},
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
            // free requires list and entries
            for (d.requires) |r| self.allocator.free(r);
            self.allocator.free(@constCast(d.requires));
            // worldgen slice points to static defs; do not free
        }
        self.by_key.deinit();
    }

    // Idempotent add: a repeated key is ignored.
    pub fn add(self: *Registry, key: []const u8, display_name: []const u8, version: Version, requires: []const []const u8, worldgen_defs: []const WorldGen.Def) !void {
        if (self.by_key.get(key) != null) return;
        const k = try self.allocator.dupe(u8, key);
        const dn = try self.allocator.dupe(u8, display_name);

        // own the requires list and entries
        const req = try self.allocator.alloc([]const u8, requires.len);
        for (requires, 0..) |s, i| req[i] = try self.allocator.dupe(u8, s);

        try self.by_key.put(k, .{ .key = k, .display_name = dn, .version = version, .requires = req, .worldgen = worldgen_defs });
    }

    pub fn get(self: *const Registry, key: []const u8) ?Def {
        if (self.by_key.get(key)) |d| return d;
        return null;
    }

    pub fn count(self: *const Registry) usize {
        return self.by_key.count();
    }
};

const testing = std.testing;

test "module registry: add and retrieve with version, requires, and worldgen" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer {
        const ok = gpa.deinit();
        testing.expect(ok == .ok) catch {};
    }
    var reg = Registry.init(gpa.allocator());
    defer reg.deinit();

    const empty_wg = &[_]WorldGen.Def{};
    try reg.add("vox/blocks", "Blocks for Vox Aetatum", .{ .major = 1, .minor = 0, .patch = 0 }, &[_][]const u8{}, empty_wg);
    try reg.add("core/combat", "Combat Mechanics", .{ .major = 0, .minor = 1, .patch = 0 }, &[_][]const u8{"vox/blocks"}, empty_wg);

    try testing.expectEqual(@as(usize, 2), reg.count());

    const a = reg.get("vox/blocks") orelse return error.UnexpectedNull;
    try testing.expectEqualStrings("vox/blocks", a.key);
    try testing.expectEqualStrings("Blocks for Vox Aetatum", a.display_name);
    try testing.expectEqual(@as(u16, 1), a.version.major);
    try testing.expectEqual(@as(u16, 0), a.version.minor);
    try testing.expectEqual(@as(u16, 0), a.version.patch);
    try testing.expectEqual(@as(usize, 0), a.requires.len);
    try testing.expectEqual(@as(usize, 0), a.worldgen.len);

    const b = reg.get("core/combat") orelse return error.UnexpectedNull;
    try testing.expectEqualStrings("core/combat", b.key);
    try testing.expectEqualStrings("Combat Mechanics", b.display_name);
    try testing.expectEqual(@as(u16, 0), b.version.major);
    try testing.expectEqual(@as(u16, 1), b.version.minor);
    try testing.expectEqual(@as(u16, 0), b.version.patch);
    try testing.expectEqual(@as(usize, 1), b.requires.len);
    try testing.expectEqualStrings("vox/blocks", b.requires[0]);
    try testing.expectEqual(@as(usize, 0), b.worldgen.len);
}
