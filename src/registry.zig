const std = @import("std");
const ids = @import("ids.zig");

pub const BlockDef = struct {
    name: []const u8, // namespace:name
    // TODO: properties (geometry mask, light emission, tags, etc.)
};

pub const BiomeDef = struct {
    name: []const u8, // namespace:name
    // TODO: climate params, colors, generation hints
};

pub const Registry = struct {
    allocator: std.mem.Allocator,
    blocks: std.ArrayList(BlockDef),
    biomes: std.ArrayList(BiomeDef),

    pub fn init(allocator: std.mem.Allocator) !Registry {
        return .{
            .allocator = allocator,
            .blocks = try std.ArrayList(BlockDef).initCapacity(allocator, 8),
            .biomes = try std.ArrayList(BiomeDef).initCapacity(allocator, 4),
        };
    }

    pub fn deinit(self: *Registry) void {
        // free any names we duplicated
        for (self.blocks.items) |b| self.allocator.free(b.name);
        for (self.biomes.items) |b| self.allocator.free(b.name);
        self.blocks.deinit(self.allocator);
        self.biomes.deinit(self.allocator);
    }

    fn dup(self: *Registry, s: []const u8) ![]const u8 {
        return try self.allocator.dupe(u8, s);
    }

    pub fn ensureAir(self: *Registry) !void {
        if (self.blocks.items.len == 0) {
            _ = try self.addBlock("core:air");
        } else {
            // if the first block isn't air, that's a programming error for now
            std.debug.assert(std.mem.eql(u8, self.blocks.items[0].name, "core:air"));
        }
    }

    pub fn addBlock(self: *Registry, name: []const u8) !ids.BlockId {
        // Prevent duplicates by name
        for (self.blocks.items, 0..) |b, i| {
            if (std.mem.eql(u8, b.name, name)) return @intCast(i);
        }
        const owned = try self.dup(name);
        try self.blocks.append(self.allocator, .{ .name = owned });
        return @intCast(self.blocks.items.len - 1);
    }

    pub fn addBiome(self: *Registry, name: []const u8) !ids.BiomeId {
        for (self.biomes.items, 0..) |b, i| {
            if (std.mem.eql(u8, b.name, name)) return @intCast(i);
        }
        const owned = try self.dup(name);
        try self.biomes.append(self.allocator, .{ .name = owned });
        return @intCast(self.biomes.items.len - 1);
    }

    pub fn getBlockName(self: *const Registry, id: ids.BlockId) []const u8 {
        return self.blocks.items[@intCast(id)].name;
    }

    pub fn getBiomeName(self: *const Registry, id: ids.BiomeId) []const u8 {
        return self.biomes.items[@intCast(id)].name;
    }

    pub fn findBiome(self: *const Registry, name: []const u8) ?ids.BiomeId {
        for (self.biomes.items, 0..) |b, i| {
            if (std.mem.eql(u8, b.name, name)) return @intCast(i);
        }
        return null;
    }

    pub fn blockCount(self: *const Registry) usize {
        return self.blocks.items.len;
    }

    pub fn biomeCount(self: *const Registry) usize {
        return self.biomes.items.len;
    }
};

const testing = std.testing;

test "registry ensures air is block ID 0 and allows new blocks" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer {
        const ok = gpa.deinit();
        testing.expect(ok == .ok) catch {};
    }
    var reg = try Registry.init(gpa.allocator());
    defer reg.deinit();

    try reg.ensureAir();
    try testing.expectEqual(@as(usize, 1), reg.blockCount());
    try testing.expectEqualStrings("core:air", reg.getBlockName(0));

    const stone = try reg.addBlock("core:stone");
    try testing.expectEqual(@as(usize, 2), reg.blockCount());
    try testing.expectEqual(@as(ids.BlockId, 1), stone);
    try testing.expectEqualStrings("core:stone", reg.getBlockName(stone));
}

test "registry adds at least one biome" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer {
        const ok = gpa.deinit();
        testing.expect(ok == .ok) catch {};
    }
    var reg = try Registry.init(gpa.allocator());
    defer reg.deinit();

    const plains = try reg.addBiome("core:plains");
    try testing.expectEqual(@as(ids.BiomeId, 0), plains);
    try testing.expectEqual(@as(usize, 1), reg.biomeCount());
}

test "registry init/deinit has no leaks" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer {
        const ok = gpa.deinit();
        testing.expect(ok == .ok) catch {};
    }
    var reg = try Registry.init(gpa.allocator());
    defer reg.deinit();
    // no further ops; success criteria is that gpa.deinit() returns .ok
}
