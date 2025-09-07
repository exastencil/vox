const std = @import("std");
const ids = @import("ids");

pub const GenSpec = struct {
    key: []const u8,
    blocks: []const ids.BlockStateId = &[_]ids.BlockStateId{},
    biomes: []const ids.BiomeId = &[_]ids.BiomeId{},
};

pub const Def = struct {
    key: []const u8, // unique world key, e.g. "vox:overworld"
    display_name: []const u8, // localizable name, e.g. "The Overworld"
    section_count_y: u16, // number of vertical sections per chunk
    gen_key: []const u8,
    gen_blocks: []const ids.BlockStateId,
    gen_biomes: []const ids.BiomeId,
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
            self.allocator.free(wd.gen_key);
            self.allocator.free(@constCast(wd.gen_blocks));
            self.allocator.free(@constCast(wd.gen_biomes));
        }
        self.by_key.deinit();
    }

    pub fn addWorld(self: *Registry, key: []const u8, display_name: []const u8, section_count_y: u16) !void {
        // default worldgen: core:void with no params
        const spec = GenSpec{ .key = "core:void" };
        try self.addWorldWithGen(key, display_name, section_count_y, spec);
    }

    pub fn addWorldWithGen(self: *Registry, key: []const u8, display_name: []const u8, section_count_y: u16, gen: GenSpec) !void {
        if (self.by_key.get(key) != null) return; // ignore duplicates for now
        const k = try self.allocator.dupe(u8, key);
        const dn = try self.allocator.dupe(u8, display_name);
        const gk = try self.allocator.dupe(u8, gen.key);
        const blocks = try self.allocator.alloc(ids.BlockStateId, gen.blocks.len);
        @memcpy(blocks, gen.blocks);
        const biomes = try self.allocator.alloc(ids.BiomeId, gen.biomes.len);
        @memcpy(biomes, gen.biomes);
        try self.by_key.put(k, .{
            .key = k,
            .display_name = dn,
            .section_count_y = section_count_y,
            .gen_key = gk,
            .gen_blocks = blocks,
            .gen_biomes = biomes,
        });
    }

    pub fn get(self: *const Registry, key: []const u8) ?Def {
        if (self.by_key.get(key)) |wd| return wd;
        return null;
    }
};
