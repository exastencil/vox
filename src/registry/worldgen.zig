const std = @import("std");
const gs = @import("../gs.zig");
const ids = @import("../ids.zig");

pub const Params = struct {
    blocks: []const ids.BlockStateId = &[_]ids.BlockStateId{},
    biomes: []const ids.BiomeId = &[_]ids.BiomeId{},
};

pub const SelectBiomeFn = *const fn (seed: u64, pos: gs.BlockPos, params: Params) ids.BiomeId;

pub const BlockLookup = struct {
    ctx: ?*anyopaque,
    call: *const fn (ctx: ?*anyopaque, name: []const u8) ?ids.BlockStateId,
};

pub const SelectBlockFn = *const fn (seed: u64, biome: ids.BiomeId, pos: gs.BlockPos, params: Params, lookup: BlockLookup) ids.BlockStateId;

pub const Def = struct {
    key: []const u8,
    display_name: []const u8,
    select_biome: SelectBiomeFn,
    select_block: SelectBlockFn,
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

    pub fn add(self: *Registry, key: []const u8, display_name: []const u8, select_biome: SelectBiomeFn, select_block: SelectBlockFn) !void {
        if (self.by_key.get(key) != null) return;
        const k = try self.allocator.dupe(u8, key);
        const dn = try self.allocator.dupe(u8, display_name);
        try self.by_key.put(k, .{ .key = k, .display_name = dn, .select_biome = select_biome, .select_block = select_block });
    }

    pub fn get(self: *const Registry, key: []const u8) ?Def {
        if (self.by_key.get(key)) |d| return d;
        return null;
    }
};

// Built-in selectors
pub fn selectVoid(seed: u64, pos: gs.BlockPos, params: Params) ids.BiomeId {
    _ = seed;
    _ = pos;
    if (params.biomes.len > 0) return params.biomes[0];
    return 0;
}

pub fn selectSuperflat(seed: u64, pos: gs.BlockPos, params: Params) ids.BiomeId {
    _ = seed;
    _ = pos;
    if (params.biomes.len > 0) return params.biomes[0];
    return 0;
}

pub fn selectBlockVoid(seed: u64, biome: ids.BiomeId, pos: gs.BlockPos, params: Params, lookup: BlockLookup) ids.BlockStateId {
    _ = seed;
    _ = biome;
    _ = pos;
    _ = params;
    return lookup.call(lookup.ctx, "core:air") orelse 0;
}

pub fn selectBlockSuperflat(seed: u64, biome: ids.BiomeId, pos: gs.BlockPos, params: Params, lookup: BlockLookup) ids.BlockStateId {
    _ = seed;
    _ = biome;
    const surface = if (params.blocks.len >= 1) params.blocks[0] else (lookup.call(lookup.ctx, "core:grass") orelse 0);
    const depth = if (params.blocks.len >= 2) params.blocks[1] else (lookup.call(lookup.ctx, "core:stone") orelse 0);
    if (pos.y < 0) return depth;
    if (pos.y <= 2) return surface;
    return lookup.call(lookup.ctx, "core:air") orelse 0;
}
