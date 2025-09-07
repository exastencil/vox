const Module = @import("../../registry/module.zig");
const WorldGen = @import("../../registry/worldgen.zig");
const BlockPos = @import("../../gs.zig").BlockPos;

const BiomeId = @import("../../ids.zig").BiomeId;
const BlockStateId = @import("../../ids.zig").BlockStateId;

const WG_SUPERFLAT = WorldGen.Def{
    .key = "core:superflat",
    .display_name = "Superflat",
    .select_biome = selectBiome,
    .select_block = selectBlock,
};

pub const module: Module.Def = .{
    .key = "core/worldgen/superflat",
    .display_name = "Worldgen: Superflat",
    .version = .{ .major = 0, .minor = 1, .patch = 0 },
    .requires = &[_][]const u8{},
    .worldgen = &[_]WorldGen.Def{WG_SUPERFLAT},
};

pub fn selectBiome(seed: u64, pos: BlockPos, params: WorldGen.Params) BiomeId {
    _ = seed;
    _ = pos;
    if (params.biomes.len > 0) return params.biomes[0];
    return 0;
}

pub fn selectBlock(seed: u64, biome: BiomeId, pos: BlockPos, params: WorldGen.Params, lookup: WorldGen.BlockLookup) BlockStateId {
    _ = seed;
    _ = biome;
    const surface = if (params.blocks.len >= 1) params.blocks[0] else (lookup.call(lookup.ctx, "core:grass") orelse 0);
    const depth = if (params.blocks.len >= 2) params.blocks[1] else (lookup.call(lookup.ctx, "core:dirt") orelse 0);
    if (pos.y < 0) return depth;
    if (pos.y <= 2) return surface;
    return lookup.call(lookup.ctx, "core:air") orelse 0;
}
