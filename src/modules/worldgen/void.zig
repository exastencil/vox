const Module = @import("../../registry/module.zig");
const WorldGen = @import("../../registry/worldgen.zig");
const BlockPos = @import("../../gs.zig").BlockPos;

const BiomeId = @import("../../ids.zig").BiomeId;
const BlockStateId = @import("../../ids.zig").BlockStateId;

const WG_VOID = WorldGen.Def{
    .key = "core:void",
    .display_name = "Void",
    .select_biome = selectBiome,
    .select_block = selectBlock,
};

pub const module: Module.Def = .{
    .key = "core/worldgen/void",
    .display_name = "Worldgen: Void",
    .version = .{ .major = 0, .minor = 1, .patch = 0 },
    .requires = &[_][]const u8{},
    .worldgen = &[_]WorldGen.Def{WG_VOID},
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
    _ = pos;
    _ = params;
    return lookup.call(lookup.ctx, "core:air") orelse 0;
}
