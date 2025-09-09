const Module = @import("../../registry/module.zig");
const WorldGen = @import("../../registry/worldgen.zig");
const wapi = @import("../../worldgen_api.zig");
const gs = @import("gs");
const ids = @import("ids");
const constants = @import("constants");

fn hook_biomes(seed: u64, proto: *wapi.ProtoChunk, params: WorldGen.Params) !void {
    _ = seed;
    try wapi.ensureBiomesAllocated(proto);
    const count = wapi.totalVoxels(proto.sections_below, proto.sections_above);
    const biome: ids.BiomeId = if (params.biomes.len > 0) params.biomes[0] else 0;
    var i: usize = 0;
    while (i < count) : (i += 1) proto.biomes_buf.?[i] = biome;
}

fn hook_noise(seed: u64, proto: *wapi.ProtoChunk, params: WorldGen.Params, lookup: WorldGen.BlockLookup) !void {
    _ = seed;
    try wapi.ensureBlocksAllocated(proto);
    const air: ids.BlockStateId = lookup.call(lookup.ctx, "core:air") orelse 0;
    const surface: ids.BlockStateId = if (params.blocks.len >= 1) params.blocks[0] else (lookup.call(lookup.ctx, "minecraft:grass") orelse 0);
    const depth: ids.BlockStateId = if (params.blocks.len >= 2) params.blocks[1] else (lookup.call(lookup.ctx, "minecraft:dirt") orelse 0);

    const scy: usize = @intCast(proto.totalSections());
    var idx: usize = 0;
    for (0..scy) |sy| {
        const y_base: i32 = (@as(i32, @intCast(sy)) - @as(i32, @intCast(proto.sections_below))) * @as(i32, @intCast(constants.section_height));
        for (0..constants.chunk_size_z) |lz| {
            for (0..constants.chunk_size_x) |lx| {
                for (0..constants.section_height) |ly| {
                    const wy: i32 = y_base + @as(i32, @intCast(ly));
                    const bid: ids.BlockStateId = if (wy < 0) depth else if (wy <= 2) surface else air;
                    proto.blocks_buf.?[idx] = bid;
                    if (bid != air) {
                        const col_idx = @as(usize, @intCast(lz)) * constants.chunk_size_x + @as(usize, @intCast(lx));
                        if (wy > proto.tops[col_idx]) proto.tops[col_idx] = wy;
                    }
                    idx += 1;
                }
            }
        }
    }
}

const WG_SUPERFLAT = WorldGen.Def{
    .key = "core:superflat",
    .display_name = "Superflat",
    .biomes = hook_biomes,
    .noise = hook_noise,
};

pub const module: Module.Def = .{
    .key = "core/worldgen/superflat",
    .display_name = "Worldgen: Superflat",
    .version = .{ .major = 0, .minor = 1, .patch = 0 },
    .requires = &[_][]const u8{},
    .worldgen = &[_]WorldGen.Def{WG_SUPERFLAT},
};
