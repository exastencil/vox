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
    // Require at least one section below the origin; superflat generates only below Y=0
    if (proto.sections_below < 1) return error.RequiresAtLeastOneSectionBelow;

    try wapi.ensureBlocksAllocated(proto);
    const air: ids.BlockId = lookup.call(lookup.ctx, "core:air") orelse 0;

    const nb: usize = params.blocks.len;
    const last_idx: usize = if (nb > 0) nb - 1 else 0;
    const penult_idx: usize = if (nb >= 2) nb - 2 else last_idx; // if only one block provided, use it for all below

    const scy: usize = @intCast(proto.totalSections());
    const bottom_depth: usize = @as(usize, @intCast(proto.sections_below)) * constants.section_height; // number of layers below 0

    var idx: usize = 0;
    for (0..scy) |sy| {
        const y_base: i32 = (@as(i32, @intCast(sy)) - @as(i32, @intCast(proto.sections_below))) * @as(i32, @intCast(constants.section_height));
        for (0..constants.chunk_size_z) |lz| {
            for (0..constants.chunk_size_x) |lx| {
                for (0..constants.section_height) |ly| {
                    const wy: i32 = y_base + @as(i32, @intCast(ly));
                    var bid: ids.BlockId = air;
                    if (wy < 0 and nb > 0) {
                        // Distance below 0 in layers, 1..bottom_depth
                        const d: usize = @intCast(-wy);
                        if (d == bottom_depth) {
                            bid = params.blocks[last_idx]; // bottom layer: final block
                        } else {
                            // Top unique layers: one layer per block except the last two (penultimate repeats)
                            if (nb >= 2 and d <= (nb - 2)) {
                                // d starts at 1 => map to blocks[d-1]
                                bid = params.blocks[d - 1];
                            } else {
                                bid = params.blocks[penult_idx];
                            }
                        }
                    } else {
                        bid = air; // no generation at or above Y=0
                    }

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
