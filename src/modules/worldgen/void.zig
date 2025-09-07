const Module = @import("../../registry/module.zig");
const WorldGen = @import("../../registry/worldgen.zig");
const wapi = @import("../../worldgen_api.zig");
const gs = @import("../../gs.zig");
const constants = @import("../../constants.zig");

fn hook_biomes(seed: u64, proto: *wapi.ProtoChunk, params: WorldGen.Params) !void {
    _ = seed;
    try wapi.ensureBiomesAllocated(proto);
    const count = wapi.totalVoxels(proto.section_count_y);
    const biome: u32 = if (params.biomes.len > 0) params.biomes[0] else 0;
    var i: usize = 0;
    while (i < count) : (i += 1) proto.biomes_buf.?[i] = @intCast(biome);
}

fn hook_noise(seed: u64, proto: *wapi.ProtoChunk, params: WorldGen.Params, lookup: WorldGen.BlockLookup) !void {
    _ = seed;
    _ = params;
    try wapi.ensureBlocksAllocated(proto);
    const count = wapi.totalVoxels(proto.section_count_y);
    const air: u32 = @intCast(lookup.call(lookup.ctx, "core:air") orelse 0);
    var i: usize = 0;
    while (i < count) : (i += 1) proto.blocks_buf.?[i] = @intCast(air);
    // tops remain -1 (void)
}

const WG_VOID = WorldGen.Def{
    .key = "core:void",
    .display_name = "Void",
    .biomes = hook_biomes,
    .noise = hook_noise,
};

pub const module: Module.Def = .{
    .key = "core/worldgen/void",
    .display_name = "Worldgen: Void",
    .version = .{ .major = 0, .minor = 1, .patch = 0 },
    .requires = &[_][]const u8{},
    .worldgen = &[_]WorldGen.Def{WG_VOID},
};
