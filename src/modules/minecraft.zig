const Module = @import("../registry/module.zig");
const World = @import("../registry/world.zig");
const ids = @import("ids");
const registry = @import("../registry.zig");

// Minecraft reference module (early versions baseline)
// NOTE (docs): This is an initial stub for a Minecraft reference implementation.
// It currently registers a small set of overworld blocks and the minecraft:overworld
// world using the existing core:superflat worldgen. As functionality evolves, keep
// the wiki updated and reconcile differences with docs/wiki/minecraft-technical-summary.md.
// If the code diverges from the wiki, prefer adding TODOs here and follow up in docs.
//
// Future: expand with full block/state system, items, entities, dimensions, etc.

pub const module: Module.Def = .{
    .key = "minecraft/base",
    .display_name = "Minecraft Example",
    .version = .{ .major = 0, .minor = 1, .patch = 0 },
    .requires = &[_][]const u8{},
    .init = init,
    .worldgen = &[_]Module.WorldGen.Def{},
};

/// Module initializer called by host via vox_modules
fn init(ctx: ?*anyopaque) !void {
    const regp: *registry.Registry = @ptrCast(@alignCast(ctx.?));
    _ = regp.resources;

    // Biomes
    const plains_biome: ids.BiomeId = regp.addBiome("minecraft:plains") catch 0;
    // Add a grass tint to plains (approx Minecraft plains: 0x7ABD64 sRGB). Use simple linearized approximation.
    regp.setBiomeTint(plains_biome, "grass", .{ 0.482, 0.741, 0.392 });

    // Blocks commonly present in the overworld. Keep minimal for now.
    // NOTE: For now BlockId is used where BlockStateId would be in a more
    // detailed system. This matches current Registry APIs.
    const stone_id = regp.addBlock("minecraft:stone") catch 0;

    const dirt_id: ids.BlockStateId = regp.addBlock("minecraft:dirt") catch 0;
    const grass_block_id: ids.BlockStateId = regp.addBlock("minecraft:grass_block") catch 0;
    _ = regp.addBlock("minecraft:bedrock") catch {};
    _ = regp.addBlock("minecraft:sand") catch {};
    _ = regp.addBlock("minecraft:gravel") catch {};
    _ = regp.addBlock("minecraft:oak_log") catch {};
    _ = regp.addBlock("minecraft:oak_leaves") catch {};
    _ = regp.addBlock("minecraft:oak_planks") catch {};
    _ = regp.addBlock("minecraft:cobblestone") catch {};
    _ = regp.addBlock("minecraft:glass") catch {};
    const water_id: ids.BlockStateId = regp.addBlock("minecraft:water") catch 0;

    // Register texture identifiers for uniform blocks (client/full will load .txtr by id)
    regp.resources.setUniformId("minecraft:dirt", "minecraft:dirt") catch {};
    regp.resources.setUniformId("minecraft:bedrock", "minecraft:bedrock") catch {};
    regp.resources.setUniformId("minecraft:sand", "minecraft:sand") catch {};
    regp.resources.setUniformId("minecraft:gravel", "minecraft:gravel") catch {};
    regp.resources.setUniformId("minecraft:glass", "minecraft:glass") catch {};
    regp.resources.setUniformId("minecraft:oak_planks", "minecraft:planks_oak") catch {};
    regp.resources.setUniformId("minecraft:oak_leaves", "minecraft:leaves_oak") catch {};
    regp.resources.setUniformId("minecraft:stone", "minecraft:stone") catch {};
    regp.resources.setUniformId("minecraft:cobblestone", "minecraft:cobblestone") catch {};

    // Axis-aligned tinted primary: top uses grass_top and is tinted by biome key "grass";
    // bottom and sides use dirt
    regp.resources.setAxisAlignedTintedPrimaryIds(
        "minecraft:grass_block",
        "minecraft:grass_block_top", // +Y primary
        "minecraft:dirt", // -Y bottom should be dirt
        "minecraft:grass_block_side", // sides use grass_side
        "grass",
    ) catch {};

    // Basic physics tweak: fluids shouldn't be full block collision.
    regp.setBlockFullCollisionById(@intCast(water_id), false);

    // World: minecraft:overworld using the existing superflat generator.
    // Refactor: require at least 1 section below and generate only below Y=0.
    // Use layers: grass_block, dirt, dirt, stone (repeat), bedrock at bottom.
    const bedrock_id: ids.BlockStateId = regp.addBlock("minecraft:bedrock") catch 0;

    // Sections: put 1 below and 4 above (-31 -> 127)
    regp.worlds.addWorldWithGen(
        "minecraft:overworld",
        "Overworld",
        1,
        4,
        .{ .key = "core:superflat", .blocks = &[_]ids.BlockStateId{ grass_block_id, dirt_id, dirt_id, stone_id, bedrock_id }, .biomes = &[_]ids.BiomeId{plains_biome} },
    ) catch {};
}
