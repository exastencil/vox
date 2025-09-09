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

    // Blocks commonly present in the overworld. Keep minimal for now.
    // NOTE: For now BlockId is used where BlockStateId would be in a more
    // detailed system. This matches current Registry APIs.
    const stone_id = regp.addBlock("minecraft:stone") catch 0;

    const dirt_id: ids.BlockStateId = regp.addBlock("minecraft:dirt") catch 0;
    _ = regp.addBlock("minecraft:grass_block") catch {};
    _ = regp.addBlock("minecraft:bedrock") catch {};
    _ = regp.addBlock("minecraft:sand") catch {};
    _ = regp.addBlock("minecraft:gravel") catch {};
    _ = regp.addBlock("minecraft:oak_log") catch {};
    _ = regp.addBlock("minecraft:oak_leaves") catch {};
    _ = regp.addBlock("minecraft:oak_planks") catch {};
    _ = regp.addBlock("minecraft:cobblestone") catch {};
    _ = regp.addBlock("minecraft:glass") catch {};
    const water_id: ids.BlockStateId = regp.addBlock("minecraft:water") catch 0;

    // Register texture paths for uniform blocks (client/full will load these)
    regp.resources.setUniformPath("minecraft:dirt", "resources/textures/blocks/minecraft/dirt.png") catch {};
    regp.resources.setUniformPath("minecraft:bedrock", "resources/textures/blocks/minecraft/bedrock.png") catch {};
    regp.resources.setUniformPath("minecraft:sand", "resources/textures/blocks/minecraft/sand.png") catch {};
    regp.resources.setUniformPath("minecraft:gravel", "resources/textures/blocks/minecraft/gravel.png") catch {};
    regp.resources.setUniformPath("minecraft:glass", "resources/textures/blocks/minecraft/glass.png") catch {};
    regp.resources.setUniformPath("minecraft:oak_planks", "resources/textures/blocks/minecraft/planks_oak.png") catch {};
    regp.resources.setUniformPath("minecraft:oak_leaves", "resources/textures/blocks/minecraft/leaves_oak.png") catch {};

    // Keep stone and grass mappings for current renderer expectations
    regp.resources.setUniformPath("minecraft:stone", "resources/textures/blocks/minecraft/stone.png") catch {};
    regp.resources.setUniformPath("minecraft:cobblestone", "resources/textures/blocks/minecraft/cobblestone.png") catch {};
    regp.resources.setFacingPaths(
        "minecraft:grass_block",
        "resources/textures/blocks/minecraft/grass_top.png",
        "resources/textures/blocks/minecraft/dirt.png",
    ) catch {};

    // Basic physics tweak: fluids shouldn't be full block collision.
    regp.setBlockFullCollisionById(@intCast(water_id), false);

    // World: minecraft:overworld using the existing superflat generator.
    // Sections: mirror the default of 4 for now (4*32=128 height).
    regp.worlds.addWorldWithGen(
        "minecraft:overworld",
        "Overworld",
        0,
        4,
        .{ .key = "core:superflat", .blocks = &[_]ids.BlockStateId{ dirt_id, stone_id }, .biomes = &[_]ids.BiomeId{plains_biome} },
    ) catch {};
}
