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
    const stone_id: ids.BlockId = regp.addBlock("minecraft:stone") catch 0;
    const dirt_id: ids.BlockId = regp.addBlock("minecraft:dirt") catch 0;
    const grass_block_id: ids.BlockId = regp.addBlock("minecraft:grass_block") catch 0;
    const bedrock_id: ids.BlockId = regp.addBlock("minecraft:bedrock") catch 0;
    const sand_id: ids.BlockId = regp.addBlock("minecraft:sand") catch 0;
    const gravel_id: ids.BlockId = regp.addBlock("minecraft:gravel") catch 0;
    const oak_log_id: ids.BlockId = regp.addBlock("minecraft:oak_log") catch 0;
    const oak_leaves_id: ids.BlockId = regp.addBlock("minecraft:oak_leaves") catch 0;
    const oak_planks_id: ids.BlockId = regp.addBlock("minecraft:oak_planks") catch 0;
    const cobblestone_id: ids.BlockId = regp.addBlock("minecraft:cobblestone") catch 0;
    const glass_id: ids.BlockId = regp.addBlock("minecraft:glass") catch 0;
    const water_id: ids.BlockId = regp.addBlock("minecraft:water") catch 0;

    // Populate Block.Def visuals using FaceConfigs with texture IDs that match .txtr IDs
    const B = registry.Block;
    // stone
    {
        const new_def = try B.Def.uniformTexture(regp.allocator, "minecraft:stone", "minecraft:stone", null, true);
        regp.blocks.items[@intCast(stone_id)].deinit(regp.allocator, true);
        regp.blocks.items[@intCast(stone_id)] = new_def;
    }
    // dirt
    {
        const new_def = try B.Def.uniformTexture(regp.allocator, "minecraft:dirt", "minecraft:dirt", null, true);
        regp.blocks.items[@intCast(dirt_id)].deinit(regp.allocator, true);
        regp.blocks.items[@intCast(dirt_id)] = new_def;
    }
    // bedrock
    {
        const new_def = try B.Def.uniformTexture(regp.allocator, "minecraft:bedrock", "minecraft:bedrock", null, true);
        regp.blocks.items[@intCast(bedrock_id)].deinit(regp.allocator, true);
        regp.blocks.items[@intCast(bedrock_id)] = new_def;
    }
    // sand
    {
        const new_def = try B.Def.uniformTexture(regp.allocator, "minecraft:sand", "minecraft:sand", null, true);
        regp.blocks.items[@intCast(sand_id)].deinit(regp.allocator, true);
        regp.blocks.items[@intCast(sand_id)] = new_def;
    }
    // gravel
    {
        const new_def = try B.Def.uniformTexture(regp.allocator, "minecraft:gravel", "minecraft:gravel", null, true);
        regp.blocks.items[@intCast(gravel_id)].deinit(regp.allocator, true);
        regp.blocks.items[@intCast(gravel_id)] = new_def;
    }
    // cobblestone
    {
        const new_def = try B.Def.uniformTexture(regp.allocator, "minecraft:cobblestone", "minecraft:cobblestone", null, true);
        regp.blocks.items[@intCast(cobblestone_id)].deinit(regp.allocator, true);
        regp.blocks.items[@intCast(cobblestone_id)] = new_def;
    }
    // glass
    {
        const new_def = try B.Def.uniformTexture(regp.allocator, "minecraft:glass", "minecraft:glass", null, true);
        regp.blocks.items[@intCast(glass_id)].deinit(regp.allocator, true);
        regp.blocks.items[@intCast(glass_id)] = new_def;
    }
    // oak planks
    {
        const new_def = try B.Def.uniformTexture(regp.allocator, "minecraft:oak_planks", "minecraft:planks_oak", null, true);
        regp.blocks.items[@intCast(oak_planks_id)].deinit(regp.allocator, true);
        regp.blocks.items[@intCast(oak_planks_id)] = new_def;
    }
    // oak leaves (no tint for now)
    {
        const new_def = try B.Def.uniformTexture(regp.allocator, "minecraft:oak_leaves", "minecraft:leaves_oak", null, true);
        regp.blocks.items[@intCast(oak_leaves_id)].deinit(regp.allocator, true);
        regp.blocks.items[@intCast(oak_leaves_id)] = new_def;
    }
    // oak log: top/bottom log_oak_top, sides log_oak
    {
        const new_def = try B.Def.topBottomSides(regp.allocator, "minecraft:oak_log", "minecraft:log_oak_top", "minecraft:log_oak", null, null, true);
        regp.blocks.items[@intCast(oak_log_id)].deinit(regp.allocator, true);
        regp.blocks.items[@intCast(oak_log_id)] = new_def;
    }
    // grass block: top tinted by biome key "grass", bottom dirt, sides grass_block_side
    {
        const new_def = try B.Def.topBottomSides(regp.allocator, "minecraft:grass_block", "minecraft:grass_block_top", "minecraft:grass_block_side", "minecraft:dirt", "grass", true);
        regp.blocks.items[@intCast(grass_block_id)].deinit(regp.allocator, true);
        regp.blocks.items[@intCast(grass_block_id)] = new_def;
    }
    // water (non-colliding); simple uniform for now
    {
        const new_def = try B.Def.uniformTexture(regp.allocator, "minecraft:water", "minecraft:water", null, false);
        regp.blocks.items[@intCast(water_id)].deinit(regp.allocator, true);
        regp.blocks.items[@intCast(water_id)] = new_def;
    }

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

    // Sections: put 1 below and 4 above (-31 -> 127)
    regp.worlds.addWorldWithGen("minecraft:overworld", "Overworld", 1, 4, .{ .key = "core:superflat", .blocks = &[_]ids.BlockId{ grass_block_id, dirt_id, dirt_id, stone_id, bedrock_id }, .biomes = &[_]ids.BiomeId{plains_biome} }) catch {};
}
