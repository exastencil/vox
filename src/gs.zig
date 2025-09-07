const std = @import("std");
const constants = @import("constants");
const ids = @import("ids");

pub const BlockPos = struct {
    x: i32,
    y: i32,
    z: i32,
};

pub const ChunkPos = struct {
    x: i32,
    z: i32,
};

pub const Weather = enum {
    clear,
    rain,
    thunder,
};

pub const Environment = struct {
    skybox_id: ids.SkyboxId,
    ambient_light: f32 = 0.0,
    time_scale: f32 = 1.0,
    gravity: f32 = 9.80665,
    // TODO: fluid rules, temperature, etc.
};

pub const Heightmaps = struct {
    // per-column height values (arrays sized 16x16)
    motion_blocking: [constants.chunk_size_x * constants.chunk_size_z]i32,
    world_surface: [constants.chunk_size_x * constants.chunk_size_z]i32,
};

pub const BlockEntityRecord = struct {
    pos: BlockPos,
    kind: ids.BlockEntityTypeId,
    // serialized block-entity payload, schema'd or NBT-like
    data: []const u8,
};

pub const EntityFlags = packed struct(u32) {
    on_ground: bool = false,
    in_fluid: bool = false,
    _reserved: u30 = 0,
};

pub const EntityRecord = struct {
    eid: u64,
    kind: ids.EntityTypeId,
    // transform
    pos: [3]f32,
    vel: [3]f32,
    yaw_pitch_roll: [3]f32,
    // The direction this entity is looking (normalized). Used to drive camera and movement.
    look_dir: [3]f32 = .{ 1, 0, 0 },
    aabb_half_extents: [3]f32,
    flags: EntityFlags = .{},
    // attributes or component data can be referenced/serialized separately
};

pub const Section = struct {
    // Constants for convenience
    pub const width = constants.chunk_size_x;
    pub const depth = constants.chunk_size_z;
    pub const height = constants.section_height;
    pub const voxel_count = width * height * depth;

    // Blocks: palette + bit-packed indices
    palette: []const ids.BlockStateId,
    blocks_indices_bits: []const u32, // bitpacked indices into the palette

    // Lighting: nibble-packed arrays (two voxels per byte), length = voxel_count/2
    skylight: []const u8,
    blocklight: []const u8,

    // Biomes: per-voxel 3D biomes via palette + bitpacked indices
    biome_palette: []const ids.BiomeId,
    biome_indices_bits: []const u32,

    // Block entities (sparse)
    block_entities: []const BlockEntityRecord,
};

// Chunk generation status follows the staged process documented in docs/wiki/worldgen-stages.md
// Note: UI/screen coordinate system remains top-left origin per WARP.md; not directly relevant here.
pub const ChunkStatus = enum(u4) {
    empty,
    structures_starts,
    structures_references,
    biomes,
    noise,
    surface,
    carvers,
    features,
    initialize_light,
    light,
    spawn,
    full,
};

pub const Chunk = struct {
    pos: ChunkPos,
    sections: []const Section, // length == world.section_count_y
    heightmaps: Heightmaps,
    entities: []const EntityRecord, // per-chunk ownership (<= constants.entity_soft_cap_per_chunk)
};

pub const Seeds = struct {
    world_seed: u64,
};

pub const World = struct {
    world_id: u128,
    name: []const u8,
    seeds: Seeds,
    section_count_y: u16,
    time_ticks: u64,
    weather_state: Weather = .clear,
    spawn_point: BlockPos,
    environment: Environment,
    chunks: []const Chunk,
};

pub const GameState = struct {
    tick_counter: u64,
    worlds: []const World,
};
