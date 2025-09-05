# Game State (GS) Model

This document defines the canonical data model the Simulation (SIM) mutates and the Client (CLI) reads via snapshots. It establishes names and structure to keep mods and engine code consistent.

Scope: Pure state. No simulation logic, threading, or networking here.

## Top-Level Concepts and Names

- GS (Game State): The aggregate of all worlds and global metadata.
- World: A self-contained dimension (akin to Minecraft dimensions) with its own chunks, environment, and config.
- Chunk: A horizontal 2D grid cell at (chunk_x, chunk_z) containing vertical Sections.
- Section: A vertical slice of a Chunk with fixed height in blocks, storing blocks, lighting, and per-section metadata.
- Block: A voxel cell identified by a BlockState (registry ID + properties). May be accompanied by a BlockEntity.
- BlockState: A concrete state of a Block type (e.g., oak_log axis=y, slab top/bottom/waterlogged, power=7).
- BlockEntity: Extra per-block data and optional per-tick logic hooks (e.g., containers, signs). Stored sparsely.
- Entity: Dynamic objects (players, mobs, projectiles, item orbs, etc.).
- Registries: Stable, ordered registries mapping (namespace:name) → numeric IDs for blocks, items, entities, biomes, etc.

Naming conventions (public docs):

- Types: CamelCase (World, Chunk, Section, BlockState, BlockEntity, Entity).
- Fields: snake_case (chunk_x, section_y, world_seed, skybox_id).
- Positions:
  - BlockPos: (x, y, z) in world block coordinates.
  - ChunkPos: (chunk_x, chunk_z) in world chunk coordinates.
  - Section index: section_y in [0 … world.section_count_y-1].

## Coordinate Conventions

- UI/screen coordinates: origin at top-left (0, 0), consistent with Sokol.
- World/block coordinates: right-handed with y-up. BlockPos uses integer coordinates for addressing voxels.
- Chunk addressing: chunk_x = floor(x / 16), chunk_z = floor(z / 16). Local coordinates within a chunk are x & 15, z & 15.

## World

Configurable per-world. Key fields:

- identity:
  - world_id: stable UUID-like identifier.
  - name: human-readable name.
- seeds:
  - world_seed: 64-bit seed used for deterministic worldgen and random ticks.
- dimensions/config:
  - section_height_blocks: 32 (constant). Fixed height per Section.
  - section_count_y: number of Sections stacked vertically. World build height = 32 * section_count_y.
  - chunk_size_blocks_xz: always 16 for now (consistent with Minecraft’s 16×16 horizontal chunk size).
  - environment: skybox_id, ambient_light, time_scale, weather_rules, gravity, fluid rules.
- runtime state:
  - time_ticks: current world time in ticks.
  - weather_state: clear/rain/thunder + timers.
  - spawn_point: BlockPos.
- chunk index:
  - loaded_chunks: mapping (chunk_x, chunk_z) → Chunk.
  - region/IO indices for persistence.

Note: Simulation distance/config belongs to SIM, not GS. GS only stores the state; SIM decides which chunks to advance.

## Chunk

- key: (chunk_x, chunk_z)
- geometry: 16 × (32 * section_count_y) × 16 blocks.
- sections: contiguous array sections[section_count_y] ordered bottom to top (section_y increasing with y).
- heightmaps: precomputed per-column values for terrain, motion blocking, and skylight heuristics.
- entities: list of Entity IDs within or intersecting this chunk (spatial index may live alongside GS for fast queries).
- dirty flags: fine-grained flags for persistence and remeshing (set by SIM; GS just carries them).

## Section

- size: 16 × 32 × 16 blocks. Section height is fixed at 32.
- block storage: palette + bit-packed indices.
  - palette: array of BlockState IDs (small integers) for this section.
  - data: tightly packed indices into the palette (bits-per-block grows with unique states; shrinks when possible).
- lighting:
  - skylight: 4-bit per-voxel values (0–15) if applicable for the dimension.
  - blocklight: 4-bit per-voxel values (0–15).
- metadata:
  - random_tick_mask (optional): hints for which positions may receive random ticks.
  - occupancy/hollow flags for meshing and culling.
  - version/schema markers for migration.
- block entities: sparse map BlockPosWithinSection → BlockEntity ID/data ref.

## Blocks, BlockStates, and Geometry

- block_id: stable numeric ID from registry (namespace:name → u32 ID).
- state encoding: properties packed into a state_id or mapped via palette to minimize storage.
- geometry: Engine supports partial-blocks. Vox Aetatum will internally support subdividing a block into eighths to represent slabs and small shapes more consistently. Geometry and collision come from the block registry definition, not per-voxel data.
- light emission: per-state property contributing to blocklight.
- tags: functional grouping (e.g., logs, planks, mineable/axe) for recipes and behaviors.

## BlockEntities

- Sparse data records holding serialized state (e.g., inventories, text, timers).
- Identified by a block_entity_id (registry) and position.
- Serialization: schema’d or NBT-like tagged format with versioning.

## Entities

- entity_id: stable numeric ID (registry) for the type.
- eid: unique runtime identifier.
- transform: position (floating-point), velocity, orientation.
- AABB: collision bounds; flags (on_ground, in_fluid, etc.).
- attributes/components: health, inventory ref, AI state refs (componentized design is acceptable so long as GS remains serializable).

## Ticks, Schedules, and Randomness Anchors

- tick_counter: monotonically increasing world tick number.
- scheduled_block_updates: priority queue keyed by due_tick and position.
- random_tick_seed: seeded per chunk/section; SIM derives deterministic random tick positions each tick.
- event_queues: SIM-owned queues may live adjacent to GS; GS stores only persisted/schedulable events.

## Snapshots and Views

- GS is mutated by SIM only; CLI reads immutable snapshots.
- Snapshots are double/triple-buffered views consisting of:
  - snapshot header (tick_counter, snapshot_id)
  - world metadata and per-chunk immutable views (palette references and block/state/light arrays)
- Lifetime: CLI never holds pointers to mutable GS; snapshot views manage ownership until replaced.

## Persistence (Format Sketch)

- Regions group chunks (e.g., 32×32 chunks per region file). Each chunk stores:
  - section array with palette+bits data
  - lighting arrays
  - block entities
  - entities (or entity references if stored separately)
  - heightmaps and chunk metadata
- Compression: per-chunk section streams can be compressed independently.
- Versioning: file header + per-record schema versions; migrations on load.

## Registries and IDs

- Global ordered registries built from (namespace:name) pairs → stable numeric IDs.
- Hash/signature of registry contents included in connection handshake to ensure client/server alignment.
- Per-section palettes reference global BlockState IDs to keep wire/save formats consistent.

## Out of Scope (SIM Concerns)

- Simulation distance/size, chunk ticking policies, threading, and network deltas are SIM responsibilities.
- Prediction, reconciliation, and authoritative conflict resolution are not part of GS.

## Open Decisions

- Exact representation for eighth-subdivision geometry in the block registry (bitmask vs shape ID).
- Whether to store per-voxel metadata channels beyond lighting (e.g., biome ID per column/voxel).
- Entity storage co-location with chunks vs global entity graph with spatial indexing.
- None for section height: it is fixed at 32 (constant).
