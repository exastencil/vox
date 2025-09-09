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
- Section index: section_y in [0 … total_sections-1], where total_sections = world.sections_below + world.sections_above. World Y mapping for a voxel within a section: world_y = (section_y - world.sections_below) * section_height + local_y (0 ≤ local_y < section_height).

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
  - sections_below: number of Sections whose Y-range lies below world Y=0.
  - sections_above: number of Sections whose Y-range lies at or above world Y=0.
  - total_sections = sections_below + sections_above. World build height = 32 × total_sections.
  - World origin: Y=0 is the design origin (e.g., sea level). Section index sections_below corresponds to world Y ∈ [0, 31].
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
- geometry: 16 × (32 × total_sections) × 16 blocks, where total_sections = sections_below + sections_above.
- sections: contiguous array sections[total_sections] ordered bottom to top (section_y increasing with y). World-space Y for a voxel at (section_y, local_y) is world_y = (section_y - sections_below) × 32 + local_y.
- trimming: during worldgen, trailing all-air sections (palette == [0]) may be trimmed from the end to reduce memory and meshing work. A chunk’s sections.len can therefore be ≤ total_sections.
- heightmaps: precomputed per-column values for terrain, motion blocking, and skylight heuristics.
- entities: in-chunk storage of entities owned by this chunk; soft cap 256 entities per chunk. Entities transfer ownership when crossing chunk boundaries. An optional global read-only index may exist for queries, but GS ownership is per-chunk.
- dirty flags: fine-grained flags for persistence and remeshing (set by SIM; GS just carries them).

## Section

- size: 16 × 32 × 16 blocks. Section height is fixed at 32.
- block storage: palette + bit-packed indices.
  - palette: array of BlockState IDs (small integers) for this section.
  - data: tightly packed indices into the palette (bits-per-block grows with unique states; shrinks when possible).
- lighting:
  - skylight: 4-bit per-voxel values (0–15) if applicable for the dimension.
  - blocklight: 4-bit per-voxel values (0–15).
- biomes:
  - per-voxel biome IDs using a section-level palette + bit-packed per-voxel indices (3D biomes).
  - palette entries reference the global biome registry; single-biome sections compress to palette size 1.
- metadata:
  - random_tick_mask (optional): hints for which positions may receive random ticks.
  - occupancy/hollow flags for meshing and culling.
  - version/schema markers for migration.
- block entities: sparse map BlockPosWithinSection → BlockEntity ID/data ref.

## Blocks, BlockStates, and Geometry

- block_id: stable numeric ID from registry (namespace:name → u32 ID).
- state encoding: properties packed into a state_id or mapped via palette to minimize storage.
- geometry: Engine supports partial-blocks. Vox Aetatum uses an eighth-subdivision occupancy mask per block:
  - sub_mask: u8 where each bit represents one octant (eighth) of the block.
  - Bit index mapping (xyz → bit): i = (x_bit << 0) | (y_bit << 1) | (z_bit << 2),
    where x_bit,y_bit,z_bit ∈ {0,1} with 0 = negative half, 1 = positive half along the axis.
  - Example masks: bottom slab (y=0): 0b00110011; top slab (y=1): 0b11001100; neg-x half: 0b01010101; pos-x half: 0b10101010.
  - Geometry and collision are derived from the block registry definition (shape assembled from occupied octants).
- light emission: per-state property contributing to blocklight.
- tags: functional grouping (e.g., logs, planks, mineable/axe) for recipes and behaviors.

## BlockEntities

- Sparse data records holding serialized state (e.g., inventories, text, timers).
- Identified by a block_entity_id (registry) and position.
- Serialization: schema’d or NBT-like tagged format with versioning.

## Entities

- entity_id: stable numeric ID (registry) for the type.
- eid: unique runtime identifier.
- residency: owned by a single chunk (chunk_x, chunk_z); SIM updates residency when crossing chunk boundaries; per-chunk soft cap: 256 entities.
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
  - biome arrays (section-level palette + per-voxel indices)
  - block entities
  - entities (in-chunk array; up to 256 entries per chunk)
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

- Eighth-subdivision representation: fixed as an 8-bit occupancy mask with xyz bit mapping (i = x | y<<1 | z<<2).
- 3D biomes: store biome IDs per voxel using section-level palette + per-voxel indices (decided).
- Entity storage: decided — entities are stored in their owning chunk with a soft cap of 256 per chunk; an optional global index may exist for queries.
- None for section height: it is fixed at 32 (constant).
