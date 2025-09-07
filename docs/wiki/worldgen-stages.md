# World Generation Stages

This document describes the staged world generation pipeline adopted by Vox Aetatum, modeled on Minecraft Java Edition’s process. Chunks progress through discrete statuses. Far-away chunks may be frozen as proto-chunks at intermediate steps for performance, and resume when players approach. When generation completes, proto-chunks are converted to level chunks.

Chunk statuses (in order):

- empty: The chunk has been created but no generation work has run yet.
- structures_starts: Reserve structure start positions (placeholder; not yet implemented in core).
- structures_references: Record references to nearby structure starts (placeholder).
- biomes: Per-voxel biomes are selected and stored in a 3D buffer.
- noise: Base terrain is generated into a 3D blocks buffer; column tops are tracked.
- surface: Perform biome-dependent surface replacement (placeholder).
- carvers: Carve caves, ravines, etc. (placeholder).
- features: Place features (ores, trees) and prepare heightmaps (placeholder for feature placement; heightmaps are already populated from the noise step in the current core pipeline).
- initialize_light: Initialize lighting engine state (placeholder).
- light: Compute block and sky light (placeholder; currently zero-initialized).
- spawn: Perform initial mob-spawn pass (placeholder).
- full: The chunk is finalized into a level chunk and is now loadable.

Terminology and data flow

- Proto-chunk: Transient, incremental generation state. Holds per-voxel biome and block buffers and the per-column top heights. Status advances one step at a time.
- Level chunk: Finalized gs.Chunk containing palettized sections, heightmaps, lighting arrays, entities, and is immutable for rendering/consumption.

Current core implementation

- The core pipeline is implemented in src/worldgen.zig via ProtoChunk and staged functions. It supports advancing a proto-chunk one phase per call, or doing a single-shot generateChunk which internally executes all phases.
- The Simulation worldgen worker advances nearby proto-chunks incrementally each pass and finalizes them when reaching the spawn→full transition. Far-away desired chunks may be created and frozen at early steps.
- Several phases (structures, surface, carvers, features, lighting, spawn) are exposed as hook points but are currently no-ops in the core engine. Mods can implement them as they are surfaced via the registry API (see below).

Registry hooks for mods

Module worldgen hooks (new API):

Your worldgen module provides a WorldGen.Def with any subset of these hooks. The engine calls them while advancing a proto-chunk. Implement at least biomes and noise.

- biomes(seed: u64, proto: *ProtoChunk, params: Params) -> !void
  - Allocate (ensureBiomesAllocated) and fill proto.biomes_buf for all voxels.
- noise(seed: u64, proto: *ProtoChunk, params: Params, lookup: BlockLookup) -> !void
  - Allocate (ensureBlocksAllocated), fill proto.blocks_buf, and update proto.tops per (x,z) column. Use lookup to resolve block IDs (e.g., "core:air").
- surface(seed, proto, params) -> !void
- carvers(seed, proto, params) -> !void
- features(seed, proto, params) -> !void
- initialize_light(seed, proto, params) -> !void
- light(seed, proto, params) -> !void
- spawn(seed, proto, params) -> !void

Notes:
- The legacy selector API (select_biome/select_block) has been removed in favor of phase hooks.
- Params carries world-configured blocks/biomes; BlockLookup resolves named blocks.

Notes:

- The built-in pipeline will invoke these callbacks if present when advancing through the corresponding status. For now, these hooks are placeholders (no core behavior attached) and exist to establish mod integration points and ordering.
- As phases graduate from placeholders, this doc will be updated to specify data contracts for reading/writing proto-chunk buffers (e.g., feature placement writing into the blocks buffer between noise and finalize).

Scheduling and performance

- Simulation increments a bounded number of proto-chunk steps per worker pass (currently 8) to avoid long stalls.
- When a proto-chunk reaches spawn, it is finalized into a level chunk in the next step and inserted into the world indices; its proto state is deallocated.

Determinism

- The pipeline uses a single world seed. All hooks should be deterministic functions of (seed, ChunkPos/BlockPos, Params) to allow reproducible generation across clients/servers.

Developer references

- Types and enums: see src/gs.zig (ChunkStatus). See src/worldgen.zig for ProtoChunk and advanceOnePhase.
- Simulation integration: see src/simulation.zig worldgen worker.

TODOs and placeholders

- Surface, carvers, features, and lighting are currently stubs. Mods may register hooks, but the core engine makes no guarantees yet beyond ordering. When these phases are implemented, we will update this document and the code comments accordingly.
