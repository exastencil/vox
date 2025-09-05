# Minecraft Technical Summary (Reference)

This document is a technical baseline for Minecraft-like functionality that the Vox Aetatum engine and its base-game mods aim to support or emulate. It is descriptive and version-agnostic (mechanics vary across Minecraft versions). We will deviate where it benefits the engine’s design and moddability.

Planned deviations (Vox Aetatum):
- Block subdivision into eighths for better support of vertical slabs and partial-block geometry.
- UI/screen coordinate origin at top-left (0,0), consistent with Sokol; world-space conventions will be documented explicitly.


## Core Loop and Ticks
- Fixed-step simulation at 20 ticks per second (TPS) in vanilla. Game logic, physics, random ticks, and scheduled updates advance on ticks.
- Random ticks: A subset of blocks receive random updates used for growth, decay, and spread (e.g., crops, leaves decay, fire spread).
- Scheduled block updates: Certain blocks schedule future updates (e.g., redstone, fluids).


## World and Chunks
- Chunked voxel world with a grid layout. Vanilla uses 16×N×16 chunks (width×height×depth), with N depending on version (e.g., historically 256, newer versions higher with extended vertical range). Columns identified by (chunkX, chunkZ).
- Region/area streaming: The client loads and unloads chunks around the player; server streams chunk data incrementally.
- Storage: Palette + bit-packing is common for compact block-state storage at the chunk or section level.
- Heightmaps: Precomputed height info for lighting and worldgen heuristics.


## Blocks and Block States
- Blocks are voxel cells with block states (properties) such as orientation, variant, waterlogged, power level, age/growth stage, etc.
- Collision shapes can be full cubes or arbitrary AABBs composing partial shapes (stairs, slabs, fences). Some shapes depend on neighboring blocks (e.g., fence connections).
- Transparency and render layers affect meshing and face culling. Block light emission is a property of certain blocks.
- Block entities (a.k.a. tile entities) carry extra data beyond the block state (e.g., chests, furnaces, signs). They have per-tick logic in many cases.
- Interactions include placement, breaking, use/activate, neighbor updates, and redstone updates.
- Fluids flow via discrete simulation with levels (e.g., 0–7). Waterlogging enables blocks to coexist with water.


## Lighting
- Dual lighting system with sky light and block light, each typically 0–15.
- Propagation is flood-fill-like with attenuation per block; skylight also depends on exposure to the sky and height information.
- Smooth lighting is a rendering technique (vertex lighting) rather than a change to the propagation model.


## Items and Inventory
- Stackable items with maximum stack sizes; durability for tools/armor.
- Pickup entities representing dropped items with simple physics and merge behavior.
- Container UIs (chests, furnaces, crafting, etc.) with server-authoritative state and client-side prediction limits.


## Crafting and Processing
- Crafting grid shaped recipes; shapeless recipes; smelting/blasting/smoking; stonecutting; anvil and enchanting systems.
- Recipe resolution is data-driven: pattern matching and tags/groups.
- Fuel/time cost models for processing blocks (furnaces, smokers, etc.).


## Entities, Physics, and AI
- Entity types include players, mobs, projectiles, vehicles, and item orbs. Entities have AABBs, position, velocity, and flags (on_ground, swimming, etc.).
- Physics: Gravity, friction, fluid buoyancy/drag, step-up mechanics, ladder mechanics.
- Pathfinding: Grid/voxel-based with navigation rules (walkable blocks, doors, water, etc.).
- AI behaviors: Goal/Task system (wander, follow, attack, flee, breed, work blocks, avoid sunlight, etc.).


## Redstone-like Systems (Circuits)
- Power levels 0–15; signal attenuation by distance and components.
- Components: wires, torches (inverters), repeaters (delay/diodes), comparators (measurement/memory), sensors, pistons.
- Update order and quasi-connectivity edge cases matter; block updates ripple through the network.


## World Generation
- Noise-based terrain (multi-octave noise, domain warping in newer variants), biome assignment, and climate parameters.
- Carvers (caves, ravines), surface decoration (vegetation, ores), structures (villages, temples, dungeons, monuments, strongholds).
- Deterministic per world seed; structure placement rules and biome constraints.
- Chunk-level pipelines with staging: base heightfield, biome sampling, carving, surface, features, structures.


## Dimensions and Travel
- Multiple dimensions with distinct rules (terrain, skybox, lighting, time flow). Canonically: Overworld, Nether, End.
- Coordinate scaling between dimensions for portals (e.g., 1:8 mapping in Nether travel, version-dependent specifics).


## Time, Weather, and Environment
- Day/night cycle tied to world time. Light levels change based on sun/moon position.
- Weather states like clear, rain, thunder; biome-specific precipitation (e.g., snow vs rain) and temperature effects.


## Saving and Persistence
- Chunk-based saves grouped into region files; per-chunk data includes block states, block entities, heightmaps, lighting, entities, and scheduled ticks.
- Game-wide player data and dimension metadata. Vanilla uses a tagged binary format for rich structured data.


## Networking Model
- Client/server architecture; server authoritative over simulation.
- Client receives chunk data, block/entity updates, and UI/container states; sends input and action requests.
- Compression and delta updates; capability negotiation is out-of-band in vanilla but is essential for modded ecosystems.


## Rendering Notes
- Chunk meshing with face culling and greedy meshing to reduce polygons.
- Multiple render layers for opaque, cutout, transparent; sorted rendering for transparency.
- Particle systems and simple billboarded quads for many effects.
- UI coordinates typically use a top-left origin; world coordinates are right-handed with y-up in many implementations (documented per engine).


## Performance Considerations
- Keep chunk updates incremental; async mesh generation and I/O.
- Use palettes for compact block storage; section-based lighting for faster updates.
- Culling (frustum, occlusion) and LOD strategies for distant terrain.


## Mapping to Vox Aetatum
- World/Chunks: Chunked storage with palette+bitpacking, async meshing, streaming.
- Blocks: Extended partial-geometry via eighth-subdivision for better slabs/stairs/variants; block states and block entities supported.
- Lighting: Dual-channel 0–15 with incremental propagation.
- Ticks: Fixed simulation step (target 20 TPS) with random ticks and scheduled updates exposed as mod hooks.
- Entities/AI: AABB physics and a goal-based AI system; fluid and ladder hooks.
- Redstone-like Circuits: Power 0–15 with components registered by mods; deterministic update order.
- Worldgen: Seeded, staged pipelines; biomes, features, and structures exposed via mod APIs.
- Persistence: Chunk-based save format with forward-compatible schemas.
- Networking: Authoritative server; mod capability negotiation protocol.
- Rendering/UI: Sokol-backed renderer; top-left origin for UI; documented world coordinate system.


## Open Questions for Design
- Exact chunk height and sectioning strategy in the engine (fixed vs configurable).
- Script runtime choice(s) and ABI stability surface for Zig mods.
- Redstone-like update order guarantees for determinism and performance.
- Cross-mod compatibility and conflict resolution (ID spaces, tags, capabilities).

