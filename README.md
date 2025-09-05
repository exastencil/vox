# Vox Aetatum

A modular, first-person, voxel-based, open‑world game engine written in Zig. Inspired by the open‑world survival and exploration design of games like Minecraft, but with a core focus on being a moddable engine first. The base game itself is implemented entirely as mods, serving as a reference implementation you can keep, extend, or replace.


## Vision

- Engine‑first: All gameplay content (blocks, entities, items, recipes, worldgen, UI, etc.) is implemented as mods.
- Replaceable base game: Ship a free, reference "base game" as a set of mods. Players and creators can remove, replace, or remix it.
- Flexible mod licensing: The base engine will be free. The license will allow mod authors to choose their own distribution model (free, paid, subscriptions, donations, etc.) provided their content depends on the base engine.
- Showcase parity: To sanity‑check the engine and act as documentation, the reference mods will implement as much of Minecraft’s functionality as possible without using the Minecraft trademark or assets.
- Clear separation of concerns: Engine provides rendering, physics, networking, persistence, mod loading, and deterministic APIs; mods provide content and behavior.


## Guiding Principles

- Modularity by default: Every engine feature should be usable or replaceable from mods.
- Data‑driven: Prefer declarative definitions where possible; provide escape hatches for custom logic.
- Determinism (where feasible): Keep core simulation deterministic to support replays, networking, and authoritative servers.
- Performance: Lean into Zig’s strengths; prioritize memory locality, cache‑coherent data layouts, and zero‑cost abstractions.
- Clear coordinate conventions: Use a screen/UI coordinate system with the origin (0,0) at the top‑left, consistent with Sokol, and document world coordinates explicitly.
- Safety & stability: Robust versioning and capability negotiation between engine and mods.


## High‑Level Architecture

- Core Engine (Zig)
  - Rendering: Likely via sokol_gfx or similar backend; chunked meshing, greedy meshing, and instancing where appropriate.
  - World Representation: Chunked voxel world with efficient storage (palette/bit‑packing), streaming, and LOD options.
  - Block Geometry: Internally subdivide blocks into eighths to better support vertical slabs and fine‑grained partial blocks.
  - Physics: Broadphase/narrowphase for axis‑aligned voxel environments, character controller, fluid and ladder behavior hooks.
  - Worldgen: Pluggable pipelines (noise, structures, biomes) with deterministic seeds.
  - Persistence: Region files/chunk serialization with forward‑compatible schemas.
  - Networking: Client/server with authority on the server; delta compression; modded protocol capabilities.
  - Input & UI: Input mapping layer; UI toolkit compatible with the rendering backend and top‑left UI coordinates.
  - Audio: Event‑driven audio with spatialization hooks.
  - Mod Runtime: ABI‑stable interfaces for Zig libraries; optional scripting runtime (language TBD based on requirements).

- Mod SDK
  - Content Registration: Blocks, items, entities, recipes, worldgen features, GUIs, sounds, advancements/achievements, etc.
  - Behavior Hooks: Tick/update events, interaction callbacks, crafting/furnace/processing recipes, loot tables, AI tasks.
  - Data‑Driven Paths: JSON/TOML or Zig‑native declarative APIs for common cases; scripting/compiled Zig for custom logic.
  - Capabilities & Versioning: Semantic versioning for engine APIs, feature flags, and compatibility guards.
  - Tooling: Build, package, and dependency management for modpacks.


## Reference Implementation ("Base Game")

- Purpose: Serve as a living example and test suite for the engine’s APIs.
- Scope: Reproduce major Minecraft‑like systems (worldgen, crafting, redstone‑like circuits, mobs, day/night cycle, weather, dimensions, structures, etc.) while avoiding any trademarked names or proprietary assets.
- Deviations: Where it improves engine design or modding, we will deviate (e.g., block eighths for better partial blocks/vertical slabs).


## Modding Model

- Zig Mods: First‑class support for mods written as Zig libraries against a stable engine API.
- Scripting Mods: One or more scripting runtimes (TBD) for rapid iteration and user accessibility.
- Hot Reload (stretch): Where practical, support hot‑reloading of content and scripts.
- Security: Sandboxing where applicable for scripting environments; clear permission model for IO/network.


## Licensing (Intent)

- Engine: Free license (exact license TBD) encouraging broad adoption and enabling commercial or free mods.
- Base Game Mods: Free, open reference implementation.
- Mods: Authors choose their own licensing/pricing; must depend on the engine rather than bundling it.


## Roadmap (Initial)

1. Seed the repository (this README) and project scaffolding.
2. Decide on rendering backend and windowing (likely Sokol + Zig bindings).
3. World/Chunk core representation and meshing pipeline.
4. Minimal player controller and camera.
5. Mod API skeleton + Zig mod loader.
6. Decide on scripting runtime(s) after requirements solidify.
7. Persistence format and world save/load.
8. Networking prototype (local + LAN), capability negotiation for mods.
9. Base game reference mods: worldgen, basic blocks/items/tools, crafting, mobs.


## Trademark Notice

This project aims to replicate certain gameplay features for educational and compatibility purposes. We will not use the term "Minecraft" or any trademarked names or assets in shipped content unless separately licensed. References to third‑party games here are purely descriptive.


## Contributing

- Discussions and technical design will be captured in the project wiki.
- PRs are welcome once the API stabilizes; early contributions may involve rapid iteration.
- Please adhere to the coding style and patterns established in the codebase.


## Status

Early planning. README seeded; initial commit will follow. Wiki and technical reference will be created next.

