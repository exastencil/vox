# WARP.md

This file provides guidance to WARP (warp.dev) when working with code in this repository.

Project-specific Rules

- UI/screen coordinate system uses origin (0,0) at the top-left, consistent with Sokol. Follow this when working on rendering or UI.
- Keep the wiki in the docs folder up to date. If a feature is described in the docs that is missing or not fully implemented, add a comment in the code. If functionality is changed such that the code conflicts with the wiki, summarize the difference and ask what the preferred resolution should be (either changing the docs or changing the functionality).

Common commands

- Build (also regenerates shaders via sokol-shdc and fetches Zig deps if needed):
  - zig build
- Run the game:
  - zig build run
  - Pass args to the program:
    - zig build run -- <args>
- Run all tests (multiple test targets are wired into the build):
  - zig build test
- Run tests for a single file (worldgen example) or filter by name:
  - zig test src/worldgen.zig
  - zig test src/worldgen.zig --test-filter <substring>
    Notes:
  - Some tests (e.g., in src/registry.zig, src/registry/resource.zig) are configured through build.zig with additional module imports; prefer zig build test for those.
- Format code (write changes):
  - zig fmt .
- Check formatting (no changes):
  - zig fmt --check .

Dependency and build notes

- Zig package dependencies are declared in build.zig.zon (sokol, sokol-tools-bin). A placeholder hash for sokol-tools-bin may require updating; if a hash mismatch occurs during build, follow Zig’s error guidance to update the dependency hash (e.g., resolve with zig fetch/--save) and rebuild.
- Shader pipeline:
  - Source: shaders/chunk.glsl
  - Generated: src/shaders/chunk_shd.zig (created by sokol-shdc during zig build)
  - The build selects an shdc binary from sokol-tools-bin based on the host OS/arch.

High-level architecture

- Entry/Executable
  - src/main.zig is the entry point. The build creates an executable named vox-aetatum.
  - The executable is integrated with a build “run” step (zig build run) that forwards CLI args.
- Client/Render Loop
  - src/client.zig orchestrates the frame loop and rendering setup using Sokol (sokol-zig). UI/screen coordinates use top-left origin.
  - Shaders live under shaders/ (GLSL source); the generated Zig shader module is in src/shaders/chunk_shd.zig and is used by the renderer.
- Game State and Simulation
  - src/gs.zig defines the game state model (“GS”).
  - src/simulation.zig updates the GS in a deterministic order (see docs/wiki for the simulation update order reference).
  - src/player.zig contains player representation and related logic.
- World and Worldgen
  - src/worldgen.zig contains world generation logic and has unit tests wired via the build.
  - src/registry/worldgen.zig provides registration points for worldgen features.
- Registry and Resources
  - src/registry.zig provides the central registry facade.
  - src/registry/resource.zig, src/registry/block.zig, src/registry/biome.zig, src/registry/world.zig define resource types and their registration APIs.
  - src/ids.zig provides stable identifiers for registrable content.
  - This registry system underpins the planned mod-first design where content is declared and looked up via IDs.
- Utilities
  - src/constants.zig collects engine-wide constants and limits.
  - src/png.zig provides PNG loading/utility functionality used by assets/textures.

Documentation anchors

- docs/wiki/README.md links to architectural notes:
  - Client/Server split and runtime topology
  - Game State (GS) model
  - Simulation update order
  - Eighth-subdivision geometry (octant bitmask)
  - Constants and limits quick reference
  - Minecraft-compat technical summary for baseline parity in the reference mods

Vision and guiding principles (from README)

- Engine-first and moddable by default; the “base game” is implemented as replaceable mods.
- Data-driven with clear separation between engine (rendering, physics, networking, persistence, mod loading, deterministic APIs) and mods (content and behavior).
- Determinism where feasible, and performance-oriented design leveraging Zig.

Notes for future enhancements (non-blocking)

- If frequent single-test filtering across all test targets is needed, expose a build option (e.g., -Dtest-filter) in build.zig and forward it to each addTest runner via --test-filter.
