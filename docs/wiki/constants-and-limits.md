# Constants and Limits (Reference)

Quick reference for core constants, limits, and statuses. Status legend:

- decided: fixed as part of the current design
- target: intended default (may be configurable)
- provisional: likely to evolve pending benchmarks/experience

## Coordinate Systems

- UI origin: top-left (0,0) — decided
- World axes: right-handed, y-up — decided

## Chunks and Sections

- Chunk size (x,z): 16×16 blocks — decided
- Section height (y): 32 blocks — decided
- World build height: 32 × (sections_below + sections_above) — decided (split per-world config around Y=0)
- Chunk addressing: chunk_x = floor(x/16), chunk_z = floor(z/16); local x,z = x & 15, z & 15 — decided
- Region grouping for saves: 32×32 chunks per region — provisional

## Blocks and Geometry

- Eighth-subdivision mask: u8 occupancy, bit i = (x<<0)|(y<<1)|(z<<2) with 0=negative half, 1=positive half — decided
- Full cube mask: 0xFF; empty: 0x00 — decided

## Lighting

- Skylight levels: 0–15 — decided
- Blocklight levels: 0–15 — decided

## Biomes

- Per-voxel (3D) biomes: section-level palette + per-voxel indices — decided

## Entities

- Ownership: stored in owning chunk — decided
- Per-chunk soft cap: 256 entities — provisional (initial default; may adjust after performance metrics)

## Simulation (SIM) Defaults

- Tick rate: 20 TPS — target
- Simulation distance: engine/runtime setting (not GS) — provisional (tunable)
- Redstone-like power range: 0–15 — provisional
- Snapshot buffering: at least double-buffering for CLI reads — target

## Persistence

- Section storage: palette+bit-packed blocks; lighting arrays; 3D biomes (palette+indices); sparse block entities — decided
- Entities: stored in-chunk (subject to cap) — decided
- Compression: per-chunk streams compressible — provisional

Notes:

- “Provisional” items will be revisited once we gather profiling data and implementation experience.
- Constants that affect on-disk or on-wire formats will be versioned with migration paths when changed.
