# World Registry for Mods

This registry is the integration point for content packs and mods to define worlds available in a save (e.g. Overworld, Nether-like, Sky worlds).

Each world definition provides:

- key: unique identifier (e.g. "vox:overworld"). Use a namespace you control to avoid collisions.
- display_name: user-facing name, localizable (e.g. "The Overworld"). Mods may provide localization files.
- sections_below: number of vertical sections below Y=0.
- sections_above: number of vertical sections at or above Y=0.
- total build height = section_height (32) × (sections_below + sections_above).

Later extensions (reserved, not implemented yet):

- environment parameters (skybox id, ambient light, gravity, time scale)
- generation rules and biome sets
- allowed dimensions, portals, entry rules

## Authoring API (Zig)

The core engine exposes the API in `src/registry/world.zig`:

- WorldDef: the definition record
- WorldRegistry:
  - init(allocator)
  - addWorld(key, display_name, sections_below, sections_above)
  - addWorldWithGen(key, display_name, sections_below, sections_above, gen_spec)
  - get(key) -> ?WorldDef

Engine or mods are expected to populate the registry during initialization. The Simulation selects a `world_key` (default `vox:overworld`) and uses the world’s (sections_below, sections_above) to derive total sections. Chunk and meshing code use that split to map section indices to world Y around origin at Y=0.

Example:

```zig path=null start=null
var worlds = world_registry.WorldRegistry.init(allocator);
// Place origin at Y=0 with 0 sections below and 4 sections above (total height 128 blocks)
try worlds.addWorld("vox:overworld", "The Overworld", 0, 4);
const wd = worlds.get("vox:overworld") orelse unreachable;
// wd.sections_below == 0; wd.sections_above == 4
```

This is a stable integration surface for mod developers; future fields will be additive. Mods should treat strings as UTF-8 and avoid mutating registry records after registration.
