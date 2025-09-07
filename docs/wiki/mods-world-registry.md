# World Registry for Mods

This registry is the integration point for content packs and mods to define worlds available in a save (e.g. Overworld, Nether-like, Sky worlds).

Each world definition provides:

- key: unique identifier (e.g. "vox:overworld"). Use a namespace you control to avoid collisions.
- display_name: user-facing name, localizable (e.g. "The Overworld"). Mods may provide localization files.
- section_count_y: number of vertical sections per chunk. Combined with section height, this determines build height.

Later extensions (reserved, not implemented yet):

- environment parameters (skybox id, ambient light, gravity, time scale)
- generation rules and biome sets
- allowed dimensions, portals, entry rules

## Authoring API (Zig)

The core engine exposes the API in `src/registry/world.zig`:

- WorldDef: the definition record
- WorldRegistry:
  - init(allocator)
  - addWorld(key, display_name, section_count_y)
  - get(key) -> ?WorldDef

Engine or mods are expected to populate the registry during initialization. The Simulation selects a `world_key` (default `vox:overworld`) and uses the world’s `section_count_y` to configure chunk heights when `SimulationOptions.section_count_y` is not explicitly provided.

Example:

```zig path=null start=null
var worlds = world_registry.WorldRegistry.init(allocator);
try worlds.addWorld("vox:overworld", "The Overworld", 4);
const wd = worlds.get("vox:overworld") orelse unreachable;
// wd.section_count_y == 4
```

This is a stable integration surface for mod developers; future fields will be additive. Mods should treat strings as UTF-8 and avoid mutating registry records after registration.
