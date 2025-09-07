# Noise Utilities

This engine provides a deterministic Simplex Noise implementation for world generation and mods.

API (Zig)

- Import
  - const noise = @import("noise");
  - const Simplex = noise.SimplexNoise;
- Construct a generator with a 64-bit seed:
  - var sn = Simplex.init(world_seed);
- Sample noise:
  - 2D: sn.noise2(x, y) -> f64 in ~[-1, 1]
  - 3D: sn.noise3(x, y, z) -> f64 in ~[-1, 1]
- Fractal helpers (fBm):
  - sn.fbm2(x, y, octaves, lacunarity, gain) -> f64 in ~[-1, 1]
  - sn.fbm3(x, y, z, octaves, lacunarity, gain) -> f64 in ~[-1, 1]

Determinism

- The generator uses a SplitMix64-seeded permutation table for platform-stable results.
- Given identical inputs (seed, coordinates), results are bit-for-bit reproducible across runs and platforms.
- For chunk-stable terrain, derive domain coordinates from world/chunk positions and pass the world seed from the worldgen pipeline.

Usage inside worldgen hooks

- Biomes phase typically uses low-frequency 2D noise (domain in chunk/world XZ) to pick regions.
- Noise phase can combine 2D/3D noise and fBm to build heightfields and density fields.

Example (pseudo)

```zig path=null start=null
const noise = @import("noise");

fn noiseHook(seed: u64, proto: *wapi.ProtoChunk, params: Params, lookup: BlockLookup) !void {
    var sn = noise.SimplexNoise.init(seed);
    const scale = 0.01; // frequency
    for (0..constants.chunk_size_z) |lz| {
        for (0..constants.chunk_size_x) |lx| {
            const wx = @as(f64, @floatFromInt(proto.pos.x * constants.chunk_size_x + @as(i32, @intCast(lx))));
            const wz = @as(f64, @floatFromInt(proto.pos.z * constants.chunk_size_z + @as(i32, @intCast(lz))));
            const h = sn.fbm2(wx * scale, wz * scale, 4, 2.0, 0.5);
            const height = @as(i32, @intFromFloat(40.0 + 20.0 * h));
            // fill blocks up to height ...
        }
    }
}
```

Notes

- Outputs are approximately in [-1, 1]; scale/shift as needed for your domain.
- Use f64 for consistent results; f32 wrappers are provided for convenience.

