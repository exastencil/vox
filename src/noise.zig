// Aggregated noise utilities for mods and worldgen.
// Import as: const noise = @import("noise");
// Then use: const Simplex = noise.SimplexNoise;

pub const SimplexNoise = @import("noise/simplex.zig").SimplexNoise;
