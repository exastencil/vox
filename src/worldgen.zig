const std = @import("std");
const gs = @import("gs.zig");
const ids = @import("ids.zig");
const constants = @import("constants.zig");

pub const ChunkGenConfig = struct {
    air: ids.BlockStateId,
    ground: ids.BlockStateId,
};

pub const Error = error{
    EmptyBiomeList,
};

fn splitmix64(x_in: u64) u64 {
    var z: u64 = x_in + 0x9E3779B97F4A7C15;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EB;
    return z ^ (z >> 31);
}

fn packBitsSet(dst: []u32, idx: usize, bits_per_index: u6, value: u32) void {
    // writes value (lower bits_per_index bits) at logical element index idx
    const bit_off: usize = idx * @as(usize, bits_per_index);
    const word_idx: usize = bit_off >> 5; // /32
    const bit_idx: u5 = @intCast(bit_off & 31);
    const mask: u32 = if (bits_per_index == 32) 0xFFFF_FFFF else (@as(u32, 1) << @intCast(bits_per_index)) - 1;
    const vmasked: u32 = value & mask;
    const rem: u6 = @intCast(32 - bit_idx);
    if (bits_per_index <= rem) {
        // fits into single word
        const clear_mask: u32 = ~(mask << bit_idx);
        dst[word_idx] = (dst[word_idx] & clear_mask) | (vmasked << bit_idx);
    } else {
        // straddles two words
        const lo_bits: u6 = rem;
        const hi_bits: u6 = @intCast(bits_per_index - lo_bits);
        const lo_mask: u32 = (@as(u32, 1) << lo_bits) - 1;
        const lo_val: u32 = vmasked & lo_mask;
        const hi_val: u32 = vmasked >> lo_bits;
        // low part into word_idx
        const clear_lo: u32 = ~(lo_mask << bit_idx);
        dst[word_idx] = (dst[word_idx] & clear_lo) | (lo_val << bit_idx);
        // high part into next word at bit 0
        const hi_mask: u32 = (@as(u32, 1) << hi_bits) - 1;
        dst[word_idx + 1] = (dst[word_idx + 1] & ~hi_mask) | hi_val;
    }
}

fn ceilDiv(a: usize, b: usize) usize { return (a + b - 1) / b; }

fn worldColumnHeight(seed: u64, chunk_pos: gs.ChunkPos, local_x: u32, local_z: u32, max_y: u32) u32 {
    // Simple deterministic pseudo-height based on hashed world-space x/z
    const wx: i64 = @as(i64, chunk_pos.x) * @as(i64, constants.chunk_size_x) + @as(i64, local_x);
    const wz: i64 = @as(i64, chunk_pos.z) * @as(i64, constants.chunk_size_z) + @as(i64, local_z);
    const mix: u64 = splitmix64(seed ^ @as(u64, @bitCast(wx)) ^ (splitmix64(@as(u64, @bitCast(wz))) << 1));
    const base: u32 = max_y / 3; // one third base level
    const amp: u32 = max_y / 6; // +/- amplitude
    return base + @as(u32, @intCast(mix % (amp + 1)));
}

pub fn generateChunk(
    allocator: std.mem.Allocator,
    seed: u64,
    section_count_y: u16,
    chunk_pos: gs.ChunkPos,
    cfg: ChunkGenConfig,
    biomes: []const ids.BiomeId,
) Error!gs.Chunk {
    if (biomes.len == 0) return Error.EmptyBiomeList;

    const sections_len: usize = section_count_y;
    var sections = try allocator.alloc(gs.Section, sections_len);

    // Choose a single biome per chunk for now (compresses to palette size 1 per section)
    const biome_idx_chunk: usize = @intCast(splitmix64(seed ^ @as(u64, @bitCast(chunk_pos.x)) ^ (@as(u64, @bitCast(chunk_pos.z)) << 1)) % @as(u64, biomes.len));
    const chunk_biome: ids.BiomeId = biomes[biome_idx_chunk];

    const max_y: u32 = @intCast(constants.section_height * sections_len);

    // precompute per-column heights
    var heights: [constants.chunk_size_x * constants.chunk_size_z]u32 = undefined;
    for (0..constants.chunk_size_z) |lz| {
        for (0..constants.chunk_size_x) |lx| {
            const idx = lz * constants.chunk_size_x + lx;
            heights[idx] = worldColumnHeight(seed, chunk_pos, @intCast(lx), @intCast(lz), max_y);
        }
    }

    // For blocks we only use 2 states: air and ground -> bpi = 1
    const bpi: u6 = 1;
    const voxel_count: usize = constants.chunk_size_x * constants.section_height * constants.chunk_size_z;
    const words_per_section: usize = ceilDiv(voxel_count * bpi, 32);
    const light_bytes_per_section: usize = voxel_count / 2; // nibble-packed

    // Build each section
    for (0..sections_len) |sy| {
        // allocate arrays
        var palette = try allocator.alloc(ids.BlockStateId, 2);
        palette[0] = cfg.air;
        palette[1] = cfg.ground;
        const indices_bits = try allocator.alloc(u32, words_per_section);
        @memset(indices_bits, 0);
        const skylight = try allocator.alloc(u8, light_bytes_per_section);
        const blocklight = try allocator.alloc(u8, light_bytes_per_section);
        @memset(skylight, 0);
        @memset(blocklight, 0);
        const biome_palette = try allocator.alloc(ids.BiomeId, 1);
        biome_palette[0] = chunk_biome;
        const biome_indices_bits = try allocator.alloc(u32, 0);
        const block_entities = try allocator.alloc(gs.BlockEntityRecord, 0);

        // fill blocks
        const y_base: u32 = @intCast(sy * constants.section_height);
        for (0..constants.chunk_size_z) |lz| {
            for (0..constants.chunk_size_x) |lx| {
                const h = heights[lz * constants.chunk_size_x + lx];
                for (0..constants.section_height) |ly| {
                    const y: u32 = y_base + @as(u32, @intCast(ly));
                    const voxel_idx_in_section: usize = ly * constants.chunk_size_x * constants.chunk_size_z + lz * constants.chunk_size_x + lx;
                    const is_ground: bool = y <= h;
                    const v: u32 = if (is_ground) 1 else 0; // 1=ground, 0=air
                    packBitsSet(indices_bits, voxel_idx_in_section, bpi, v);
                    // lighting left at zero for now
                }
            }
        }

        sections[sy] = .{
            .palette = palette,
            .blocks_indices_bits = indices_bits,
            .skylight = skylight,
            .blocklight = blocklight,
            .biome_palette = biome_palette,
            .biome_indices_bits = biome_indices_bits,
            .block_entities = block_entities,
        };
    }

    // Compute heightmaps (motion_blocking == surface for now)
    var heightmaps: gs.Heightmaps = .{
        .motion_blocking = undefined,
        .world_surface = undefined,
    };
    for (0..constants.chunk_size_z) |lz| {
        for (0..constants.chunk_size_x) |lx| {
            const idx = lz * constants.chunk_size_x + lx;
            const h = heights[idx];
            heightmaps.motion_blocking[idx] = @intCast(h);
            heightmaps.world_surface[idx] = @intCast(h);
        }
    }

    const entities = try allocator.alloc(gs.EntityRecord, 0);

    return .{
        .pos = chunk_pos,
        .sections = sections,
        .heightmaps = heightmaps,
        .entities = entities,
    };
}

