const std = @import("std");
const gs = @import("gs.zig");
const ids = @import("ids.zig");
const constants = @import("constants.zig");

pub const ChunkGenConfig = struct {
    air: ids.BlockStateId,
    ground: ids.BlockStateId,
};

pub const Error = error{ EmptyBiomeList } || std.mem.Allocator.Error;

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
    const rem: u6 = @as(u6, 32) - @as(u6, bit_idx);
    if (bits_per_index <= rem) {
        // fits into single word
        const clear_mask: u32 = ~(mask << bit_idx);
        dst[word_idx] = (dst[word_idx] & clear_mask) | (vmasked << bit_idx);
    } else {
        // straddles two words
        const lo_bits: u6 = rem;
        const hi_bits: u6 = @intCast(bits_per_index - lo_bits);
        const lo_mask: u32 = (@as(u32, 1) << @as(u5, @intCast(lo_bits))) - 1;
        const lo_val: u32 = vmasked & lo_mask;
        const hi_val: u32 = vmasked >> @as(u5, @intCast(lo_bits));
        // low part into word_idx
        const clear_lo: u32 = ~(lo_mask << bit_idx);
        dst[word_idx] = (dst[word_idx] & clear_lo) | (lo_val << bit_idx);
        // high part into next word at bit 0
        const hi_mask: u32 = (@as(u32, 1) << @as(u5, @intCast(hi_bits))) - 1;
        dst[word_idx + 1] = (dst[word_idx + 1] & ~hi_mask) | hi_val;
    }
}

fn ceilDiv(a: usize, b: usize) usize {
    return (a + b - 1) / b;
}

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
    const cx64: i64 = chunk_pos.x;
    const cz64: i64 = chunk_pos.z;
    const biome_idx_chunk: usize = @intCast(splitmix64(seed ^ @as(u64, @bitCast(cx64)) ^ (@as(u64, @bitCast(cz64)) << 1)) % @as(u64, biomes.len));
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

pub fn deinitChunk(allocator: std.mem.Allocator, chunk: *const gs.Chunk) void {
    // free per-section allocations, then the sections slice and entities slice
    for (chunk.sections) |s| {
        allocator.free(s.palette);
        allocator.free(s.blocks_indices_bits);
        allocator.free(s.skylight);
        allocator.free(s.blocklight);
        allocator.free(s.biome_palette);
        allocator.free(s.biome_indices_bits);
        allocator.free(s.block_entities);
    }
    allocator.free(chunk.sections);
    allocator.free(chunk.entities);
}

test "generateChunk determinism and basic structure" {
    const testing = std.testing;
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer {
        const ok = gpa.deinit();
        testing.expect(ok == .ok) catch {};
    }
    const allocator = gpa.allocator();

    const seed: u64 = 0xDEADBEEFCAFEBABE;
    const section_count_y: u16 = 4; // 4*32 = 128 world height
    const pos = gs.ChunkPos{ .x = 0, .z = 0 };
    const cfg = ChunkGenConfig{ .air = 0, .ground = 1 };
    const biomes = [_]ids.BiomeId{ 7 };

    var chunk1 = try generateChunk(allocator, seed, section_count_y, pos, cfg, &biomes);
    defer deinitChunk(allocator, &chunk1);
    var chunk2 = try generateChunk(allocator, seed, section_count_y, pos, cfg, &biomes);
    defer deinitChunk(allocator, &chunk2);

    try testing.expectEqual(@as(usize, section_count_y), chunk1.sections.len);
    try testing.expectEqual(@as(usize, section_count_y), chunk2.sections.len);

    // verify per-section sizes and palette contents
    const voxel_count = constants.chunk_size_x * constants.section_height * constants.chunk_size_z;
    const words_per_section = (voxel_count * 1 + 31) / 32; // bpi=1
    const light_bytes = voxel_count / 2;

    for (chunk1.sections, 0..) |s, i| {
        _ = i;
        try testing.expectEqual(@as(usize, 2), s.palette.len);
        try testing.expectEqual(@as(ids.BlockStateId, 0), s.palette[0]);
        try testing.expectEqual(@as(ids.BlockStateId, 1), s.palette[1]);
        try testing.expectEqual(@as(usize, words_per_section), s.blocks_indices_bits.len);
        try testing.expectEqual(@as(usize, light_bytes), s.skylight.len);
        try testing.expectEqual(@as(usize, light_bytes), s.blocklight.len);
        try testing.expectEqual(@as(usize, 1), s.biome_palette.len);
        try testing.expectEqual(@as(ids.BiomeId, 7), s.biome_palette[0]);
        try testing.expectEqual(@as(usize, 0), s.biome_indices_bits.len);
        try testing.expectEqual(@as(usize, 0), s.block_entities.len);
    }

    // determinism: compare indices_bits and heightmaps between chunk1 and chunk2
    for (0..chunk1.sections.len) |i| {
        const a = chunk1.sections[i].blocks_indices_bits;
        const b = chunk2.sections[i].blocks_indices_bits;
        try testing.expect(std.mem.eql(u32, a, b));
    }
    try testing.expectEqualDeep(chunk1.heightmaps, chunk2.heightmaps);

    // basic voxel check at center column matches heightmap
    const cx: usize = constants.chunk_size_x / 2;
    const cz: usize = constants.chunk_size_z / 2;
    const col_h = @as(u32, @intCast(chunk1.heightmaps.world_surface[cz * constants.chunk_size_x + cx]));
    // helper to read a bit-packed value (bpi=1)
    const getBit = struct {
        fn at(buf: []const u32, idx: usize) u32 {
            const bit_off: usize = idx * 1;
            const word_idx: usize = bit_off >> 5;
            const bit_idx: u5 = @intCast(bit_off & 31);
            return (buf[word_idx] >> bit_idx) & 1;
        }
    };
    // find which section contains y and verify ground/air bits
    for (0..chunk1.sections.len) |sy| {
        const y_base: u32 = @intCast(sy * constants.section_height);
        const s = chunk1.sections[sy];
        for (0..constants.section_height) |ly| {
            const y: u32 = y_base + @as(u32, @intCast(ly));
            const voxel_idx_in_section: usize = ly * constants.chunk_size_x * constants.chunk_size_z + cz * constants.chunk_size_x + cx;
            const bit = getBit.at(s.blocks_indices_bits, voxel_idx_in_section);
            if (y <= col_h) {
                try testing.expectEqual(@as(u32, 1), bit);
            } else {
                try testing.expectEqual(@as(u32, 0), bit);
            }
        }
    }
}

test "generateChunk fails with empty biome list" {
    const testing = std.testing;
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer {
        const ok = gpa.deinit();
        testing.expect(ok == .ok) catch {};
    }
    const allocator = gpa.allocator();
    const seed: u64 = 1;
    const cfg = ChunkGenConfig{ .air = 0, .ground = 1 };
    const pos = gs.ChunkPos{ .x = 0, .z = 0 };

    const res = generateChunk(allocator, seed, 1, pos, cfg, &[_]ids.BiomeId{});
    try testing.expectError(Error.EmptyBiomeList, res);
}
