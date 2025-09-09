const std = @import("std");
const gs = @import("gs");
const ids = @import("ids");
const constants = @import("constants");
const wgen = @import("registry/worldgen.zig");
const wapi = @import("worldgen_api.zig");

fn totalVoxels(sections_below: u16, sections_above: u16) usize {
    return wapi.totalVoxels(sections_below, sections_above);
}

pub fn protoInit(allocator: std.mem.Allocator, sections_below: u16, sections_above: u16, pos: gs.ChunkPos) !wapi.ProtoChunk {
    const tops_len: usize = constants.chunk_size_x * constants.chunk_size_z;
    const tops = try allocator.alloc(i32, tops_len);
    @memset(tops, -1);
    return .{ .allocator = allocator, .sections_below = sections_below, .sections_above = sections_above, .pos = .{ .x = pos.x, .z = pos.z }, .tops = tops };
}

pub fn protoDeinit(proto: *wapi.ProtoChunk) void {
    if (proto.biomes_buf) |b| proto.allocator.free(b);
    if (proto.blocks_buf) |b2| proto.allocator.free(b2);
    proto.allocator.free(proto.tops);
    proto.* = undefined;
}

fn callBiomesHook(proto: *wapi.ProtoChunk, seed: u64, def: wgen.Def, params: wgen.Params) !void {
    if (@as(gs.ChunkStatus, @enumFromInt(proto.status)) != .empty) return;
    try wapi.ensureBiomesAllocated(proto);
    if (def.biomes) |f| {
        try f(seed, proto, params);
    }
    proto.status = @intFromEnum(gs.ChunkStatus.biomes);
}

fn callNoiseHook(proto: *wapi.ProtoChunk, seed: u64, def: wgen.Def, params: wgen.Params, lookup: wgen.BlockLookup) !void {
    if (@as(gs.ChunkStatus, @enumFromInt(proto.status)) != .biomes) return;
    try wapi.ensureBlocksAllocated(proto);
    @memset(proto.tops, -1);
    if (def.noise) |f| {
        try f(seed, proto, params, lookup);
    }
    proto.status = @intFromEnum(gs.ChunkStatus.noise);
}

fn advanceNoop(proto: *wapi.ProtoChunk, next: gs.ChunkStatus) void {
    proto.status = @intFromEnum(next);
}

pub fn advanceOnePhase(
    proto: *wapi.ProtoChunk,
    seed: u64,
    def: wgen.Def,
    params: wgen.Params,
    lookup: wgen.BlockLookup,
    air_id: ids.BlockStateId,
) !?gs.Chunk {
    _ = air_id; // not used by hook APIs directly; blocks should be set explicitly by noise/surface hooks
    switch (@as(gs.ChunkStatus, @enumFromInt(proto.status))) {
        .empty => {
            try callBiomesHook(proto, seed, def, params);
            return null;
        },
        .biomes => {
            try callNoiseHook(proto, seed, def, params, lookup);
            return null;
        },
        .noise => {
            advanceNoop(proto, .surface);
            return null;
        },
        .surface => {
            if (def.surface) |f| _ = f(seed, proto, params) catch {};
            advanceNoop(proto, .carvers);
            return null;
        },
        .carvers => {
            if (def.carvers) |f| _ = f(seed, proto, params) catch {};
            advanceNoop(proto, .features);
            return null;
        },
        .features => {
            if (def.features) |f| _ = f(seed, proto, params) catch {};
            advanceNoop(proto, .initialize_light);
            return null;
        },
        .initialize_light => {
            if (def.initialize_light) |f| _ = f(seed, proto, params) catch {};
            advanceNoop(proto, .light);
            return null;
        },
        .light => {
            if (def.light) |f| _ = f(seed, proto, params) catch {};
            advanceNoop(proto, .spawn);
            return null;
        },
        .spawn => {
            if (def.spawn) |f| _ = f(seed, proto, params) catch {};
            const ch = try buildChunkFromBuffers(proto);
            proto.status = @intFromEnum(gs.ChunkStatus.full);
            return ch;
        },
        .structures_starts, .structures_references => {
            advanceNoop(proto, .biomes);
            return null;
        },
        .full => return null,
    }
}

fn buildChunkFromBuffers(proto: *const wapi.ProtoChunk) !gs.Chunk {
    if (proto.blocks_buf == null or proto.biomes_buf == null) return error.WorldgenNotReady;

    const allocator = proto.allocator;
    const sections_len: usize = @intCast(proto.totalSections());
    var sections = try allocator.alloc(gs.Section, sections_len);

    const voxel_count: usize = constants.chunk_size_x * constants.section_height * constants.chunk_size_z;

    // build sections from buffers
    var idx_global: usize = 0;
    var sy: usize = 0;
    while (sy < sections_len) : (sy += 1) {
        // Build palettes for this section
        var block_map = std.AutoHashMap(ids.BlockStateId, u32).init(allocator);
        defer block_map.deinit();
        var biome_map = std.AutoHashMap(ids.BiomeId, u32).init(allocator);
        defer biome_map.deinit();

        var block_palette_list = try std.ArrayList(ids.BlockStateId).initCapacity(allocator, 0);
        defer block_palette_list.deinit(allocator);
        var biome_palette_list = try std.ArrayList(ids.BiomeId).initCapacity(allocator, 0);
        defer biome_palette_list.deinit(allocator);

        // First pass: collect unique values
        var i: usize = 0;
        while (i < voxel_count) : (i += 1) {
            const bid = proto.blocks_buf.?[idx_global + i];
            if (block_map.get(bid) == null) {
                const pi: u32 = @intCast(block_palette_list.items.len);
                try block_palette_list.append(allocator, bid);
                try block_map.put(bid, pi);
            }
            const bm = proto.biomes_buf.?[idx_global + i];
            if (biome_map.get(bm) == null) {
                const pi2: u32 = @intCast(biome_palette_list.items.len);
                try biome_palette_list.append(allocator, bm);
                try biome_map.put(bm, pi2);
            }
        }

        // Allocate outputs for section
        const bpi: u6 = bitsFor(block_palette_list.items.len);
        const words_per_section: usize = ceilDiv(voxel_count * bpi, 32);
        const light_bytes_per_section: usize = voxel_count / 2;

        const palette = try allocator.dupe(ids.BlockStateId, block_palette_list.items);
        const indices_bits = try allocator.alloc(u32, words_per_section);
        @memset(indices_bits, 0);
        const skylight = try allocator.alloc(u8, light_bytes_per_section);
        const blocklight = try allocator.alloc(u8, light_bytes_per_section);
        @memset(skylight, 0);
        @memset(blocklight, 0);

        const biome_palette = try allocator.dupe(ids.BiomeId, biome_palette_list.items);
        const biome_bpi: u6 = bitsFor(biome_palette_list.items.len);
        const biome_words: usize = ceilDiv(voxel_count * biome_bpi, 32);
        const biome_indices_bits = try allocator.alloc(u32, biome_words);
        @memset(biome_indices_bits, 0);
        const block_entities = try allocator.alloc(gs.BlockEntityRecord, 0);

        // Second pass: pack indices
        i = 0;
        while (i < voxel_count) : (i += 1) {
            const bid = proto.blocks_buf.?[idx_global + i];
            const pid = block_map.get(bid).?;
            packBitsSet(indices_bits, i, bpi, pid);
            const bm = proto.biomes_buf.?[idx_global + i];
            const bpi2 = biome_map.get(bm).?;
            packBitsSet(biome_indices_bits, i, biome_bpi, bpi2);
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

        idx_global += voxel_count;
    }

    // Trim trailing all-air sections (palette == [0])
    var new_len: usize = sections.len;
    while (new_len > 0) {
        const last = sections[new_len - 1];
        if (last.palette.len == 1 and last.palette[0] == 0 and last.block_entities.len == 0) {
            // free this section's memory and shrink
            allocator.free(last.palette);
            allocator.free(last.blocks_indices_bits);
            allocator.free(last.skylight);
            allocator.free(last.blocklight);
            allocator.free(last.biome_palette);
            allocator.free(last.biome_indices_bits);
            allocator.free(last.block_entities);
            new_len -= 1;
        } else break;
    }
    if (new_len < sections.len) {
        var trimmed = try allocator.alloc(gs.Section, new_len);
        @memcpy(trimmed[0..new_len], sections[0..new_len]);
        allocator.free(sections);
        sections = trimmed;
    }

    // compute heightmaps from proto.tops
    var heightmaps: gs.Heightmaps = .{ .motion_blocking = undefined, .world_surface = undefined };
    for (0..constants.chunk_size_z) |lz| {
        for (0..constants.chunk_size_x) |lx| {
            const col_idx = lz * constants.chunk_size_x + lx;
            heightmaps.motion_blocking[col_idx] = proto.tops[col_idx];
            heightmaps.world_surface[col_idx] = proto.tops[col_idx];
        }
    }

    const entities = try allocator.alloc(gs.EntityRecord, 0);
    return .{ .pos = .{ .x = proto.pos.x, .z = proto.pos.z }, .sections = sections, .heightmaps = heightmaps, .entities = entities };
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

fn bitsFor(n: usize) u6 {
    if (n <= 1) return 1;
    var v: usize = n - 1;
    var b: u6 = 0;
    while (v > 0) : (v >>= 1) b += 1;
    return if (b == 0) 1 else b;
}

pub fn generateChunk(
    allocator: std.mem.Allocator,
    section_count_y: u16,
    chunk_pos: gs.ChunkPos,
    seed: u64,
    def: wgen.Def,
    params: wgen.Params,
    lookup: wgen.BlockLookup,
) !gs.Chunk {
    // Back-compat wrapper: interpret section_count_y as sections above origin, with 0 below
    var proto = try protoInit(allocator, 0, section_count_y, chunk_pos);
    defer protoDeinit(&proto);

    try callBiomesHook(&proto, seed, def, params);
    try callNoiseHook(&proto, seed, def, params, lookup);
    advanceNoop(&proto, .surface);
    advanceNoop(&proto, .carvers);
    advanceNoop(&proto, .features);
    advanceNoop(&proto, .initialize_light);
    advanceNoop(&proto, .light);
    advanceNoop(&proto, .spawn);

    const chunk = try buildChunkFromBuffers(&proto);
    proto.status = @intFromEnum(gs.ChunkStatus.full);
    return chunk;
}

const testing = std.testing;

test "worldgen: void generator produces all-air sections and -1 heightmap" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer {
        const ok = gpa.deinit();
        testing.expect(ok == .ok) catch {};
    }
    const allocator = gpa.allocator();

    // Selectors: single biome, always air blocks
    const selBiome = struct {
        fn call(seed: u64, pos: gs.BlockPos, params: wgen.Params) ids.BiomeId {
            _ = seed;
            _ = pos;
            return if (params.biomes.len > 0) params.biomes[0] else 0;
        }
    }.call;

    const selBlock = struct {
        fn call(seed: u64, biome: ids.BiomeId, pos: gs.BlockPos, params: wgen.Params, lookup: wgen.BlockLookup) ids.BlockStateId {
            _ = seed;
            _ = biome;
            _ = pos;
            _ = params;
            _ = lookup;
            return 0; // air
        }
    }.call;

    const air_id: ids.BlockStateId = 0;
    const biome_id: ids.BiomeId = 7;
    const params: wgen.Params = .{ .blocks = &[_]ids.BlockStateId{}, .biomes = &[_]ids.BiomeId{biome_id} };
    const dummy_lookup: wgen.BlockLookup = .{ .ctx = null, .call = undefined };

    var chunk = try generateChunk(allocator, 2, .{ .x = 0, .z = 0 }, 42, selBiome, selBlock, params, dummy_lookup, air_id);
    defer deinitChunk(allocator, &chunk);

    // All sections should have only air in the palette
    for (chunk.sections) |s| {
        try testing.expectEqual(@as(usize, 1), s.palette.len);
        try testing.expectEqual(air_id, s.palette[0]);
        try testing.expectEqual(@as(usize, 1), s.biome_palette.len);
        try testing.expectEqual(biome_id, s.biome_palette[0]);
    }

    // Heightmaps should be -1 everywhere (no solid blocks)
    for (chunk.heightmaps.motion_blocking) |h| try testing.expectEqual(@as(i32, -1), h);
    for (chunk.heightmaps.world_surface) |h| try testing.expectEqual(@as(i32, -1), h);
}

test "worldgen: flat surface at y<=2 yields top height 2 and two-block palette" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer {
        const ok = gpa.deinit();
        testing.expect(ok == .ok) catch {};
    }
    const allocator = gpa.allocator();

    const selBiome = struct {
        fn call(seed: u64, pos: gs.BlockPos, params: wgen.Params) ids.BiomeId {
            _ = seed;
            _ = pos;
            return if (params.biomes.len > 0) params.biomes[0] else 0;
        }
    }.call;

    const selBlock = struct {
        fn call(seed: u64, biome: ids.BiomeId, pos: gs.BlockPos, params: wgen.Params, lookup: wgen.BlockLookup) ids.BlockStateId {
            _ = seed;
            _ = biome;
            _ = lookup;
            const air: ids.BlockStateId = 0;
            const surface: ids.BlockStateId = if (params.blocks.len >= 1) params.blocks[0] else 2;
            if (pos.y <= 2) return surface;
            return air;
        }
    }.call;

    const air_id: ids.BlockStateId = 0;
    const surface_id: ids.BlockStateId = 2;
    const biome_id: ids.BiomeId = 3;
    const params: wgen.Params = .{ .blocks = &[_]ids.BlockStateId{surface_id}, .biomes = &[_]ids.BiomeId{biome_id} };
    const dummy_lookup: wgen.BlockLookup = .{ .ctx = null, .call = undefined };

    var chunk = try generateChunk(allocator, 1, .{ .x = 0, .z = 0 }, 99, selBiome, selBlock, params, dummy_lookup, air_id);
    defer deinitChunk(allocator, &chunk);

    // Each section should have exactly two block states: air and surface (order not guaranteed)
    for (chunk.sections) |s| {
        try testing.expectEqual(@as(usize, 2), s.palette.len);
        var saw_air = false;
        var saw_surface = false;
        for (s.palette) |bid| {
            if (bid == air_id) saw_air = true;
            if (bid == surface_id) saw_surface = true;
        }
        try testing.expect(saw_air);
        try testing.expect(saw_surface);
        try testing.expectEqual(@as(usize, 1), s.biome_palette.len);
        try testing.expectEqual(biome_id, s.biome_palette[0]);
    }

    // Heightmaps top should be 2 everywhere (surface layer)
    for (chunk.heightmaps.motion_blocking) |h| try testing.expectEqual(@as(i32, 2), h);
    for (chunk.heightmaps.world_surface) |h| try testing.expectEqual(@as(i32, 2), h);
}

test "worldgen: alternating biomes across columns produce two-biome palette and mixed surface blocks" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer {
        const ok = gpa.deinit();
        testing.expect(ok == .ok) catch {};
    }
    const allocator = gpa.allocator();

    const biome_a: ids.BiomeId = 5;
    const biome_b: ids.BiomeId = 6;
    const surface_a: ids.BlockStateId = 10;
    const surface_b: ids.BlockStateId = 11;
    const air_id: ids.BlockStateId = 0;

    const selBiome = struct {
        fn call(seed: u64, pos: gs.BlockPos, params: wgen.Params) ids.BiomeId {
            _ = seed;
            const alt: usize = @intCast(@as(i64, pos.x) + @as(i64, pos.z) & 1);
            // require two biomes provided
            if (params.biomes.len >= 2) return params.biomes[alt];
            return 0;
        }
    }.call;

    const selBlock = struct {
        fn call(seed: u64, biome: ids.BiomeId, pos: gs.BlockPos, params: wgen.Params, lookup: wgen.BlockLookup) ids.BlockStateId {
            _ = seed;
            _ = lookup;
            if (pos.y == 0) {
                // choose surface block by biome
                if (params.blocks.len >= 2) {
                    return if (biome == params.biomes[0]) params.blocks[0] else params.blocks[1];
                }
                return if (biome == 0) 2 else 3;
            }
            return 0; // air elsewhere
        }
    }.call;

    const params: wgen.Params = .{ .blocks = &[_]ids.BlockStateId{ surface_a, surface_b }, .biomes = &[_]ids.BiomeId{ biome_a, biome_b } };
    const dummy_lookup: wgen.BlockLookup = .{ .ctx = null, .call = undefined };

    var chunk = try generateChunk(allocator, 1, .{ .x = 0, .z = 0 }, 12345, selBiome, selBlock, params, dummy_lookup, air_id);
    defer deinitChunk(allocator, &chunk);

    try testing.expectEqual(@as(usize, 1), chunk.sections.len);
    const s = chunk.sections[0];

    // Expect air + two surface blocks in palette (order agnostic)
    try testing.expectEqual(@as(usize, 3), s.palette.len);
    var saw_air = false;
    var saw_a = false;
    var saw_b = false;
    for (s.palette) |bid| {
        if (bid == air_id) saw_air = true;
        if (bid == surface_a) saw_a = true;
        if (bid == surface_b) saw_b = true;
    }
    try testing.expect(saw_air);
    try testing.expect(saw_a);
    try testing.expect(saw_b);

    // Biome palette should have both biomes
    try testing.expectEqual(@as(usize, 2), s.biome_palette.len);
    var saw_ba = false;
    var saw_bb = false;
    for (s.biome_palette) |bm| {
        if (bm == biome_a) saw_ba = true;
        if (bm == biome_b) saw_bb = true;
    }
    try testing.expect(saw_ba);
    try testing.expect(saw_bb);

    // Heightmaps should be 0 everywhere (top solid at y==0)
    for (chunk.heightmaps.motion_blocking) |h| try testing.expectEqual(@as(i32, 0), h);
    for (chunk.heightmaps.world_surface) |h| try testing.expectEqual(@as(i32, 0), h);
}
