const std = @import("std");
const ids = @import("ids");
const constants = @import("constants");

pub const Params = struct {
    blocks: []const ids.BlockStateId = &[_]ids.BlockStateId{},
    biomes: []const ids.BiomeId = &[_]ids.BiomeId{},
};

pub const BlockLookup = struct {
    ctx: ?*anyopaque,
    call: *const fn (ctx: ?*anyopaque, name: []const u8) ?ids.BlockStateId,
};

// Shared proto-chunk used during staged worldgen.
pub const ProtoChunk = struct {
    allocator: std.mem.Allocator,
    sections_below: u16,
    sections_above: u16,
    pos: struct { x: i32, z: i32 },
    status: u4 = 0, // gs.ChunkStatus-like; 0==empty
    biomes_buf: ?[]ids.BiomeId = null,
    blocks_buf: ?[]ids.BlockStateId = null,
    tops: []i32, // per-column height tracking

    pub fn totalSections(self: *const ProtoChunk) u16 {
        return self.sections_below + self.sections_above;
    }
};

pub fn totalVoxels(sections_below: u16, sections_above: u16) usize {
    const total: usize = @as(usize, sections_below) + @as(usize, sections_above);
    return total * constants.chunk_size_x * constants.section_height * constants.chunk_size_z;
}

pub fn ensureBiomesAllocated(proto: *ProtoChunk) !void {
    if (proto.biomes_buf == null) {
        const count = totalVoxels(proto.sections_below, proto.sections_above);
        proto.biomes_buf = try proto.allocator.alloc(ids.BiomeId, count);
    }
}

pub fn ensureBlocksAllocated(proto: *ProtoChunk) !void {
    if (proto.blocks_buf == null) {
        const count = totalVoxels(proto.sections_below, proto.sections_above);
        proto.blocks_buf = try proto.allocator.alloc(ids.BlockStateId, count);
    }
}

// Phase hook signatures used by modules.
pub const StructuresStartsFn = *const fn (seed: u64, proto: *ProtoChunk, params: Params) anyerror!void;
pub const StructuresRefsFn = *const fn (seed: u64, proto: *ProtoChunk, params: Params) anyerror!void;
pub const BiomesFn = *const fn (seed: u64, proto: *ProtoChunk, params: Params) anyerror!void;
pub const NoiseFn = *const fn (seed: u64, proto: *ProtoChunk, params: Params, lookup: BlockLookup) anyerror!void;
pub const SurfaceFn = *const fn (seed: u64, proto: *ProtoChunk, params: Params) anyerror!void;
pub const CarversFn = *const fn (seed: u64, proto: *ProtoChunk, params: Params) anyerror!void;
pub const FeaturesFn = *const fn (seed: u64, proto: *ProtoChunk, params: Params) anyerror!void;
pub const InitLightFn = *const fn (seed: u64, proto: *ProtoChunk, params: Params) anyerror!void;
pub const LightFn = *const fn (seed: u64, proto: *ProtoChunk, params: Params) anyerror!void;
pub const SpawnFn = *const fn (seed: u64, proto: *ProtoChunk, params: Params) anyerror!void;
