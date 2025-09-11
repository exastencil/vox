// Client background mesher, snapshot and cache management
const std = @import("std");
const constants = @import("constants");
const gs = @import("gs");
const ids = @import("ids");
const block = @import("../registry/block.zig");
const simulation = @import("../simulation.zig");
const uvmap = @import("../uvmap.zig");
const client_atlas_builder_mod = @import("atlas_builder.zig");

// Shared CPU vertex type for meshing (must match client renderer layout)
pub const Vertex = struct {
    pos: [3]f32,
    uv: [2]f32,
    rect_min: [2]f32,
    rect_size: [2]f32,
    layer: f32,
    apply_tint: f32,
};

pub const SectionDraw = struct { chunk_pos: gs.ChunkPos, sy: u16, first: u32, count: u32 };

pub const MeshingJob = struct { rpos: simulation.RegionPos };
pub const MeshingResult = struct { rpos: simulation.RegionPos, built_chunk_count: usize, verts: []Vertex, draws: []SectionDraw };

// Snapshot types used for background meshing (owned, outside world lock)
pub const SectionSnapshot = struct {
    palette: []gs.BlockState,
    blocks_indices_bits: []u32,
};

pub const ChunkSnapshot = struct {
    pos: gs.ChunkPos,
    sections: []SectionSnapshot,
};

pub const RegionSnapshot = struct {
    rpos: simulation.RegionPos,
    base_cx: i32,
    base_cz: i32,
    section_count_y: u16,
    sections_below: u16,
    chunks: []ChunkSnapshot,
    grid_map: []i32, // 32x32 mapping local (x,z) -> chunk index or -1
};

// Temporary slice-ref structs for capturing under lock, then copying outside the lock
pub const SectionSliceRefs = struct { pal_ptr: [*]const gs.BlockState, pal_len: usize, bits_ptr: [*]const u32, bits_len: usize };
pub const ChunkSliceRefs = struct { pos: gs.ChunkPos, sections: []SectionSliceRefs };
pub const RegionSliceRefs = struct { rpos: simulation.RegionPos, section_count_y: u16, sections_below: u16, base_cx: i32, base_cz: i32, chunks: []ChunkSliceRefs };

fn bitsFor(n: usize) u6 {
    if (n <= 1) return 1;
    var v: usize = n - 1;
    var b: u6 = 0;
    while (v > 0) : (v >>= 1) b += 1;
    return if (b == 0) 1 else b;
}
fn unpackBitsGet(src: []const u32, idx: usize, bits_per_index: u6) u32 {
    const bit_off: usize = idx * @as(usize, bits_per_index);
    const word_idx: usize = bit_off >> 5;
    const bit_idx: u5 = @intCast(bit_off & 31);
    if (bits_per_index == 32) return src[word_idx];
    const mask: u32 = (@as(u32, 1) << @intCast(bits_per_index)) - 1;
    const rem: u6 = @as(u6, 32) - @as(u6, bit_idx);
    if (bits_per_index <= rem) {
        return (src[word_idx] >> bit_idx) & mask;
    } else {
        const lo_bits: u6 = rem;
        const hi_bits: u6 = @intCast(bits_per_index - lo_bits);
        const lo_mask: u32 = (@as(u32, 1) << @as(u5, @intCast(lo_bits))) - 1;
        const lo_val: u32 = (src[word_idx] >> bit_idx) & lo_mask;
        const hi_val: u32 = (src[word_idx + 1] & ((@as(u32, 1) << @as(u5, @intCast(hi_bits))) - 1)) << @as(u5, @intCast(lo_bits));
        return lo_val | hi_val;
    }
}

inline fn worldToRelativeFace(dir: ids.Direction, world_face: u3) block.Face {
    const BF = block.Face;
    const bf_to_wf: [6]u3 = switch (dir) {
        .up => .{ 5, 4, 0, 1, 3, 2 },
        .down => .{ 5, 4, 1, 0, 2, 3 },
        .front => .{ 3, 2, 0, 1, 5, 4 },
        .back => .{ 3, 2, 1, 0, 4, 5 },
        .right => .{ 5, 4, 3, 2, 1, 0 },
        .left => .{ 5, 4, 2, 3, 0, 1 },
    };
    var i: u3 = 0;
    while (i < 6) : (i += 1) {
        if (bf_to_wf[i] == world_face) return @enumFromInt(i);
    }
    return BF.up;
}

inline fn emitQuad(allocator: std.mem.Allocator, list: *std.ArrayList(Vertex), v0: [3]f32, v1: [3]f32, v2: [3]f32, v3: [3]f32, uv0: [2]f32, uv1: [2]f32, uv2: [2]f32, uv3: [2]f32, rect_min: [2]f32, rect_size: [2]f32, layer: f32, apply_tint: f32) void {
    list.append(allocator, .{ .pos = v0, .uv = uv0, .rect_min = rect_min, .rect_size = rect_size, .layer = layer, .apply_tint = apply_tint }) catch return;
    list.append(allocator, .{ .pos = v1, .uv = uv1, .rect_min = rect_min, .rect_size = rect_size, .layer = layer, .apply_tint = apply_tint }) catch return;
    list.append(allocator, .{ .pos = v2, .uv = uv2, .rect_min = rect_min, .rect_size = rect_size, .layer = layer, .apply_tint = apply_tint }) catch return;
    list.append(allocator, .{ .pos = v0, .uv = uv0, .rect_min = rect_min, .rect_size = rect_size, .layer = layer, .apply_tint = apply_tint }) catch return;
    list.append(allocator, .{ .pos = v2, .uv = uv2, .rect_min = rect_min, .rect_size = rect_size, .layer = layer, .apply_tint = apply_tint }) catch return;
    list.append(allocator, .{ .pos = v3, .uv = uv3, .rect_min = rect_min, .rect_size = rect_size, .layer = layer, .apply_tint = apply_tint }) catch return;
}

pub fn buildRegionRefsUnderLock(self: anytype, alloc: std.mem.Allocator, ws: *const simulation.WorldState, rs: *const simulation.RegionState, rpos: simulation.RegionPos) RegionSliceRefs {
    _ = self;
    var refs: RegionSliceRefs = .{ .rpos = rpos, .section_count_y = ws.sections_below + ws.sections_above, .sections_below = ws.sections_below, .base_cx = rpos.x * 32, .base_cz = rpos.z * 32, .chunks = &[_]ChunkSliceRefs{} };
    const count: usize = rs.chunks.items.len;
    if (count == 0) return refs;
    var chunks = alloc.alloc(ChunkSliceRefs, count) catch return refs;
    var i: usize = 0;
    while (i < count) : (i += 1) {
        const ch = rs.chunks.items[i];
        const sec_count = ch.sections.len;
        var secs: []SectionSliceRefs = &[_]SectionSliceRefs{};
        if (sec_count > 0) secs = alloc.alloc(SectionSliceRefs, sec_count) catch &[_]SectionSliceRefs{};
        var sy: usize = 0;
        while (sy < secs.len) : (sy += 1) {
            const s = ch.sections[sy];
            secs[sy] = .{ .pal_ptr = s.palette.ptr, .pal_len = s.palette.len, .bits_ptr = s.blocks_indices_bits.ptr, .bits_len = s.blocks_indices_bits.len };
        }
        chunks[i] = .{ .pos = ch.pos, .sections = secs };
    }
    refs.chunks = chunks;
    return refs;
}

pub fn freeRegionRefs(self: anytype, alloc: std.mem.Allocator, refs: *RegionSliceRefs) void {
    _ = self;
    var i: usize = 0;
    while (i < refs.chunks.len) : (i += 1) {
        const ch = refs.chunks[i];
        if (ch.sections.len > 0) alloc.free(ch.sections);
    }
    if (refs.chunks.len > 0) alloc.free(refs.chunks);
    refs.* = .{ .rpos = refs.rpos, .section_count_y = refs.section_count_y, .sections_below = refs.sections_below, .base_cx = refs.base_cx, .base_cz = refs.base_cz, .chunks = &[_]ChunkSliceRefs{} };
}

pub fn copySnapshotFromRefs(self: anytype, alloc: std.mem.Allocator, refs: *const RegionSliceRefs) RegionSnapshot {
    _ = self;
    var snap: RegionSnapshot = .{ .rpos = refs.rpos, .base_cx = refs.base_cx, .base_cz = refs.base_cz, .section_count_y = refs.section_count_y, .sections_below = refs.sections_below, .chunks = &[_]ChunkSnapshot{}, .grid_map = &[_]i32{} };
    const count: usize = refs.chunks.len;
    if (count == 0) return snap;
    var chunks = alloc.alloc(ChunkSnapshot, count) catch return snap;
    var grid = alloc.alloc(i32, 32 * 32) catch {
        alloc.free(chunks);
        return snap;
    };
    var gi: usize = 0;
    while (gi < grid.len) : (gi += 1) grid[gi] = -1;
    var i: usize = 0;
    while (i < count) : (i += 1) {
        const chref = refs.chunks[i];
        const pos = chref.pos;
        const lx: i32 = pos.x - snap.base_cx;
        const lz: i32 = pos.z - snap.base_cz;
        if (!(lx >= 0 and lx < 32 and lz >= 0 and lz < 32)) {
            chunks[i] = .{ .pos = pos, .sections = &[_]SectionSnapshot{} };
            continue;
        }
        const m_idx: usize = @intCast(@as(usize, @intCast(lz)) * 32 + @as(usize, @intCast(lx)));
        grid[m_idx] = @intCast(i);
        var secs: []SectionSnapshot = &[_]SectionSnapshot{};
        if (chref.sections.len > 0) secs = alloc.alloc(SectionSnapshot, chref.sections.len) catch &[_]SectionSnapshot{};
        var sy: usize = 0;
        while (sy < secs.len) : (sy += 1) {
            const sref = chref.sections[sy];
            var pal: []gs.BlockState = &[_]gs.BlockState{};
            var bits: []u32 = &[_]u32{};
            if (sref.pal_len > 0) {
                pal = alloc.alloc(gs.BlockState, sref.pal_len) catch &[_]gs.BlockState{};
                if (pal.len == sref.pal_len) @memcpy(pal[0..pal.len], sref.pal_ptr[0..sref.pal_len]);
            }
            if (sref.bits_len > 0) {
                bits = alloc.alloc(u32, sref.bits_len) catch &[_]u32{};
                if (bits.len == sref.bits_len) @memcpy(bits[0..bits.len], sref.bits_ptr[0..sref.bits_len]);
            }
            secs[sy] = .{ .palette = pal, .blocks_indices_bits = bits };
        }
        chunks[i] = .{ .pos = pos, .sections = secs };
    }
    snap.chunks = chunks;
    snap.grid_map = grid;
    return snap;
}

pub fn freeRegionSnapshot(self: anytype, alloc: std.mem.Allocator, snap: *RegionSnapshot) void {
    _ = self;
    var i: usize = 0;
    while (i < snap.chunks.len) : (i += 1) {
        const ch = snap.chunks[i];
        var sy: usize = 0;
        while (sy < ch.sections.len) : (sy += 1) {
            const s = ch.sections[sy];
            if (s.palette.len > 0) alloc.free(s.palette);
            if (s.blocks_indices_bits.len > 0) alloc.free(s.blocks_indices_bits);
        }
        if (ch.sections.len > 0) alloc.free(ch.sections);
    }
    if (snap.chunks.len > 0) alloc.free(snap.chunks);
    if (snap.grid_map.len > 0) alloc.free(snap.grid_map);
    snap.* = .{ .rpos = snap.rpos, .base_cx = snap.base_cx, .base_cz = snap.base_cz, .section_count_y = snap.section_count_y, .sections_below = snap.sections_below, .chunks = &[_]ChunkSnapshot{}, .grid_map = &[_]i32{} };
}

pub fn snapGetChunkIndex(snap: *const RegionSnapshot, cx: i32, cz: i32) ?usize {
    const lx: i32 = cx - snap.base_cx;
    const lz: i32 = cz - snap.base_cz;
    if (!(lx >= 0 and lx < 32 and lz >= 0 and lz < 32)) return null;
    const idx: usize = @intCast(@as(usize, @intCast(lz)) * 32 + @as(usize, @intCast(lx)));
    const val: i32 = snap.grid_map[idx];
    if (val < 0) return null;
    return @intCast(val);
}

pub fn isSolidAtSnapshot(self: anytype, snap: *const RegionSnapshot, start_cx: i32, start_cz: i32, lx_in: i32, abs_y_in: i32, lz_in: i32) bool {
    var lx = lx_in;
    var lz = lz_in;
    var cx = start_cx;
    var cz = start_cz;
    if (lx < 0) {
        cx -= 1;
        lx += @as(i32, @intCast(constants.chunk_size_x));
    } else if (lx >= @as(i32, @intCast(constants.chunk_size_x))) {
        cx += 1;
        lx -= @as(i32, @intCast(constants.chunk_size_x));
    }
    if (lz < 0) {
        cz -= 1;
        lz += @as(i32, @intCast(constants.chunk_size_z));
    } else if (lz >= @as(i32, @intCast(constants.chunk_size_z))) {
        cz += 1;
        lz -= @as(i32, @intCast(constants.chunk_size_z));
    }
    const idx_opt = snapGetChunkIndex(snap, cx, cz);
    // Treat neighbors outside the captured snapshot as non-solid so faces at the
    // snapshot boundary are rendered instead of culled.
    if (idx_opt == null) return false;
    const ch = snap.chunks[idx_opt.?];
    const total_y: i32 = @as(i32, @intCast(snap.section_count_y)) * @as(i32, @intCast(constants.section_height));
    const ny0 = abs_y_in + @as(i32, @intCast(snap.sections_below)) * @as(i32, @intCast(constants.section_height));
    if (ny0 < 0 or ny0 >= total_y) return false;
    const sy: usize = @intCast(@divTrunc(ny0, @as(i32, @intCast(constants.section_height))));
    const ly: usize = @intCast(@mod(ny0, @as(i32, @intCast(constants.section_height))));
    if (sy >= ch.sections.len) return false;
    const s = ch.sections[sy];
    const bpi: u6 = bitsFor(s.palette.len);
    const idx3d: usize = @as(usize, @intCast(lz)) * (constants.chunk_size_x * constants.section_height) + @as(usize, @intCast(lx)) * constants.section_height + ly;
    const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, idx3d, bpi));
    if (pidx >= s.palette.len) return false;
    const bid: u32 = s.palette[pidx].block_id;
    if (bid == 0) return false;
    if (bid < self.sim.reg.blocks.items.len) return self.sim.reg.blocks.items[@intCast(bid)].collision;
    return true;
}

pub fn buildSectionVerticesFromSnapshot(self: anytype, snap: *const RegionSnapshot, ch: *const ChunkSnapshot, sy: usize, base_x: f32, base_z: f32, out: *std.ArrayList(Vertex), alloc: std.mem.Allocator) void {
    const s = ch.sections[sy];
    const pal_len = s.palette.len;
    const bpi: u6 = bitsFor(pal_len);
    // Build per-palette info (draw, solid, per-face UV/tint)
    const PalInfo = struct {
        draw: bool,
        solid: bool,
        top_uv: client_atlas_builder_mod.AtlasUv,
        top_apply_tint: bool,
        bot_uv: client_atlas_builder_mod.AtlasUv,
        bot_apply_tint: bool,
        nx_uv: client_atlas_builder_mod.AtlasUv,
        nx_apply_tint: bool,
        px_uv: client_atlas_builder_mod.AtlasUv,
        px_apply_tint: bool,
        nz_uv: client_atlas_builder_mod.AtlasUv,
        nz_apply_tint: bool,
        pz_uv: client_atlas_builder_mod.AtlasUv,
        pz_apply_tint: bool,
    };
    var pal_info = std.ArrayList(PalInfo).initCapacity(alloc, pal_len) catch {
        return;
    };
    defer pal_info.deinit(alloc);
    var pi: usize = 0;
    while (pi < pal_len) : (pi += 1) {
        const bid: u32 = s.palette[pi].block_id;
        var pinfo: PalInfo = .{
            .draw = true,
            .solid = true,
            .top_uv = .{ .aidx = 0, .u0 = 0, .v0 = 0, .u1 = 1, .v1 = 1 },
            .top_apply_tint = false,
            .bot_uv = .{ .aidx = 0, .u0 = 0, .v0 = 0, .u1 = 1, .v1 = 1 },
            .bot_apply_tint = false,
            .nx_uv = .{ .aidx = 0, .u0 = 0, .v0 = 0, .u1 = 1, .v1 = 1 },
            .nx_apply_tint = false,
            .px_uv = .{ .aidx = 0, .u0 = 0, .v0 = 0, .u1 = 1, .v1 = 1 },
            .px_apply_tint = false,
            .nz_uv = .{ .aidx = 0, .u0 = 0, .v0 = 0, .u1 = 1, .v1 = 1 },
            .nz_apply_tint = false,
            .pz_uv = .{ .aidx = 0, .u0 = 0, .v0 = 0, .u1 = 1, .v1 = 1 },
            .pz_apply_tint = false,
        };
        if (bid < self.sim.reg.blocks.items.len) pinfo.solid = self.sim.reg.blocks.items[@intCast(bid)].collision;
        if (bid == 0) {
            pinfo.draw = false;
            pinfo.solid = false;
        } else {
            const def = self.sim.reg.blocks.items[@intCast(bid)];
            const state = s.palette[pi];
            inline for (.{ 0, 1, 2, 3, 4, 5 }) |wf| {
                const rel: block.Face = worldToRelativeFace(state.direction, @intCast(wf));
                const fidx: usize = @intCast(def.face_config_index[@intFromEnum(rel)]);
                if (fidx < def.face_configs.len) {
                    const fc = def.face_configs[fidx];
                    if (fc.layers.len > 0) switch (fc.layers[0]) {
                        .Texture => |t| {
                            if (self.atlas_uvs_by_id.get(t.tex)) |uv| {
                                switch (wf) {
                                    3 => {
                                        pinfo.top_uv = uv;
                                        pinfo.top_apply_tint = (t.tint != null);
                                    },
                                    2 => {
                                        pinfo.bot_uv = uv;
                                        pinfo.bot_apply_tint = (t.tint != null);
                                    },
                                    0 => {
                                        pinfo.nx_uv = uv;
                                        pinfo.nx_apply_tint = (t.tint != null);
                                    },
                                    1 => {
                                        pinfo.px_uv = uv;
                                        pinfo.px_apply_tint = (t.tint != null);
                                    },
                                    4 => {
                                        pinfo.nz_uv = uv;
                                        pinfo.nz_apply_tint = (t.tint != null);
                                    },
                                    5 => {
                                        pinfo.pz_uv = uv;
                                        pinfo.pz_apply_tint = (t.tint != null);
                                    },
                                    else => {},
                                }
                            }
                        },
                    };
                }
            }
        }
        pal_info.appendAssumeCapacity(pinfo);
    }

    const MaskCell = struct { present: bool, uv: @TypeOf(pal_info.items[0].top_uv), tint: bool };
    const section_base_y: i32 = @as(i32, @intCast(sy * constants.section_height)) - @as(i32, @intCast(snap.sections_below)) * @as(i32, @intCast(constants.section_height));

    // +Y faces per Y-slice
    var ly_top: usize = 0;
    while (ly_top < constants.section_height) : (ly_top += 1) {
        const abs_y: i32 = section_base_y + @as(i32, @intCast(ly_top));
        var mask: [constants.chunk_size_z * constants.chunk_size_x]MaskCell = undefined;
        var mz: usize = 0;
        while (mz < constants.chunk_size_z) : (mz += 1) {
            var mx: usize = 0;
            while (mx < constants.chunk_size_x) : (mx += 1) {
                const idxm: usize = mz * constants.chunk_size_x + mx;
                mask[idxm].present = false;
                const src_idx: usize = mz * (constants.chunk_size_x * constants.section_height) + mx * constants.section_height + ly_top;
                const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, src_idx, bpi));
                if (pidx >= pal_info.items.len) continue;
                const info = pal_info.items[pidx];
                if (!info.draw) continue;
                if (!isSolidAtSnapshot(self, snap, ch.pos.x, ch.pos.z, @as(i32, @intCast(mx)), abs_y + 1, @as(i32, @intCast(mz)))) {
                    mask[idxm] = .{ .present = true, .uv = info.top_uv, .tint = info.top_apply_tint };
                }
            }
        }
        mz = 0;
        while (mz < constants.chunk_size_z) : (mz += 1) {
            var mx: usize = 0;
            while (mx < constants.chunk_size_x) : (mx += 1) {
                const base_idx: usize = mz * constants.chunk_size_x + mx;
                if (!mask[base_idx].present) continue;
                const mkey = mask[base_idx];
                var w: usize = 1;
                while ((mx + w) < constants.chunk_size_x) : (w += 1) {
                    const idx2 = mz * constants.chunk_size_x + (mx + w);
                    if (!(mask[idx2].present and mask[idx2].uv.u0 == mkey.uv.u0 and mask[idx2].uv.v0 == mkey.uv.v0 and mask[idx2].uv.u1 == mkey.uv.u1 and mask[idx2].uv.v1 == mkey.uv.v1 and mask[idx2].tint == mkey.tint)) break;
                }
                var h: usize = 1;
                outer: while ((mz + h) < constants.chunk_size_z) : (h += 1) {
                    var kx: usize = 0;
                    while (kx < w) : (kx += 1) {
                        const idx3 = (mz + h) * constants.chunk_size_x + (mx + kx);
                        if (!(mask[idx3].present and mask[idx3].uv.u0 == mkey.uv.u0 and mask[idx3].uv.v0 == mkey.uv.v0 and mask[idx3].uv.u1 == mkey.uv.u1 and mask[idx3].uv.v1 == mkey.uv.v1 and mask[idx3].tint == mkey.tint)) break :outer;
                    }
                }
                const wx0: f32 = base_x + @as(f32, @floatFromInt(mx));
                const wz0: f32 = base_z + @as(f32, @floatFromInt(mz));
                const wx1: f32 = base_x + @as(f32, @floatFromInt(mx + w));
                const wz1: f32 = base_z + @as(f32, @floatFromInt(mz + h));
                const wy1: f32 = @as(f32, @floatFromInt(abs_y + 1));
                const du: f32 = mkey.uv.u1 - mkey.uv.u0;
                const dv: f32 = mkey.uv.v1 - mkey.uv.v0;
                const rmin = [2]f32{ mkey.uv.u0, mkey.uv.v0 };
                const rsize = [2]f32{ du, dv };
                const uv00: [2]f32 = .{ 0, 0 };
                const uvh0: [2]f32 = .{ @floatFromInt(h), 0 };
                const uvhw: [2]f32 = .{ @floatFromInt(h), @floatFromInt(w) };
                const uv0w: [2]f32 = .{ 0, @floatFromInt(w) };
                emitQuad(alloc, out, .{ wx0, wy1, wz0 }, .{ wx0, wy1, wz1 }, .{ wx1, wy1, wz1 }, .{ wx1, wy1, wz0 }, uv00, uvh0, uvhw, uv0w, rmin, rsize, @floatFromInt(mkey.uv.aidx), if (mkey.tint) 1.0 else 0.0);
                var zz: usize = 0;
                while (zz < h) : (zz += 1) {
                    var xx: usize = 0;
                    while (xx < w) : (xx += 1) mask[(mz + zz) * constants.chunk_size_x + (mx + xx)].present = false;
                }
            }
        }
    }

    // -Y faces per Y-slice
    ly_top = 0;
    while (ly_top < constants.section_height) : (ly_top += 1) {
        const abs_y: i32 = section_base_y + @as(i32, @intCast(ly_top));
        var mask: [constants.chunk_size_z * constants.chunk_size_x]MaskCell = undefined;
        var mz: usize = 0;
        while (mz < constants.chunk_size_z) : (mz += 1) {
            var mx: usize = 0;
            while (mx < constants.chunk_size_x) : (mx += 1) {
                const idxm: usize = mz * constants.chunk_size_x + mx;
                mask[idxm].present = false;
                const src_idx: usize = mz * (constants.chunk_size_x * constants.section_height) + mx * constants.section_height + ly_top;
                const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, src_idx, bpi));
                if (pidx >= pal_info.items.len) continue;
                const info = pal_info.items[pidx];
                if (!info.draw) continue;
                if (!isSolidAtSnapshot(self, snap, ch.pos.x, ch.pos.z, @as(i32, @intCast(mx)), abs_y - 1, @as(i32, @intCast(mz)))) {
                    mask[idxm] = .{ .present = true, .uv = info.bot_uv, .tint = info.bot_apply_tint };
                }
            }
        }
        mz = 0;
        while (mz < constants.chunk_size_z) : (mz += 1) {
            var mx: usize = 0;
            while (mx < constants.chunk_size_x) : (mx += 1) {
                const base_idx: usize = mz * constants.chunk_size_x + mx;
                if (!mask[base_idx].present) continue;
                const mkey = mask[base_idx];
                var w: usize = 1;
                while ((mx + w) < constants.chunk_size_x) : (w += 1) {
                    const idx2 = mz * constants.chunk_size_x + (mx + w);
                    if (!(mask[idx2].present and mask[idx2].uv.u0 == mkey.uv.u0 and mask[idx2].uv.v0 == mkey.uv.v0 and mask[idx2].uv.u1 == mkey.uv.u1 and mask[idx2].uv.v1 == mkey.uv.v1 and mask[idx2].tint == mkey.tint)) break;
                }
                var h: usize = 1;
                outer_b: while ((mz + h) < constants.chunk_size_z) : (h += 1) {
                    var kx: usize = 0;
                    while (kx < w) : (kx += 1) {
                        const idx3 = (mz + h) * constants.chunk_size_x + (mx + kx);
                        if (!(mask[idx3].present and mask[idx3].uv.u0 == mkey.uv.u0 and mask[idx3].uv.v0 == mkey.uv.v0 and mask[idx3].uv.u1 == mkey.uv.u1 and mask[idx3].uv.v1 == mkey.uv.v1 and mask[idx3].tint == mkey.tint)) break :outer_b;
                    }
                }
                const wx0: f32 = base_x + @as(f32, @floatFromInt(mx));
                const wz0: f32 = base_z + @as(f32, @floatFromInt(mz));
                const wx1: f32 = base_x + @as(f32, @floatFromInt(mx + w));
                const wz1: f32 = base_z + @as(f32, @floatFromInt(mz + h));
                const wy0: f32 = @as(f32, @floatFromInt(abs_y));
                const du: f32 = mkey.uv.u1 - mkey.uv.u0;
                const dv: f32 = mkey.uv.v1 - mkey.uv.v0;
                const rmin = [2]f32{ mkey.uv.u0, mkey.uv.v0 };
                const rsize = [2]f32{ du, dv };
                const uv00_b: [2]f32 = .{ 0, 0 };
                const uvw0: [2]f32 = .{ @floatFromInt(w), 0 };
                const uvwh: [2]f32 = .{ @floatFromInt(w), @floatFromInt(h) };
                const uv0h: [2]f32 = .{ 0, @floatFromInt(h) };
                emitQuad(alloc, out, .{ wx0, wy0, wz0 }, .{ wx1, wy0, wz0 }, .{ wx1, wy0, wz1 }, .{ wx0, wy0, wz1 }, uv00_b, uvw0, uvwh, uv0h, rmin, rsize, @floatFromInt(mkey.uv.aidx), if (mkey.tint) 1.0 else 0.0);
                var zz: usize = 0;
                while (zz < h) : (zz += 1) {
                    var xx: usize = 0;
                    while (xx < w) : (xx += 1) mask[(mz + zz) * constants.chunk_size_x + (mx + xx)].present = false;
                }
            }
        }
    }

    // -X and +X per X-slice
    var lx_face: usize = 0;
    while (lx_face < constants.chunk_size_x) : (lx_face += 1) {
        // -X
        {
            const MaskSide = struct { present: bool, uv: client_atlas_builder_mod.AtlasUv, tint: bool };
            var mask_x: [constants.section_height * constants.chunk_size_z]MaskSide = undefined;
            var mz: usize = 0;
            while (mz < constants.chunk_size_z) : (mz += 1) {
                var ly: usize = 0;
                while (ly < constants.section_height) : (ly += 1) {
                    const idxm: usize = mz * constants.section_height + ly;
                    mask_x[idxm].present = false;
                    const src_idx: usize = mz * (constants.chunk_size_x * constants.section_height) + lx_face * constants.section_height + ly;
                    const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, src_idx, bpi));
                    if (pidx >= pal_info.items.len) continue;
                    const info = pal_info.items[pidx];
                    if (!info.draw) continue;
                    const abs_y: i32 = section_base_y + @as(i32, @intCast(ly));
                    if (!isSolidAtSnapshot(self, snap, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx_face)) - 1, abs_y, @as(i32, @intCast(mz))))
                        mask_x[idxm] = .{ .present = true, .uv = info.nx_uv, .tint = info.nx_apply_tint };
                }
            }
            mz = 0;
            while (mz < constants.chunk_size_z) : (mz += 1) {
                var ly0: usize = 0;
                while (ly0 < constants.section_height) : (ly0 += 1) {
                    const base_idx: usize = mz * constants.section_height + ly0;
                    if (!mask_x[base_idx].present) continue;
                    const mkey = mask_x[base_idx];
                    var w: usize = 1;
                    while ((ly0 + w) < constants.section_height) : (w += 1) {
                        const idx2 = mz * constants.section_height + (ly0 + w);
                        const c = mask_x[idx2];
                        if (!(c.present and c.uv.u0 == mkey.uv.u0 and c.uv.v0 == mkey.uv.v0 and c.uv.u1 == mkey.uv.u1 and c.uv.v1 == mkey.uv.v1 and c.tint == mkey.tint)) break;
                    }
                    var h: usize = 1;
                    outer_xn: while ((mz + h) < constants.chunk_size_z) : (h += 1) {
                        var ky: usize = 0;
                        while (ky < w) : (ky += 1) {
                            const idx3 = (mz + h) * constants.section_height + (ly0 + ky);
                            const c = mask_x[idx3];
                            if (!(c.present and c.uv.u0 == mkey.uv.u0 and c.uv.v0 == mkey.uv.v0 and c.uv.u1 == mkey.uv.u1 and c.uv.v1 == mkey.uv.v1 and c.tint == mkey.tint)) break :outer_xn;
                        }
                    }
                    const x0: f32 = base_x + @as(f32, @floatFromInt(lx_face));
                    const y0: f32 = @as(f32, @floatFromInt(section_base_y)) + @as(f32, @floatFromInt(ly0));
                    const y1: f32 = y0 + @as(f32, @floatFromInt(w));
                    const z0: f32 = base_z + @as(f32, @floatFromInt(mz));
                    const z1: f32 = z0 + @as(f32, @floatFromInt(h));
                    const uvs_nx = uvmap.sideTileUVs(0, w, h);
                    const rmin = [2]f32{ mkey.uv.u0, mkey.uv.v0 };
                    const rsize = [2]f32{ mkey.uv.u1 - mkey.uv.u0, mkey.uv.v1 - mkey.uv.v0 };
                    emitQuad(alloc, out, .{ x0, y0, z0 }, .{ x0, y0, z1 }, .{ x0, y1, z1 }, .{ x0, y1, z0 }, uvs_nx[0], uvs_nx[1], uvs_nx[2], uvs_nx[3], rmin, rsize, @floatFromInt(mkey.uv.aidx), if (mkey.tint) 1.0 else 0.0);
                    var hz: usize = 0;
                    while (hz < h) : (hz += 1) {
                        var wy: usize = 0;
                        while (wy < w) : (wy += 1) mask_x[(mz + hz) * constants.section_height + (ly0 + wy)].present = false;
                    }
                }
            }
        }
        // +X
        {
            const MaskSide = struct { present: bool, uv: client_atlas_builder_mod.AtlasUv, tint: bool };
            var mask_x: [constants.section_height * constants.chunk_size_z]MaskSide = undefined;
            var mz: usize = 0;
            while (mz < constants.chunk_size_z) : (mz += 1) {
                var ly: usize = 0;
                while (ly < constants.section_height) : (ly += 1) {
                    const idxm: usize = mz * constants.section_height + ly;
                    mask_x[idxm].present = false;
                    const src_idx: usize = mz * (constants.chunk_size_x * constants.section_height) + lx_face * constants.section_height + ly;
                    const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, src_idx, bpi));
                    if (pidx >= pal_info.items.len) continue;
                    const info = pal_info.items[pidx];
                    if (!info.draw) continue;
                    const abs_y: i32 = section_base_y + @as(i32, @intCast(ly));
                    if (!isSolidAtSnapshot(self, snap, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx_face)) + 1, abs_y, @as(i32, @intCast(mz))))
                        mask_x[idxm] = .{ .present = true, .uv = info.px_uv, .tint = info.px_apply_tint };
                }
            }
            mz = 0;
            while (mz < constants.chunk_size_z) : (mz += 1) {
                var ly0: usize = 0;
                while (ly0 < constants.section_height) : (ly0 += 1) {
                    const base_idx: usize = mz * constants.section_height + ly0;
                    if (!mask_x[base_idx].present) continue;
                    const mkey = mask_x[base_idx];
                    var w: usize = 1;
                    while ((ly0 + w) < constants.section_height) : (w += 1) {
                        const idx2 = mz * constants.section_height + (ly0 + w);
                        const c = mask_x[idx2];
                        if (!(c.present and c.uv.u0 == mkey.uv.u0 and c.uv.v0 == mkey.uv.v0 and c.uv.u1 == mkey.uv.u1 and c.uv.v1 == mkey.uv.v1 and c.tint == mkey.tint)) break;
                    }
                    var h: usize = 1;
                    outer_xp: while ((mz + h) < constants.chunk_size_z) : (h += 1) {
                        var ky: usize = 0;
                        while (ky < w) : (ky += 1) {
                            const idx3 = (mz + h) * constants.section_height + (ly0 + ky);
                            const c = mask_x[idx3];
                            if (!(c.present and c.uv.u0 == mkey.uv.u0 and c.uv.v0 == mkey.uv.v0 and c.uv.u1 == mkey.uv.u1 and c.uv.v1 == mkey.uv.v1 and c.tint == mkey.tint)) break :outer_xp;
                        }
                    }
                    const x1: f32 = base_x + @as(f32, @floatFromInt(lx_face + 1));
                    const y0: f32 = @as(f32, @floatFromInt(section_base_y)) + @as(f32, @floatFromInt(ly0));
                    const y1: f32 = y0 + @as(f32, @floatFromInt(w));
                    const z0: f32 = base_z + @as(f32, @floatFromInt(mz));
                    const z1: f32 = z0 + @as(f32, @floatFromInt(h));
                    const uvs_px = uvmap.sideTileUVs(1, w, h);
                    const rmin = [2]f32{ mkey.uv.u0, mkey.uv.v0 };
                    const rsize = [2]f32{ mkey.uv.u1 - mkey.uv.u0, mkey.uv.v1 - mkey.uv.v0 };
                    emitQuad(alloc, out, .{ x1, y0, z0 }, .{ x1, y1, z0 }, .{ x1, y1, z1 }, .{ x1, y0, z1 }, uvs_px[0], uvs_px[1], uvs_px[2], uvs_px[3], rmin, rsize, @floatFromInt(mkey.uv.aidx), if (mkey.tint) 1.0 else 0.0);
                    var hz: usize = 0;
                    while (hz < h) : (hz += 1) {
                        var wy: usize = 0;
                        while (wy < w) : (wy += 1) mask_x[(mz + hz) * constants.section_height + (ly0 + wy)].present = false;
                    }
                }
            }
        }
    }

    // -Z / +Z per Z-slice
    var lz_face: usize = 0;
    while (lz_face < constants.chunk_size_z) : (lz_face += 1) {
        // -Z
        {
            const MaskSide = struct { present: bool, uv: client_atlas_builder_mod.AtlasUv, tint: bool };
            var mask_z: [constants.section_height * constants.chunk_size_x]MaskSide = undefined;
            var lx: usize = 0;
            while (lx < constants.chunk_size_x) : (lx += 1) {
                var ly: usize = 0;
                while (ly < constants.section_height) : (ly += 1) {
                    const idxm: usize = lx * constants.section_height + ly;
                    mask_z[idxm].present = false;
                    const src_idx: usize = lz_face * (constants.chunk_size_x * constants.section_height) + lx * constants.section_height + ly;
                    const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, src_idx, bpi));
                    if (pidx >= pal_info.items.len) continue;
                    const info = pal_info.items[pidx];
                    if (!info.draw) continue;
                    const abs_y: i32 = section_base_y + @as(i32, @intCast(ly));
                    if (!isSolidAtSnapshot(self, snap, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y, @as(i32, @intCast(lz_face)) - 1))
                        mask_z[idxm] = .{ .present = true, .uv = info.nz_uv, .tint = info.nz_apply_tint };
                }
            }
            lx = 0;
            while (lx < constants.chunk_size_x) : (lx += 1) {
                var ly0: usize = 0;
                while (ly0 < constants.section_height) : (ly0 += 1) {
                    const base_idx: usize = lx * constants.section_height + ly0;
                    if (!mask_z[base_idx].present) continue;
                    const mkey = mask_z[base_idx];
                    var w: usize = 1;
                    while ((lx + w) < constants.chunk_size_x) : (w += 1) {
                        const idx2 = (lx + w) * constants.section_height + ly0;
                        const c = mask_z[idx2];
                        if (!(c.present and c.uv.u0 == mkey.uv.u0 and c.uv.v0 == mkey.uv.v0 and c.uv.u1 == mkey.uv.u1 and c.uv.v1 == mkey.uv.v1 and c.tint == mkey.tint)) break;
                    }
                    var h: usize = 1;
                    outer_zn: while ((ly0 + h) < constants.section_height) : (h += 1) {
                        var kx: usize = 0;
                        while (kx < w) : (kx += 1) {
                            const idx3 = (lx + kx) * constants.section_height + (ly0 + h);
                            const c = mask_z[idx3];
                            if (!(c.present and c.uv.u0 == mkey.uv.u0 and c.uv.v0 == mkey.uv.v0 and c.uv.u1 == mkey.uv.u1 and c.uv.v1 == mkey.uv.v1 and c.tint == mkey.tint)) break :outer_zn;
                        }
                    }
                    const z0: f32 = base_z + @as(f32, @floatFromInt(lz_face));
                    const x0: f32 = base_x + @as(f32, @floatFromInt(lx));
                    const x1: f32 = x0 + @as(f32, @floatFromInt(w));
                    const y0: f32 = @as(f32, @floatFromInt(section_base_y)) + @as(f32, @floatFromInt(ly0));
                    const y1: f32 = y0 + @as(f32, @floatFromInt(h));
                    const uvs_nz = uvmap.sideTileUVs(4, w, h);
                    const rmin = [2]f32{ mkey.uv.u0, mkey.uv.v0 };
                    const rsize = [2]f32{ mkey.uv.u1 - mkey.uv.u0, mkey.uv.v1 - mkey.uv.v0 };
                    emitQuad(alloc, out, .{ x0, y0, z0 }, .{ x0, y1, z0 }, .{ x1, y1, z0 }, .{ x1, y0, z0 }, uvs_nz[0], uvs_nz[1], uvs_nz[2], uvs_nz[3], rmin, rsize, @floatFromInt(mkey.uv.aidx), if (mkey.tint) 1.0 else 0.0);
                    var wy: usize = 0;
                    while (wy < h) : (wy += 1) {
                        var wx: usize = 0;
                        while (wx < w) : (wx += 1) mask_z[(lx + wx) * constants.section_height + (ly0 + wy)].present = false;
                    }
                }
            }
        }
        // +Z
        {
            const MaskSide = struct { present: bool, uv: client_atlas_builder_mod.AtlasUv, tint: bool };
            var mask_z: [constants.section_height * constants.chunk_size_x]MaskSide = undefined;
            var lx: usize = 0;
            while (lx < constants.chunk_size_x) : (lx += 1) {
                var ly: usize = 0;
                while (ly < constants.section_height) : (ly += 1) {
                    const idxm: usize = lx * constants.section_height + ly;
                    mask_z[idxm].present = false;
                    const src_idx: usize = lz_face * (constants.chunk_size_x * constants.section_height) + lx * constants.section_height + ly;
                    const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, src_idx, bpi));
                    if (pidx >= pal_info.items.len) continue;
                    const info = pal_info.items[pidx];
                    if (!info.draw) continue;
                    const abs_y: i32 = section_base_y + @as(i32, @intCast(ly));
                    if (!isSolidAtSnapshot(self, snap, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y, @as(i32, @intCast(lz_face)) + 1))
                        mask_z[idxm] = .{ .present = true, .uv = info.pz_uv, .tint = info.pz_apply_tint };
                }
            }
            lx = 0;
            while (lx < constants.chunk_size_x) : (lx += 1) {
                var ly0: usize = 0;
                while (ly0 < constants.section_height) : (ly0 += 1) {
                    const base_idx: usize = lx * constants.section_height + ly0;
                    if (!mask_z[base_idx].present) continue;
                    const mkey = mask_z[base_idx];
                    var w: usize = 1;
                    while ((lx + w) < constants.chunk_size_x) : (w += 1) {
                        const idx2 = (lx + w) * constants.section_height + ly0;
                        const c = mask_z[idx2];
                        if (!(c.present and c.uv.u0 == mkey.uv.u0 and c.uv.v0 == mkey.uv.v0 and c.uv.u1 == mkey.uv.u1 and c.uv.v1 == mkey.uv.v1 and c.tint == mkey.tint)) break;
                    }
                    var h: usize = 1;
                    outer_zp: while ((ly0 + h) < constants.section_height) : (h += 1) {
                        var kx: usize = 0;
                        while (kx < w) : (kx += 1) {
                            const idx3 = (lx + kx) * constants.section_height + (ly0 + h);
                            const c = mask_z[idx3];
                            if (!(c.present and c.uv.u0 == mkey.uv.u0 and c.uv.v0 == mkey.uv.v0 and c.uv.u1 == mkey.uv.u1 and c.uv.v1 == mkey.uv.v1 and c.tint == mkey.tint)) break :outer_zp;
                        }
                    }
                    const z1: f32 = base_z + @as(f32, @floatFromInt(lz_face + 1));
                    const x0: f32 = base_x + @as(f32, @floatFromInt(lx));
                    const x1: f32 = x0 + @as(f32, @floatFromInt(w));
                    const y0: f32 = @as(f32, @floatFromInt(section_base_y)) + @as(f32, @floatFromInt(ly0));
                    const y1: f32 = y0 + @as(f32, @floatFromInt(h));
                    const uvs_pz = uvmap.sideTileUVs(5, w, h);
                    // Ensure CCW for +Z face: use (v0, v1, v2, v3) -> (0,3,2,1)
                    const rmin = [2]f32{ mkey.uv.u0, mkey.uv.v0 };
                    const rsize = [2]f32{ mkey.uv.u1 - mkey.uv.u0, mkey.uv.v1 - mkey.uv.v0 };
                    emitQuad(alloc, out, .{ x0, y0, z1 }, .{ x1, y0, z1 }, .{ x1, y1, z1 }, .{ x0, y1, z1 }, uvs_pz[0], uvs_pz[3], uvs_pz[2], uvs_pz[1], rmin, rsize, @floatFromInt(mkey.uv.aidx), if (mkey.tint) 1.0 else 0.0);
                    var wy: usize = 0;
                    while (wy < h) : (wy += 1) {
                        var wx: usize = 0;
                        while (wx < w) : (wx += 1) mask_z[(lx + wx) * constants.section_height + (ly0 + wy)].present = false;
                    }
                }
            }
        }
    }
}

pub fn meshRegionFromSnapshot(self: anytype, alloc: std.mem.Allocator, snap: *const RegionSnapshot) MeshingResult {
    var draws = self.allocator.alloc(SectionDraw, snap.chunks.len * @as(usize, @intCast(snap.section_count_y))) catch return .{ .rpos = snap.rpos, .built_chunk_count = 0, .verts = &[_]Vertex{}, .draws = &[_]SectionDraw{} };
    var draws_len: usize = 0;
    var verts = std.ArrayList(Vertex).initCapacity(alloc, 0) catch {
        self.allocator.free(draws);
        return .{ .rpos = snap.rpos, .built_chunk_count = 0, .verts = &[_]Vertex{}, .draws = &[_]SectionDraw{} };
    };
    defer verts.deinit(alloc);
    for (snap.chunks) |ch| {
        const base_x: f32 = @floatFromInt(ch.pos.x * @as(i32, @intCast(constants.chunk_size_x)));
        const base_z: f32 = @floatFromInt(ch.pos.z * @as(i32, @intCast(constants.chunk_size_z)));
        var sy: usize = 0;
        while (sy < ch.sections.len) : (sy += 1) {
            const start: usize = verts.items.len;
            buildSectionVerticesFromSnapshot(self, snap, &ch, sy, base_x, base_z, &verts, alloc);
            const count: usize = verts.items.len - start;
            if (count > 0) {
                draws[draws_len] = .{ .chunk_pos = ch.pos, .sy = @intCast(sy), .first = @intCast(start), .count = @intCast(count) };
                draws_len += 1;
            }
        }
    }
    if (draws_len < draws.len) {
        const new_draws = self.allocator.alloc(SectionDraw, draws_len) catch {
            return .{ .rpos = snap.rpos, .built_chunk_count = snap.chunks.len, .verts = blk: {
                if (verts.items.len == 0) break :blk &[_]Vertex{};
                const owned = alloc.alloc(Vertex, verts.items.len) catch break :blk &[_]Vertex{};
                @memcpy(owned[0..verts.items.len], verts.items[0..verts.items.len]);
                break :blk owned;
            }, .draws = draws };
        };
        @memcpy(new_draws[0..draws_len], draws[0..draws_len]);
        self.allocator.free(draws);
        draws = new_draws;
    }
    var verts_owned: []Vertex = &[_]Vertex{};
    if (verts.items.len > 0) {
        verts_owned = alloc.alloc(Vertex, verts.items.len) catch &[_]Vertex{};
        if (verts_owned.len == verts.items.len) @memcpy(verts_owned[0..verts.items.len], verts.items[0..verts.items.len]);
    }
    return .{ .rpos = snap.rpos, .built_chunk_count = snap.chunks.len, .verts = verts_owned, .draws = draws };
}
