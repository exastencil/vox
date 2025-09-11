// Client renderer: pipelines, draws, tint cache glue
const std = @import("std");
const sokol = @import("sokol");
const sg = sokol.gfx;
const shd_mod = @import("../shaders/chunk_shd.zig");
const constants = @import("constants");
const simulation = @import("../simulation.zig");
const gs = @import("gs");
const client_camera_mod = @import("camera.zig");

// Local copy of Vertex layout used by the client renderer; must match src/client.zig
const Vertex = struct {
    pos: [3]f32,
    uv: [2]f32,
    rect_min: [2]f32,
    rect_size: [2]f32,
    layer: f32,
    apply_tint: f32,
};

pub fn drawPlayerAabb(self: anytype, vs_in: *const shd_mod.VsParams) void {
    const hx = self.local_aabb_half_extents[0];
    const hy = self.local_aabb_half_extents[1];
    const hz = self.local_aabb_half_extents[2];
    const x0: f32 = self.local_pos[0] - hx;
    const x1: f32 = self.local_pos[0] + hx;
    const y0: f32 = self.local_pos[1] - hy;
    const y1: f32 = self.local_pos[1] + hy;
    const z0: f32 = self.local_pos[2] - hz;
    const z1: f32 = self.local_pos[2] + hz;
    var verts: [36]Vertex = undefined;
    var i: usize = 0;
    const uv00 = [2]f32{ 0, 0 };
    const uv01 = [2]f32{ 0, 1 };
    const uv11 = [2]f32{ 1, 1 };
    const uv10 = [2]f32{ 1, 0 };
    const addTri = struct {
        fn call(list: *[36]Vertex, idx: *usize, p0: [3]f32, p1: [3]f32, p2: [3]f32, t0: [2]f32, t1: [2]f32, t2: [2]f32) void {
            list.*[idx.*] = .{ .pos = p0, .uv = t0, .rect_min = .{ 0, 0 }, .rect_size = .{ 1, 1 }, .layer = 0, .apply_tint = 0 };
            idx.* += 1;
            list.*[idx.*] = .{ .pos = p1, .uv = t1, .rect_min = .{ 0, 0 }, .rect_size = .{ 1, 1 }, .layer = 0, .apply_tint = 0 };
            idx.* += 1;
            list.*[idx.*] = .{ .pos = p2, .uv = t2, .rect_min = .{ 0, 0 }, .rect_size = .{ 1, 1 }, .layer = 0, .apply_tint = 0 };
            idx.* += 1;
        }
    }.call;
    // +X
    addTri(&verts, &i, .{ x1, y0, z0 }, .{ x1, y1, z0 }, .{ x1, y1, z1 }, uv00, uv01, uv11);
    addTri(&verts, &i, .{ x1, y0, z0 }, .{ x1, y1, z1 }, .{ x1, y0, z1 }, uv00, uv11, uv10);
    // -X
    addTri(&verts, &i, .{ x0, y0, z1 }, .{ x0, y1, z1 }, .{ x0, y1, z0 }, uv00, uv01, uv11);
    addTri(&verts, &i, .{ x0, y0, z1 }, .{ x0, y1, z0 }, .{ x0, y0, z0 }, uv00, uv11, uv10);
    // +Y (top)
    addTri(&verts, &i, .{ x0, y1, z0 }, .{ x0, y1, z1 }, .{ x1, y1, z1 }, uv00, uv01, uv11);
    addTri(&verts, &i, .{ x0, y1, z0 }, .{ x1, y1, z1 }, .{ x1, y1, z0 }, uv00, uv11, uv10);
    // -Y (bottom)
    addTri(&verts, &i, .{ x0, y0, z1 }, .{ x0, y0, z0 }, .{ x1, y0, z0 }, uv00, uv01, uv11);
    addTri(&verts, &i, .{ x0, y0, z1 }, .{ x1, y0, z0 }, .{ x1, y0, z1 }, uv00, uv11, uv10);
    // +Z
    addTri(&verts, &i, .{ x1, y0, z1 }, .{ x1, y1, z1 }, .{ x0, y1, z1 }, uv00, uv01, uv11);
    addTri(&verts, &i, .{ x1, y0, z1 }, .{ x0, y1, z1 }, .{ x0, y0, z1 }, uv00, uv11, uv10);
    // -Z
    addTri(&verts, &i, .{ x0, y0, z0 }, .{ x0, y1, z0 }, .{ x1, y1, z0 }, uv00, uv01, uv11);
    addTri(&verts, &i, .{ x0, y0, z0 }, .{ x1, y1, z0 }, .{ x1, y0, z0 }, uv00, uv11, uv10);
    if (i != verts.len) return; // safety
    // Use the 3D pipeline (depth test on). Override binding to use green texture and UI buffer.
    sg.applyPipeline(self.pip);
    var vs_params: shd_mod.VsParams = .{ .mvp = vs_in.mvp, .region_info = .{ 0, 0, 1.0 / 512.0, 0 } };
    sg.applyUniforms(0, sg.asRange(&vs_params));
    var b = self.bind;
    b.vertex_buffers[0] = self.aabb_vbuf;
    if (self.green_view.id != 0) b.views[1] = self.green_view;
    sg.applyBindings(b);
    const needed_bytes: usize = verts.len * @sizeOf(Vertex);
    if (self.aabb_vbuf_capacity_bytes < needed_bytes) {
        if (self.aabb_vbuf.id != 0) sg.destroyBuffer(self.aabb_vbuf);
        self.aabb_vbuf = sg.makeBuffer(.{ .usage = .{ .vertex_buffer = true, .stream_update = true }, .size = @intCast(needed_bytes) });
        self.aabb_vbuf_capacity_bytes = needed_bytes;
    }
    sg.updateBuffer(self.aabb_vbuf, sg.asRange(&verts));
    sg.draw(0, @intCast(verts.len), 1);
}

pub fn ensureRegionTintView(self: anytype, rpos: simulation.RegionPos) sg.View {
    if (self.region_tint_cache.getPtr(rpos)) |entry| {
        entry.last_used = self.frame_index;
        if (entry.dirty) {
            var pixels: [512 * 512 * 4]u8 = [_]u8{0} ** (512 * 512 * 4);
            fillRegionTintPixels(self, rpos, &pixels);
            if (entry.view.id != 0) sg.destroyView(entry.view);
            if (entry.img.id != 0) sg.destroyImage(entry.img);
            const new_img = sg.makeImage(.{ .width = 512, .height = 512, .pixel_format = .RGBA8, .data = .{ .subimage = .{
                .{ sg.asRange(pixels[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
            } } });
            const new_view = if (new_img.id != 0) sg.makeView(.{ .texture = .{ .image = new_img } }) else self.white_view;
            entry.img = new_img;
            entry.view = new_view;
            entry.dirty = false;
        }
        return entry.view;
    }
    var pixels: [512 * 512 * 4]u8 = [_]u8{0} ** (512 * 512 * 4);
    fillRegionTintPixels(self, rpos, &pixels);
    const img = sg.makeImage(.{ .width = 512, .height = 512, .pixel_format = .RGBA8, .data = .{ .subimage = .{
        .{ sg.asRange(pixels[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
        .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
        .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
        .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
        .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
        .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
    } } });
    const view = if (img.id != 0) sg.makeView(.{ .texture = .{ .image = img } }) else self.white_view;
    _ = self.region_tint_cache.put(rpos, .{ .img = img, .view = view, .last_used = self.frame_index, .dirty = false }) catch {};
    return if (view.id != 0) view else self.white_view;
}

pub fn fillRegionTintPixels(self: anytype, rpos: simulation.RegionPos, pixels: *[512 * 512 * 4]u8) void {
    const wk = "minecraft:overworld";
    var i: usize = 0;
    while (i < pixels.len) : (i += 4) {
        pixels.*[i + 0] = 255;
        pixels.*[i + 1] = 255;
        pixels.*[i + 2] = 255;
        pixels.*[i + 3] = 255;
    }
    self.sim.worlds_mutex.lock();
    const ws = self.sim.worlds_state.getPtr(wk);
    if (ws) |wsp| {
        const base_cx: i32 = rpos.x * 32;
        const base_cz: i32 = rpos.z * 32;
        var cz_off: i32 = 0;
        while (cz_off < 32) : (cz_off += 1) {
            var cx_off: i32 = 0;
            while (cx_off < 32) : (cx_off += 1) {
                const cpos = gs.ChunkPos{ .x = base_cx + cx_off, .z = base_cz + cz_off };
                const rp = simulation.RegionPos{ .x = @divFloor(cpos.x, 32), .z = @divFloor(cpos.z, 32) };
                if (wsp.regions.getPtr(rp)) |rs| {
                    if (rs.chunk_index.get(cpos)) |cidx| {
                        const ch = rs.chunks.items[cidx];
                        const sb: i32 = @as(i32, @intCast(wsp.sections_below));
                        const sy0: usize = if (sb > 0) @intCast(sb - 1) else 0;
                        if (sy0 < ch.sections.len) {
                            const s = ch.sections[sy0];
                            const bpi_biome: u6 = bitsFor(s.biome_palette.len);
                            var lz: usize = 0;
                            while (lz < 16) : (lz += 1) {
                                var lx: usize = 0;
                                while (lx < 16) : (lx += 1) {
                                    const idx3d: usize = lz * (constants.chunk_size_x * constants.section_height) + lx * constants.section_height + 0;
                                    var tint_rgb: [3]f32 = .{ 1, 1, 1 };
                                    if (s.biome_palette.len > 0 and s.biome_indices_bits.len > 0) {
                                        const pidx_u32: u32 = unpackBitsGet(s.biome_indices_bits, idx3d, bpi_biome);
                                        const pidx: usize = @intCast(pidx_u32);
                                        if (pidx < s.biome_palette.len) {
                                            const biome_id = s.biome_palette[pidx];
                                            if (self.sim.reg.getBiomeTint(biome_id, "grass")) |c|
                                                tint_rgb = c;
                                        }
                                    }
                                    const px: usize = (@as(usize, @intCast(cx_off)) * 16) + lx;
                                    const pz: usize = (@as(usize, @intCast(cz_off)) * 16) + lz;
                                    const off: usize = (pz * 512 + px) * 4;
                                    const r = std.math.clamp(tint_rgb[0], 0.0, 1.0);
                                    const g = std.math.clamp(tint_rgb[1], 0.0, 1.0);
                                    const b = std.math.clamp(tint_rgb[2], 0.0, 1.0);
                                    pixels.*[off + 0] = @intFromFloat(r * 255.0);
                                    pixels.*[off + 1] = @intFromFloat(g * 255.0);
                                    pixels.*[off + 2] = @intFromFloat(b * 255.0);
                                    pixels.*[off + 3] = 255;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    self.sim.worlds_mutex.unlock();
}

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

pub fn evictRegionMeshes(self: anytype, max_count: usize, ttl_frames: u32) void {
    var to_remove = std.ArrayList(simulation.RegionPos).initCapacity(self.allocator, 0) catch {
        return;
    };
    defer to_remove.deinit(self.allocator);
    var it = self.region_mesh_cache.iterator();
    while (it.next()) |entry| {
        const pos = entry.key_ptr.*;
        const last_opt = self.last_visible_frame_region.get(pos);
        const last_seen: u32 = last_opt orelse 0;
        const age = self.frame_index - last_seen;
        if (age > ttl_frames) {
            to_remove.append(self.allocator, pos) catch {};
        }
    }
    var i: usize = 0;
    while (i < to_remove.items.len) : (i += 1) {
        const pos = to_remove.items[i];
        if (self.region_mesh_cache.get(pos)) |rm| {
            if (rm.vbuf.id != 0) sg.destroyBuffer(rm.vbuf);
            self.allocator.free(rm.draws);
        }
        _ = self.region_mesh_cache.remove(pos);
    }
    // Count current entries
    var count: usize = 0;
    var itc = self.region_mesh_cache.keyIterator();
    while (itc.next()) |_| count += 1;
    if (count <= max_count) return;
    var candidates = std.ArrayList(struct { pos: simulation.RegionPos, last: u32 }).initCapacity(self.allocator, 0) catch {
        return;
    };
    defer candidates.deinit(self.allocator);
    var it2 = self.region_mesh_cache.iterator();
    while (it2.next()) |entry| {
        const pos = entry.key_ptr.*;
        const last = self.last_visible_frame_region.get(pos) orelse 0;
        candidates.append(self.allocator, .{ .pos = pos, .last = last }) catch {};
    }
    while (count > max_count and candidates.items.len > 0) {
        var min_idx: usize = 0;
        var min_last: u32 = candidates.items[0].last;
        var j: usize = 1;
        while (j < candidates.items.len) : (j += 1) {
            if (candidates.items[j].last < min_last) {
                min_last = candidates.items[j].last;
                min_idx = j;
            }
        }
        const pos = candidates.items[min_idx].pos;
        if (self.region_mesh_cache.get(pos)) |rm| {
            if (rm.vbuf.id != 0) sg.destroyBuffer(rm.vbuf);
            self.allocator.free(rm.draws);
        }
        _ = self.region_mesh_cache.remove(pos);
        _ = candidates.swapRemove(min_idx);
        count -= 1;
    }
}

pub fn renderWorldCached(self: anytype, vs_in: *const shd_mod.VsParams) void {
    const wk = "minecraft:overworld";
    var regions = std.ArrayList(struct { rpos: simulation.RegionPos, chunk_count: usize }).initCapacity(self.allocator, 0) catch return;
    defer regions.deinit(self.allocator);
    self.sim.worlds_mutex.lock();
    const ws = self.sim.worlds_state.getPtr(wk);
    var world_sections_below: u16 = 0;
    if (ws) |ws_ptr| {
        world_sections_below = ws_ptr.sections_below;
        var reg_it = ws_ptr.regions.iterator();
        while (reg_it.next()) |rentry| {
            const rpos = rentry.key_ptr.*;
            const rs_ptr = rentry.value_ptr;
            const count = rs_ptr.chunks.items.len;
            regions.append(self.allocator, .{ .rpos = rpos, .chunk_count = count }) catch {};
        }
    }
    self.sim.worlds_mutex.unlock();

    var vs_loc: shd_mod.VsParams = .{ .mvp = vs_in.mvp, .region_info = .{ 0, 0, 1.0 / 512.0, 0 } };
    self.rebuilds_issued_this_frame = 0;

    var i: usize = 0;
    while (i < regions.items.len) : (i += 1) {
        const rp = regions.items[i].rpos;
        const expected_chunks = regions.items[i].chunk_count;
        self.ensureRegionMesh(rp, expected_chunks);
        const rmesh = self.region_mesh_cache.getPtr(rp) orelse continue;
        if (rmesh.vbuf.id == 0 or rmesh.draws.len == 0) continue;

        const region_origin_x: f32 = @floatFromInt(rp.x * 32 * @as(i32, @intCast(constants.chunk_size_x)));
        const region_origin_z: f32 = @floatFromInt(rp.z * 32 * @as(i32, @intCast(constants.chunk_size_z)));
        vs_loc.region_info = .{ region_origin_x, region_origin_z, 1.0 / 512.0, 0 };

        var b = self.bind;
        b.vertex_buffers[0] = rmesh.vbuf;

        var di: usize = 0;
        while (di < rmesh.draws.len) : (di += 1) {
            const d = rmesh.draws[di];
            if (self.frustum_valid) {
                const base_x: f32 = @floatFromInt(d.chunk_pos.x * @as(i32, @intCast(constants.chunk_size_x)));
                const base_z: f32 = @floatFromInt(d.chunk_pos.z * @as(i32, @intCast(constants.chunk_size_z)));
                const y_off_sections: f32 = -@as(f32, @floatFromInt(@as(i32, @intCast(world_sections_below)) * @as(i32, @intCast(constants.section_height))));
                const minp_s: [3]f32 = .{ base_x, y_off_sections + @as(f32, @floatFromInt((@as(usize, d.sy) * constants.section_height))), base_z };
                const maxp_s: [3]f32 = .{ base_x + 16.0, y_off_sections + @as(f32, @floatFromInt(((@as(usize, d.sy) + 1) * constants.section_height))), base_z + 16.0 };
                const cam = self.camera.pos;
                const vis_sec = client_camera_mod.aabbContainsPoint(minp_s, maxp_s, cam) or client_camera_mod.aabbInFrustum(self.frustum_planes, minp_s, maxp_s, 0.5);
                if (!vis_sec) continue;
            }
            const tint_view = ensureRegionTintView(self, rp);
            var bdraw = self.bind;
            bdraw.vertex_buffers[0] = rmesh.vbuf;
            if (self.atlas_count > 0) bdraw.views[1] = self.atlas_views[0];
            if (self.atlas_count > 1) bdraw.views[5] = self.atlas_views[1];
            if (self.atlas_count > 2) bdraw.views[6] = self.atlas_views[2];
            if (self.atlas_count > 3) bdraw.views[7] = self.atlas_views[3];
            if (tint_view.id != 0) bdraw.views[3] = tint_view;
            sg.applyBindings(bdraw);
            sg.applyUniforms(0, sg.asRange(&vs_loc));
            sg.draw(d.first, d.count, 1);
            _ = self.last_visible_frame_chunk.put(d.chunk_pos, self.frame_index) catch {};
        }
        _ = self.last_visible_frame_region.put(rp, self.frame_index) catch {};
    }

    evictRegionMeshes(self, 128, 10);
}
