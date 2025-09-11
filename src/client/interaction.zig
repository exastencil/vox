// Client interaction and targeting
const std = @import("std");
const sokol = @import("sokol");
const sapp = sokol.app;
const sg = sokol.gfx;
const shd_mod = @import("../shaders/chunk_shd.zig");
const constants = @import("constants");
const simulation = @import("../simulation.zig");
const gs = @import("gs");
const client_camera_mod = @import("camera.zig");

// Local copy of the vertex layout used by the client renderer
// Must match the layout in src/client.zig
const Vertex = struct {
    pos: [3]f32,
    uv: [2]f32,
    rect_min: [2]f32,
    rect_size: [2]f32,
    layer: f32,
    apply_tint: f32,
};

// Bit-pack helpers (duplicated from client.zig for now)
fn bitsFor(n: usize) u6 {
    if (n <= 1) return 1;
    var v: usize = n - 1;
    var b: u6 = 0;
    while (v > 0) : (v >>= 1) b += 1;
    return if (b == 0) 1 else b;
}

fn unpackBitsGet(src: []const u32, idx: usize, bits_per_index: u6) u32 {
    const bit_off: usize = idx * @as(usize, bits_per_index);
    const word_idx: usize = bit_off >> 5; // /32
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

fn packBitsSet(dst: []u32, idx: usize, bits_per_index: u6, value: u32) void {
    const bit_off: usize = idx * @as(usize, bits_per_index);
    const word_idx: usize = bit_off >> 5; // /32
    const bit_idx: u5 = @intCast(bit_off & 31);
    const mask: u32 = if (bits_per_index == 32) 0xFFFF_FFFF else (@as(u32, 1) << @intCast(bits_per_index)) - 1;
    const vmasked: u32 = value & mask;
    const rem: u6 = @as(u6, 32) - @as(u6, bit_idx);
    if (bits_per_index <= rem) {
        const clear_mask: u32 = ~(mask << bit_idx);
        dst[word_idx] = (dst[word_idx] & clear_mask) | (vmasked << bit_idx);
    } else {
        const lo_bits: u6 = rem;
        const hi_bits: u6 = @intCast(bits_per_index - lo_bits);
        const lo_mask: u32 = (@as(u32, 1) << @as(u5, @intCast(lo_bits))) - 1;
        const lo_val: u32 = vmasked & lo_mask;
        const hi_val: u32 = vmasked >> @as(u5, @intCast(lo_bits));
        const clear_lo: u32 = ~(lo_mask << bit_idx);
        dst[word_idx] = (dst[word_idx] & clear_lo) | (lo_val << bit_idx);
        const hi_mask: u32 = (@as(u32, 1) << @as(u5, @intCast(hi_bits))) - 1;
        dst[word_idx + 1] = (dst[word_idx + 1] & ~hi_mask) | hi_val;
    }
}

// Expects a `self` with the fields/methods used below (Client)
pub fn updateHighlightTarget(self: anytype) void {
    self.highlight_has = false;
    if (!self.ready or !self.textures_ready) return;
    const wk = "minecraft:overworld";
    const eye = self.camera.pos;
    var dir = self.local_dir;
    const dl: f32 = @sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
    if (dl < 0.000001) return;
    dir[0] /= dl;
    dir[1] /= dl;
    dir[2] /= dl;
    const reach: f32 = self.reach_distance;
    const step: f32 = 0.05;
    self.sim.worlds_mutex.lock();
    const ws = self.sim.worlds_state.getPtr(wk);
    if (ws) |wsp| {
        var t: f32 = 0.0;
        while (t <= reach) : (t += step) {
            const px = eye[0] + dir[0] * t;
            const py = eye[1] + dir[1] * t;
            const pz = eye[2] + dir[2] * t;
            const ix: i32 = @intFromFloat(@floor(px));
            const iy: i32 = @intFromFloat(@floor(py));
            const iz: i32 = @intFromFloat(@floor(pz));
            const cx: i32 = @divFloor(ix, @as(i32, @intCast(constants.chunk_size_x)));
            const cz: i32 = @divFloor(iz, @as(i32, @intCast(constants.chunk_size_z)));
            const lx_i: i32 = @mod(ix, @as(i32, @intCast(constants.chunk_size_x)));
            const lz_i: i32 = @mod(iz, @as(i32, @intCast(constants.chunk_size_z)));
            const lx: usize = @intCast(lx_i);
            const lz: usize = @intCast(lz_i);
            const rp = simulation.RegionPos{ .x = @divFloor(cx, 32), .z = @divFloor(cz, 32) };
            const rs = wsp.regions.getPtr(rp) orelse continue;
            const cidx = rs.chunk_index.get(.{ .x = cx, .z = cz }) orelse continue;
            const ch = rs.chunks.items[cidx];
            const total_y: i32 = @intCast(ch.sections.len * constants.section_height);
            const ny0: i32 = iy + @as(i32, @intCast(wsp.sections_below)) * @as(i32, @intCast(constants.section_height));
            if (ny0 < 0 or ny0 >= total_y) continue;
            const sy: usize = @intCast(@divTrunc(ny0, @as(i32, @intCast(constants.section_height))));
            const ly: usize = @intCast(@mod(ny0, @as(i32, @intCast(constants.section_height))));
            if (sy >= ch.sections.len) continue;
            const s = ch.sections[sy];
            const bpi: u6 = bitsFor(s.palette.len);
            const idx3d: usize = lz * (constants.chunk_size_x * constants.section_height) + lx * constants.section_height + ly;
            const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, idx3d, bpi));
            if (pidx >= s.palette.len) continue;
            const bid: u32 = s.palette[pidx].block_id;
            if (bid == 0) continue;
            if (bid < self.sim.reg.blocks.items.len and !self.sim.reg.blocks.items[@intCast(bid)].collision) continue;
            // set highlight
            self.highlight_has = true;
            self.highlight_min = .{ @as(f32, @floatFromInt(ix)), @as(f32, @floatFromInt(iy)), @as(f32, @floatFromInt(iz)) };
            self.highlight_max = .{ self.highlight_min[0] + 1.0, self.highlight_min[1] + 1.0, self.highlight_min[2] + 1.0 };
            break;
        }
    }
    self.sim.worlds_mutex.unlock();
}

pub fn drawTargetOutline(self: anytype, vs_in: *const shd_mod.VsParams) void {
    if (!self.highlight_has) return;
    // Compute 6px screen-space thickness in world units at the block center
    const mn = self.highlight_min;
    const mx = self.highlight_max;
    const center = [3]f32{ (mn[0] + mx[0]) * 0.5, (mn[1] + mx[1]) * 0.5, (mn[2] + mx[2]) * 0.5 };
    const cam = self.camera.pos;
    const dx = center[0] - cam[0];
    const dy = center[1] - cam[1];
    const dz = center[2] - cam[2];
    const depth: f32 = @sqrt(dx * dx + dy * dy + dz * dz);
    const h_px_f: f32 = @floatFromInt(sapp.height());
    if (h_px_f <= 0) return;
    const fovy: f32 = 60.0 * std.math.pi / 180.0;
    const px: f32 = 3.0;
    const thickness_world: f32 = (2.0 * depth * @tan(fovy * 0.5)) * (px / h_px_f);
    const half_t: f32 = thickness_world * 0.5;

    // Unit vector from camera to block center (used to determine front-facing faces)
    const cam_to_center = if (depth > 0.000001)
        .{ dx / depth, dy / depth, dz / depth }
    else
        .{ 0.0, 0.0, 1.0 };
    // Face order: 0:-X(minX), 1:+X(maxX), 2:-Y(minY), 3:+Y(maxY), 4:-Z(minZ), 5:+Z(maxZ)
    const front_faces = [_]bool{
        cam_to_center[0] > 0.0, // -X
        cam_to_center[0] < 0.0, // +X
        cam_to_center[1] > 0.0, // -Y
        cam_to_center[1] < 0.0, // +Y
        cam_to_center[2] > 0.0, // -Z
        cam_to_center[2] < 0.0, // +Z
    };

    // Determine per-face visibility using frustum and occlusion (ray to face center)
    var face_visible = [_]bool{false} ** 6;
    const hx: i32 = @intFromFloat(@floor(mn[0]));
    const hy: i32 = @intFromFloat(@floor(mn[1]));
    const hz: i32 = @intFromFloat(@floor(mn[2]));
    const face_centers = [_][3]f32{
        .{ @as(f32, @floatFromInt(hx)), @as(f32, @floatFromInt(hy)) + 0.5, @as(f32, @floatFromInt(hz)) + 0.5 }, // -X
        .{ @as(f32, @floatFromInt(hx)) + 1.0, @as(f32, @floatFromInt(hy)) + 0.5, @as(f32, @floatFromInt(hz)) + 0.5 }, // +X
        .{ @as(f32, @floatFromInt(hx)) + 0.5, @as(f32, @floatFromInt(hy)), @as(f32, @floatFromInt(hz)) + 0.5 }, // -Y
        .{ @as(f32, @floatFromInt(hx)) + 0.5, @as(f32, @floatFromInt(hy)) + 1.0, @as(f32, @floatFromInt(hz)) + 0.5 }, // +Y
        .{ @as(f32, @floatFromInt(hx)) + 0.5, @as(f32, @floatFromInt(hy)) + 0.5, @as(f32, @floatFromInt(hz)) }, // -Z
        .{ @as(f32, @floatFromInt(hx)) + 0.5, @as(f32, @floatFromInt(hy)) + 0.5, @as(f32, @floatFromInt(hz)) + 1.0 }, // +Z
    };
    const face_normals = [_][3]f32{
        .{ -1.0, 0.0, 0.0 }, .{ 1.0, 0.0, 0.0 }, .{ 0.0, -1.0, 0.0 }, .{ 0.0, 1.0, 0.0 }, .{ 0.0, 0.0, -1.0 }, .{ 0.0, 0.0, 1.0 },
    };
    const wk = "minecraft:overworld";
    const ray_visible = struct {
        fn call(cli: anytype, ws: *const simulation.WorldState, tx: i32, ty: i32, tz: i32, face_center: [3]f32, face_normal: [3]f32, cam_pos: [3]f32) bool {
            // Ray from camera to slightly in front of face center
            const eps: f32 = 0.001;
            const target = .{ face_center[0] - face_normal[0] * eps, face_center[1] - face_normal[1] * eps, face_center[2] - face_normal[2] * eps };
            var rx = target[0] - cam_pos[0];
            var ry = target[1] - cam_pos[1];
            var rz = target[2] - cam_pos[2];
            const dist: f32 = @sqrt(rx * rx + ry * ry + rz * rz);
            if (dist <= 0.00001) return true;
            rx /= dist;
            ry /= dist;
            rz /= dist;
            var t: f32 = 0.0;
            const step: f32 = 0.05;
            while (t <= dist) : (t += step) {
                const rxp = cam_pos[0] + rx * t;
                const ryp = cam_pos[1] + ry * t;
                const rzp = cam_pos[2] + rz * t;
                const ix: i32 = @intFromFloat(@floor(rxp));
                const iy: i32 = @intFromFloat(@floor(ryp));
                const iz: i32 = @intFromFloat(@floor(rzp));
                // Reached target block => visible
                if (ix == tx and iy == ty and iz == tz) return true;
                // Query world occupancy (solid blocks occlude)
                const cx: i32 = @divFloor(ix, @as(i32, @intCast(constants.chunk_size_x)));
                const cz: i32 = @divFloor(iz, @as(i32, @intCast(constants.chunk_size_z)));
                const rp = simulation.RegionPos{ .x = @divFloor(cx, 32), .z = @divFloor(cz, 32) };
                const rs = ws.regions.getPtr(rp) orelse continue;
                const cidx = rs.chunk_index.get(.{ .x = cx, .z = cz }) orelse continue;
                const ch = rs.chunks.items[cidx];
                const total_y: i32 = @intCast(ch.sections.len * constants.section_height);
                const ny0: i32 = iy + @as(i32, @intCast(ws.sections_below)) * @as(i32, @intCast(constants.section_height));
                if (ny0 < 0 or ny0 >= total_y) continue;
                const sy: usize = @intCast(@divTrunc(ny0, @as(i32, @intCast(constants.section_height))));
                const ly: usize = @intCast(@mod(ny0, @as(i32, @intCast(constants.section_height))));
                if (sy >= ch.sections.len) continue;
                const s = ch.sections[sy];
                const bpi: u6 = bitsFor(s.palette.len);
                const lx_i: i32 = @mod(ix, @as(i32, @intCast(constants.chunk_size_x)));
                const lz_i: i32 = @mod(iz, @as(i32, @intCast(constants.chunk_size_z)));
                const lx: usize = @intCast(lx_i);
                const lz: usize = @intCast(lz_i);
                const idx3d: usize = lz * (constants.chunk_size_x * constants.section_height) + lx * constants.section_height + ly;
                const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, idx3d, bpi));
                if (pidx >= s.palette.len) continue;
                const bid: u32 = s.palette[pidx].block_id;
                if (bid == 0) continue;
                if (bid < cli.sim.reg.blocks.items.len) {
                    if (cli.sim.reg.blocks.items[@intCast(bid)].collision) {
                        // occluded by another solid block
                        return false;
                    }
                } else {
                    // unknown: treat as solid
                    return false;
                }
            }
            return true; // no occluders found until target
        }
    }.call;

    // Acquire world state to test occlusions
    self.sim.worlds_mutex.lock();
    const ws = self.sim.worlds_state.getPtr(wk);
    if (ws) |wsp| {
        var fi: usize = 0;
        while (fi < 6) : (fi += 1) {
            if (!front_faces[fi]) {
                face_visible[fi] = false;
                continue;
            }
            // Frustum test on face center
            const fc = face_centers[fi];
            var in_frustum = true;
            if (self.frustum_valid) {
                in_frustum = client_camera_mod.aabbInFrustum(self.frustum_planes, fc, fc, 0.0);
            }
            if (!in_frustum) {
                face_visible[fi] = false;
                continue;
            }
            // Occlusion ray test
            face_visible[fi] = ray_visible(self, wsp, hx, hy, hz, fc, face_normals[fi], cam);
        }
    }
    self.sim.worlds_mutex.unlock();

    // Define all 12 edges of the AABB
    const p000 = [3]f32{ mn[0], mn[1], mn[2] };
    const p001 = [3]f32{ mn[0], mn[1], mx[2] };
    const p010 = [3]f32{ mn[0], mx[1], mn[2] };
    const p011 = [3]f32{ mn[0], mx[1], mx[2] };
    const p100 = [3]f32{ mx[0], mn[1], mn[2] };
    const p101 = [3]f32{ mx[0], mn[1], mx[2] };
    const p110 = [3]f32{ mx[0], mx[1], mn[2] };
    const p111 = [3]f32{ mx[0], mx[1], mx[2] };
    const edges = [_][2][3]f32{
        .{ p000, p100 }, .{ p100, p101 }, .{ p101, p001 }, .{ p001, p000 }, // bottom
        .{ p010, p110 }, .{ p110, p111 }, .{ p111, p011 }, .{ p011, p010 }, // top
        .{ p000, p010 }, .{ p100, p110 }, .{ p101, p111 }, .{ p001, p011 }, // verticals
    };
    // For each edge, list the two adjacent faces using the face order above
    const edge_faces = [_][2]u3{
        .{ 2, 4 }, .{ 2, 1 }, .{ 2, 5 }, .{ 2, 0 },
        .{ 3, 4 }, .{ 3, 1 }, .{ 3, 5 }, .{ 3, 0 },
        .{ 0, 4 }, .{ 1, 4 }, .{ 1, 5 }, .{ 0, 5 },
    };

    // Build quads (two triangles per edge) facing the camera using a screen-space thickness
    var verts: [12 * 6]Vertex = undefined; // 12 edges * 2 tris * 3 verts
    var vi: usize = 0;
    const make_vert = struct {
        fn call(p: [3]f32) Vertex {
            return .{ .pos = p, .uv = .{ 0, 0 }, .rect_min = .{ 0, 0 }, .rect_size = .{ 1, 1 }, .layer = 0, .apply_tint = 0 };
        }
    }.call;
    const normalize3 = struct {
        fn call(v: [3]f32) [3]f32 {
            const l: f32 = @sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            if (l > 0.000001) return .{ v[0] / l, v[1] / l, v[2] / l };
            return .{ 0, 0, 0 };
        }
    }.call;
    const cross3 = struct {
        fn call(a: [3]f32, b: [3]f32) [3]f32 {
            return .{ a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };
        }
    }.call;
    const addTri = struct {
        fn call(list: *[12 * 6]Vertex, idx: *usize, a: [3]f32, b: [3]f32, c: [3]f32) void {
            list.*[idx.*] = make_vert(a);
            idx.* += 1;
            list.*[idx.*] = make_vert(b);
            idx.* += 1;
            list.*[idx.*] = make_vert(c);
            idx.* += 1;
        }
    }.call;

    var ei: usize = 0;
    while (ei < edges.len) : (ei += 1) {
        const a = edges[ei][0];
        const b = edges[ei][1];
        // Edge visibility: draw if either adjacent face is visible (including both)
        const f0: usize = @intCast(edge_faces[ei][0]);
        const f1: usize = @intCast(edge_faces[ei][1]);
        if (!face_visible[f0] and !face_visible[f1]) continue;

        const mid = [3]f32{ (a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5 };
        const d = normalize3(.{ b[0] - a[0], b[1] - a[1], b[2] - a[2] });
        const vdir = normalize3(.{ mid[0] - cam[0], mid[1] - cam[1], mid[2] - cam[2] });
        var n = normalize3(cross3(d, vdir));
        // Fallback if edge is parallel to view direction
        if (@abs(n[0]) + @abs(n[1]) + @abs(n[2]) < 0.00001) {
            n = normalize3(cross3(d, .{ 0, 1, 0 }));
            if (@abs(n[0]) + @abs(n[1]) + @abs(n[2]) < 0.00001) {
                n = normalize3(cross3(d, .{ 1, 0, 0 }));
            }
        }
        const off = .{ n[0] * half_t, n[1] * half_t, n[2] * half_t };
        const q0 = [3]f32{ a[0] - off[0], a[1] - off[1], a[2] - off[2] };
        const q1 = [3]f32{ a[0] + off[0], a[1] + off[1], a[2] + off[2] };
        const q2 = [3]f32{ b[0] + off[0], b[1] + off[1], b[2] + off[2] };
        const q3 = [3]f32{ b[0] - off[0], b[1] - off[1], b[2] - off[2] };
        // Two triangles (culling disabled in pipeline, winding doesn't matter)
        addTri(&verts, &vi, q0, q1, q2);
        addTri(&verts, &vi, q0, q2, q3);
    }
    if (vi == 0) return;

    // Use outline triangle pipeline and black texture; renders through geometry (depth compare: ALWAYS)
    sg.applyPipeline(self.pip_outline);
    var vs_params: shd_mod.VsParams = .{ .mvp = vs_in.mvp, .region_info = .{ 0, 0, 1.0 / 512.0, 0 } };
    sg.applyUniforms(0, sg.asRange(&vs_params));
    var b = self.bind;
    b.vertex_buffers[0] = self.outline_vbuf;
    if (self.black_view.id != 0) b.views[1] = self.black_view;
    sg.applyBindings(b);
    const needed_bytes: usize = vi * @sizeOf(Vertex);
    if (self.outline_vbuf_capacity_bytes < needed_bytes) {
        if (self.outline_vbuf.id != 0) sg.destroyBuffer(self.outline_vbuf);
        self.outline_vbuf = sg.makeBuffer(.{ .usage = .{ .vertex_buffer = true, .stream_update = true }, .size = @intCast(needed_bytes) });
        self.outline_vbuf_capacity_bytes = needed_bytes;
    }
    sg.updateBuffer(self.outline_vbuf, sg.asRange(verts[0..vi]));
    sg.draw(0, @intCast(vi), 1);
}

pub fn tryBreakBlockUnderCrosshair(self: anytype) void {
    if (!self.ready or !self.textures_ready) return;
    const wk = "minecraft:overworld";
    const eye = self.camera.pos;
    var dir = self.local_dir;
    const dl: f32 = @sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
    if (dl < 0.000001) return;
    dir[0] /= dl;
    dir[1] /= dl;
    dir[2] /= dl;
    const reach: f32 = self.reach_distance;
    const step: f32 = 0.05;

    var hit_found = false;
    var hit_x: i32 = 0;
    var hit_y: i32 = 0;
    var hit_z: i32 = 0;
    var hit_cx: i32 = 0;
    var hit_cz: i32 = 0;
    var hit_lx: usize = 0;
    var hit_lz: usize = 0;

    self.sim.worlds_mutex.lock();
    const ws = self.sim.worlds_state.getPtr(wk);
    if (ws) |wsp| {
        var t: f32 = 0.0;
        while (t <= reach) : (t += step) {
            const px = eye[0] + dir[0] * t;
            const py = eye[1] + dir[1] * t;
            const pz = eye[2] + dir[2] * t;
            const ix: i32 = @intFromFloat(@floor(px));
            const iy: i32 = @intFromFloat(@floor(py));
            const iz: i32 = @intFromFloat(@floor(pz));
            const cx: i32 = @divFloor(ix, @as(i32, @intCast(constants.chunk_size_x)));
            const cz: i32 = @divFloor(iz, @as(i32, @intCast(constants.chunk_size_z)));
            const lx_i: i32 = @mod(ix, @as(i32, @intCast(constants.chunk_size_x)));
            const lz_i: i32 = @mod(iz, @as(i32, @intCast(constants.chunk_size_z)));
            const lx: usize = @intCast(lx_i);
            const lz: usize = @intCast(lz_i);
            const rp = simulation.RegionPos{ .x = @divFloor(cx, 32), .z = @divFloor(cz, 32) };
            const rs = wsp.regions.getPtr(rp) orelse continue;
            const cidx = rs.chunk_index.get(.{ .x = cx, .z = cz }) orelse continue;
            const ch = rs.chunks.items[cidx];
            const total_y: i32 = @intCast(ch.sections.len * constants.section_height);
            const ny0: i32 = iy + @as(i32, @intCast(wsp.sections_below)) * @as(i32, @intCast(constants.section_height));
            if (ny0 < 0 or ny0 >= total_y) continue;
            const sy: usize = @intCast(@divTrunc(ny0, @as(i32, @intCast(constants.section_height))));
            const ly: usize = @intCast(@mod(ny0, @as(i32, @intCast(constants.section_height))));
            if (sy >= ch.sections.len) continue;
            const s = ch.sections[sy];
            const bpi: u6 = bitsFor(s.palette.len);
            const idx3d: usize = lz * (constants.chunk_size_x * constants.section_height) + lx * constants.section_height + ly;
            const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, idx3d, bpi));
            if (pidx >= s.palette.len) continue;
            const bid: u32 = s.palette[pidx].block_id;
            if (bid == 0) continue;
            if (bid < self.sim.reg.blocks.items.len and !self.sim.reg.blocks.items[@intCast(bid)].collision) continue;
            // Remember hit
            hit_found = true;
            hit_x = ix;
            hit_y = iy;
            hit_z = iz;
            hit_cx = cx;
            hit_cz = cz;
            hit_lx = lx;
            hit_lz = lz;
            break;
        }
    }
    self.sim.worlds_mutex.unlock();

    if (!hit_found) return;
    // Replace block with air (by writing the palette index for air) and update heightmaps; mark meshes dirty
    self.sim.worlds_mutex.lock();
    const ws2 = self.sim.worlds_state.getPtr(wk);
    if (ws2) |wsp_mut| {
        const rp = simulation.RegionPos{ .x = @divFloor(hit_cx, 32), .z = @divFloor(hit_cz, 32) };
        if (wsp_mut.regions.getPtr(rp)) |rs_mut| {
            if (rs_mut.chunk_index.get(.{ .x = hit_cx, .z = hit_cz })) |cidx| {
                var chp = &rs_mut.chunks.items[cidx];
                const ny0h: i32 = hit_y + @as(i32, @intCast(wsp_mut.sections_below)) * @as(i32, @intCast(constants.section_height));
                if (ny0h >= 0) {
                    const syh: usize = @intCast(@divTrunc(ny0h, @as(i32, @intCast(constants.section_height))));
                    const lyh: usize = @intCast(@mod(ny0h, @as(i32, @intCast(constants.section_height))));
                    if (syh < chp.sections.len) {
                        const sref = chp.sections[syh];
                        const pal_len = sref.palette.len;
                        const bpi_set: u6 = bitsFor(pal_len);
                        // find air palette index
                        var air_pidx: ?u32 = null;
                        var pi: usize = 0;
                        while (pi < pal_len) : (pi += 1) {
                            if (sref.palette[pi].block_id == 0) {
                                air_pidx = @intCast(pi);
                                break;
                            }
                        }
                        if (air_pidx) |ap| {
                            const idx3d_set: usize = hit_lz * (constants.chunk_size_x * constants.section_height) + hit_lx * constants.section_height + lyh;
                            // write into mutable bit-packed buffer
                            const bits_mut: []u32 = @constCast(sref.blocks_indices_bits);
                            packBitsSet(bits_mut, idx3d_set, bpi_set, ap);
                            // Update heightmaps for this column (recompute top)
                            const col_idx: usize = hit_lz * constants.chunk_size_x + hit_lx;
                            const new_top: i32 = blk: {
                                var best: i32 = -1;
                                var sy_scan: i32 = @intCast(chp.sections.len);
                                while (sy_scan > 0) : (sy_scan -= 1) {
                                    const sy_u: usize = @intCast(sy_scan - 1);
                                    const sscan = chp.sections[sy_u];
                                    const bpi_scan: u6 = bitsFor(sscan.palette.len);
                                    var ly_scan: i32 = @intCast(constants.section_height);
                                    while (ly_scan > 0) : (ly_scan -= 1) {
                                        const ly_u: usize = @intCast(ly_scan - 1);
                                        const idx_scan: usize = hit_lz * (constants.chunk_size_x * constants.section_height) + hit_lx * constants.section_height + ly_u;
                                        const p_scan: usize = @intCast(unpackBitsGet(sscan.blocks_indices_bits, idx_scan, bpi_scan));
                                        if (p_scan >= sscan.palette.len) continue;
                                        const bid_scan: u32 = sscan.palette[p_scan].block_id;
                                        if (bid_scan != 0) {
                                            // respect solid flag
                                            if (bid_scan < self.sim.reg.blocks.items.len and !self.sim.reg.blocks.items[@intCast(bid_scan)].collision) {
                                                continue;
                                            }
                                            const abs_y: i32 = (@as(i32, @intCast(sy_u)) * @as(i32, @intCast(constants.section_height)) + @as(i32, @intCast(ly_u))) - @as(i32, @intCast(wsp_mut.sections_below)) * @as(i32, @intCast(constants.section_height));
                                            best = abs_y;
                                            break;
                                        }
                                    }
                                    if (best >= 0) break;
                                }
                                break :blk best;
                            };
                            chp.heightmaps.world_surface[col_idx] = new_top;
                            chp.heightmaps.motion_blocking[col_idx] = new_top;
                            // mark region mesh dirty so it gets rebuilt
                            if (self.region_mesh_cache.getPtr(rp)) |rm| {
                                rm.dirty = true;
                            }
                            // Also dirty neighbor region meshes if we edited a chunk-edge column
                            var dxs: [2]i32 = .{ 0, 0 };
                            var dzs: [2]i32 = .{ 0, 0 };
                            var ncount: usize = 0;
                            if (hit_lx == 0) {
                                dxs[ncount] = -1;
                                dzs[ncount] = 0;
                                ncount += 1;
                            }
                            if (hit_lx == (@as(usize, @intCast(constants.chunk_size_x)) - 1)) {
                                dxs[ncount] = 1;
                                dzs[ncount] = 0;
                                ncount += 1;
                            }
                            if (hit_lz == 0) {
                                dxs[ncount] = 0;
                                dzs[ncount] = -1;
                                ncount += 1;
                            }
                            if (hit_lz == (@as(usize, @intCast(constants.chunk_size_z)) - 1)) {
                                dxs[ncount] = 0;
                                dzs[ncount] = 1;
                                ncount += 1;
                            }
                            var ni: usize = 0;
                            while (ni < ncount) : (ni += 1) {
                                const ncx: i32 = hit_cx + dxs[ni];
                                const ncz: i32 = hit_cz + dzs[ni];
                                const nrp = simulation.RegionPos{ .x = @divFloor(ncx, 32), .z = @divFloor(ncz, 32) };
                                if (self.region_mesh_cache.getPtr(nrp)) |rmn| rmn.dirty = true;
                            }
                        }
                    }
                }
            }
        }
    }
    self.sim.worlds_mutex.unlock();
}
