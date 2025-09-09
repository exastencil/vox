const std = @import("std");
const simulation = @import("simulation.zig");
const player = @import("player.zig");
const constants = @import("constants");
const gs = @import("gs");
const sokol = @import("sokol");
const physics = @import("physics.zig");
const sg = sokol.gfx;
const sapp = sokol.app;
const sglue = sokol.glue;
const sdtx = sokol.debugtext;
const shd_mod = @import("shaders/chunk_shd.zig");
const builtin = @import("builtin");

// macOS-only: warp the mouse cursor to a point that is guaranteed to be inside our app window.
// For now, we use the center of the main display, which is typically within the window
// (our window defaults to 1920x1080 and is centered on launch). This avoids the first click
// going to another app if the cursor started outside our window.
inline fn macosWarpCursorIntoAppWindow() void {
    if (builtin.os.tag != .macos) return;
    const cg = @cImport({
        @cInclude("CoreGraphics/CoreGraphics.h");
    });
    const display = cg.CGMainDisplayID();
    const bounds = cg.CGDisplayBounds(display);
    const cx: cg.CGFloat = bounds.origin.x + (bounds.size.width / 2.0);
    const cy: cg.CGFloat = bounds.origin.y + (bounds.size.height / 2.0);
    const pt = cg.CGPointMake(cx, cy);
    _ = cg.CGWarpMouseCursorPosition(pt);
}

fn loadOrFallback(allocator: std.mem.Allocator, path: []const u8) sg.Image {
    const png = @import("png.zig");
    const loaded: ?png.Image = blk: {
        const res = png.loadFileRGBA8(allocator, path) catch |e| {
            std.log.warn("texture: failed to load '{s}': {s}", .{ path, @errorName(e) });
            break :blk null;
        };
        break :blk res;
    };
    if (loaded) |img| {
        defer allocator.free(img.pixels);
        return sg.makeImage(.{
            .width = @intCast(img.width),
            .height = @intCast(img.height),
            .pixel_format = .RGBA8,
            .data = .{ .subimage = .{
                .{ sg.asRange(img.pixels[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
            } },
        });
    } else {
        // Generate a magenta/black checkerboard fallback with exactly 4 squares (2x2) per face
        const w: usize = 16;
        const h: usize = 16;
        const tile: usize = 8; // 2 squares across each axis, 8x8 each
        var pixels = allocator.alloc(u8, w * h * 4) catch {
            var one = [_]u8{ 255, 0, 255, 255 };
            return sg.makeImage(.{
                .width = 1,
                .height = 1,
                .pixel_format = .RGBA8,
                .data = .{ .subimage = .{
                    .{ sg.asRange(one[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                    .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                    .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                    .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                    .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                    .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                } },
            });
        };
        defer allocator.free(pixels);
        var y: usize = 0;
        while (y < h) : (y += 1) {
            var x: usize = 0;
            while (x < w) : (x += 1) {
                const cx = (x / tile) & 1;
                const cy = (y / tile) & 1;
                const is_magenta = (cx ^ cy) == 0;
                const idx = (y * w + x) * 4;
                if (is_magenta) {
                    pixels[idx + 0] = 255;
                    pixels[idx + 1] = 0;
                    pixels[idx + 2] = 255;
                    pixels[idx + 3] = 255;
                } else {
                    pixels[idx + 0] = 0;
                    pixels[idx + 1] = 0;
                    pixels[idx + 2] = 0;
                    pixels[idx + 3] = 255;
                }
            }
        }
        return sg.makeImage(.{
            .width = @intCast(w),
            .height = @intCast(h),
            .pixel_format = .RGBA8,
            .data = .{ .subimage = .{
                .{ sg.asRange(pixels[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
                .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
            } },
        });
    }
}

const Camera = struct {
    pos: [3]f32 = .{ 0, 0, 0 },
    yaw: f32 = 0.0,
    pitch: f32 = 0.0,
    roll: f32 = 0.0,
    // Derived forward direction from yaw/pitch
    dir: [3]f32 = .{ 0, 0, -1 },

    fn computeDir(yaw: f32, pitch: f32) [3]f32 {
        // Assuming yaw around +Y, pitch around +X, radians
        const cy = @cos(yaw);
        const sy = @sin(yaw);
        const cp = @cos(pitch);
        const sp = @sin(pitch);
        // Forward in right-handed Y-up: x = cp*cy, y = sp, z = cp*sy
        return .{ cp * cy, sp, cp * sy };
    }

    fn setYawPitch(self: *Camera, yaw: f32, pitch: f32) void {
        self.yaw = yaw;
        self.pitch = pitch;
        self.dir = computeDir(yaw, pitch);
    }

    fn setDir(self: *Camera, dir_in: [3]f32) void {
        // normalize and update yaw/pitch to match
        const dx = dir_in[0];
        const dy = dir_in[1];
        const dz = dir_in[2];
        const len: f32 = @sqrt(dx * dx + dy * dy + dz * dz);
        if (len > 0.000001) {
            const nx = dx / len;
            const ny = dy / len;
            const nz = dz / len;
            self.dir = .{ nx, ny, nz };
            self.pitch = std.math.asin(std.math.clamp(ny, -1.0, 1.0));
            self.yaw = std.math.atan2(nz, nx);
        }
    }
};

// Control scheme abstraction encapsulating how input affects camera and player
const ViewProj = struct { view: [16]f32, proj: [16]f32 };
const ControlScheme = struct {
    name: []const u8,
    onMouseMove: *const fn (*anyopaque, sapp.Event) void,
    onScroll: *const fn (*anyopaque, sapp.Event) void,
    applyMovement: *const fn (*anyopaque) void,
    updateCamera: *const fn (*anyopaque) void,
    makeViewProj: *const fn (*anyopaque, f32, f32) ViewProj,
};

// FirstPersonHorizontal control scheme: horizontal + vertical mouse look (yaw + pitch),
// movement derived from yaw on XZ plane, camera mirrors local state.
fn cs_fph_onMouseMove(ctx: *anyopaque, ev: sapp.Event) void {
    const self: *Client = @ptrCast(@alignCast(ctx));
    const sens: f32 = 0.002; // radians per pixel
    const dx: f32 = ev.mouse_dx * sens;
    const dy: f32 = ev.mouse_dy * sens;

    // Update yaw from horizontal mouse motion (mouse right => turn right)
    self.local_yaw += dx;
    if (self.local_yaw > std.math.pi) self.local_yaw -= 2.0 * std.math.pi;
    if (self.local_yaw < -std.math.pi) self.local_yaw += 2.0 * std.math.pi;

    // Update pitch from vertical mouse motion.
    // Top-left origin: moving mouse up yields negative dy; looking up is +pitch => subtract dy
    const pitch_max: f32 = (std.math.pi / 2.0) - 0.01; // keep away from poles to avoid singularities
    self.local_pitch = std.math.clamp(self.local_pitch - dy, -pitch_max, pitch_max);

    // Recompute forward/look vector and sync camera
    const cp: f32 = @cos(self.local_pitch);
    const sp: f32 = @sin(self.local_pitch);
    const cy: f32 = @cos(self.local_yaw);
    const sy: f32 = @sin(self.local_yaw);
    self.local_dir = .{ cp * cy, sp, cp * sy };
    self.camera.setYawPitch(self.local_yaw, self.local_pitch);

    // If not moving, constrain facing so look stays within 90° of facing (XZ)
    if (!(self.move_forward or self.move_back or self.move_left or self.move_right)) {
        // Use camera forward
        const lpx = self.local_dir[0];
        const lpz = self.local_dir[2];
        const lplen = @sqrt(lpx * lpx + lpz * lpz);
        if (lplen > 0.0001) {
            const nlx = lpx / lplen;
            const nlz = lpz / lplen;
            // Read/adjust facing under mutex
            self.sim.connections_mutex.lock();
            const idx_opt_f = self.sim.entity_by_player.get(self.player_id);
            if (idx_opt_f) |idx_f| {
                if (idx_f < self.sim.dynamic_entities.items.len) {
                    var e_f = &self.sim.dynamic_entities.items[idx_f];
                    const fvx = e_f.facing_dir_xz[0];
                    const fvz = e_f.facing_dir_xz[1];
                    const dot = fvx * nlx + fvz * nlz;
                    if (dot < 0) { // angle > 90°
                        const facing_yaw = std.math.atan2(fvz, fvx);
                        const look_yaw = std.math.atan2(nlz, nlx);
                        var diff = look_yaw - facing_yaw;
                        // wrap to [-pi, pi]
                        if (diff > std.math.pi) diff -= 2 * std.math.pi;
                        if (diff < -std.math.pi) diff += 2 * std.math.pi;
                        const sign: f32 = if (diff >= 0) 1.0 else -1.0;
                        const delta = @abs(diff) - (std.math.pi / 2.0);
                        const new_yaw = facing_yaw + sign * delta;
                        e_f.facing_dir_xz = .{ @cos(new_yaw), @sin(new_yaw) };
                    }
                }
            }
            self.sim.connections_mutex.unlock();
        }
    }
}

fn cs_fph_applyMovement(ctx: *anyopaque) void {
    const self: *Client = @ptrCast(@alignCast(ctx));
    self.applyMovementInputsLocal();
}

fn cs_fph_updateCamera(ctx: *anyopaque) void {
    const self: *Client = @ptrCast(@alignCast(ctx));
    self.updateCameraFromLocal();
}

fn cs_fph_makeViewProj(ctx: *anyopaque, w_px: f32, h_px: f32) ViewProj {
    const self: *Client = @ptrCast(@alignCast(ctx));
    const aspect_wh: f32 = if (h_px > 0) (w_px / h_px) else 1.0;
    const proj = Client.makePerspective(60.0 * std.math.pi / 180.0, aspect_wh, 0.1, 1000.0);
    const view = self.makeViewNoRoll(self.camera.pos, self.camera.yaw, self.camera.pitch);
    return .{ .view = view, .proj = proj };
}

fn cs_fph_onScroll(ctx: *anyopaque, ev: sapp.Event) void {
    _ = ctx;
    _ = ev; // no-op for first person horizontal
}

const firstPersonHorizontalScheme: ControlScheme = .{
    .name = "FirstPersonHorizontal",
    .onMouseMove = cs_fph_onMouseMove,
    .onScroll = cs_fph_onScroll,
    .applyMovement = cs_fph_applyMovement,
    .updateCamera = cs_fph_updateCamera,
    .makeViewProj = cs_fph_makeViewProj,
};

const Vertex = struct { pos: [3]f32, uv: [2]f32, uv_off: [2]f32, uv_scale: [2]f32 };

// ThirdPersonIsometric control scheme
fn cs_iso_onMouseMove(ctx: *anyopaque, ev: sapp.Event) void {
    const self: *Client = @ptrCast(@alignCast(ctx));
    const sens: f32 = 0.002; // radians per pixel
    const dx = ev.mouse_dx * sens;
    // Horizontal orbit around the player (mouse right => turn right)
    self.iso_yaw += dx;
    // Normalize yaw to [-pi, pi]
    if (self.iso_yaw > std.math.pi) self.iso_yaw -= 2.0 * std.math.pi;
    if (self.iso_yaw < -std.math.pi) self.iso_yaw += 2.0 * std.math.pi;
}

fn cs_iso_onScroll(ctx: *anyopaque, ev: sapp.Event) void {
    const self: *Client = @ptrCast(@alignCast(ctx));
    const step: f32 = 1.0;
    // Positive scroll_y typically means scroll up; bring camera closer
    self.iso_dist = std.math.clamp(self.iso_dist - (ev.scroll_y * step), self.iso_min_dist, self.iso_max_dist);
}

fn cs_iso_applyMovement(ctx: *anyopaque) void {
    const self: *Client = @ptrCast(@alignCast(ctx));
    // Camera-relative movement in XZ using orbit yaw
    const yaw = self.iso_yaw;
    const fwd_x: f32 = @cos(yaw);
    const fwd_z: f32 = @sin(yaw);
    const right_x: f32 = -@sin(yaw);
    const right_z: f32 = @cos(yaw);

    var vx: f32 = 0;
    var vz: f32 = 0;
    if (self.move_forward) {
        vx += fwd_x;
        vz += fwd_z;
    }
    if (self.move_back) {
        vx -= fwd_x;
        vz -= fwd_z;
    }
    if (self.move_right) {
        vx += right_x;
        vz += right_z;
    }
    if (self.move_left) {
        vx -= right_x;
        vz -= right_z;
    }
    const mag: f32 = @sqrt(vx * vx + vz * vz);
    if (mag > 0.0001) {
        vx /= mag;
        vz /= mag;
    }
    const speed: f32 = 4.5;
    self.local_vel[0] = vx * speed;
    self.local_vel[2] = vz * speed;
    if (self.jump_pressed and self.local_on_ground) {
        const g: f32 = 9.80665;
        const target_h: f32 = 1.1;
        const v0: f32 = @sqrt(2.0 * g * target_h);
        self.local_vel[1] = v0;
        self.local_on_ground = false;
        self.jump_pending = true;
    }
    self.jump_pressed = false;
}

fn cs_iso_updateCamera(ctx: *anyopaque) void {
    const self: *Client = @ptrCast(@alignCast(ctx));
    // Desired isometric yaw/pitch control the orbit direction
    const yaw = self.iso_yaw;
    const pitch = self.iso_pitch;

    // Forward from yaw/pitch
    const cy = @cos(yaw);
    const sy = @sin(yaw);
    const cp = @cos(pitch);
    const sp = @sin(pitch);
    const fwd = [3]f32{ cp * cy, sp, cp * sy };

    // Orbit around the bottom center of the player's bounding box
    const bottom_y: f32 = self.local_pos[1];
    const target = [3]f32{ self.local_pos[0], bottom_y, self.local_pos[2] };
    self.camera.pos = .{ target[0] - fwd[0] * self.iso_dist, target[1] - fwd[1] * self.iso_dist, target[2] - fwd[2] * self.iso_dist };

    // Ensure the camera actually looks at the target (robust to any view debug transforms)
    var dir = [3]f32{ target[0] - self.camera.pos[0], target[1] - self.camera.pos[1], target[2] - self.camera.pos[2] };
    const dl: f32 = @sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
    if (dl > 0.000001) {
        dir[0] /= dl;
        dir[1] /= dl;
        dir[2] /= dl;
    }
    const look_pitch: f32 = std.math.asin(std.math.clamp(dir[1], -1.0, 1.0));
    const look_yaw: f32 = std.math.atan2(dir[2], dir[0]);
    self.camera.setYawPitch(look_yaw, look_pitch);
    self.camera.roll = 0;
    // Note: In isometric mode, the player's look vector remains independent of the camera.
}

fn makeOrthoCentered(half_width: f32, half_height: f32, znear: f32, zfar: f32) [16]f32 {
    var m: [16]f32 = [_]f32{0} ** 16;
    // column-major
    m[0] = if (half_width != 0) (1.0 / half_width) else 0.0;
    m[5] = if (half_height != 0) (1.0 / half_height) else 0.0;
    m[10] = if ((zfar - znear) != 0) (-2.0 / (zfar - znear)) else 0.0;
    m[15] = 1.0;
    // translate is zero for symmetric bounds
    m[12] = 0.0;
    m[13] = 0.0;
    m[14] = -(zfar + znear) / (zfar - znear);
    return m;
}

fn cs_iso_makeViewProj(ctx: *anyopaque, w_px: f32, h_px: f32) ViewProj {
    const self: *Client = @ptrCast(@alignCast(ctx));
    const aspect_wh: f32 = if (h_px > 0) (w_px / h_px) else 1.0;
    const proj = Client.makePerspective(60.0 * std.math.pi / 180.0, aspect_wh, 0.1, 1000.0);
    const view = self.makeViewNoRoll(self.camera.pos, self.iso_yaw, self.iso_pitch);
    return .{ .view = view, .proj = proj };
}

const thirdPersonIsometricScheme: ControlScheme = .{
    .name = "ThirdPersonIsometric",
    .onMouseMove = cs_iso_onMouseMove,
    .onScroll = cs_iso_onScroll,
    .applyMovement = cs_iso_applyMovement,
    .updateCamera = cs_iso_updateCamera,
    .makeViewProj = cs_iso_makeViewProj,
};

inline fn wrap01m(v: f32) f32 {
    const f = v - std.math.floor(v);
    return if (f == 0.0 and v > 0.0) 1.0 else f;
}

inline fn emitQuad(allocator: std.mem.Allocator, list: *std.ArrayList(Vertex), v0: [3]f32, v1: [3]f32, v2: [3]f32, v3: [3]f32, uv0: [2]f32, uv1: [2]f32, uv2: [2]f32, uv3: [2]f32, uv_scale_p: [2]f32, uv_offset_p: [2]f32) void {
    // Store tile-local UVs (may be >1) and atlas transform; shader applies fract and transform
    const off = uv_offset_p;
    const sc = uv_scale_p;
    list.append(allocator, .{ .pos = v0, .uv = uv0, .uv_off = off, .uv_scale = sc }) catch return;
    list.append(allocator, .{ .pos = v1, .uv = uv1, .uv_off = off, .uv_scale = sc }) catch return;
    list.append(allocator, .{ .pos = v2, .uv = uv2, .uv_off = off, .uv_scale = sc }) catch return;
    list.append(allocator, .{ .pos = v0, .uv = uv0, .uv_off = off, .uv_scale = sc }) catch return;
    list.append(allocator, .{ .pos = v2, .uv = uv2, .uv_off = off, .uv_scale = sc }) catch return;
    list.append(allocator, .{ .pos = v3, .uv = uv3, .uv_off = off, .uv_scale = sc }) catch return;
}

// Cached mesh types (region-level aggregation)
const SectionMesh = struct { first: u32 = 0, count: u32 = 0 };
const SectionDraw = struct { chunk_pos: gs.ChunkPos, sy: u16, first: u32, count: u32 };
const RegionMesh = struct {
    vbuf: sg.Buffer = .{},
    draws: []SectionDraw,
    built_chunk_count: usize = 0,
    dirty: bool = false,
    inflight: bool = false,
    last_built_frame: u32 = 0,
};

const MeshingJob = struct { rpos: simulation.RegionPos };
const MeshingResult = struct { rpos: simulation.RegionPos, built_chunk_count: usize, verts: []Vertex, draws: []SectionDraw };

// Snapshot types for background meshing
const SectionSnapshot = struct {
    palette: []u32,
    blocks_indices_bits: []u32,
};

const ChunkSnapshot = struct {
    pos: gs.ChunkPos,
    sections: []SectionSnapshot,
};

const RegionSnapshot = struct {
    rpos: simulation.RegionPos,
    base_cx: i32,
    base_cz: i32,
    section_count_y: u16,
    sections_below: u16,
    chunks: []ChunkSnapshot,
    // 32x32 grid mapping local (x,z) -> chunk index or -1
    grid_map: []i32,
};

// Temporary slice-ref structs for capturing under lock, then copying outside the lock
const SectionSliceRefs = struct { pal_ptr: [*]const u32, pal_len: usize, bits_ptr: [*]const u32, bits_len: usize };
const ChunkSliceRefs = struct { pos: gs.ChunkPos, sections: []SectionSliceRefs };
const RegionSliceRefs = struct { rpos: simulation.RegionPos, section_count_y: u16, sections_below: u16, base_cx: i32, base_cz: i32, chunks: []ChunkSliceRefs };

const REGION_MESH_BUDGET: usize = 128; // generous, regions are fewer than chunks

const AtlasTransform = struct {
    offset: [2]f32,
    scale: [2]f32,
};

fn bitsFor(n: usize) u6 {
    if (n <= 1) return 1;
    var v: usize = n - 1;
    var b: u6 = 0;
    while (v > 0) : (v >>= 1) b += 1;
    return if (b == 0) 1 else b;
}

fn unpackBitsGet(src: []const u32, idx: usize, bits_per_index: u6) u32 {
    // read logical element at index idx where each element has bits_per_index bits
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

fn buildQuadVerts(out: []Vertex, x0: f32, y0: f32, size: f32) usize {
    if (out.len < 6) return 0;
    const x1: f32 = x0 + size;
    const y1: f32 = y0 + size;
    out[0] = .{ .pos = .{ x0, y0 }, .uv = .{ 0, 0 } };
    out[1] = .{ .pos = .{ x1, y0 }, .uv = .{ 1, 0 } };
    out[2] = .{ .pos = .{ x1, y1 }, .uv = .{ 1, 1 } };
    out[3] = .{ .pos = .{ x0, y0 }, .uv = .{ 0, 0 } };
    out[4] = .{ .pos = .{ x1, y1 }, .uv = .{ 1, 1 } };
    out[5] = .{ .pos = .{ x0, y1 }, .uv = .{ 0, 1 } };
    return 6;
}

const UvRect = struct { u0: f32, v0: f32, u1: f32, v1: f32, w: u32, h: u32 };

fn makeFallbackTile(allocator: std.mem.Allocator, size: usize) !struct { w: u32, h: u32, pixels: []u8 } {
    const w: usize = size;
    const h: usize = size;
    const tile: usize = @max(1, size / 2); // exactly 2x2 squares per face
    var pixels = try allocator.alloc(u8, w * h * 4);
    errdefer allocator.free(pixels);
    var y: usize = 0;
    while (y < h) : (y += 1) {
        var x: usize = 0;
        while (x < w) : (x += 1) {
            const cx = (x / tile) & 1;
            const cy = (y / tile) & 1;
            const is_magenta = (cx ^ cy) == 0;
            const idx = (y * w + x) * 4;
            if (is_magenta) {
                pixels[idx + 0] = 255;
                pixels[idx + 1] = 0;
                pixels[idx + 2] = 255;
                pixels[idx + 3] = 255;
            } else {
                pixels[idx + 0] = 0;
                pixels[idx + 1] = 0;
                pixels[idx + 2] = 0;
                pixels[idx + 3] = 255;
            }
        }
    }
    return .{ .w = @intCast(w), .h = @intCast(h), .pixels = pixels };
}

fn buildAtlasFromPaths(allocator: std.mem.Allocator, paths: []const []const u8) !struct {
    atlas_pixels: []u8,
    atlas_w: u32,
    atlas_h: u32,
    rects: []UvRect,
} {
    const png = @import("png.zig");
    // Load all images (or fallback tiles) and record dims
    const N = paths.len;
    var loaded = try allocator.alloc(struct { w: u32, h: u32, pixels: []u8 }, N);
    errdefer {
        // free any allocated pixels
        var j: usize = 0;
        while (j < loaded.len) : (j += 1) {
            if (loaded[j].pixels.len > 0) allocator.free(loaded[j].pixels);
        }
        allocator.free(loaded);
    }

    var max_w: u32 = 0;
    var max_h: u32 = 0;
    var i: usize = 0;
    while (i < N) : (i += 1) {
        const img: ?png.Image = blk: {
            const res = png.loadFileRGBA8(allocator, paths[i]) catch |e| {
                std.log.warn("texture: failed to load '{s}': {s}", .{ paths[i], @errorName(e) });
                break :blk null;
            };
            break :blk res;
        };
        if (img) |im| {
            loaded[i].w = im.width;
            loaded[i].h = im.height;
            loaded[i].pixels = im.pixels; // take ownership
        } else {
            const fb = try makeFallbackTile(allocator, 16);
            loaded[i].w = fb.w;
            loaded[i].h = fb.h;
            loaded[i].pixels = fb.pixels;
        }
        if (loaded[i].w > max_w) max_w = loaded[i].w;
        if (loaded[i].h > max_h) max_h = loaded[i].h;
    }
    const tile_w: u32 = if (max_w == 0) 16 else max_w;
    const tile_h: u32 = if (max_h == 0) 16 else max_h;

    // Simple grid packing: smallest cols where cols*cols >= N
    var cols: u32 = 1;
    while (@as(usize, cols) * @as(usize, cols) < N) cols += 1;
    const rows: u32 = @intCast((N + cols - 1) / cols);
    const atlas_w: u32 = cols * tile_w;
    const atlas_h: u32 = rows * tile_h;
    var atlas_pixels = try allocator.alloc(u8, @as(usize, atlas_w) * @as(usize, atlas_h) * 4);
    @memset(atlas_pixels, 0);

    var rects = try allocator.alloc(UvRect, N);

    // Blit each tile
    i = 0;
    while (i < N) : (i += 1) {
        const c: u32 = @intCast(i % cols);
        const r: u32 = @intCast(i / cols);
        const dst_x: u32 = c * tile_w;
        const dst_y: u32 = r * tile_h;
        // Copy row by row
        var py: u32 = 0;
        while (py < loaded[i].h) : (py += 1) {
            const src_off: usize = @as(usize, py) * @as(usize, loaded[i].w) * 4;
            const dst_row_off: usize = (@as(usize, dst_y + py) * @as(usize, atlas_w) + @as(usize, dst_x)) * 4;
            const copy_bytes: usize = @as(usize, loaded[i].w) * 4;
            @memcpy(atlas_pixels[dst_row_off .. dst_row_off + copy_bytes], loaded[i].pixels[src_off .. src_off + copy_bytes]);
        }
        rects[i] = .{
            .u0 = @as(f32, @floatFromInt(dst_x)) / @as(f32, @floatFromInt(atlas_w)),
            .v0 = @as(f32, @floatFromInt(dst_y)) / @as(f32, @floatFromInt(atlas_h)),
            .u1 = @as(f32, @floatFromInt(dst_x + loaded[i].w)) / @as(f32, @floatFromInt(atlas_w)),
            .v1 = @as(f32, @floatFromInt(dst_y + loaded[i].h)) / @as(f32, @floatFromInt(atlas_h)),
            .w = loaded[i].w,
            .h = loaded[i].h,
        };
    }

    // Free individual tiles; keep atlas_pixels and rects
    i = 0;
    while (i < N) : (i += 1) allocator.free(loaded[i].pixels);
    allocator.free(loaded);

    return .{ .atlas_pixels = atlas_pixels, .atlas_w = atlas_w, .atlas_h = atlas_h, .rects = rects };
}

fn addUniquePath(allocator: std.mem.Allocator, list: *std.ArrayList([]const u8), s: []const u8) !void {
    // Avoid duplicates by linear scan (small N)
    for (list.items) |p| if (std.mem.eql(u8, p, s)) return;
    try list.append(allocator, s);
}

fn findRectForPath(paths: []const []const u8, rects: []const UvRect, s: []const u8) ?UvRect {
    var i: usize = 0;
    while (i < paths.len) : (i += 1) if (std.mem.eql(u8, paths[i], s)) return rects[i];
    return null;
}

fn buildTextureAtlas(self: *Client) void {
    // Collect unique paths from resource registry
    var paths = std.ArrayList([]const u8).initCapacity(self.allocator, 0) catch {
        self.textures_ready = true;
        return;
    };
    defer paths.deinit(self.allocator);
    var it = self.sim.reg.resources.by_block.valueIterator();
    while (it.next()) |res| {
        switch (res.*) {
            .Uniform => |u| addUniquePath(self.allocator, &paths, u.all_path) catch {},
            .Facing => |f| {
                addUniquePath(self.allocator, &paths, f.face_path) catch {};
                addUniquePath(self.allocator, &paths, f.other_path) catch {};
            },
            .Void => {},
        }
    }
    if (paths.items.len == 0) {
        // Nothing to do: create a 1x1 fallback atlas
        var one = [_]u8{ 255, 0, 255, 255 };
        self.grass_img = sg.makeImage(.{ .width = 1, .height = 1, .pixel_format = .RGBA8, .data = .{ .subimage = .{ .{ sg.asRange(one[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} } } } });
        self.atlas_w = 1;
        self.atlas_h = 1;
        self.atlas_uv_scale = .{ 1, 1 };
        self.atlas_uv_offset = .{ 0, 0 };
        // Set per-face defaults to match the selected default region
        self.atlas_uv_top_scale = self.atlas_uv_scale;
        self.atlas_uv_top_offset = self.atlas_uv_offset;
        self.atlas_uv_side_scale = self.atlas_uv_scale;
        self.atlas_uv_side_offset = self.atlas_uv_offset;
        self.textures_ready = true;
        return;
    }

    // Clear any previous atlas map entries
    self.atlas_uv_by_path.clearRetainingCapacity();

    const built = buildAtlasFromPaths(self.allocator, paths.items) catch {
        // Fallback as above on failure
        var one = [_]u8{ 255, 0, 255, 255 };
        self.grass_img = sg.makeImage(.{ .width = 1, .height = 1, .pixel_format = .RGBA8, .data = .{ .subimage = .{ .{ sg.asRange(one[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} } } } });
        self.atlas_w = 1;
        self.atlas_h = 1;
        self.atlas_uv_scale = .{ 1, 1 };
        self.atlas_uv_offset = .{ 0, 0 };
        self.atlas_uv_top_scale = self.atlas_uv_scale;
        self.atlas_uv_top_offset = self.atlas_uv_offset;
        self.atlas_uv_side_scale = self.atlas_uv_scale;
        self.atlas_uv_side_offset = self.atlas_uv_offset;
        self.textures_ready = true;
        return;
    };
    defer self.allocator.free(built.atlas_pixels);
    defer self.allocator.free(built.rects);

    // Upload atlas image to GPU
    self.grass_img = sg.makeImage(.{
        .width = @intCast(built.atlas_w),
        .height = @intCast(built.atlas_h),
        .pixel_format = .RGBA8,
        .data = .{ .subimage = .{
            .{ sg.asRange(built.atlas_pixels[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
            .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
            .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
            .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
            .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
            .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} },
        } },
    });
    self.atlas_w = built.atlas_w;
    self.atlas_h = built.atlas_h;

    // Populate atlas UV map for each source path
    var i_paths: usize = 0;
    while (i_paths < paths.items.len) : (i_paths += 1) {
        const rc = built.rects[i_paths];
        const tx: AtlasTransform = .{ .offset = .{ rc.u0, rc.v0 }, .scale = .{ rc.u1 - rc.u0, rc.v1 - rc.v0 } };
        // Note: keys are owned by registry; we do not dup or free them here
        _ = self.atlas_uv_by_path.put(paths.items[i_paths], tx) catch {};
    }

    // Choose a default atlas region to use for the current single-material demo mesh.
    // Prefer minecraft:dirt (uniform), otherwise fall back to grass_block face, then stone, then first tile.
    var chosen: bool = false;
    if (!chosen) {
        if (self.sim.reg.resources.get("minecraft:dirt")) |r_dirt| switch (r_dirt) {
            .Uniform => |u| if (findRectForPath(paths.items, built.rects, u.all_path)) |rc| {
                self.atlas_uv_offset = .{ rc.u0, rc.v0 };
                self.atlas_uv_scale = .{ rc.u1 - rc.u0, rc.v1 - rc.v0 };
                chosen = true;
            },
            else => {},
        };
    }
    if (!chosen) {
        if (self.sim.reg.resources.get("minecraft:grass")) |res| switch (res) {
            .Facing => |f| if (findRectForPath(paths.items, built.rects, f.face_path)) |rc| {
                self.atlas_uv_offset = .{ rc.u0, rc.v0 };
                self.atlas_uv_scale = .{ rc.u1 - rc.u0, rc.v1 - rc.v0 };
                chosen = true;
            },
            .Uniform => |u| if (findRectForPath(paths.items, built.rects, u.all_path)) |rc| {
                self.atlas_uv_offset = .{ rc.u0, rc.v0 };
                self.atlas_uv_scale = .{ rc.u1 - rc.u0, rc.v1 - rc.v0 };
                chosen = true;
            },
            .Void => {},
        };
    }
    if (!chosen) {
        if (self.sim.reg.resources.get("minecraft:stone")) |r_stone| switch (r_stone) {
            .Uniform => |u| if (findRectForPath(paths.items, built.rects, u.all_path)) |rc| {
                self.atlas_uv_offset = .{ rc.u0, rc.v0 };
                self.atlas_uv_scale = .{ rc.u1 - rc.u0, rc.v1 - rc.v0 };
                chosen = true;
            },
            else => {},
        };
    }
    if (!chosen and built.rects.len > 0) {
        const rc = built.rects[0];
        self.atlas_uv_offset = .{ rc.u0, rc.v0 };
        self.atlas_uv_scale = .{ rc.u1 - rc.u0, rc.v1 - rc.v0 };
    }

    // Compute per-face transforms
    // Top: prefer grass_block face, fallback to dirt, then stone, then default
    var top_set = false;
    if (!top_set) {
        if (self.sim.reg.resources.get("minecraft:grass_block")) |res| switch (res) {
            .Facing => |f| if (findRectForPath(paths.items, built.rects, f.face_path)) |rc| {
                self.atlas_uv_top_offset = .{ rc.u0, rc.v0 };
                self.atlas_uv_top_scale = .{ rc.u1 - rc.u0, rc.v1 - rc.v0 };
                top_set = true;
            },
            else => {},
        };
    }
    if (!top_set) {
        if (self.sim.reg.resources.get("minecraft:dirt")) |r_dirt2| switch (r_dirt2) {
            .Uniform => |u| if (findRectForPath(paths.items, built.rects, u.all_path)) |rc| {
                self.atlas_uv_top_offset = .{ rc.u0, rc.v0 };
                self.atlas_uv_top_scale = .{ rc.u1 - rc.u0, rc.v1 - rc.v0 };
                top_set = true;
            },
            else => {},
        };
    }
    if (!top_set) {
        self.atlas_uv_top_offset = self.atlas_uv_offset;
        self.atlas_uv_top_scale = self.atlas_uv_scale;
    }

    // Side: prefer grass_block other (dirt), fallback to dirt, then stone, then default
    var side_set = false;
    if (!side_set) {
        if (self.sim.reg.resources.get("minecraft:grass_block")) |res2| switch (res2) {
            .Facing => |f| if (findRectForPath(paths.items, built.rects, f.other_path)) |rc| {
                self.atlas_uv_side_offset = .{ rc.u0, rc.v0 };
                self.atlas_uv_side_scale = .{ rc.u1 - rc.u0, rc.v1 - rc.v0 };
                side_set = true;
            },
            else => {},
        };
    }
    if (!side_set) {
        if (self.sim.reg.resources.get("minecraft:dirt")) |r_dirt3| switch (r_dirt3) {
            .Uniform => |u| if (findRectForPath(paths.items, built.rects, u.all_path)) |rc| {
                self.atlas_uv_side_offset = .{ rc.u0, rc.v0 };
                self.atlas_uv_side_scale = .{ rc.u1 - rc.u0, rc.v1 - rc.v0 };
                side_set = true;
            },
            else => {},
        };
    }
    if (!side_set) {
        self.atlas_uv_side_offset = self.atlas_uv_offset;
        self.atlas_uv_side_scale = self.atlas_uv_scale;
    }

    self.textures_ready = true;
}

pub const Client = struct {
    allocator: std.mem.Allocator,
    sim: *simulation.Simulation,
    player_id: player.PlayerId,
    account_name: []const u8,
    connected: bool = false,
    // Connection/ready state
    ready: bool = false,

    // Texture/atlas preloading state
    textures_ready: bool = false,
    // UV transform defaults for the current demo material
    atlas_uv_scale: [2]f32 = .{ 1, 1 },
    atlas_uv_offset: [2]f32 = .{ 0, 0 },
    // Per-face transforms for the simple terrain surface (top uses grass top; sides use dirt)
    atlas_uv_top_scale: [2]f32 = .{ 1, 1 },
    atlas_uv_top_offset: [2]f32 = .{ 0, 0 },
    atlas_uv_side_scale: [2]f32 = .{ 1, 1 },
    atlas_uv_side_offset: [2]f32 = .{ 0, 0 },

    // UI / rendering state
    pass_action: sg.PassAction = .{},
    ui_scale: f32 = 2.0,
    show_debug: bool = true,
    // Backend quirks
    metal_transpose_view3x3: bool = false,

    // Debug view basis capture (for diagnostics)
    dbg_s: [3]f32 = .{ 0, 0, 0 },
    dbg_u: [3]f32 = .{ 0, 0, 0 },
    dbg_f: [3]f32 = .{ 0, 0, 0 },

    // Minimal chunk renderer state
    shd: sg.Shader = .{},
    pip: sg.Pipeline = .{},
    pip_lines: sg.Pipeline = .{},
    pip_outline: sg.Pipeline = .{},
    vbuf: sg.Buffer = .{},
    // dedicated dynamic buffer for 3D debug (e.g., player AABB)
    aabb_vbuf: sg.Buffer = .{},
    // dedicated dynamic buffer for outline (lines) to avoid multiple updates per buffer per frame
    outline_vbuf: sg.Buffer = .{},
    bind: sg.Bindings = .{},
    // We reuse grass_* for the atlas image/view binding for now
    grass_img: sg.Image = .{},
    grass_view: sg.View = .{},
    sampler: sg.Sampler = .{},
    // Debug solid-color textures
    green_img: sg.Image = .{},
    green_view: sg.View = .{},
    black_img: sg.Image = .{},
    black_view: sg.View = .{},
    atlas_w: u32 = 1,
    atlas_h: u32 = 1,
    // Atlas UV transforms per texture path
    atlas_uv_by_path: std.StringHashMap(AtlasTransform) = undefined,

    // Cached region meshes keyed by region position
    region_mesh_cache: std.AutoHashMap(simulation.RegionPos, RegionMesh) = undefined,

    // GPU buffer capacity tracking
    vbuf_capacity_bytes: usize = 0,
    aabb_vbuf_capacity_bytes: usize = 0,
    outline_vbuf_capacity_bytes: usize = 0,

    // View frustum (derived each frame from MVP) for culling
    frustum_planes: [6][4]f32 = [_][4]f32{.{ 0, 0, 0, 0 }} ** 6,
    frustum_valid: bool = false,
    prev_frustum_planes: [6][4]f32 = [_][4]f32{.{ 0, 0, 0, 0 }} ** 6,
    prev_frustum_valid: bool = false,
    // Last visible frame for chunks and regions
    last_visible_frame_chunk: std.AutoHashMap(gs.ChunkPos, u32) = undefined,
    last_visible_frame_region: std.AutoHashMap(simulation.RegionPos, u32) = undefined,
    frame_index: u32 = 0,

    // Highlight target for interaction
    highlight_has: bool = false,
    highlight_min: [3]f32 = .{ 0, 0, 0 },
    highlight_max: [3]f32 = .{ 0, 0, 0 },

    // Meshing throttle
    rebuild_budget_per_frame: u32 = 1,
    rebuilds_issued_this_frame: u32 = 0,
    rebuild_cooldown_frames: u32 = 8,

    // Background mesher
    mesher_mutex: std.Thread.Mutex = .{},
    mesher_cv: std.Thread.Condition = .{},
    mesher_jobs: std.ArrayList(MeshingJob) = undefined,
    mesher_results: std.ArrayList(MeshingResult) = undefined,
    mesher_threads: std.ArrayList(std.Thread) = undefined,
    mesher_running: bool = false,
    mesher_started: bool = false,
    adopt_budget_per_frame: u32 = 1,

    // Debug counters
    dbg_last_scheduled: u32 = 0,
    dbg_last_adopted: u32 = 0,

    // Camera mirrored from player entity (position only; orientation is client-local)
    camera: Camera = .{},

    // Control schemes
    control_schemes: []const ControlScheme = &[_]ControlScheme{},
    current_scheme: usize = 0,

    // Isometric (third-person) parameters
    iso_dist: f32 = 12.0,
    iso_min_dist: f32 = 3.0,
    iso_max_dist: f32 = 50.0,
    iso_yaw: f32 = std.math.pi / 4.0, // 45 degrees
    iso_pitch: f32 = -0.8, // pitched down a bit more to avoid horizon
    iso_ortho_half_h: f32 = 20.0,

    // Local view state (client-predicted, decoupled from simulation)
    local_yaw: f32 = 0.0,
    local_pitch: f32 = 0.0,
    local_dir: [3]f32 = .{ 1, 0, 0 },
    local_orient_initialized: bool = false,
    last_look_tick_sent: u64 = 0,

    // Local predicted player kinematics used for rendering
    local_pos: [3]f32 = .{ 0, 0, 0 },
    local_vel: [3]f32 = .{ 0, 0, 0 },
    local_aabb_half_extents: [3]f32 = .{ 0.375, 0.875, 0.2 }, // keep in sync with Simulation default
    local_on_ground: bool = false,
    last_move_tick_sent: u64 = 0,
    jump_pending: bool = false,

    // Debug: track last observed position for delta reporting
    dbg_last_pos: [3]f32 = .{ 0, 0, 0 },

    // Input state
    move_forward: bool = false,
    move_back: bool = false,
    move_left: bool = false,
    move_right: bool = false,
    jump_pressed: bool = false, // edge-triggered

    // Interaction
    reach_distance: f32 = 4.0,

    // Snapshot monitoring
    latest_snapshot: simulation.Snapshot = .{
        .tick = 0,
        .actual_tps = 0,
        .rolling_tps = 0,
        .dt_sec = 0,
        .world_seed = 0,
        .section_count_y = 0,
        .chunk_count = 0,
        .player_count = 0,
        .dynamic_entity_count = 0,
    },
    last_seen_tick: u64 = 0,

    // FPS measurement
    fps_last_ns: i128 = 0,
    fps_accum_ns: i128 = 0,
    fps_accum_frames: u32 = 0,
    fps_value: f32 = 0.0,

    pub fn init(allocator: std.mem.Allocator, sim: *simulation.Simulation, player_id: player.PlayerId, account_name: []const u8) !Client {
        const acc = try allocator.dupe(u8, account_name);
        var cli: Client = .{
            .allocator = allocator,
            .sim = sim,
            .player_id = player_id,
            .account_name = acc,
            .atlas_uv_by_path = std.StringHashMap(AtlasTransform).init(allocator),
            .region_mesh_cache = std.AutoHashMap(simulation.RegionPos, RegionMesh).init(allocator),
            .last_visible_frame_chunk = std.AutoHashMap(gs.ChunkPos, u32).init(allocator),
            .last_visible_frame_region = std.AutoHashMap(simulation.RegionPos, u32).init(allocator),
            .mesher_jobs = std.ArrayList(MeshingJob).initCapacity(allocator, 0) catch unreachable,
            .mesher_results = std.ArrayList(MeshingResult).initCapacity(allocator, 0) catch unreachable,
            .mesher_threads = std.ArrayList(std.Thread).initCapacity(allocator, 0) catch unreachable,
        };
        // Initialize control schemes (default to FirstPersonHorizontal)
        cli.control_schemes = &[_]ControlScheme{ firstPersonHorizontalScheme, thirdPersonIsometricScheme };
        cli.current_scheme = 0; // first person by default
        return cli;
    }

    fn mesherThreadMain(self: *Client) void {
        // mark started (debug)
        self.mesher_started = true;
        const alloc = std.heap.page_allocator;
        while (true) {
            // Wait for a job
            self.mesher_mutex.lock();
            while (self.mesher_running and self.mesher_jobs.items.len == 0) self.mesher_cv.wait(&self.mesher_mutex);
            if (!self.mesher_running) {
                self.mesher_mutex.unlock();
                return;
            }
            const job = self.mesher_jobs.items[self.mesher_jobs.items.len - 1];
            _ = self.mesher_jobs.pop();
            self.mesher_mutex.unlock();

            // Under world lock: gather slice refs only (no large allocations or copies)
            self.sim.worlds_mutex.lock();
            const ws_ptr = self.sim.worlds_state.getPtr("minecraft:overworld") orelse {
                self.sim.worlds_mutex.unlock();
                continue;
            };
            const rs_ptr = ws_ptr.regions.getPtr(job.rpos) orelse {
                self.sim.worlds_mutex.unlock();
                continue;
            };
            var refs = self.buildRegionRefsUnderLock(alloc, ws_ptr, rs_ptr, job.rpos);
            self.sim.worlds_mutex.unlock();

            if (refs.chunks.len == 0) {
                self.freeRegionRefs(alloc, &refs);
                continue;
            }

            // Outside the world lock: copy data into an owned snapshot and free refs
            var snap = self.copySnapshotFromRefs(alloc, &refs);
            self.freeRegionRefs(alloc, &refs);

            if (snap.chunks.len == 0) {
                self.freeRegionSnapshot(alloc, &snap);
                continue;
            }

            // Mesh from snapshot (no locks)
            const result = self.meshRegionFromSnapshot(alloc, &snap);
            self.freeRegionSnapshot(alloc, &snap);

            // Publish result
            self.mesher_mutex.lock();
            self.mesher_results.append(self.allocator, result) catch {
                if (result.verts.len > 0) alloc.free(result.verts);
                if (result.draws.len > 0) self.allocator.free(result.draws);
            };
            self.mesher_mutex.unlock();
        }
    }

    // Enable/disable raw mouse input if supported by the sokol app wrapper
    fn setRawMouse(on: bool) void {
        if (@hasDecl(sapp, "rawMouseSupported") and @hasDecl(sapp, "enableRawMouse")) {
            if (sapp.rawMouseSupported()) {
                sapp.enableRawMouse(on);
            }
        } else {}
    }

    pub fn deinit(self: *Client) void {
        // Release GPU resources if created
        if (self.vbuf.id != 0) sg.destroyBuffer(self.vbuf);
        if (self.aabb_vbuf.id != 0) sg.destroyBuffer(self.aabb_vbuf);
        if (self.outline_vbuf.id != 0) sg.destroyBuffer(self.outline_vbuf);
        if (self.grass_view.id != 0) sg.destroyView(self.grass_view);
        if (self.green_view.id != 0) sg.destroyView(self.green_view);
        if (self.black_view.id != 0) sg.destroyView(self.black_view);
        if (self.sampler.id != 0) sg.destroySampler(self.sampler);
        if (self.grass_img.id != 0) sg.destroyImage(self.grass_img);
        if (self.green_img.id != 0) sg.destroyImage(self.green_img);
        if (self.black_img.id != 0) sg.destroyImage(self.black_img);
        if (self.pip_lines.id != 0) sg.destroyPipeline(self.pip_lines);
        if (self.pip_outline.id != 0) sg.destroyPipeline(self.pip_outline);
        if (self.pip.id != 0) sg.destroyPipeline(self.pip);
        if (self.shd.id != 0) sg.destroyShader(self.shd);

        // Stop mesher threads
        if (self.mesher_running) {
            self.mesher_mutex.lock();
            self.mesher_running = false;
            self.mesher_cv.broadcast();
            self.mesher_mutex.unlock();
            var i: usize = 0;
            while (i < self.mesher_threads.items.len) : (i += 1) self.mesher_threads.items[i].join();
            self.mesher_threads.deinit(self.allocator);
            // Free any pending results
            var r: usize = 0;
            while (r < self.mesher_results.items.len) : (r += 1) {
                const mr = self.mesher_results.items[r];
                std.heap.page_allocator.free(mr.verts);
                self.allocator.free(mr.draws);
            }
            self.mesher_results.deinit(self.allocator);
            self.mesher_jobs.deinit(self.allocator);
        }

        // Deinitialize atlas map (values only; keys are borrowed from registry)
        // Destroy cached region meshes
        var it = self.region_mesh_cache.valueIterator();
        while (it.next()) |rm| {
            if (rm.vbuf.id != 0) sg.destroyBuffer(rm.vbuf);
            self.allocator.free(rm.draws);
        }
        self.region_mesh_cache.deinit();

        self.atlas_uv_by_path.deinit();
        self.last_visible_frame_chunk.deinit();
        self.last_visible_frame_region.deinit();

        // Owned string
        self.allocator.free(self.account_name);
    }

    pub fn connect(self: *Client) !void {
        if (self.connected) return;
        try self.sim.connectPlayer(self.player_id, self.account_name);
        self.connected = true;
        self.ready = false;
        // Ensure server creates/associates an entity for this player
        _ = self.sim.ensurePlayerEntity(self.player_id) catch {};
        // Do not block here; frame() will show "Connecting…" and poll readiness until the
        // player's chunk is loaded.
    }

    pub fn initGfx(self: *Client) void {
        // setup debug text and default pass action
        sdtx.setup(.{ .fonts = .{
            sdtx.fontKc853(),
            sdtx.fontKc854(),
            sdtx.fontZ1013(),
            sdtx.fontCpc(),
            sdtx.fontC64(),
            sdtx.fontOric(),
            .{},
            .{},
        } });
        sdtx.font(4);
        self.ui_scale = 2.0;
        self.pass_action.colors[0] = .{ .load_action = .CLEAR, .clear_value = .{ .r = 0, .g = 0, .b = 0, .a = 1 } };
        self.show_debug = true;

        // shader: use shdc-generated cross-platform shader
        const backend = sg.queryBackend();
        // Metal backend needs a transposed view 3x3 (no runtime toggle)
        if (backend == .METAL_MACOS) {
            self.metal_transpose_view3x3 = true;
        }
        if (@hasDecl(shd_mod, "shaderDesc")) {
            self.shd = sg.makeShader(shd_mod.shaderDesc(backend));
        } else if (@hasDecl(shd_mod, "shader_desc")) {
            self.shd = sg.makeShader(shd_mod.shader_desc(backend));
        } else if (@hasDecl(shd_mod, "chunkShaderDesc")) {
            self.shd = sg.makeShader(shd_mod.chunkShaderDesc(backend));
        } else if (@hasDecl(shd_mod, "chunk_shader_desc")) {
            self.shd = sg.makeShader(shd_mod.chunk_shader_desc(backend));
        } else {
            @compileError("shdc-generated shader module does not expose a known descriptor function");
        }

        var pdesc: sg.PipelineDesc = .{};
        pdesc.label = "chunk-pipeline";
        pdesc.shader = self.shd;
        pdesc.layout.attrs[0].format = .FLOAT3;
        pdesc.layout.attrs[1].format = .FLOAT2;
        pdesc.primitive_type = .TRIANGLES;
        pdesc.cull_mode = .BACK;
        // Front-face winding should match our emitted CCW triangles
        pdesc.face_winding = .CCW;
        pdesc.depth = .{ .compare = .LESS_EQUAL, .write_enabled = true };
        // ensure pipeline color/depth target matches swapchain format
        const desc = sg.queryDesc();
        pdesc.color_count = 1;
        pdesc.colors[0].pixel_format = desc.environment.defaults.color_format;
        pdesc.layout.attrs[2].format = .FLOAT2;
        pdesc.layout.attrs[3].format = .FLOAT2;
        self.pip = sg.makePipeline(pdesc);

        // A separate pipeline for line rendering (block outlines)
        var ldesc: sg.PipelineDesc = pdesc;
        ldesc.label = "outline-lines";
        ldesc.primitive_type = .LINES;
        self.pip_lines = sg.makePipeline(ldesc);

        // A triangle pipeline for thick outlines (render-through: depth test disabled for compare, writes off)
        var odesc: sg.PipelineDesc = pdesc;
        odesc.label = "outline-tris";
        odesc.primitive_type = .TRIANGLES;
        odesc.cull_mode = .NONE;
        odesc.depth = .{ .compare = .ALWAYS, .write_enabled = false };
        self.pip_outline = sg.makePipeline(odesc);

        // Defer starting mesher threads until first frame when this Client is stored in state
        self.mesher_running = false;
        self.mesher_started = false;
        // honor user's request: adopt 1 result per frame
        self.adopt_budget_per_frame = 1;

        // dynamic vertex buffer; start with a generous capacity (e.g., 16384 verts)
        const initial_vcount: usize = 16 * 16 * 16; // plenty for top + sides on flat worlds
        const initial_bytes: usize = initial_vcount * @sizeOf(Vertex);
        self.vbuf = sg.makeBuffer(.{ .usage = .{ .vertex_buffer = true, .stream_update = true }, .size = @intCast(initial_bytes) });
        self.vbuf_capacity_bytes = initial_bytes;
        self.bind.vertex_buffers[0] = self.vbuf;
        // separate dynamic vertex buffer for 3D debug geometry (player AABB: 12 tris = 36 verts)
        const aabb_initial_vcount: usize = 36;
        const aabb_initial_bytes: usize = aabb_initial_vcount * @sizeOf(Vertex);
        self.aabb_vbuf = sg.makeBuffer(.{ .usage = .{ .vertex_buffer = true, .stream_update = true }, .size = @intCast(aabb_initial_bytes) });
        self.aabb_vbuf_capacity_bytes = aabb_initial_bytes;
        // separate dynamic vertex buffer for outline (lines: 12 lines = 24 verts)
        const outline_initial_vcount: usize = 24;
        const outline_initial_bytes: usize = outline_initial_vcount * @sizeOf(Vertex);
        self.outline_vbuf = sg.makeBuffer(.{ .usage = .{ .vertex_buffer = true, .stream_update = true }, .size = @intCast(outline_initial_bytes) });
        self.outline_vbuf_capacity_bytes = outline_initial_bytes;
        self.pass_action.depth = .{ .load_action = .CLEAR, .clear_value = 1.0 };

        // Build texture atlas from registry resource paths (map already initialized in init)
        buildTextureAtlas(self);

        // create a view and sampler for the atlas (reusing grass_* fields)
        self.grass_view = sg.makeView(.{ .texture = .{ .image = self.grass_img } });
        self.sampler = sg.makeSampler(.{ .min_filter = .NEAREST, .mag_filter = .NEAREST, .mipmap_filter = .NEAREST, .wrap_u = .CLAMP_TO_EDGE, .wrap_v = .CLAMP_TO_EDGE });
        // shader expects VIEW_tex_texture at slot 1 and SMP_tex_sampler at slot 2
        self.bind.views[1] = self.grass_view;
        self.bind.samplers[2] = self.sampler;

        // Create a 1x1 green texture/view for debug solid color draws
        {
            var green = [_]u8{ 0, 255, 0, 255 };
            self.green_img = sg.makeImage(.{
                .width = 1,
                .height = 1,
                .pixel_format = .RGBA8,
                .data = .{ .subimage = .{ .{ sg.asRange(green[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} } } },
            });
            self.green_view = sg.makeView(.{ .texture = .{ .image = self.green_img } });
        }

        // Initialize local state and camera from player entity if available
        self.initLocalFromSim();
        self.control_schemes[self.current_scheme].updateCamera(self);
        // Defer mouse capture until world is ready (see checkReady)

        // Create a 1x1 black texture/view for debug outlines
        {
            var black = [_]u8{ 0, 0, 0, 255 };
            self.black_img = sg.makeImage(.{
                .width = 1,
                .height = 1,
                .pixel_format = .RGBA8,
                .data = .{ .subimage = .{ .{ sg.asRange(black[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} } } },
            });
            self.black_view = sg.makeView(.{ .texture = .{ .image = self.black_img } });
        }
    }

    fn buildChunkSurfaceMesh(self: *Client, out: *std.ArrayList(Vertex)) usize {
        const uv_scale_top = self.atlas_uv_top_scale;
        const uv_offset_top = self.atlas_uv_top_offset;
        const uv_scale_side = self.atlas_uv_side_scale;
        const uv_offset_side = self.atlas_uv_side_offset;
        // Pad in atlas units to avoid cross-tile sampling (use full 1.0 texel)
        const pad_u: f32 = if (self.atlas_w > 0) (1.0 / @as(f32, @floatFromInt(self.atlas_w))) else 0.0;
        const pad_v: f32 = if (self.atlas_h > 0) (1.0 / @as(f32, @floatFromInt(self.atlas_h))) else 0.0;
        // Render all loaded chunks in the overworld
        const wk = "minecraft:overworld";
        self.sim.worlds_mutex.lock();
        defer self.sim.worlds_mutex.unlock();
        const ws = self.sim.worlds_state.getPtr(wk) orelse return 0;

        // Helpers to append a quad (two triangles) with CCW winding
        const addQuad = struct {
            fn wrap01(v: f32) f32 {
                const f = v - std.math.floor(v);
                // keep upper edge at 1.0 instead of wrapping to 0.0 when v is an exact integer > 0
                return if (f == 0.0 and v > 0.0) 1.0 else f;
            }
            fn call(list: *std.ArrayList(Vertex), v0: [3]f32, v1: [3]f32, v2: [3]f32, v3: [3]f32, uv0: [2]f32, uv1: [2]f32, uv2: [2]f32, uv3: [2]f32, uv_scale_p: [2]f32, uv_offset_p: [2]f32, pad_u_in: f32, pad_v_in: f32) void {
                // Wrap UVs into [0,1] (inclusive upper edge) per tile
                const w0x: f32 = wrap01(uv0[0]);
                const w0y: f32 = wrap01(uv0[1]);
                const w1x: f32 = wrap01(uv1[0]);
                const w1y: f32 = wrap01(uv1[1]);
                const w2x: f32 = wrap01(uv2[0]);
                const w2y: f32 = wrap01(uv2[1]);
                const w3x: f32 = wrap01(uv3[0]);
                const w3y: f32 = wrap01(uv3[1]);
                // inset by full texel in atlas space to avoid bleeding, and clamp scale accordingly
                const min_u = uv_offset_p[0] + pad_u_in;
                const min_v = uv_offset_p[1] + pad_v_in;
                const range_u = @max(0.0, uv_scale_p[0] - 2.0 * pad_u_in);
                const range_v = @max(0.0, uv_scale_p[1] - 2.0 * pad_v_in);
                const t0 = .{ min_u + w0x * range_u, min_v + w0y * range_v };
                const t1 = .{ min_u + w1x * range_u, min_v + w1y * range_v };
                const t2 = .{ min_u + w2x * range_u, min_v + w2y * range_v };
                const t3 = .{ min_u + w3x * range_u, min_v + w3y * range_v };
                // Emit triangles with standard CCW winding (v0->v1->v2, v0->v2->v3)
                list.appendAssumeCapacity(.{ .pos = v0, .uv = t0 });
                list.appendAssumeCapacity(.{ .pos = v1, .uv = t1 });
                list.appendAssumeCapacity(.{ .pos = v2, .uv = t2 });
                list.appendAssumeCapacity(.{ .pos = v0, .uv = t0 });
                list.appendAssumeCapacity(.{ .pos = v2, .uv = t2 });
                list.appendAssumeCapacity(.{ .pos = v3, .uv = t3 });
            }
        }.call;

        // convenience to get heightmap value with bounds check (outside => -1)
        const getH = struct {
            fn call(hm: *const [16 * 16]i32, x: i32, z: i32) i32 {
                if (x < 0 or x >= 16 or z < 0 or z >= 16) return -1;
                return hm[@as(usize, @intCast(z)) * 16 + @as(usize, @intCast(x))];
            }
        }.call;

        const verts_before: usize = out.items.len;

        // Iterate all regions and their chunks
        var reg_it = ws.regions.iterator();
        while (reg_it.next()) |rentry| {
            const rs_val = rentry.value_ptr.*; // copy for read-only access
            for (rs_val.chunks.items) |ch| {
                const base_x: f32 = @floatFromInt(ch.pos.x * 16);
                const base_z: f32 = @floatFromInt(ch.pos.z * 16);

                // Upper bound capacity per chunk (36 verts per column)
                const cur_len: usize = out.items.len;
                out.ensureTotalCapacity(self.allocator, cur_len + 16 * 16 * 6 * 6) catch {};

                // Build top faces and simple side faces from heightmap
                const hm = ch.heightmaps.world_surface;
                for (0..16) |z| {
                    for (0..16) |x| {
                        const xi: i32 = @intCast(x);
                        const zi: i32 = @intCast(z);
                        const h = hm[z * 16 + x];
                        // Allow negative tops when the world has sections below 0; still skip if world has no below sections
                        if (h < 0 and ws.sections_below == 0) continue; // empty column in worlds with no below-zero terrain
                        const y_top: f32 = @floatFromInt(h + 1);
                        const x0: f32 = base_x + @as(f32, @floatFromInt(x));
                        const x1: f32 = x0 + 1.0;
                        const z0: f32 = base_z + @as(f32, @floatFromInt(z));
                        const z1: f32 = z0 + 1.0;
                        // Top face (upward normal), CCW seen from +Y
                        addQuad(
                            out,
                            .{ x0, y_top, z0 },
                            .{ x0, y_top, z1 },
                            .{ x1, y_top, z1 },
                            .{ x1, y_top, z0 },
                            .{ 0, 0 },
                            .{ 0, 1 },
                            .{ 1, 1 },
                            .{ 1, 0 },
                            uv_scale_top,
                            uv_offset_top,
                            pad_u,
                            pad_v,
                        );
                        // Sides if neighbor lower (within the same chunk). For inter-chunk gaps, we'll
                        // handle neighbors later — this renders a simple "wall" at chunk borders for now.
                        const h_w = getH(&hm, xi - 1, zi);
                        if (h_w < h) {
                            var yb: i32 = h_w + 1;
                            const yt: i32 = h;
                            while (yb <= yt) : (yb += 1) {
                                const y0s: f32 = @floatFromInt(yb);
                                const y1s: f32 = y0s + 1.0;
                                addQuad(
                                    out,
                                    .{ x0, y0s, z1 },
                                    .{ x0, y1s, z1 },
                                    .{ x0, y1s, z0 },
                                    .{ x0, y0s, z0 }, // -X face (one-block tall)
                                    .{ 0, 0 },
                                    .{ 0, 1 },
                                    .{ 1, 1 },
                                    .{ 1, 0 },
                                    uv_scale_side,
                                    uv_offset_side,
                                    pad_u,
                                    pad_v,
                                );
                            }
                        }
                        const h_e = getH(&hm, xi + 1, zi);
                        if (h_e < h) {
                            var yb: i32 = h_e + 1;
                            const yt: i32 = h;
                            while (yb <= yt) : (yb += 1) {
                                const y0s: f32 = @floatFromInt(yb);
                                const y1s: f32 = y0s + 1.0;
                                addQuad(
                                    out,
                                    .{ x1, y0s, z0 },
                                    .{ x1, y1s, z0 },
                                    .{ x1, y1s, z1 },
                                    .{ x1, y0s, z1 }, // +X face (one-block tall)
                                    .{ 0, 0 },
                                    .{ 0, 1 },
                                    .{ 1, 1 },
                                    .{ 1, 0 },
                                    uv_scale_side,
                                    uv_offset_side,
                                    pad_u,
                                    pad_v,
                                );
                            }
                        }
                        const h_n = getH(&hm, xi, zi - 1);
                        if (h_n < h) {
                            var yb: i32 = h_n + 1;
                            const yt: i32 = h;
                            while (yb <= yt) : (yb += 1) {
                                const y0s: f32 = @floatFromInt(yb);
                                const y1s: f32 = y0s + 1.0;
                                addQuad(
                                    out,
                                    .{ x0, y0s, z0 },
                                    .{ x0, y1s, z0 },
                                    .{ x1, y1s, z0 },
                                    .{ x1, y0s, z0 }, // -Z face (one-block tall)
                                    .{ 0, 0 },
                                    .{ 0, 1 },
                                    .{ 1, 1 },
                                    .{ 1, 0 },
                                    uv_scale_side,
                                    uv_offset_side,
                                    pad_u,
                                    pad_v,
                                );
                            }
                        }
                        const h_s = getH(&hm, xi, zi + 1);
                        if (h_s < h) {
                            var yb: i32 = h_s + 1;
                            const yt: i32 = h;
                            while (yb <= yt) : (yb += 1) {
                                const y0s: f32 = @floatFromInt(yb);
                                const y1s: f32 = y0s + 1.0;
                                addQuad(
                                    out,
                                    .{ x1, y0s, z1 },
                                    .{ x1, y1s, z1 },
                                    .{ x0, y1s, z1 },
                                    .{ x0, y0s, z1 }, // +Z face (one-block tall)
                                    .{ 0, 0 },
                                    .{ 0, 1 },
                                    .{ 1, 1 },
                                    .{ 1, 0 },
                                    uv_scale_side,
                                    uv_offset_side,
                                    pad_u,
                                    pad_v,
                                );
                            }
                        }
                    }
                }
            }
        }
        return out.items.len - verts_before;
    }

    fn checkReady(self: *Client) void {
        if (self.ready) return;
        // Determine the player's current chunk and require a 5x5 area around it to be loaded
        const wk = "minecraft:overworld";
        // Read player position under the connections mutex
        self.sim.connections_mutex.lock();
        var has_pos = false;
        var cx: i32 = 0;
        var cz: i32 = 0;
        const idx_opt = self.sim.entity_by_player.get(self.player_id);
        if (idx_opt) |idx| {
            if (idx < self.sim.dynamic_entities.items.len) {
                const e = self.sim.dynamic_entities.items[idx];
                cx = @divFloor(@as(i32, @intFromFloat(@floor(e.pos[0]))), 16);
                cz = @divFloor(@as(i32, @intFromFloat(@floor(e.pos[2]))), 16);
                has_pos = true;
            }
        }
        self.sim.connections_mutex.unlock();
        if (!has_pos) return;

        // Require all chunks in a 5x5 square centered on (cx,cz)
        const radius: i32 = 2;
        var all_loaded = true;
        var dz: i32 = -radius;
        while (dz <= radius) : (dz += 1) {
            var dx: i32 = -radius;
            while (dx <= radius) : (dx += 1) {
                if (!self.sim.isChunkLoadedAt(wk, .{ .x = cx + dx, .z = cz + dz })) {
                    all_loaded = false;
                    break;
                }
            }
            if (!all_loaded) break;
        }

        if (all_loaded) {
            self.ready = true;
            // Initialize local player and camera now that the surrounding world exists
            self.initLocalFromSim();
            self.control_schemes[self.current_scheme].updateCamera(self);
            // Now that we are ready, warp the cursor inside the app (macOS) and capture the mouse
            macosWarpCursorIntoAppWindow();
            sapp.lockMouse(true);
            setRawMouse(true);
        }
    }

    fn startMesherIfNeeded(self: *Client) void {
        if (!self.mesher_started) {
            self.mesher_running = true;
            const thread_count: usize = 4;
            self.mesher_threads.ensureTotalCapacity(self.allocator, thread_count) catch return;
            var ti: usize = 0;
            while (ti < thread_count) : (ti += 1) {
                const th = std.Thread.spawn(.{}, mesherThreadMain, .{self}) catch break;
                self.mesher_threads.appendAssumeCapacity(th);
            }
            self.mesher_started = true;
        }
    }

    fn pumpMesherScheduleAndAdopt(self: *Client) void {
        // Snapshot regions and chunk counts, schedule jobs, and adopt results without drawing
        const wk = "minecraft:overworld";
        var regions = std.ArrayList(struct { rpos: simulation.RegionPos, chunk_count: usize }).initCapacity(self.allocator, 0) catch return;
        defer regions.deinit(self.allocator);
        self.sim.worlds_mutex.lock();
        const ws = self.sim.worlds_state.getPtr(wk);
        if (ws) |ws_ptr| {
            var reg_it = ws_ptr.regions.iterator();
            while (reg_it.next()) |rentry| {
                regions.append(self.allocator, .{ .rpos = rentry.key_ptr.*, .chunk_count = rentry.value_ptr.chunks.items.len }) catch {};
            }
        }
        self.sim.worlds_mutex.unlock();
        // reset issuance counter for this prepass
        self.rebuilds_issued_this_frame = 0;
        var i: usize = 0;
        while (i < regions.items.len) : (i += 1) self.ensureRegionMesh(regions.items[i].rpos, regions.items[i].chunk_count);
        self.dbg_last_scheduled = self.rebuilds_issued_this_frame;
        // adopt after scheduling
        self.adoptMesherResults();
    }

    pub fn frame(self: *Client) void {
        // Bump frame index (used for culling hysteresis)
        self.frame_index +%= 1;

        // Frame timing and FPS
        const t_now: i128 = std.time.nanoTimestamp();
        var dt_s: f32 = 0.0;
        if (self.fps_last_ns != 0) {
            const dt_ns: i128 = t_now - self.fps_last_ns;
            if (dt_ns > 0) {
                dt_s = @as(f32, @floatFromInt(dt_ns)) / 1_000_000_000.0;
                self.fps_accum_ns += dt_ns;
                self.fps_accum_frames += 1;
                if (self.fps_accum_ns >= 500_000_000) { // 0.5s window
                    const seconds: f32 = @as(f32, @floatFromInt(self.fps_accum_ns)) / 1_000_000_000.0;
                    self.fps_value = @as(f32, @floatFromInt(self.fps_accum_frames)) / seconds;
                    self.fps_accum_ns = 0;
                    self.fps_accum_frames = 0;
                }
            }
        }
        self.fps_last_ns = t_now;

        sg.beginPass(.{ .action = self.pass_action, .swapchain = sglue.swapchain() });

        // Ensure mesher thread(s) started after this Client is fully stored
        self.startMesherIfNeeded();
        // Even if not ready, keep background mesher pumping so meshes are ready when world appears
        self.pumpMesherScheduleAndAdopt();

        // If textures not ready, draw a Texturing overlay and return early
        if (!self.textures_ready) {
            const scale: f32 = self.ui_scale;
            const w_px: f32 = @floatFromInt(sapp.width());
            const h_px: f32 = @floatFromInt(sapp.height());
            sdtx.canvas(w_px / scale, h_px / scale);
            sdtx.origin(0, 0);
            sdtx.font(4);
            const msg: [:0]const u8 = "Texturing...";
            const cols: f32 = (w_px / scale) / 8.0;
            const rows: f32 = (h_px / scale) / 8.0;
            const col_start: f32 = @max(0.0, (cols - @as(f32, @floatFromInt(msg.len))) / 2.0);
            const row_start: f32 = @max(0.0, rows / 2.0);
            sdtx.pos(col_start, row_start);
            sdtx.color3b(255, 255, 255);
            sdtx.puts(msg);
            sdtx.draw();
            sg.endPass();
            sg.commit();
            return;
        }

        // If not ready, keep mesher pumping and draw a simple Connecting overlay, then return early
        if (!self.ready) {
            // one more pump (in case new regions appeared this frame)
            self.pumpMesherScheduleAndAdopt();
            const scale: f32 = self.ui_scale;
            const w_px: f32 = @floatFromInt(sapp.width());
            const h_px: f32 = @floatFromInt(sapp.height());
            sdtx.canvas(w_px / scale, h_px / scale);
            sdtx.origin(0, 0);
            sdtx.font(4);
            // Center-ish text: compute columns and rows (8x8 font)
            const msg: [:0]const u8 = "Connecting...";
            const cols: f32 = (w_px / scale) / 8.0;
            const rows: f32 = (h_px / scale) / 8.0;
            const col_start: f32 = @max(0.0, (cols - @as(f32, @floatFromInt(msg.len))) / 2.0);
            const row_start: f32 = @max(0.0, rows / 2.0);
            sdtx.pos(col_start, row_start);
            sdtx.color3b(255, 255, 255);
            sdtx.puts(msg);
            sdtx.draw();
            sg.endPass();
            sg.commit();
            // Poll readiness for next frame
            self.checkReady();
            return;
        }

        // Apply movement/input via current control scheme and predict physics, then update camera
        self.control_schemes[self.current_scheme].applyMovement(self);
        const dt_clamped: f32 = std.math.clamp(dt_s, 0.0, 0.05);
        if (dt_clamped > 0.0) self.localPhysicsStep(dt_clamped);
        self.control_schemes[self.current_scheme].updateCamera(self);
        // Update highlight target for this frame
        self.updateHighlightTarget();
        // Notify simulation no more than once per tick
        self.maybeSendLookToSim();
        self.maybeSendMoveToSim();

        // Prepare MVP and frustum before building the mesh (used for culling)
        const w_px: f32 = @floatFromInt(sapp.width());
        const h_px: f32 = @floatFromInt(sapp.height());
        const vp = self.control_schemes[self.current_scheme].makeViewProj(self, w_px, h_px);
        var view_adj = vp.view;
        if (self.metal_transpose_view3x3) {
            view_adj = self.transposeView3x3WithEye(view_adj);
        }
        // standard MVP (proj*view)
        const mvp: [16]f32 = matMul(vp.proj, view_adj);
        self.updateFrustum(mvp);
        var vs_params: shd_mod.VsParams = .{ .mvp = mvp, .atlas_pad = .{ 0, 0 } };

        // Render cached meshes (build on demand)
        if (self.grass_img.id != 0) {
            sg.applyPipeline(self.pip);
            sg.applyUniforms(0, sg.asRange(&vs_params));
            self.renderWorldCached(&vs_params);
        }

        // Optional continuous probe logging

        // Cached rendering handled above

        // Draw targeted block outline (if any)
        self.drawTargetOutline(&vs_params);

        // Draw player bounding box in solid green for debugging camera orbit (disabled in first-person)
        if (!std.mem.eql(u8, self.control_schemes[self.current_scheme].name, "FirstPersonHorizontal")) {
            self.drawPlayerAabb(&vs_params);
        }

        if (self.show_debug) {
            const scale: f32 = self.ui_scale;
            const w_px_dbg: f32 = @floatFromInt(sapp.width());
            const h_px_dbg: f32 = @floatFromInt(sapp.height());
            sdtx.canvas(w_px_dbg / scale, h_px_dbg / scale);
            sdtx.origin(0, 0);
            sdtx.font(4);

            const snap = self.pollSnapshot().*;
            const cols_f: f32 = (w_px / scale) / 8.0;
            // Right-aligned, one datum per line
            var tbuf: [64:0]u8 = undefined;
            var slice = std.fmt.bufPrint(tbuf[0 .. tbuf.len - 1], "Tick: {d}", .{snap.tick}) catch "Tick: ?";
            tbuf[slice.len] = 0;
            var line: [:0]const u8 = tbuf[0..slice.len :0];
            var col_start_f: f32 = @max(0.0, cols_f - @as(f32, @floatFromInt(line.len)) - 1.0);
            sdtx.pos(col_start_f, 1.0);
            sdtx.color3b(255, 255, 255);
            sdtx.puts(line);

            slice = std.fmt.bufPrint(tbuf[0 .. tbuf.len - 1], "TPS: {d:.2}", .{snap.rolling_tps}) catch "TPS: ?";
            tbuf[slice.len] = 0;
            line = tbuf[0..slice.len :0];
            col_start_f = @max(0.0, cols_f - @as(f32, @floatFromInt(line.len)) - 1.0);
            sdtx.pos(col_start_f, 2.0);
            sdtx.color3b(255, 255, 255);
            sdtx.puts(line);

            slice = std.fmt.bufPrint(tbuf[0 .. tbuf.len - 1], "FPS: {d:.1}", .{self.fps_value}) catch "FPS: ?";
            tbuf[slice.len] = 0;
            line = tbuf[0..slice.len :0];
            col_start_f = @max(0.0, cols_f - @as(f32, @floatFromInt(line.len)) - 1.0);
            sdtx.pos(col_start_f, 3.0);
            sdtx.color3b(255, 255, 255);
            sdtx.puts(line);

            // Left-side debug: player's position and look vector
            // Gather current player entity safely
            self.sim.connections_mutex.lock();
            var pos: [3]f32 = .{ 0, 0, 0 };
            var look: [3]f32 = .{ 0, 0, 0 };
            const idx_opt_dbg = self.sim.entity_by_player.get(self.player_id);
            if (idx_opt_dbg) |idx_dbg| {
                if (idx_dbg < self.sim.dynamic_entities.items.len) {
                    const e_dbg = self.sim.dynamic_entities.items[idx_dbg];
                    pos = e_dbg.pos;
                    look = e_dbg.look_dir;
                }
            }
            self.sim.connections_mutex.unlock();

            var pbuf: [96:0]u8 = undefined;
            const ptxt_slice = std.fmt.bufPrint(pbuf[0 .. pbuf.len - 1], "Position: {d:.1} {d:.1} {d:.1}", .{ pos[0], pos[1], pos[2] }) catch "Position: ?";
            pbuf[ptxt_slice.len] = 0;
            const ptxt: [:0]const u8 = pbuf[0..ptxt_slice.len :0];
            sdtx.pos(1.0, 1.0);
            sdtx.color3b(255, 255, 255);
            sdtx.puts(ptxt);

            var lbuf: [96:0]u8 = undefined;
            const ltxt_slice = std.fmt.bufPrint(lbuf[0 .. lbuf.len - 1], "Look: {d:.2} {d:.2} {d:.2}", .{ look[0], look[1], look[2] }) catch "Look: ?";
            lbuf[ltxt_slice.len] = 0;
            const ltxt: [:0]const u8 = lbuf[0..ltxt_slice.len :0];
            sdtx.pos(1.0, 2.0);
            sdtx.color3b(255, 255, 255);
            sdtx.puts(ltxt);

            // World and mesher stats
            // Count regions and chunks under lock
            var regions_count: usize = 0;
            var chunks_count: usize = 0;
            self.sim.worlds_mutex.lock();
            const ws_dbg = self.sim.worlds_state.getPtr("minecraft:overworld");
            if (ws_dbg) |wsp| {
                var it = wsp.regions.iterator();
                while (it.next()) |entry| {
                    regions_count += 1;
                    chunks_count += entry.value_ptr.chunks.items.len;
                }
            }
            self.sim.worlds_mutex.unlock();

            // Mesher queues and cache counts
            var jobs_len: usize = 0;
            var results_len: usize = 0;
            self.mesher_mutex.lock();
            jobs_len = self.mesher_jobs.items.len;
            results_len = self.mesher_results.items.len;
            self.mesher_mutex.unlock();
            var rcache_count: usize = 0;
            var inflight_count: usize = 0;
            var it_rm = self.region_mesh_cache.valueIterator();
            while (it_rm.next()) |rmc| {
                rcache_count += 1;
                if (rmc.inflight) inflight_count += 1;
            }

            // World group header
            const world_hdr: [:0]const u8 = "World";
            sdtx.pos(1.0, 3.0);
            sdtx.color3b(180, 220, 255);
            sdtx.puts(world_hdr);

            // World stats under header
            var wbuf: [64:0]u8 = undefined;
            var wslice = std.fmt.bufPrint(wbuf[0 .. wbuf.len - 1], "regions: {d}", .{regions_count}) catch "regions: ?";
            wbuf[wslice.len] = 0;
            var wline: [:0]const u8 = wbuf[0..wslice.len :0];
            sdtx.pos(1.0, 4.0);
            sdtx.color3b(180, 220, 255);
            sdtx.puts(wline);

            wslice = std.fmt.bufPrint(wbuf[0 .. wbuf.len - 1], "chunks: {d}", .{chunks_count}) catch "chunks: ?";
            wbuf[wslice.len] = 0;
            wline = wbuf[0..wslice.len :0];
            sdtx.pos(1.0, 5.0);
            sdtx.color3b(180, 220, 255);
            sdtx.puts(wline);

            wslice = std.fmt.bufPrint(wbuf[0 .. wbuf.len - 1], "ready: {s}", .{if (self.ready) "true" else "false"}) catch "ready: ?";
            wbuf[wslice.len] = 0;
            wline = wbuf[0..wslice.len :0];
            sdtx.pos(1.0, 6.0);
            sdtx.color3b(180, 220, 255);
            sdtx.puts(wline);

            // Mesher group header
            const mesher_hdr: [:0]const u8 = "Mesher";
            sdtx.pos(1.0, 7.0);
            sdtx.color3b(180, 255, 180);
            sdtx.puts(mesher_hdr);

            // Mesher stats under header
            var mbuf: [72:0]u8 = undefined;
            var mslice = std.fmt.bufPrint(mbuf[0 .. mbuf.len - 1], "jobs: {d}", .{jobs_len}) catch "jobs: ?";
            mbuf[mslice.len] = 0;
            var mline: [:0]const u8 = mbuf[0..mslice.len :0];
            sdtx.pos(1.0, 8.0);
            sdtx.color3b(180, 255, 180);
            sdtx.puts(mline);

            mslice = std.fmt.bufPrint(mbuf[0 .. mbuf.len - 1], "results: {d}", .{results_len}) catch "results: ?";
            mbuf[mslice.len] = 0;
            mline = mbuf[0..mslice.len :0];
            sdtx.pos(1.0, 9.0);
            sdtx.color3b(180, 255, 180);
            sdtx.puts(mline);

            mslice = std.fmt.bufPrint(mbuf[0 .. mbuf.len - 1], "inflight: {d}", .{inflight_count}) catch "inflight: ?";
            mbuf[mslice.len] = 0;
            mline = mbuf[0..mslice.len :0];
            sdtx.pos(1.0, 10.0);
            sdtx.color3b(180, 255, 180);
            sdtx.puts(mline);

            mslice = std.fmt.bufPrint(mbuf[0 .. mbuf.len - 1], "rcache: {d}", .{rcache_count}) catch "rcache: ?";
            mbuf[mslice.len] = 0;
            mline = mbuf[0..mslice.len :0];
            sdtx.pos(1.0, 11.0);
            sdtx.color3b(180, 255, 180);
            sdtx.puts(mline);

            mslice = std.fmt.bufPrint(mbuf[0 .. mbuf.len - 1], "scheduled: {d}", .{self.dbg_last_scheduled}) catch "scheduled: ?";
            mbuf[mslice.len] = 0;
            mline = mbuf[0..mslice.len :0];
            sdtx.pos(1.0, 12.0);
            sdtx.color3b(180, 255, 180);
            sdtx.puts(mline);

            mslice = std.fmt.bufPrint(mbuf[0 .. mbuf.len - 1], "adopted: {d}", .{self.dbg_last_adopted}) catch "adopted: ?";
            mbuf[mslice.len] = 0;
            mline = mbuf[0..mslice.len :0];
            sdtx.pos(1.0, 13.0);
            sdtx.color3b(180, 255, 180);
            sdtx.puts(mline);

            mslice = std.fmt.bufPrint(mbuf[0 .. mbuf.len - 1], "threads: {d}", .{self.mesher_threads.items.len}) catch "threads: ?";
            mbuf[mslice.len] = 0;
            mline = mbuf[0..mslice.len :0];
            sdtx.pos(1.0, 14.0);
            sdtx.color3b(180, 255, 180);
            sdtx.puts(mline);

            mslice = std.fmt.bufPrint(mbuf[0 .. mbuf.len - 1], "running: {s}", .{if (self.mesher_running) "true" else "false"}) catch "running: ?";
            mbuf[mslice.len] = 0;
            mline = mbuf[0..mslice.len :0];
            sdtx.pos(1.0, 15.0);
            sdtx.color3b(180, 255, 180);
            sdtx.puts(mline);

            sdtx.draw();
        }

        sg.endPass();
        sg.commit();
    }

    pub fn event(self: *Client, ev: sapp.Event) void {
        // Ensure mouse lock state is managed even before the client is fully ready.
        if (!self.ready) {
            switch (ev.type) {
                .FOCUSED => {
                    // On macOS, the system may require a user interaction before lock fully takes effect.
                    // Warp the cursor inside the app to ensure the next click cannot hit another app.
                    macosWarpCursorIntoAppWindow();
                    // Lock the mouse immediately when the window gains focus to avoid stray first clicks
                    sapp.lockMouse(true);
                    setRawMouse(true);
                },
                .UNFOCUSED => {
                    // Always release on focus loss
                    sapp.lockMouse(false);
                    setRawMouse(false);
                    self.clearMovementInputs();
                },
                else => {},
            }
            // Ignore all other input until the client is ready (prevents affecting the simulation while connecting/loading)
            return;
        }
        switch (ev.type) {
            .KEY_UP => {
                if (ev.key_code == .F3) self.show_debug = !self.show_debug;
                if (ev.key_code == .F5) {
                    const next_idx: usize = (self.current_scheme + 1) % self.control_schemes.len;
                    const next = self.control_schemes[next_idx];
                    // If switching to first-person, sync camera from the player's look vector
                    if (std.mem.eql(u8, next.name, "FirstPersonHorizontal")) {
                        var look: [3]f32 = self.local_dir;
                        // Try to fetch latest from SIM
                        self.sim.connections_mutex.lock();
                        const idx_opt_cam = self.sim.entity_by_player.get(self.player_id);
                        if (idx_opt_cam) |idx_cam| {
                            if (idx_cam < self.sim.dynamic_entities.items.len) {
                                const e_cam = self.sim.dynamic_entities.items[idx_cam];
                                look = e_cam.look_dir;
                            }
                        }
                        self.sim.connections_mutex.unlock();
                        // Compute yaw/pitch from look
                        const dx = look[0];
                        const dy = look[1];
                        const dz = look[2];
                        const len: f32 = @sqrt(dx * dx + dy * dy + dz * dz);
                        if (len > 0.000001) {
                            const nx = dx / len;
                            const ny = dy / len;
                            const nz = dz / len;
                            self.local_pitch = std.math.asin(std.math.clamp(ny, -1.0, 1.0));
                            self.local_yaw = std.math.atan2(nz, nx);
                            const cp = @cos(self.local_pitch);
                            const sp = @sin(self.local_pitch);
                            const cy = @cos(self.local_yaw);
                            const sy = @sin(self.local_yaw);
                            self.local_dir = .{ cp * cy, sp, cp * sy };
                        }
                    }
                    self.current_scheme = next_idx;
                    self.control_schemes[self.current_scheme].updateCamera(self);
                }
                if (ev.key_code == .ESCAPE) {
                    // Release mouse when ESC is pressed
                    sapp.lockMouse(false);
                    setRawMouse(false);
                    self.clearMovementInputs();
                }
                switch (ev.key_code) {
                    .W => self.move_forward = false,
                    .S => self.move_back = false,
                    .A => self.move_left = false,
                    .D => self.move_right = false,
                    else => {},
                }
            },
            .FOCUSED => {
                // On macOS, warp the cursor inside the app window to prevent the first click going elsewhere
                macosWarpCursorIntoAppWindow();
                // Lock the mouse when the window gains focus
                sapp.lockMouse(true);
                // Enable raw mouse input if supported
                setRawMouse(true);
            },
            .UNFOCUSED => {
                // Release the mouse when the window loses focus
                sapp.lockMouse(false);
                setRawMouse(false);
                self.clearMovementInputs();
            },
            .KEY_DOWN => {
                switch (ev.key_code) {
                    .W => self.move_forward = true,
                    .S => self.move_back = true,
                    .A => self.move_left = true,
                    .D => self.move_right = true,
                    .SPACE => self.jump_pressed = true,
                    else => {},
                }
            },
            .MOUSE_DOWN => {
                // If the mouse is unlocked (e.g., after ESC), clicking the window should re-lock it
                if (!sapp.mouseLocked()) {
                    // Before re-locking, warp cursor inside window on macOS to keep interaction local
                    macosWarpCursorIntoAppWindow();
                    sapp.lockMouse(true);
                    setRawMouse(true);
                } else {
                    // When in first-person scheme, left-click breaks the targeted block within reach
                    if (std.mem.eql(u8, self.control_schemes[self.current_scheme].name, "FirstPersonHorizontal")) {
                        if (ev.mouse_button == .LEFT) {
                            self.tryBreakBlockUnderCrosshair();
                        }
                    }
                }
            },
            .MOUSE_MOVE => {
                if (sapp.mouseLocked()) {
                    self.control_schemes[self.current_scheme].onMouseMove(self, ev);
                }
            },
            .MOUSE_SCROLL => {
                self.control_schemes[self.current_scheme].onScroll(self, ev);
            },
            else => {},
        }
    }

    pub fn shutdownGfx(self: *Client) void {
        _ = self; // currently nothing to do except sdtx, which main will shut down
    }

    // Pull the most recent snapshot from the simulation. Returns a pointer to the
    // internal latest_snapshot for convenience.
    pub fn pollSnapshot(self: *Client) *const simulation.Snapshot {
        const snap = self.sim.getSnapshot();
        if (snap.tick != self.last_seen_tick) {
            self.latest_snapshot = snap;
            self.last_seen_tick = snap.tick;
        }
        return &self.latest_snapshot;
    }

    // Update camera to mirror the player's entity position; keep orientation client-local
    fn maybeSendLookToSim(self: *Client) void {
        const snap = self.pollSnapshot().*;
        if (snap.tick == self.last_look_tick_sent) return;
        self.last_look_tick_sent = snap.tick;
        self.sim.connections_mutex.lock();
        const idx_opt = self.sim.entity_by_player.get(self.player_id);
        if (idx_opt) |idx| {
            if (idx < self.sim.dynamic_entities.items.len) {
                var e = &self.sim.dynamic_entities.items[idx];
                e.look_dir = self.local_dir;
            }
        }
        self.sim.connections_mutex.unlock();
    }

    // Initialize local predicted player state from the authoritative simulation
    fn initLocalFromSim(self: *Client) void {
        self.sim.connections_mutex.lock();
        defer self.sim.connections_mutex.unlock();
        const idx_opt = self.sim.entity_by_player.get(self.player_id);
        if (idx_opt) |idx| {
            if (idx < self.sim.dynamic_entities.items.len) {
                const e = self.sim.dynamic_entities.items[idx];
                self.local_pos = e.pos;
                self.local_vel = e.vel;
                self.local_aabb_half_extents = e.aabb_half_extents;
                self.local_on_ground = e.flags.on_ground;
                if (!self.local_orient_initialized) {
                    // Seed local orientation from sim
                    const dx = e.look_dir[0];
                    const dy = e.look_dir[1];
                    const dz = e.look_dir[2];
                    const len: f32 = @sqrt(dx * dx + dy * dy + dz * dz);
                    if (len > 0.000001) {
                        const nx = dx / len;
                        const ny = dy / len;
                        const nz = dz / len;
                        self.local_pitch = std.math.asin(std.math.clamp(ny, -1.0, 1.0));
                        self.local_yaw = std.math.atan2(nz, nx);
                        const cp = @cos(self.local_pitch);
                        const sp = @sin(self.local_pitch);
                        const cy = @cos(self.local_yaw);
                        const sy = @sin(self.local_yaw);
                        self.local_dir = .{ cp * cy, sp, cp * sy };
                        self.local_orient_initialized = true;
                    }
                }
            }
        }
    }

    // Update camera to mirror the client-local predicted player position; keep orientation client-local
    fn updateCameraFromLocal(self: *Client) void {
        const eye_y: f32 = self.local_pos[1] + self.local_aabb_half_extents[1];
        self.camera.pos = .{ self.local_pos[0], eye_y, self.local_pos[2] };
        self.camera.setYawPitch(self.local_yaw, self.local_pitch);
        self.camera.roll = 0;
    }

    fn updateHighlightTarget(self: *Client) void {
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
                const bid: u32 = s.palette[pidx];
                if (bid == 0) continue;
                if (bid < self.sim.reg.blocks.items.len and !self.sim.reg.blocks.items[@intCast(bid)].full_block_collision) continue;
                // set highlight
                self.highlight_has = true;
                self.highlight_min = .{ @as(f32, @floatFromInt(ix)), @as(f32, @floatFromInt(iy)), @as(f32, @floatFromInt(iz)) };
                self.highlight_max = .{ self.highlight_min[0] + 1.0, self.highlight_min[1] + 1.0, self.highlight_min[2] + 1.0 };
                break;
            }
        }
        self.sim.worlds_mutex.unlock();
    }

    fn drawTargetOutline(self: *Client, vs_in: *const shd_mod.VsParams) void {
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
            fn call(cli: *Client, ws: *const simulation.WorldState, tx: i32, ty: i32, tz: i32, face_center: [3]f32, face_normal: [3]f32, cam_pos: [3]f32) bool {
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
                    const bid: u32 = s.palette[pidx];
                    if (bid == 0) continue;
                    if (bid < cli.sim.reg.blocks.items.len) {
                        if (cli.sim.reg.blocks.items[@intCast(bid)].full_block_collision) {
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
                    in_frustum = aabbInFrustum(self.frustum_planes, fc, fc, 0.0);
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
            .{ 2, 4 }, // p000-p100: -Y, -Z
            .{ 2, 1 }, // p100-p101: -Y, +X
            .{ 2, 5 }, // p101-p001: -Y, +Z
            .{ 2, 0 }, // p001-p000: -Y, -X
            .{ 3, 4 }, // p010-p110: +Y, -Z
            .{ 3, 1 }, // p110-p111: +Y, +X
            .{ 3, 5 }, // p111-p011: +Y, +Z
            .{ 3, 0 }, // p011-p010: +Y, -X
            .{ 0, 4 }, // p000-p010: -X, -Z
            .{ 1, 4 }, // p100-p110: +X, -Z
            .{ 1, 5 }, // p101-p111: +X, +Z
            .{ 0, 5 }, // p001-p011: -X, +Z
        };

        // Build quads (two triangles per edge) facing the camera using a screen-space thickness
        var verts: [12 * 6]Vertex = undefined; // 12 edges * 2 tris * 3 verts
        var vi: usize = 0;
        const make_vert = struct {
            fn call(p: [3]f32) Vertex {
                return .{ .pos = p, .uv = .{ 0, 0 }, .uv_off = .{ 0, 0 }, .uv_scale = .{ 0, 0 } };
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
        var vs_params: shd_mod.VsParams = .{ .mvp = vs_in.mvp, .atlas_pad = .{ 0, 0 } };
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

    fn tryBreakBlockUnderCrosshair(self: *Client) void {
        // Only operate when textures are ready and world is ready
        if (!self.ready or !self.textures_ready) return;
        const wk = "minecraft:overworld";
        // Ray origin and direction
        const eye = self.camera.pos;
        var dir = self.local_dir;
        // normalize dir
        const dl: f32 = @sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
        if (dl < 0.000001) return;
        dir[0] /= dl;
        dir[1] /= dl;
        dir[2] /= dl;
        const reach: f32 = self.reach_distance;
        const step: f32 = 0.05; // small step for short reach

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
                // y -> section-space
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
                const bid: u32 = s.palette[pidx];
                if (bid == 0) continue;
                // respect collision flag
                if (bid < self.sim.reg.blocks.items.len) {
                    if (!self.sim.reg.blocks.items[@intCast(bid)].full_block_collision) continue;
                }
                // hit!
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

            if (hit_found) {
                // Set to air within the chunk under lock
                const rp_hit = simulation.RegionPos{ .x = @divFloor(hit_cx, 32), .z = @divFloor(hit_cz, 32) };
                if (wsp.regions.getPtr(rp_hit)) |rs_hit| {
                    if (rs_hit.chunk_index.get(.{ .x = hit_cx, .z = hit_cz })) |cidx_hit| {
                        var chp = &rs_hit.chunks.items[cidx_hit];
                        // y -> section-space
                        const ny0h: i32 = hit_y + @as(i32, @intCast(wsp.sections_below)) * @as(i32, @intCast(constants.section_height));
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
                                    if (sref.palette[pi] == 0) {
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
                                                const bid_scan: u32 = sscan.palette[p_scan];
                                                if (bid_scan != 0) {
                                                    // respect solid flag
                                                    if (bid_scan < self.sim.reg.blocks.items.len and !self.sim.reg.blocks.items[@intCast(bid_scan)].full_block_collision) {
                                                        continue;
                                                    }
                                                    const abs_y: i32 = (@as(i32, @intCast(sy_u)) * @as(i32, @intCast(constants.section_height)) + @as(i32, @intCast(ly_u))) - @as(i32, @intCast(wsp.sections_below)) * @as(i32, @intCast(constants.section_height));
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
                                    if (self.region_mesh_cache.getPtr(rp_hit)) |rm| {
                                        rm.dirty = true;
                                    }
                                    // Also dirty neighbor region meshes if we edited a chunk-edge column,
                                    // since their meshing samples cross-chunk solids.
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
        }
        self.sim.worlds_mutex.unlock();
    }

    // Apply currently-pressed movement keys to the client-local predicted player
    fn applyMovementInputsLocal(self: *Client) void {
        // Derive forward from yaw only (horizontal heading), independent of pitch
        const fwd_x: f32 = @cos(self.local_yaw);
        const fwd_z: f32 = @sin(self.local_yaw);
        // Screen-right (world right) is cross(forward, up) on XZ: (-fwd_z, fwd_x)
        const right_x: f32 = -fwd_z;
        const right_z: f32 = fwd_x;

        var vx: f32 = 0;
        var vz: f32 = 0;
        if (self.move_forward) {
            vx += fwd_x;
            vz += fwd_z;
        }
        if (self.move_back) {
            vx -= fwd_x;
            vz -= fwd_z;
        }
        if (self.move_right) {
            vx += right_x;
            vz += right_z;
        }
        if (self.move_left) {
            vx -= right_x;
            vz -= right_z;
        }
        // Normalize if any input
        const mag: f32 = @sqrt(vx * vx + vz * vz);
        if (mag > 0.0001) {
            vx /= mag;
            vz /= mag;
        }
        const speed: f32 = 4.5; // m/s walking speed
        self.local_vel[0] = vx * speed;
        self.local_vel[2] = vz * speed;

        // If moving, keep facing synced to the horizontal component of the look vector
        if (self.move_forward or self.move_back or self.move_left or self.move_right) {
            // Note: facing_dir_xz is only written back to SIM in maybeSendMoveToSim()
        }

        // Jump impulse on edge press if on ground (client-local)
        if (self.jump_pressed and self.local_on_ground) {
            const g: f32 = 9.80665;
            const target_h: f32 = 1.1; // slightly over 1 block
            const v0: f32 = @sqrt(2.0 * g * target_h);
            self.local_vel[1] = v0;
            self.local_on_ground = false;
            self.jump_pending = true; // request SIM jump on next tick
        }
        // consume jump edge
        self.jump_pressed = false;
    }

    fn isSolidAt(self: *Client, ws: *const simulation.WorldState, start_chunk: *const gs.Chunk, cpos_x_in: i32, cpos_z_in: i32, lx_in: i32, abs_y_in: i32, lz_in: i32) bool {
        var nx = lx_in;
        const ny0 = abs_y_in + @as(i32, @intCast(ws.sections_below)) * @as(i32, @intCast(constants.section_height));
        var nz = lz_in;
        var cpos_x = cpos_x_in;
        var cpos_z = cpos_z_in;
        var chn: *const gs.Chunk = start_chunk;

        // Cross-chunk in X
        if (nx < 0) {
            cpos_x -= 1;
            const rp_w = simulation.RegionPos{ .x = @divFloor(cpos_x, 32), .z = @divFloor(cpos_z, 32) };
            const rsn = ws.regions.getPtr(rp_w) orelse return false;
            const idxn = rsn.chunk_index.get(.{ .x = cpos_x, .z = cpos_z }) orelse return false;
            chn = &rsn.chunks.items[idxn];
            nx += @as(i32, @intCast(constants.chunk_size_x));
        } else if (nx >= @as(i32, @intCast(constants.chunk_size_x))) {
            cpos_x += 1;
            const rp_e = simulation.RegionPos{ .x = @divFloor(cpos_x, 32), .z = @divFloor(cpos_z, 32) };
            const rsn2 = ws.regions.getPtr(rp_e) orelse return false;
            const idxn2 = rsn2.chunk_index.get(.{ .x = cpos_x, .z = cpos_z }) orelse return false;
            chn = &rsn2.chunks.items[idxn2];
            nx -= @as(i32, @intCast(constants.chunk_size_x));
        }
        // Cross-chunk in Z
        if (nz < 0) {
            cpos_z -= 1;
            const rp_n = simulation.RegionPos{ .x = @divFloor(cpos_x, 32), .z = @divFloor(cpos_z, 32) };
            const rsn3 = ws.regions.getPtr(rp_n) orelse return false;
            const idxn3 = rsn3.chunk_index.get(.{ .x = cpos_x, .z = cpos_z }) orelse return false;
            chn = &rsn3.chunks.items[idxn3];
            nz += @as(i32, @intCast(constants.chunk_size_z));
        } else if (nz >= @as(i32, @intCast(constants.chunk_size_z))) {
            cpos_z += 1;
            const rp_s = simulation.RegionPos{ .x = @divFloor(cpos_x, 32), .z = @divFloor(cpos_z, 32) };
            const rsn4 = ws.regions.getPtr(rp_s) orelse return false;
            const idxn4 = rsn4.chunk_index.get(.{ .x = cpos_x, .z = cpos_z }) orelse return false;
            chn = &rsn4.chunks.items[idxn4];
            nz -= @as(i32, @intCast(constants.chunk_size_z));
        }

        // Y in-bounds
        const total_y: i32 = @intCast(chn.sections.len * constants.section_height);
        if (ny0 < 0 or ny0 >= total_y) return false;
        const n_sy_idx: usize = @intCast(@divTrunc(ny0, @as(i32, @intCast(constants.section_height))));
        const n_ly: usize = @intCast(@mod(ny0, @as(i32, @intCast(constants.section_height))));
        const s = chn.sections[n_sy_idx];
        const bpi: u6 = bitsFor(s.palette.len);
        const idx: usize = @as(usize, @intCast(nz)) * (constants.chunk_size_x * constants.section_height) + @as(usize, @intCast(nx)) * constants.section_height + n_ly;
        const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, idx, bpi));
        if (pidx >= s.palette.len) return false;
        const bid: u32 = s.palette[pidx];
        if (bid == 0) return false;
        if (bid < self.sim.reg.blocks.items.len) {
            return self.sim.reg.blocks.items[@intCast(bid)].full_block_collision;
        }
        return true;
    }

    fn buildSectionVertices(self: *Client, ws: *const simulation.WorldState, ch: *const gs.Chunk, sy: usize, base_x: f32, base_z: f32, out: *std.ArrayList(Vertex)) void {
        const s = ch.sections[sy];
        const pal_len = s.palette.len;
        const bpi: u6 = bitsFor(pal_len);
        // Build per-palette info (draw, solid, top/side transforms)
        const PalInfo = struct {
            draw: bool,
            solid: bool,
            top_tx: AtlasTransform,
            side_tx: AtlasTransform,
        };
        var pal_info = std.ArrayList(PalInfo).initCapacity(self.allocator, pal_len) catch {
            return;
        };
        defer pal_info.deinit(self.allocator);
        var pi: usize = 0;
        while (pi < pal_len) : (pi += 1) {
            const bid: u32 = s.palette[pi];
            var pinfo: PalInfo = .{
                .draw = true,
                .solid = true,
                .top_tx = .{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale },
                .side_tx = .{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale },
            };
            if (bid < self.sim.reg.blocks.items.len) {
                pinfo.solid = self.sim.reg.blocks.items[@intCast(bid)].full_block_collision;
            }
            if (bid == 0) {
                pinfo.draw = false;
                pinfo.solid = false;
            } else {
                const name = self.sim.reg.getBlockName(@intCast(bid));
                if (self.sim.reg.resources.get(name)) |res| switch (res) {
                    .Void => {
                        pinfo.draw = false;
                        pinfo.solid = false;
                    },
                    .Uniform => |u| {
                        const tx: AtlasTransform = self.atlas_uv_by_path.get(u.all_path) orelse AtlasTransform{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale };
                        pinfo.top_tx = tx;
                        pinfo.side_tx = tx;
                    },
                    .Facing => |f| {
                        const top_tx: AtlasTransform = self.atlas_uv_by_path.get(f.face_path) orelse AtlasTransform{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale };
                        const side_tx: AtlasTransform = self.atlas_uv_by_path.get(f.other_path) orelse AtlasTransform{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale };
                        pinfo.top_tx = top_tx;
                        pinfo.side_tx = side_tx;
                    },
                };
            }
            pal_info.appendAssumeCapacity(pinfo);
        }

        // Greedy meshing for +Y (top) faces per Y-slice to drastically reduce vertex count
        const FaceMat = struct { off: [2]f32, sc: [2]f32 };
        const Mat = struct {
            fn eq(a: FaceMat, b: FaceMat) bool {
                return a.off[0] == b.off[0] and a.off[1] == b.off[1] and a.sc[0] == b.sc[0] and a.sc[1] == b.sc[1];
            }
        };
        const MaskCell = struct { present: bool, m: FaceMat };
        const section_base_y: i32 = @as(i32, @intCast(sy * constants.section_height)) - @as(i32, @intCast(ws.sections_below)) * @as(i32, @intCast(constants.section_height));
        var ly_top: usize = 0;
        while (ly_top < constants.section_height) : (ly_top += 1) {
            const abs_y: i32 = section_base_y + @as(i32, @intCast(ly_top));
            var mask: [constants.chunk_size_z * constants.chunk_size_x]MaskCell = undefined;
            // fill mask
            var mz: usize = 0;
            while (mz < constants.chunk_size_z) : (mz += 1) {
                var mx: usize = 0;
                while (mx < constants.chunk_size_x) : (mx += 1) {
                    const idx: usize = mz * constants.chunk_size_x + mx;
                    mask[idx].present = false;
                    // palette index
                    const src_idx: usize = mz * (constants.chunk_size_x * constants.section_height) + mx * constants.section_height + ly_top;
                    const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, src_idx, bpi));
                    if (pidx >= pal_info.items.len) continue;
                    const info = pal_info.items[pidx];
                    if (!info.draw) continue;
                    // top visible if neighbor above not solid
                    if (!self.isSolidAt(ws, ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(mx)), abs_y + 1, @as(i32, @intCast(mz)))) {
                        mask[idx] = .{ .present = true, .m = .{ .off = info.top_tx.offset, .sc = info.top_tx.scale } };
                    }
                }
            }
            // greedy rectangles over mask (x along width, z along height)
            mz = 0;
            while (mz < constants.chunk_size_z) : (mz += 1) {
                var mx: usize = 0;
                while (mx < constants.chunk_size_x) : (mx += 1) {
                    const base_idx: usize = mz * constants.chunk_size_x + mx;
                    if (!mask[base_idx].present) continue;
                    const mkey: FaceMat = mask[base_idx].m;
                    // determine width
                    var w: usize = 1;
                    while ((mx + w) < constants.chunk_size_x) : (w += 1) {
                        const idx2 = mz * constants.chunk_size_x + (mx + w);
                        if (!(mask[idx2].present and Mat.eq(mask[idx2].m, mkey))) break;
                    }
                    // determine height
                    var h: usize = 1;
                    outer: while ((mz + h) < constants.chunk_size_z) : (h += 1) {
                        var kx: usize = 0;
                        while (kx < w) : (kx += 1) {
                            const idx3 = (mz + h) * constants.chunk_size_x + (mx + kx);
                            if (!(mask[idx3].present and Mat.eq(mask[idx3].m, mkey))) break :outer;
                        }
                    }
                    // emit quad for rectangle (mx..mx+w, mz..mz+h) at y = abs_y + 1
                    const wx0: f32 = base_x + @as(f32, @floatFromInt(mx));
                    const wz0: f32 = base_z + @as(f32, @floatFromInt(mz));
                    const wx1: f32 = base_x + @as(f32, @floatFromInt(mx + w));
                    const wz1: f32 = base_z + @as(f32, @floatFromInt(mz + h));
                    const wy1: f32 = @as(f32, @floatFromInt(abs_y + 1));
                    // UVs repeat across the merged area (0..w, 0..h)
                    const uv00: [2]f32 = .{ 0, 0 };
                    const uv0h: [2]f32 = .{ 0, @as(f32, @floatFromInt(h)) };
                    const uvwh: [2]f32 = .{ @as(f32, @floatFromInt(w)), @as(f32, @floatFromInt(h)) };
                    const uvw0: [2]f32 = .{ @as(f32, @floatFromInt(w)), 0 };
                    emitQuad(self.allocator, out, .{ wx0, wy1, wz0 }, .{ wx0, wy1, wz1 }, .{ wx1, wy1, wz1 }, .{ wx1, wy1, wz0 }, uv00, uv0h, uvwh, uvw0, mkey.sc, mkey.off);
                    // clear mask for used cells
                    var zz: usize = 0;
                    while (zz < h) : (zz += 1) {
                        var xx: usize = 0;
                        while (xx < w) : (xx += 1) {
                            mask[(mz + zz) * constants.chunk_size_x + (mx + xx)].present = false;
                        }
                    }
                }
            }
        }

        // Now emit the remaining faces (bottom and sides) per-voxel
        for (0..constants.chunk_size_z) |lz| {
            for (0..constants.chunk_size_x) |lx| {
                for (0..constants.section_height) |ly| {
                    const idx: usize = lz * (constants.chunk_size_x * constants.section_height) + lx * constants.section_height + ly;
                    const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, idx, bpi));
                    if (pidx >= pal_info.items.len) continue;
                    const info = pal_info.items[pidx];
                    if (!info.draw) continue;

                    const wx0: f32 = base_x + @as(f32, @floatFromInt(lx));
                    const wy0: f32 = @as(f32, @floatFromInt(section_base_y + @as(i32, @intCast(ly))));
                    const wz0: f32 = base_z + @as(f32, @floatFromInt(lz));
                    const wx1: f32 = wx0 + 1.0;
                    const wy1: f32 = wy0 + 1.0;
                    const wz1: f32 = wz0 + 1.0;

                    const abs_y_this: i32 = section_base_y + @as(i32, @intCast(ly));

                    const uv00: [2]f32 = .{ 0, 0 };
                    const uv01: [2]f32 = .{ 0, 1 };
                    const uv11: [2]f32 = .{ 1, 1 };
                    const uv10: [2]f32 = .{ 1, 0 };

                    if (false) {}
                    if (!self.isSolidAt(ws, ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y_this - 1, @as(i32, @intCast(lz)))) {
                        emitQuad(self.allocator, out, .{ wx0, wy0, wz0 }, .{ wx1, wy0, wz0 }, .{ wx1, wy0, wz1 }, .{ wx0, wy0, wz1 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset);
                    }
                    if (!self.isSolidAt(ws, ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)) - 1, abs_y_this, @as(i32, @intCast(lz)))) {
                        emitQuad(self.allocator, out, .{ wx0, wy0, wz1 }, .{ wx0, wy1, wz1 }, .{ wx0, wy1, wz0 }, .{ wx0, wy0, wz0 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset);
                    }
                    if (!self.isSolidAt(ws, ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)) + 1, abs_y_this, @as(i32, @intCast(lz)))) {
                        emitQuad(self.allocator, out, .{ wx1, wy0, wz0 }, .{ wx1, wy1, wz0 }, .{ wx1, wy1, wz1 }, .{ wx1, wy0, wz1 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset);
                    }
                    if (!self.isSolidAt(ws, ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y_this, @as(i32, @intCast(lz)) - 1)) {
                        emitQuad(self.allocator, out, .{ wx0, wy0, wz0 }, .{ wx0, wy1, wz0 }, .{ wx1, wy1, wz0 }, .{ wx1, wy0, wz0 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset);
                    }
                    if (!self.isSolidAt(ws, ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y_this, @as(i32, @intCast(lz)) + 1)) {
                        emitQuad(self.allocator, out, .{ wx1, wy0, wz1 }, .{ wx1, wy1, wz1 }, .{ wx0, wy1, wz1 }, .{ wx0, wy0, wz1 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset);
                    }
                }
            }
        }
    }

    // Build a single region mesh (one buffer) by concatenating all section vertices of all chunks in the region
    fn buildRegionMeshNow(self: *Client, ws: *const simulation.WorldState, rs: *const simulation.RegionState) RegionMesh {
        var draws = self.allocator.alloc(SectionDraw, rs.chunks.items.len * @as(usize, @intCast(ws.sections_below + ws.sections_above))) catch return .{ .vbuf = .{}, .draws = &[_]SectionDraw{}, .built_chunk_count = 0, .dirty = false, .inflight = false, .last_built_frame = self.frame_index };
        var draws_len: usize = 0;
        var verts = std.ArrayList(Vertex).initCapacity(self.allocator, 0) catch {
            self.allocator.free(draws);
            return .{ .vbuf = .{}, .draws = &[_]SectionDraw{}, .built_chunk_count = 0, .dirty = false, .inflight = false, .last_built_frame = self.frame_index };
        };
        defer verts.deinit(self.allocator);
        for (rs.chunks.items) |ch| {
            const base_x: f32 = @floatFromInt(ch.pos.x * @as(i32, @intCast(constants.chunk_size_x)));
            const base_z: f32 = @floatFromInt(ch.pos.z * @as(i32, @intCast(constants.chunk_size_z)));
            var sy: usize = 0;
            while (sy < ch.sections.len) : (sy += 1) {
                const start: usize = verts.items.len;
                self.buildSectionVertices(ws, &ch, sy, base_x, base_z, &verts);
                const count: usize = verts.items.len - start;
                if (count > 0) {
                    draws[draws_len] = .{ .chunk_pos = ch.pos, .sy = @intCast(sy), .first = @intCast(start), .count = @intCast(count) };
                    draws_len += 1;
                }
            }
        }
        if (draws_len < draws.len) {
            const new_draws = self.allocator.alloc(SectionDraw, draws_len) catch {
                self.allocator.free(draws);
                return .{ .vbuf = .{}, .draws = &[_]SectionDraw{}, .built_chunk_count = 0, .dirty = false, .inflight = false, .last_built_frame = self.frame_index };
            };
            @memcpy(new_draws[0..draws_len], draws[0..draws_len]);
            self.allocator.free(draws);
            draws = new_draws;
        }
        var buf: sg.Buffer = .{};
        if (verts.items.len > 0) {
            buf = sg.makeBuffer(.{ .usage = .{ .vertex_buffer = true }, .data = sg.asRange(verts.items) });
        }
        return .{ .vbuf = buf, .draws = draws, .built_chunk_count = rs.chunks.items.len, .dirty = false, .inflight = false, .last_built_frame = self.frame_index };
    }

    // Under world lock: capture slice refs for a region (no heavy copies)
    fn buildRegionRefsUnderLock(self: *Client, alloc: std.mem.Allocator, ws: *const simulation.WorldState, rs: *const simulation.RegionState, rpos: simulation.RegionPos) RegionSliceRefs {
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

    fn freeRegionRefs(self: *Client, alloc: std.mem.Allocator, refs: *RegionSliceRefs) void {
        _ = self;
        var i: usize = 0;
        while (i < refs.chunks.len) : (i += 1) {
            const ch = refs.chunks[i];
            if (ch.sections.len > 0) alloc.free(ch.sections);
        }
        if (refs.chunks.len > 0) alloc.free(refs.chunks);
        refs.* = .{ .rpos = refs.rpos, .section_count_y = refs.section_count_y, .sections_below = refs.sections_below, .base_cx = refs.base_cx, .base_cz = refs.base_cz, .chunks = &[_]ChunkSliceRefs{} };
    }

    // Outside world lock: copy refs into an owned RegionSnapshot
    fn copySnapshotFromRefs(self: *Client, alloc: std.mem.Allocator, refs: *const RegionSliceRefs) RegionSnapshot {
        _ = self;
        var snap: RegionSnapshot = .{
            .rpos = refs.rpos,
            .base_cx = refs.base_cx,
            .base_cz = refs.base_cz,
            .section_count_y = refs.section_count_y,
            .sections_below = refs.sections_below,
            .chunks = &[_]ChunkSnapshot{},
            .grid_map = &[_]i32{},
        };
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
                var pal: []u32 = &[_]u32{};
                var bits: []u32 = &[_]u32{};
                if (sref.pal_len > 0) {
                    pal = alloc.alloc(u32, sref.pal_len) catch &[_]u32{};
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

    fn freeRegionSnapshot(self: *Client, alloc: std.mem.Allocator, snap: *RegionSnapshot) void {
        _ = self;
        // free sections and their arrays
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
        // reset
        snap.* = .{ .rpos = snap.rpos, .base_cx = snap.base_cx, .base_cz = snap.base_cz, .section_count_y = snap.section_count_y, .sections_below = snap.sections_below, .chunks = &[_]ChunkSnapshot{}, .grid_map = &[_]i32{} };
    }

    fn snapGetChunkIndex(snap: *const RegionSnapshot, cx: i32, cz: i32) ?usize {
        const lx: i32 = cx - snap.base_cx;
        const lz: i32 = cz - snap.base_cz;
        if (!(lx >= 0 and lx < 32 and lz >= 0 and lz < 32)) return null;
        const idx: usize = @intCast(@as(usize, @intCast(lz)) * 32 + @as(usize, @intCast(lx)));
        const val: i32 = snap.grid_map[idx];
        if (val < 0) return null;
        return @intCast(val);
    }

    fn isSolidAtSnapshot(self: *Client, snap: *const RegionSnapshot, start_cx: i32, start_cz: i32, lx_in: i32, abs_y_in: i32, lz_in: i32) bool {
        var lx = lx_in;
        var lz = lz_in;
        var cx = start_cx;
        var cz = start_cz;
        // cross-chunk X
        if (lx < 0) {
            cx -= 1;
            lx += @as(i32, @intCast(constants.chunk_size_x));
        } else if (lx >= @as(i32, @intCast(constants.chunk_size_x))) {
            cx += 1;
            lx -= @as(i32, @intCast(constants.chunk_size_x));
        }
        // cross-chunk Z
        if (lz < 0) {
            cz -= 1;
            lz += @as(i32, @intCast(constants.chunk_size_z));
        } else if (lz >= @as(i32, @intCast(constants.chunk_size_z))) {
            cz += 1;
            lz -= @as(i32, @intCast(constants.chunk_size_z));
        }
        const idx_opt = snapGetChunkIndex(snap, cx, cz);
        if (idx_opt == null) {
            // Outside this region snapshot: fall back to reading the live world state under lock
            // so we don't create seams at region borders.
            self.sim.worlds_mutex.lock();
            const ws_ptr = self.sim.worlds_state.getPtr("minecraft:overworld");
            if (ws_ptr) |ws_live| {
                const rp = simulation.RegionPos{ .x = @divFloor(cx, 32), .z = @divFloor(cz, 32) };
                if (ws_live.regions.getPtr(rp)) |rs_live| {
                    if (rs_live.chunk_index.get(.{ .x = cx, .z = cz })) |cidx_live| {
                        const ch_live = rs_live.chunks.items[cidx_live];
                        const total_y_live: i32 = @intCast(ch_live.sections.len * constants.section_height);
                        const ny0_live: i32 = abs_y_in + @as(i32, @intCast(snap.sections_below)) * @as(i32, @intCast(constants.section_height));
                        if (!(ny0_live < 0 or ny0_live >= total_y_live)) {
                            const sy_live: usize = @intCast(@divTrunc(ny0_live, @as(i32, @intCast(constants.section_height))));
                            const ly_live: usize = @intCast(@mod(ny0_live, @as(i32, @intCast(constants.section_height))));
                            if (sy_live < ch_live.sections.len) {
                                const s_live = ch_live.sections[sy_live];
                                const bpi_live: u6 = bitsFor(s_live.palette.len);
                                const idx3d_live: usize = @as(usize, @intCast(lz)) * (constants.chunk_size_x * constants.section_height) + @as(usize, @intCast(lx)) * constants.section_height + ly_live;
                                const pidx_live: usize = @intCast(unpackBitsGet(s_live.blocks_indices_bits, idx3d_live, bpi_live));
                                if (pidx_live < s_live.palette.len) {
                                    const bid_live: u32 = s_live.palette[pidx_live];
                                    if (bid_live == 0) {
                                        self.sim.worlds_mutex.unlock();
                                        return false;
                                    }
                                    const solid_live: bool = if (bid_live < self.sim.reg.blocks.items.len) self.sim.reg.blocks.items[@intCast(bid_live)].full_block_collision else true;
                                    self.sim.worlds_mutex.unlock();
                                    return solid_live;
                                }
                            }
                        }
                    }
                }
            }
            self.sim.worlds_mutex.unlock();
            // Default: treat as solid if we couldn't read neighbor
            return true;
        }
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
        const bid: u32 = s.palette[pidx];
        if (bid == 0) return false;
        if (bid < self.sim.reg.blocks.items.len) return self.sim.reg.blocks.items[@intCast(bid)].full_block_collision;
        return true;
    }

    fn buildSectionVerticesFromSnapshot(self: *Client, snap: *const RegionSnapshot, ch: *const ChunkSnapshot, sy: usize, base_x: f32, base_z: f32, out: *std.ArrayList(Vertex), alloc: std.mem.Allocator) void {
        const s = ch.sections[sy];
        const pal_len = s.palette.len;
        const bpi: u6 = bitsFor(pal_len);
        const PalInfo = struct {
            draw: bool,
            solid: bool,
            top_tx: AtlasTransform,
            side_tx: AtlasTransform,
        };
        var pal_info = std.ArrayList(PalInfo).initCapacity(self.allocator, pal_len) catch {
            return;
        };
        defer pal_info.deinit(self.allocator);
        var pi: usize = 0;
        while (pi < pal_len) : (pi += 1) {
            const bid: u32 = s.palette[pi];
            var pinfo: PalInfo = .{
                .draw = true,
                .solid = true,
                .top_tx = .{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale },
                .side_tx = .{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale },
            };
            if (bid < self.sim.reg.blocks.items.len) {
                pinfo.solid = self.sim.reg.blocks.items[@intCast(bid)].full_block_collision;
            }
            if (bid == 0) {
                pinfo.draw = false;
                pinfo.solid = false;
            } else {
                const name = self.sim.reg.getBlockName(@intCast(bid));
                if (self.sim.reg.resources.get(name)) |res| switch (res) {
                    .Void => {
                        pinfo.draw = false;
                        pinfo.solid = false;
                    },
                    .Uniform => |u| {
                        const tx: AtlasTransform = self.atlas_uv_by_path.get(u.all_path) orelse AtlasTransform{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale };
                        pinfo.top_tx = tx;
                        pinfo.side_tx = tx;
                    },
                    .Facing => |f| {
                        const top_tx: AtlasTransform = self.atlas_uv_by_path.get(f.face_path) orelse AtlasTransform{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale };
                        const side_tx: AtlasTransform = self.atlas_uv_by_path.get(f.other_path) orelse AtlasTransform{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale };
                        pinfo.top_tx = top_tx;
                        pinfo.side_tx = side_tx;
                    },
                };
            }
            pal_info.appendAssumeCapacity(pinfo);
        }

        // Greedy top faces per Y-slice
        const FaceMat = struct { off: [2]f32, sc: [2]f32 };
        const Mat = struct {
            fn eq(a: FaceMat, b: FaceMat) bool {
                return a.off[0] == b.off[0] and a.off[1] == b.off[1] and a.sc[0] == b.sc[0] and a.sc[1] == b.sc[1];
            }
        };
        const section_base_y: i32 = @as(i32, @intCast(sy * constants.section_height)) - @as(i32, @intCast(snap.sections_below)) * @as(i32, @intCast(constants.section_height));
        var ly_top: usize = 0;
        while (ly_top < constants.section_height) : (ly_top += 1) {
            const abs_y: i32 = section_base_y + @as(i32, @intCast(ly_top));
            var mask: [constants.chunk_size_z * constants.chunk_size_x]struct { present: bool, m: FaceMat } = undefined;
            // fill mask
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
                    if (!self.isSolidAtSnapshot(snap, ch.pos.x, ch.pos.z, @as(i32, @intCast(mx)), abs_y + 1, @as(i32, @intCast(mz)))) {
                        mask[idxm] = .{ .present = true, .m = .{ .off = info.top_tx.offset, .sc = info.top_tx.scale } };
                    }
                }
            }
            // rectangles
            mz = 0;
            while (mz < constants.chunk_size_z) : (mz += 1) {
                var mx: usize = 0;
                while (mx < constants.chunk_size_x) : (mx += 1) {
                    const base_idx: usize = mz * constants.chunk_size_x + mx;
                    if (!mask[base_idx].present) continue;
                    const mkey: FaceMat = mask[base_idx].m;
                    // width
                    var w: usize = 1;
                    while ((mx + w) < constants.chunk_size_x) : (w += 1) {
                        const idx2 = mz * constants.chunk_size_x + (mx + w);
                        if (!(mask[idx2].present and Mat.eq(mask[idx2].m, mkey))) break;
                    }
                    // height
                    var h: usize = 1;
                    outer: while ((mz + h) < constants.chunk_size_z) : (h += 1) {
                        var kx: usize = 0;
                        while (kx < w) : (kx += 1) {
                            const idx3 = (mz + h) * constants.chunk_size_x + (mx + kx);
                            if (!(mask[idx3].present and Mat.eq(mask[idx3].m, mkey))) break :outer;
                        }
                    }
                    // emit quad
                    const wx0: f32 = base_x + @as(f32, @floatFromInt(mx));
                    const wz0: f32 = base_z + @as(f32, @floatFromInt(mz));
                    const wx1: f32 = base_x + @as(f32, @floatFromInt(mx + w));
                    const wz1: f32 = base_z + @as(f32, @floatFromInt(mz + h));
                    const wy1: f32 = @as(f32, @floatFromInt(abs_y + 1));
                    const uv00: [2]f32 = .{ 0, 0 };
                    const uv0h: [2]f32 = .{ 0, @as(f32, @floatFromInt(h)) };
                    const uvwh: [2]f32 = .{ @as(f32, @floatFromInt(w)), @as(f32, @floatFromInt(h)) };
                    const uvw0: [2]f32 = .{ @as(f32, @floatFromInt(w)), 0 };
                    emitQuad(alloc, out, .{ wx0, wy1, wz0 }, .{ wx0, wy1, wz1 }, .{ wx1, wy1, wz1 }, .{ wx1, wy1, wz0 }, uv00, uv0h, uvwh, uvw0, mkey.sc, mkey.off);
                    // clear mask
                    var zz: usize = 0;
                    while (zz < h) : (zz += 1) {
                        var xx: usize = 0;
                        while (xx < w) : (xx += 1) mask[(mz + zz) * constants.chunk_size_x + (mx + xx)].present = false;
                    }
                }
            }
        }

        // Remaining faces per-voxel
        for (0..constants.chunk_size_z) |lz| {
            for (0..constants.chunk_size_x) |lx| {
                for (0..constants.section_height) |ly| {
                    const idx: usize = lz * (constants.chunk_size_x * constants.section_height) + lx * constants.section_height + ly;
                    const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, idx, bpi));
                    if (pidx >= pal_info.items.len) continue;
                    const info = pal_info.items[pidx];
                    if (!info.draw) continue;

                    const wx0: f32 = base_x + @as(f32, @floatFromInt(lx));
                    const wy0: f32 = @as(f32, @floatFromInt(section_base_y + @as(i32, @intCast(ly))));
                    const wz0: f32 = base_z + @as(f32, @floatFromInt(lz));
                    const wx1: f32 = wx0 + 1.0;
                    const wy1: f32 = wy0 + 1.0;
                    const wz1: f32 = wz0 + 1.0;

                    const abs_y_this: i32 = section_base_y + @as(i32, @intCast(ly));
                    const uv00: [2]f32 = .{ 0, 0 };
                    const uv01: [2]f32 = .{ 0, 1 };
                    const uv11: [2]f32 = .{ 1, 1 };
                    const uv10: [2]f32 = .{ 1, 0 };

                    if (!self.isSolidAtSnapshot(snap, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y_this + 1, @as(i32, @intCast(lz)))) {
                        emitQuad(alloc, out, .{ wx0, wy1, wz0 }, .{ wx0, wy1, wz1 }, .{ wx1, wy1, wz1 }, .{ wx1, wy1, wz0 }, uv00, uv01, uv11, uv10, info.top_tx.scale, info.top_tx.offset);
                    }
                    if (!self.isSolidAtSnapshot(snap, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y_this - 1, @as(i32, @intCast(lz)))) {
                        emitQuad(alloc, out, .{ wx0, wy0, wz0 }, .{ wx1, wy0, wz0 }, .{ wx1, wy0, wz1 }, .{ wx0, wy0, wz1 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset);
                    }
                    if (!self.isSolidAtSnapshot(snap, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)) - 1, abs_y_this, @as(i32, @intCast(lz)))) {
                        emitQuad(alloc, out, .{ wx0, wy0, wz1 }, .{ wx0, wy1, wz1 }, .{ wx0, wy1, wz0 }, .{ wx0, wy0, wz0 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset);
                    }
                    if (!self.isSolidAtSnapshot(snap, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)) + 1, abs_y_this, @as(i32, @intCast(lz)))) {
                        emitQuad(alloc, out, .{ wx1, wy0, wz0 }, .{ wx1, wy1, wz0 }, .{ wx1, wy1, wz1 }, .{ wx1, wy0, wz1 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset);
                    }
                    if (!self.isSolidAtSnapshot(snap, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y_this, @as(i32, @intCast(lz)) - 1)) {
                        emitQuad(alloc, out, .{ wx0, wy0, wz0 }, .{ wx0, wy1, wz0 }, .{ wx1, wy1, wz0 }, .{ wx1, wy0, wz0 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset);
                    }
                    if (!self.isSolidAtSnapshot(snap, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y_this, @as(i32, @intCast(lz)) + 1)) {
                        emitQuad(alloc, out, .{ wx1, wy0, wz1 }, .{ wx1, wy1, wz1 }, .{ wx0, wy1, wz1 }, .{ wx0, wy0, wz1 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset);
                    }
                }
            }
        }
    }

    fn meshRegionFromSnapshot(self: *Client, alloc: std.mem.Allocator, snap: *const RegionSnapshot) MeshingResult {
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
                self.buildSectionVerticesFromSnapshot(snap, &ch, sy, base_x, base_z, &verts, alloc);
                const count: usize = verts.items.len - start;
                if (count > 0) {
                    draws[draws_len] = .{ .chunk_pos = ch.pos, .sy = @intCast(sy), .first = @intCast(start), .count = @intCast(count) };
                    draws_len += 1;
                }
            }
        }
        // shrink draws
        if (draws_len < draws.len) {
            const new_draws = self.allocator.alloc(SectionDraw, draws_len) catch {
                // keep existing draws oversized
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
        // copy verts into owned slice
        var verts_owned: []Vertex = &[_]Vertex{};
        if (verts.items.len > 0) {
            verts_owned = alloc.alloc(Vertex, verts.items.len) catch &[_]Vertex{};
            if (verts_owned.len == verts.items.len) @memcpy(verts_owned[0..verts.items.len], verts.items[0..verts.items.len]);
        }
        return .{ .rpos = snap.rpos, .built_chunk_count = snap.chunks.len, .verts = verts_owned, .draws = draws };
    }

    fn adoptMesherResults(self: *Client) void {
        var processed: u32 = 0;
        while (processed < self.adopt_budget_per_frame) {
            // fetch one result
            self.mesher_mutex.lock();
            if (self.mesher_results.items.len == 0) {
                self.mesher_mutex.unlock();
                break;
            }
            const mr = self.mesher_results.items[self.mesher_results.items.len - 1];
            _ = self.mesher_results.pop();
            self.mesher_mutex.unlock();

            // Upload GPU buffer
            var buf: sg.Buffer = .{};
            if (mr.verts.len > 0) {
                buf = sg.makeBuffer(.{ .usage = .{ .vertex_buffer = true }, .data = sg.asRange(mr.verts) });
            }

            // Swap into cache
            if (self.region_mesh_cache.getPtr(mr.rpos)) |existing| {
                const old = existing.*;
                existing.* = .{ .vbuf = buf, .draws = mr.draws, .built_chunk_count = mr.built_chunk_count, .dirty = false, .inflight = false, .last_built_frame = self.frame_index };
                if (old.vbuf.id != 0) sg.destroyBuffer(old.vbuf);
                self.allocator.free(old.draws);
            } else {
                if (self.regionMeshCacheCount() < REGION_MESH_BUDGET) {
                    const rm: RegionMesh = .{ .vbuf = buf, .draws = mr.draws, .built_chunk_count = mr.built_chunk_count, .dirty = false, .inflight = false, .last_built_frame = self.frame_index };
                    _ = self.region_mesh_cache.put(mr.rpos, rm) catch {
                        if (rm.vbuf.id != 0) sg.destroyBuffer(rm.vbuf);
                        self.allocator.free(mr.draws);
                    };
                } else {
                    // over budget: drop result
                    if (buf.id != 0) sg.destroyBuffer(buf);
                    self.allocator.free(mr.draws);
                }
            }

            // free CPU verts
            if (mr.verts.len > 0) std.heap.page_allocator.free(mr.verts);
            processed += 1;
        }
        self.dbg_last_adopted = processed;
    }

    fn ensureRegionMesh(self: *Client, rpos: simulation.RegionPos, expected_chunk_count: usize) void {
        // If nothing to build yet, avoid scheduling and clear any stale inflight
        if (expected_chunk_count == 0) {
            if (self.region_mesh_cache.getPtr(rpos)) |existing0| {
                existing0.built_chunk_count = 0;
                existing0.inflight = false;
            }
            return;
        }
        // If cache entry exists and is up-to-date, nothing to do
        if (self.region_mesh_cache.getPtr(rpos)) |existing| {
            // If marked dirty, schedule a rebuild immediately (ignoring cooldown), unless already inflight or out of budget
            if (existing.dirty and !existing.inflight and (self.rebuilds_issued_this_frame < self.rebuild_budget_per_frame)) {
                existing.inflight = true;
                // leave existing.dirty set; it will be cleared when adopting results
                self.mesher_mutex.lock();
                self.mesher_jobs.append(self.allocator, .{ .rpos = rpos }) catch {};
                self.mesher_cv.broadcast();
                self.mesher_mutex.unlock();
                self.rebuilds_issued_this_frame += 1;
                return;
            }
            if (existing.built_chunk_count == expected_chunk_count and existing.vbuf.id != 0) return;
            // If a rebuild is already in-flight but appears stuck, allow reschedule after cooldown
            if (existing.inflight) {
                if ((self.frame_index - existing.last_built_frame) < self.rebuild_cooldown_frames) return;
                // Reset and fall through to schedule a fresh job
                existing.inflight = false;
            }
            // Throttle job issuance
            if (self.rebuilds_issued_this_frame >= self.rebuild_budget_per_frame) return;
            if ((self.frame_index - existing.last_built_frame) < self.rebuild_cooldown_frames) return;
            // Mark inflight and enqueue job
            existing.inflight = true;
            self.mesher_mutex.lock();
            self.mesher_jobs.append(self.allocator, .{ .rpos = rpos }) catch {};
            self.mesher_cv.broadcast();
            self.mesher_mutex.unlock();
            self.rebuilds_issued_this_frame += 1;
            return;
        }
        // No cache entry: if under budget, insert placeholder and schedule build
        if (self.rebuilds_issued_this_frame >= self.rebuild_budget_per_frame) return;
        if (self.regionMeshCacheCount() >= REGION_MESH_BUDGET) return;
        const placeholder: RegionMesh = .{ .vbuf = .{}, .draws = &[_]SectionDraw{}, .built_chunk_count = 0, .dirty = false, .inflight = true, .last_built_frame = self.frame_index };
        _ = self.region_mesh_cache.put(rpos, placeholder) catch return;
        self.mesher_mutex.lock();
        self.mesher_jobs.append(self.allocator, .{ .rpos = rpos }) catch {};
        self.mesher_cv.broadcast();
        self.mesher_mutex.unlock();
        self.rebuilds_issued_this_frame += 1;
    }

    fn renderWorldCached(self: *Client, vs_in: *const shd_mod.VsParams) void {
        // Adopt up to N meshing results per frame on the render thread
        self.adoptMesherResults();
        const wk = "minecraft:overworld";
        // Snapshot region positions and chunk counts under world lock
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

        // constants for padding
        const pad_u: f32 = if (self.atlas_w > 0) (1.0 / @as(f32, @floatFromInt(self.atlas_w))) else 0.0;
        const pad_v: f32 = if (self.atlas_h > 0) (1.0 / @as(f32, @floatFromInt(self.atlas_h))) else 0.0;
        var vs_loc: shd_mod.VsParams = .{ .mvp = vs_in.mvp, .atlas_pad = .{ pad_u, pad_v } };

        // reset per-frame rebuild counter
        self.rebuilds_issued_this_frame = 0;

        // Iterate regions without holding world lock
        var i: usize = 0;
        while (i < regions.items.len) : (i += 1) {
            const rp = regions.items[i].rpos;
            const expected_chunks = regions.items[i].chunk_count;
            // Ensure mesh scheduled or up-to-date
            self.ensureRegionMesh(rp, expected_chunks);
            const rmesh = self.region_mesh_cache.getPtr(rp) orelse continue;
            if (rmesh.vbuf.id == 0 or rmesh.draws.len == 0) continue;

            // Bind region buffer once
            var b = self.bind;
            b.vertex_buffers[0] = rmesh.vbuf;
            sg.applyBindings(b);

            // Draw visible sections in this region
            var di: usize = 0;
            while (di < rmesh.draws.len) : (di += 1) {
                const d = rmesh.draws[di];
                // Section-level culling
                if (self.frustum_valid) {
                    const base_x: f32 = @floatFromInt(d.chunk_pos.x * @as(i32, @intCast(constants.chunk_size_x)));
                    const base_z: f32 = @floatFromInt(d.chunk_pos.z * @as(i32, @intCast(constants.chunk_size_z)));
                    const y_off_sections: f32 = -@as(f32, @floatFromInt(@as(i32, @intCast(world_sections_below)) * @as(i32, @intCast(constants.section_height))));
                    const minp_s: [3]f32 = .{ base_x, y_off_sections + @as(f32, @floatFromInt((@as(usize, d.sy) * constants.section_height))), base_z };
                    const maxp_s: [3]f32 = .{ base_x + 16.0, y_off_sections + @as(f32, @floatFromInt(((@as(usize, d.sy) + 1) * constants.section_height))), base_z + 16.0 };
                    const cam = self.camera.pos;
                    const vis_sec = aabbContainsPoint(minp_s, maxp_s, cam) or aabbInFrustum(self.frustum_planes, minp_s, maxp_s, 0.5);
                    if (!vis_sec) continue;
                }
                sg.applyUniforms(0, sg.asRange(&vs_loc));
                sg.draw(d.first, d.count, 1);
                _ = self.last_visible_frame_chunk.put(d.chunk_pos, self.frame_index) catch {};
            }
            _ = self.last_visible_frame_region.put(rp, self.frame_index) catch {};
        }

        // Evict old or excess region meshes to bound pool usage
        self.evictRegionMeshes(REGION_MESH_BUDGET, 10);
    }

    fn buildChunkBlockMesh(self: *Client, out: *std.ArrayList(Vertex)) usize {
        // Helpers for UV padding and wrapping
        const pad_u: f32 = if (self.atlas_w > 0) (1.0 / @as(f32, @floatFromInt(self.atlas_w))) else 0.0;
        const pad_v: f32 = if (self.atlas_h > 0) (1.0 / @as(f32, @floatFromInt(self.atlas_h))) else 0.0;
        const addQuad = struct {
            fn wrap01(v: f32) f32 {
                const f = v - std.math.floor(v);
                return if (f == 0.0 and v > 0.0) 1.0 else f;
            }
            fn call(list: *std.ArrayList(Vertex), v0: [3]f32, v1: [3]f32, v2: [3]f32, v3: [3]f32, uv0: [2]f32, uv1: [2]f32, uv2: [2]f32, uv3: [2]f32, uv_scale_p: [2]f32, uv_offset_p: [2]f32, pad_u_in: f32, pad_v_in: f32) void {
                const w0x: f32 = wrap01(uv0[0]);
                const w0y: f32 = wrap01(uv0[1]);
                const w1x: f32 = wrap01(uv1[0]);
                const w1y: f32 = wrap01(uv1[1]);
                const w2x: f32 = wrap01(uv2[0]);
                const w2y: f32 = wrap01(uv2[1]);
                const w3x: f32 = wrap01(uv3[0]);
                const w3y: f32 = wrap01(uv3[1]);
                const min_u = uv_offset_p[0] + pad_u_in;
                const min_v = uv_offset_p[1] + pad_v_in;
                const range_u = @max(0.0, uv_scale_p[0] - 2.0 * pad_u_in);
                const range_v = @max(0.0, uv_scale_p[1] - 2.0 * pad_v_in);
                const t0 = .{ min_u + w0x * range_u, min_v + w0y * range_v };
                const t1 = .{ min_u + w1x * range_u, min_v + w1y * range_v };
                const t2 = .{ min_u + w2x * range_u, min_v + w2y * range_v };
                const t3 = .{ min_u + w3x * range_u, min_v + w3y * range_v };
                list.append(self.allocator, .{ .pos = v0, .uv = t0 }) catch return;
                list.append(self.allocator, .{ .pos = v1, .uv = t1 }) catch return;
                list.append(self.allocator, .{ .pos = v2, .uv = t2 }) catch return;
                list.append(self.allocator, .{ .pos = v0, .uv = t0 }) catch return;
                list.append(self.allocator, .{ .pos = v2, .uv = t2 }) catch return;
                list.append(self.allocator, .{ .pos = v3, .uv = t3 }) catch return;
            }
        }.call;

        // Mesh all loaded chunks in the single overworld for now
        const wk = "minecraft:overworld";
        self.sim.worlds_mutex.lock();
        defer self.sim.worlds_mutex.unlock();
        const ws = self.sim.worlds_state.getPtr(wk) orelse return 0;
        const y_off_mesh: f32 = -@as(f32, @floatFromInt(@as(i32, @intCast(ws.sections_below)) * @as(i32, @intCast(constants.section_height))));

        const verts_before: usize = out.items.len;

        var reg_it = ws.regions.iterator();
        while (reg_it.next()) |rentry| {
            const rs_ptr = rentry.value_ptr; // keep pointer to access chunk_index
            for (rs_ptr.chunks.items) |*ch_ptr| {
                const ch = ch_ptr.*; // copy for convenience
                const base_x: f32 = @floatFromInt(ch.pos.x * @as(i32, @intCast(constants.chunk_size_x)));
                const base_z: f32 = @floatFromInt(ch.pos.z * @as(i32, @intCast(constants.chunk_size_z)));

                // Frustum culling per-chunk AABB
                if (self.frustum_valid) {
                    const minp: [3]f32 = .{ base_x, y_off_mesh, base_z };
                    const maxp: [3]f32 = .{
                        base_x + @as(f32, @floatFromInt(constants.chunk_size_x)),
                        y_off_mesh + @as(f32, @floatFromInt(ch.sections.len * constants.section_height)),
                        base_z + @as(f32, @floatFromInt(constants.chunk_size_z)),
                    };
                    const cull_margin: f32 = 0.5; // tighter margin; hysteresis handles stability
                    // Always keep the chunk containing the camera
                    const cam = self.camera.pos;
                    var visible = aabbContainsPoint(minp, maxp, cam);
                    if (!visible) {
                        visible = aabbInFrustum(self.frustum_planes, minp, maxp, cull_margin);
                        if (!visible) {
                            // Per-chunk linger: keep if seen within the last 2 frames
                            if (self.last_visible_frame.get(ch.pos)) |last_f| {
                                const delta = self.frame_index - last_f;
                                if (delta <= 2) visible = true;
                            }
                        }
                    }
                    if (!visible) continue;
                    // Update last visible frame index
                    _ = self.last_visible_frame.put(ch.pos, self.frame_index) catch {};
                }

                // Rough upper bound capacity: worst-case 6 faces per voxel (very high), but we reserve moderately
                const cur_len: usize = out.items.len;
                out.ensureTotalCapacity(self.allocator, cur_len + 16 * 16 * 32 * 6) catch {};

                // Iterate sections
                const sections_len: usize = ch.sections.len;
                var sy: usize = 0;
                while (sy < sections_len) : (sy += 1) {
                    const s = ch.sections[sy];
                    const pal_len = s.palette.len;
                    const bpi: u6 = bitsFor(pal_len);
                    // Build per-palette info (draw, solid, top/side transforms)
                    const PalInfo = struct {
                        draw: bool,
                        solid: bool,
                        top_tx: AtlasTransform,
                        side_tx: AtlasTransform,
                    };
                    var pal_info = std.ArrayList(PalInfo).initCapacity(self.allocator, pal_len) catch {
                        // If alloc fails, skip this section gracefully
                        continue;
                    };
                    defer pal_info.deinit(self.allocator);
                    var pi: usize = 0;
                    while (pi < pal_len) : (pi += 1) {
                        const bid: u32 = s.palette[pi];
                        var pinfo: PalInfo = .{
                            .draw = true,
                            .solid = true,
                            .top_tx = .{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale },
                            .side_tx = .{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale },
                        };
                        // Solid from registry collision flag
                        if (bid < self.sim.reg.blocks.items.len) {
                            pinfo.solid = self.sim.reg.blocks.items[@intCast(bid)].full_block_collision;
                        }
                        // Treat block 0 (air) as not drawn and not solid
                        if (bid == 0) {
                            pinfo.draw = false;
                            pinfo.solid = false;
                        } else {
                            // Resource lookup by block name
                            const name = self.sim.reg.getBlockName(@intCast(bid));
                            if (self.sim.reg.resources.get(name)) |res| switch (res) {
                                .Void => {
                                    pinfo.draw = false;
                                    pinfo.solid = false;
                                },
                                .Uniform => |u| {
                                    const tx: AtlasTransform = self.atlas_uv_by_path.get(u.all_path) orelse AtlasTransform{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale };
                                    pinfo.top_tx = tx;
                                    pinfo.side_tx = tx;
                                },
                                .Facing => |f| {
                                    const top_tx: AtlasTransform = self.atlas_uv_by_path.get(f.face_path) orelse AtlasTransform{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale };
                                    const side_tx: AtlasTransform = self.atlas_uv_by_path.get(f.other_path) orelse AtlasTransform{ .offset = self.atlas_uv_offset, .scale = self.atlas_uv_scale };
                                    pinfo.top_tx = top_tx;
                                    pinfo.side_tx = side_tx;
                                },
                            };
                        }
                        pal_info.appendAssumeCapacity(pinfo);
                    }

                    // Precompute section base Y to avoid capturing mutable 'sy'
                    const section_base_y: i32 = @intCast(sy * constants.section_height);

                    // Iterate voxels: order used in worldgen is (lz, lx, ly) with ly fastest
                    for (0..constants.chunk_size_z) |lz| {
                        for (0..constants.chunk_size_x) |lx| {
                            for (0..constants.section_height) |ly| {
                                const idx: usize = lz * (constants.chunk_size_x * constants.section_height) + lx * constants.section_height + ly;
                                const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, idx, bpi));
                                if (pidx >= pal_info.items.len) continue;
                                const info = pal_info.items[pidx];
                                if (!info.draw) continue;

                                // World coords for this block
                                const wx0: f32 = base_x + @as(f32, @floatFromInt(lx));
                                const wy0: f32 = @as(f32, @floatFromInt(@as(i32, @intCast(sy * constants.section_height)) + @as(i32, @intCast(ly))));
                                const wz0: f32 = base_z + @as(f32, @floatFromInt(lz));
                                const wx1: f32 = wx0 + 1.0;
                                const wy1: f32 = wy0 + 1.0;
                                const wz1: f32 = wz0 + 1.0;

                                // We'll use a helper that takes absolute Y and handles cross-chunk
                                const abs_y_this: i32 = section_base_y + @as(i32, @intCast(ly));

                                // Emit faces if neighbor is not solid
                                const uv00: [2]f32 = .{ 0, 0 };
                                const uv01: [2]f32 = .{ 0, 1 };
                                const uv11: [2]f32 = .{ 1, 1 };
                                const uv10: [2]f32 = .{ 1, 0 };

                                // +Y (top)
                                if (!self.isSolidAt(ws, &ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y_this + 1, @as(i32, @intCast(lz)))) {
                                    addQuad(out, .{ wx0, wy1, wz0 }, .{ wx0, wy1, wz1 }, .{ wx1, wy1, wz1 }, .{ wx1, wy1, wz0 }, uv00, uv01, uv11, uv10, info.top_tx.scale, info.top_tx.offset, pad_u, pad_v);
                                }
                                // -Y (bottom)
                                if (!self.isSolidAt(ws, &ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y_this - 1, @as(i32, @intCast(lz)))) {
                                    addQuad(out, .{ wx0, wy0, wz0 }, .{ wx1, wy0, wz0 }, .{ wx1, wy0, wz1 }, .{ wx0, wy0, wz1 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset, pad_u, pad_v);
                                }
                                // -X (west)
                                if (!self.isSolidAt(ws, &ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)) - 1, abs_y_this, @as(i32, @intCast(lz)))) {
                                    addQuad(out, .{ wx0, wy0, wz1 }, .{ wx0, wy1, wz1 }, .{ wx0, wy1, wz0 }, .{ wx0, wy0, wz0 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset, pad_u, pad_v);
                                }
                                // +X (east)
                                if (!self.isSolidAt(ws, &ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)) + 1, abs_y_this, @as(i32, @intCast(lz)))) {
                                    addQuad(out, .{ wx1, wy0, wz0 }, .{ wx1, wy1, wz0 }, .{ wx1, wy1, wz1 }, .{ wx1, wy0, wz1 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset, pad_u, pad_v);
                                }
                                // -Z (north)
                                if (!self.isSolidAt(ws, &ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y_this, @as(i32, @intCast(lz)) - 1)) {
                                    addQuad(out, .{ wx0, wy0, wz0 }, .{ wx0, wy1, wz0 }, .{ wx1, wy1, wz0 }, .{ wx1, wy0, wz0 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset, pad_u, pad_v);
                                }
                                // +Z (south)
                                if (!self.isSolidAt(ws, &ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y_this, @as(i32, @intCast(lz)) + 1)) {
                                    addQuad(out, .{ wx1, wy0, wz1 }, .{ wx1, wy1, wz1 }, .{ wx0, wy1, wz1 }, .{ wx0, wy0, wz1 }, uv00, uv01, uv11, uv10, info.side_tx.scale, info.side_tx.offset, pad_u, pad_v);
                                }
                            }
                        }
                    }
                }
            }
        }
        return out.items.len - verts_before;
    }

    fn clearMovementInputs(self: *Client) void {
        self.move_forward = false;
        self.move_back = false;
        self.move_left = false;
        self.move_right = false;
        // zero horizontal velocity (local)
        self.local_vel[0] = 0;
        self.local_vel[2] = 0;
        // Optionally also zero on SIM at next tick
        self.jump_pending = false;
    }

    // Basic 4x4 column-major matrix helpers
    fn matMul(a: [16]f32, b: [16]f32) [16]f32 {
        var r: [16]f32 = undefined;
        var i: usize = 0;
        while (i < 4) : (i += 1) {
            var j: usize = 0;
            while (j < 4) : (j += 1) {
                // column-major: r[col*4+row]
                r[i * 4 + j] =
                    a[0 * 4 + j] * b[i * 4 + 0] +
                    a[1 * 4 + j] * b[i * 4 + 1] +
                    a[2 * 4 + j] * b[i * 4 + 2] +
                    a[3 * 4 + j] * b[i * 4 + 3];
            }
        }
        return r;
    }

    fn transpose4(m: [16]f32) [16]f32 {
        var r: [16]f32 = undefined;
        var c: usize = 0;
        while (c < 4) : (c += 1) {
            var rrow: usize = 0;
            while (rrow < 4) : (rrow += 1) {
                r[c * 4 + rrow] = m[rrow * 4 + c];
            }
        }
        return r;
    }

    // Extract 6 view-frustum planes from a column-major MVP matrix.
    // Plane order: left, right, bottom, top, near, far. Each plane is (a,b,c,d) with outward normal.
    fn extractFrustumPlanes(m: [16]f32) [6][4]f32 {
        // rows in column-major storage
        const r0 = [4]f32{ m[0], m[4], m[8], m[12] };
        const r1 = [4]f32{ m[1], m[5], m[9], m[13] };
        const r2 = [4]f32{ m[2], m[6], m[10], m[14] };
        const r3 = [4]f32{ m[3], m[7], m[11], m[15] };
        var planes: [6][4]f32 = undefined;
        // left = r3 + r0
        planes[0] = .{ r3[0] + r0[0], r3[1] + r0[1], r3[2] + r0[2], r3[3] + r0[3] };
        // right = r3 - r0
        planes[1] = .{ r3[0] - r0[0], r3[1] - r0[1], r3[2] - r0[2], r3[3] - r0[3] };
        // bottom = r3 + r1
        planes[2] = .{ r3[0] + r1[0], r3[1] + r1[1], r3[2] + r1[2], r3[3] + r1[3] };
        // top = r3 - r1
        planes[3] = .{ r3[0] - r1[0], r3[1] - r1[1], r3[2] - r1[2], r3[3] - r1[3] };
        // near = r3 + r2
        planes[4] = .{ r3[0] + r2[0], r3[1] + r2[1], r3[2] + r2[2], r3[3] + r2[3] };
        // far = r3 - r2
        planes[5] = .{ r3[0] - r2[0], r3[1] - r2[1], r3[2] - r2[2], r3[3] - r2[3] };
        // normalize planes
        var pi: usize = 0;
        while (pi < 6) : (pi += 1) {
            const a = planes[pi][0];
            const b = planes[pi][1];
            const c = planes[pi][2];
            const inv_len = 1.0 / @max(0.000001, @sqrt(a * a + b * b + c * c));
            planes[pi][0] = a * inv_len;
            planes[pi][1] = b * inv_len;
            planes[pi][2] = c * inv_len;
            planes[pi][3] = planes[pi][3] * inv_len;
        }
        return planes;
    }

    fn aabbInFrustum(planes: [6][4]f32, min: [3]f32, max: [3]f32, margin: f32) bool {
        var p: [3]f32 = undefined;
        var i: usize = 0;
        while (i < 6) : (i += 1) {
            const a = planes[i][0];
            const b = planes[i][1];
            const c = planes[i][2];
            const d = planes[i][3];
            p[0] = if (a >= 0) max[0] else min[0];
            p[1] = if (b >= 0) max[1] else min[1];
            p[2] = if (c >= 0) max[2] else min[2];
            const dist = a * p[0] + b * p[1] + c * p[2] + d;
            if (dist < -margin) return false;
        }
        return true;
    }

    fn updateFrustum(self: *Client, mvp: [16]f32) void {
        if (self.frustum_valid) {
            self.prev_frustum_planes = self.frustum_planes;
            self.prev_frustum_valid = true;
        }
        self.frustum_planes = extractFrustumPlanes(mvp);
        self.frustum_valid = true;
    }

    fn aabbContainsPoint(min: [3]f32, max: [3]f32, p: [3]f32) bool {
        return p[0] >= min[0] and p[0] <= max[0] and p[1] >= min[1] and p[1] <= max[1] and p[2] >= min[2] and p[2] <= max[2];
    }

    // Local physics step using shared routine
    fn localPhysicsStep(self: *Client, dt: f32) void {
        const cfg: physics.PhysicsConfig = .{ .gravity = 9.80665 };
        // If the player's current chunk isn't loaded yet, pause client-side physics to avoid visual falling
        const chunkLoadedAt = struct {
            fn call(cli: *Client, x: f32, z: f32) bool {
                const xi: i32 = @intFromFloat(@floor(x));
                const zi: i32 = @intFromFloat(@floor(z));
                const cx: i32 = @divFloor(xi, @as(i32, @intCast(constants.chunk_size_x)));
                const cz: i32 = @divFloor(zi, @as(i32, @intCast(constants.chunk_size_z)));
                const wk = "minecraft:overworld";
                cli.sim.worlds_mutex.lock();
                defer cli.sim.worlds_mutex.unlock();
                const ws = cli.sim.worlds_state.getPtr(wk) orelse return false;
                const rp = simulation.RegionPos{ .x = @divFloor(cx, 32), .z = @divFloor(cz, 32) };
                const rs = ws.regions.getPtr(rp) orelse return false;
                const cidx = rs.chunk_index.get(.{ .x = cx, .z = cz }) orelse return false;
                const ch = rs.chunks.items[cidx];
                return ch.status == .full;
            }
        }.call;
        if (!chunkLoadedAt(self, self.local_pos[0], self.local_pos[2])) return;
        var kin: physics.EntityKinematics = .{
            .pos = self.local_pos,
            .vel = self.local_vel,
            .half_extents = self.local_aabb_half_extents,
            .on_ground = &self.local_on_ground,
        };
        const solidSampler = struct {
            fn call(ctx: *anyopaque, ix: i32, iy: i32, iz: i32) bool {
                const cli: *Client = @ptrCast(@alignCast(ctx));
                const wk = "minecraft:overworld";
                cli.sim.worlds_mutex.lock();
                defer cli.sim.worlds_mutex.unlock();
                const ws = cli.sim.worlds_state.getPtr(wk) orelse return false;
                const cx: i32 = @divFloor(ix, @as(i32, @intCast(constants.chunk_size_x)));
                const cz: i32 = @divFloor(iz, @as(i32, @intCast(constants.chunk_size_z)));
                const lx_i: i32 = @mod(ix, @as(i32, @intCast(constants.chunk_size_x)));
                const lz_i: i32 = @mod(iz, @as(i32, @intCast(constants.chunk_size_z)));
                const rp = simulation.RegionPos{ .x = @divFloor(cx, 32), .z = @divFloor(cz, 32) };
                const rs = ws.regions.getPtr(rp) orelse return false;
                const cidx = rs.chunk_index.get(.{ .x = cx, .z = cz }) orelse return false;
                const ch = rs.chunks.items[cidx];
                const total_y: i32 = @intCast(ch.sections.len * constants.section_height);
                const ny0: i32 = iy + @as(i32, @intCast(ws.sections_below)) * @as(i32, @intCast(constants.section_height));
                if (ny0 < 0 or ny0 >= total_y) return false;
                const sy: usize = @intCast(@divTrunc(ny0, @as(i32, @intCast(constants.section_height))));
                const ly: usize = @intCast(@mod(ny0, @as(i32, @intCast(constants.section_height))));
                if (sy >= ch.sections.len) return false;
                const s = ch.sections[sy];
                const bpi: u6 = bitsFor(s.palette.len);
                const lx: usize = @intCast(lx_i);
                const lz: usize = @intCast(lz_i);
                if (lx >= constants.chunk_size_x or lz >= constants.chunk_size_z) return false;
                const idx3d: usize = lz * (constants.chunk_size_x * constants.section_height) + lx * constants.section_height + ly;
                const pidx: usize = @intCast(unpackBitsGet(s.blocks_indices_bits, idx3d, bpi));
                if (pidx >= s.palette.len) return false;
                const bid: u32 = s.palette[pidx];
                if (bid == 0) return false;
                if (bid < cli.sim.reg.blocks.items.len) {
                    return cli.sim.reg.blocks.items[@intCast(bid)].full_block_collision;
                }
                return true;
            }
        }.call;
        physics.integrateStep(cfg, &kin, dt, solidSampler, self);
        self.local_pos = kin.pos;
        self.local_vel = kin.vel;
    }

    // Send movement state to SIM at most once per authoritative tick
    fn maybeSendMoveToSim(self: *Client) void {
        const snap = self.pollSnapshot().*;
        if (snap.tick == self.last_move_tick_sent) return;
        self.last_move_tick_sent = snap.tick;
        self.sim.connections_mutex.lock();
        const idx_opt = self.sim.entity_by_player.get(self.player_id);
        if (idx_opt) |idx| {
            if (idx < self.sim.dynamic_entities.items.len) {
                var e = &self.sim.dynamic_entities.items[idx];
                // Horizontal velocity from local prediction
                e.vel[0] = self.local_vel[0];
                e.vel[2] = self.local_vel[2];
                // Apply jump impulse if pending (server will clamp by on_ground)
                if (self.jump_pending) {
                    const g2: f32 = 9.80665;
                    const target_h: f32 = 1.1;
                    const v0: f32 = @sqrt(2.0 * g2 * target_h);
                    e.vel[1] = v0;
                    e.flags.on_ground = false;
                }
                // Keep facing aligned to camera forward projected onto XZ when moving
                if (self.move_forward or self.move_back or self.move_left or self.move_right) {
                    const lx = self.local_dir[0];
                    const lz = self.local_dir[2];
                    const llen: f32 = @sqrt(lx * lx + lz * lz);
                    if (llen > 0.0001) {
                        e.facing_dir_xz = .{ lx / llen, lz / llen };
                        const facing_yaw: f32 = std.math.atan2(e.facing_dir_xz[1], e.facing_dir_xz[0]);
                        e.yaw_pitch_roll = .{ facing_yaw, 0, 0 };
                    }
                }
            }
        }
        self.sim.connections_mutex.unlock();
        // clear one-shot jump after sending this tick
        self.jump_pending = false;
    }

    fn makePerspective(fovy_radians: f32, aspect: f32, znear: f32, zfar: f32) [16]f32 {
        const f: f32 = 1.0 / @tan(fovy_radians / 2.0);
        var m: [16]f32 = [_]f32{0} ** 16;
        m[0] = f / aspect;
        m[5] = f;
        m[10] = (zfar + znear) / (znear - zfar);
        m[11] = -1.0;
        m[14] = (2.0 * zfar * znear) / (znear - zfar);
        return m;
    }

    fn drawPlayerAabb(self: *Client, vs_in: *const shd_mod.VsParams) void {
        // Construct a solid box from the client's local predicted AABB
        const hx = self.local_aabb_half_extents[0];
        const hy = self.local_aabb_half_extents[1];
        const hz = self.local_aabb_half_extents[2];
        const x0: f32 = self.local_pos[0] - hx;
        const x1: f32 = self.local_pos[0] + hx;
        // Physics pos[1] is center Y; debug box should span [center - hy, center + hy]
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
        const off = [2]f32{ 0, 0 };
        const sc = [2]f32{ 1, 1 };
        const addTri = struct {
            fn call(list: *[36]Vertex, idx: *usize, p0: [3]f32, p1: [3]f32, p2: [3]f32, t0: [2]f32, t1: [2]f32, t2: [2]f32) void {
                list.*[idx.*] = .{ .pos = p0, .uv = t0, .uv_off = off, .uv_scale = sc };
                idx.* += 1;
                list.*[idx.*] = .{ .pos = p1, .uv = t1, .uv_off = off, .uv_scale = sc };
                idx.* += 1;
                list.*[idx.*] = .{ .pos = p2, .uv = t2, .uv_off = off, .uv_scale = sc };
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
        var vs_params: shd_mod.VsParams = .{ .mvp = vs_in.mvp, .atlas_pad = .{ 0, 0 } };
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

    fn makeViewConv(eye: [3]f32, forward: [3]f32, up_in: [3]f32, conventional: bool) [16]f32 {
        // normalize forward
        const f0 = forward[0];
        const f1 = forward[1];
        const f2 = forward[2];
        const flen: f32 = @sqrt(f0 * f0 + f1 * f1 + f2 * f2);
        const fx = if (flen > 0.000001) (f0 / flen) else 1.0;
        const fy = if (flen > 0.000001) (f1 / flen) else 0.0;
        const fz = if (flen > 0.000001) (f2 / flen) else 0.0;
        // s = normalize(cross(f, up)) (conventional) or cross(up, f)
        var sx0: f32 = undefined;
        var sy0: f32 = undefined;
        var sz0: f32 = undefined;
        if (conventional) {
            // cross(f, up)
            sx0 = fy * up_in[2] - fz * up_in[1];
            sy0 = fz * up_in[0] - fx * up_in[2];
            sz0 = fx * up_in[1] - fy * up_in[0];
        } else {
            // cross(up, f)
            sx0 = up_in[1] * fz - up_in[2] * fy;
            sy0 = up_in[2] * fx - up_in[0] * fz;
            sz0 = up_in[0] * fy - up_in[1] * fx;
        }
        const slen: f32 = @sqrt(sx0 * sx0 + sy0 * sy0 + sz0 * sz0);
        const sx = sx0 / slen;
        const sy = sy0 / slen;
        const sz = sz0 / slen;
        // u = cross(s, f)
        const ux = sy * fz - sz * fy;
        const uy = sz * fx - sx * fz;
        const uz = sx * fy - sy * fx;
        return makeViewFromBasis(eye, .{ sx, sy, sz }, .{ ux, uy, uz }, .{ fx, fy, fz });
    }

    fn makeViewFromBasis(eye: [3]f32, s: [3]f32, u: [3]f32, f: [3]f32) [16]f32 {
        var m: [16]f32 = undefined;
        m[0] = s[0];
        m[4] = u[0];
        // Use OpenGL-style view with -f in the third column (camera looks down -Z in view space)
        m[8] = -f[0];
        m[12] = 0;
        m[1] = s[1];
        m[5] = u[1];
        m[9] = -f[1];
        m[13] = 0;
        m[2] = s[2];
        m[6] = u[2];
        m[10] = -f[2];
        m[14] = 0;
        m[3] = 0;
        m[7] = 0;
        m[11] = 0;
        m[15] = 1;
        var t: [16]f32 = [_]f32{0} ** 16;
        t[0] = 1;
        t[5] = 1;
        t[10] = 1;
        t[15] = 1;
        t[12] = -eye[0];
        t[13] = -eye[1];
        t[14] = -eye[2];
        return matMul(m, t);
    }

    fn transposeView3x3WithEye(self: *Client, v: [16]f32) [16]f32 {
        var m = v;
        // transpose rotation 3x3 in upper-left
        const m00 = m[0];
        const m01 = m[4];
        const m02 = m[8];
        const m10 = m[1];
        const m11 = m[5];
        const m12 = m[9];
        const m20 = m[2];
        const m21 = m[6];
        const m22 = m[10];
        m[0] = m00;
        m[4] = m10;
        m[8] = m20;
        m[1] = m01;
        m[5] = m11;
        m[9] = m21;
        m[2] = m02;
        m[6] = m12;
        m[10] = m22;
        // recompute translation column using new rotation and current camera eye
        const eye = self.camera.pos;
        const neg_eye0 = -eye[0];
        const neg_eye1 = -eye[1];
        const neg_eye2 = -eye[2];
        m[12] = m[0] * neg_eye0 + m[4] * neg_eye1 + m[8] * neg_eye2;
        m[13] = m[1] * neg_eye0 + m[5] * neg_eye1 + m[9] * neg_eye2;
        m[14] = m[2] * neg_eye0 + m[6] * neg_eye1 + m[10] * neg_eye2;
        return m;
    }

    fn makeViewNoRoll(self: *Client, eye: [3]f32, yaw: f32, pitch: f32) [16]f32 {
        const cy = @cos(yaw);
        const sy = @sin(yaw);
        const cp = @cos(pitch);
        const sp = @sin(pitch);
        // forward from yaw/pitch
        const f: [3]f32 = .{ cp * cy, sp, cp * sy };
        // choose a stable world-up; if nearly parallel to forward, pick Z-up fallback
        var up: [3]f32 = .{ 0, 1, 0 };
        const dot_fu: f32 = f[0] * up[0] + f[1] * up[1] + f[2] * up[2];
        if (@abs(dot_fu) > 0.999) {
            up = .{ 0, 0, 1 };
        }
        // s = normalize(cross(f, up))
        const sx0: f32 = f[1] * up[2] - f[2] * up[1];
        const sy0: f32 = f[2] * up[0] - f[0] * up[2];
        const sz0: f32 = f[0] * up[1] - f[1] * up[0];
        const sl: f32 = @max(0.000001, @sqrt(sx0 * sx0 + sy0 * sy0 + sz0 * sz0));
        const s: [3]f32 = .{ sx0 / sl, sy0 / sl, sz0 / sl };
        // u = cross(s, f)
        const ux: f32 = s[1] * f[2] - s[2] * f[1];
        const uy: f32 = s[2] * f[0] - s[0] * f[2];
        const uz: f32 = s[0] * f[1] - s[1] * f[0];
        const u: [3]f32 = .{ ux, uy, uz };
        // capture for debug overlay
        self.dbg_f = f;
        self.dbg_s = s;
        self.dbg_u = u;
        return makeViewFromBasis(eye, s, u, f);
    }

    fn regionMeshCacheCount(self: *Client) usize {
        var cnt: usize = 0;
        var it = self.region_mesh_cache.keyIterator();
        while (it.next()) |_| cnt += 1;
        return cnt;
    }

    fn evictRegionMeshes(self: *Client, max_count: usize, ttl_frames: u32) void {
        // Collect stale regions (older than ttl_frames)
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
        // Remove stale
        var i: usize = 0;
        while (i < to_remove.items.len) : (i += 1) {
            const pos = to_remove.items[i];
            if (self.region_mesh_cache.get(pos)) |rm| {
                if (rm.vbuf.id != 0) sg.destroyBuffer(rm.vbuf);
                self.allocator.free(rm.draws);
            }
            _ = self.region_mesh_cache.remove(pos);
        }

        // If still over budget, remove oldest until under limit
        var count = self.regionMeshCacheCount();
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
};
