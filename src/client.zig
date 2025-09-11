const std = @import("std");
const simulation = @import("simulation.zig");
const player = @import("player.zig");
const constants = @import("constants");
const gs = @import("gs");
const ids = @import("ids");
const block = @import("registry/block.zig");
const sokol = @import("sokol");
const physics = @import("physics.zig");
const sg = sokol.gfx;
const sapp = sokol.app;
const sglue = sokol.glue;
const sdtx = sokol.debugtext;
const shd_mod = @import("shaders/chunk_shd.zig");
const uvmap = @import("uvmap.zig");
const builtin = @import("builtin");

// Client-only modules (facade-only imports for now)
const client_platform_mod = @import("client/platform.zig");
const client_controls_mod = @import("client/controls.zig");
const client_camera_mod = @import("client/camera.zig");
const client_interaction_mod = @import("client/interaction.zig");
const client_prediction_mod = @import("client/prediction.zig");
const client_textures_mod = @import("client/textures.zig");
const client_atlas_builder_mod = @import("client/atlas_builder.zig");
const client_renderer_mod = @import("client/renderer.zig");
const client_mesher_mod = @import("client/mesher.zig");
const client_mesher_threads_mod = @import("client/mesher_threads.zig");
const client_mesher_scheduler_mod = @import("client/mesher_scheduler.zig");
// macOS-only: warp the mouse cursor to a point that is guaranteed to be inside our app window.
// For now, we use the center of the main display, which is typically within the window
// (our window defaults to 1920x1080 and is centered on launch). This avoids the first click
// going to another app if the cursor started outside our window.

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
    const proj = client_camera_mod.makePerspective(60.0 * std.math.pi / 180.0, aspect_wh, 0.1, 1000.0);
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

const Vertex = struct { pos: [3]f32, uv: [2]f32, rect_min: [2]f32, rect_size: [2]f32, layer: f32, apply_tint: f32 };

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

fn cs_iso_makeViewProj(ctx: *anyopaque, w_px: f32, h_px: f32) ViewProj {
    const self: *Client = @ptrCast(@alignCast(ctx));
    const aspect_wh: f32 = if (h_px > 0) (w_px / h_px) else 1.0;
    const proj = client_camera_mod.makePerspective(60.0 * std.math.pi / 180.0, aspect_wh, 0.1, 1000.0);
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

inline fn emitQuad(allocator: std.mem.Allocator, list: *std.ArrayList(Vertex), v0: [3]f32, v1: [3]f32, v2: [3]f32, v3: [3]f32, uv0: [2]f32, uv1: [2]f32, uv2: [2]f32, uv3: [2]f32, rect_min: [2]f32, rect_size: [2]f32, layer: f32, apply_tint: f32) void {
    // Store tile-space UVs and atlas rect; shader applies fract and scales into atlas space
    list.append(allocator, .{ .pos = v0, .uv = uv0, .rect_min = rect_min, .rect_size = rect_size, .layer = layer, .apply_tint = apply_tint }) catch return;
    list.append(allocator, .{ .pos = v1, .uv = uv1, .rect_min = rect_min, .rect_size = rect_size, .layer = layer, .apply_tint = apply_tint }) catch return;
    list.append(allocator, .{ .pos = v2, .uv = uv2, .rect_min = rect_min, .rect_size = rect_size, .layer = layer, .apply_tint = apply_tint }) catch return;
    list.append(allocator, .{ .pos = v0, .uv = uv0, .rect_min = rect_min, .rect_size = rect_size, .layer = layer, .apply_tint = apply_tint }) catch return;
    list.append(allocator, .{ .pos = v2, .uv = uv2, .rect_min = rect_min, .rect_size = rect_size, .layer = layer, .apply_tint = apply_tint }) catch return;
    list.append(allocator, .{ .pos = v3, .uv = uv3, .rect_min = rect_min, .rect_size = rect_size, .layer = layer, .apply_tint = apply_tint }) catch return;
}

// Cached mesh types (region-level aggregation)
const SectionMesh = struct { first: u32 = 0, count: u32 = 0 };
const SectionDraw = client_mesher_mod.SectionDraw;
const cache_mod = @import("client/mesher_cache.zig");
const RegionMesh = cache_mod.RegionMesh;

const MeshingJob = client_mesher_mod.MeshingJob;
const MeshingResult = client_mesher_mod.MeshingResult;

// Snapshot types for background meshing
const SectionSnapshot = client_mesher_mod.SectionSnapshot;
const ChunkSnapshot = client_mesher_mod.ChunkSnapshot;
const RegionSnapshot = client_mesher_mod.RegionSnapshot;

// Temporary slice-ref structs for capturing under lock, then copying outside the lock
const SectionSliceRefs = client_mesher_mod.SectionSliceRefs;
const ChunkSliceRefs = client_mesher_mod.ChunkSliceRefs;
const RegionSliceRefs = client_mesher_mod.RegionSliceRefs;

const REGION_MESH_BUDGET: usize = @import("client/mesher_cache.zig").REGION_MESH_BUDGET; // generous, regions are fewer than chunks

// Texture-array layer identifier
const LayerId = u16;

// Per-region tint cache entry (512x512 covering a 32x32 chunk area)
const RegionTintEntry = struct { img: sg.Image, view: sg.View, last_used: u32, dirty: bool };

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

// Map a world-facing face index to a block-relative Face, given the block's "up" direction.
// World face order: 0:-X, 1:+X, 2:-Y, 3:+Y, 4:-Z, 5:+Z
inline fn worldToRelativeFace(dir: ids.Direction, world_face: u3) block.Face {
    const BF = block.Face;
    // For each orientation, define which world face each block face points to.
    // This arbitrarily fixes a yaw for non-up orientations; current content does not
    // distinguish front/back vs left/right in visuals, so this is sufficient.
    const bf_to_wf: [6]u3 = switch (dir) {
        .up => .{
            5, // front -> +Z
            4, // back  -> -Z
            0, // left  -> -X
            1, // right -> +X
            3, // up    -> +Y
            2, // down  -> -Y
        },
        .down => .{
            5, // front -> +Z
            4, // back  -> -Z
            1, // left  -> +X (flip)
            0, // right -> -X (flip)
            2, // up    -> -Y
            3, // down  -> +Y
        },
        .front => .{
            3, // front -> +Y
            2, // back  -> -Y
            0, // left  -> -X
            1, // right -> +X
            5, // up    -> +Z
            4, // down  -> -Z
        },
        .back => .{
            3, // front -> +Y
            2, // back  -> -Y
            1, // left  -> +X
            0, // right -> -X
            4, // up    -> -Z
            5, // down  -> +Z
        },
        .right => .{
            5, // front -> +Z
            4, // back  -> -Z
            3, // left  -> +Y
            2, // right -> -Y
            1, // up    -> +X
            0, // down  -> -X
        },
        .left => .{
            5, // front -> +Z
            4, // back  -> -Z
            2, // left  -> -Y
            3, // right -> +Y
            0, // up    -> -X
            1, // down  -> +X
        },
    };
    // Invert mapping: find which block face has the given world face index
    var i: u3 = 0;
    while (i < 6) : (i += 1) {
        if (bf_to_wf[i] == world_face) return @enumFromInt(i);
    }
    // Fallback
    return BF.up;
}

fn buildQuadVerts(out: []Vertex, x0: f32, y0: f32, size: f32) usize {
    if (out.len < 6) return 0;
    const x1: f32 = x0 + size;
    const y1: f32 = y0 + size;
    out[0] = .{ .pos = .{ x0, y0 }, .uv = .{ 0, 0 }, .rect_min = .{ 0, 0 }, .rect_size = .{ 1, 1 }, .layer = 0, .apply_tint = 0 };
    out[1] = .{ .pos = .{ x1, y0 }, .uv = .{ 1, 0 }, .rect_min = .{ 0, 0 }, .rect_size = .{ 1, 1 }, .layer = 0, .apply_tint = 0 };
    out[2] = .{ .pos = .{ x1, y1 }, .uv = .{ 1, 1 }, .rect_min = .{ 0, 0 }, .rect_size = .{ 1, 1 }, .layer = 0, .apply_tint = 0 };
    out[3] = .{ .pos = .{ x0, y0 }, .uv = .{ 0, 0 }, .rect_min = .{ 0, 0 }, .rect_size = .{ 1, 1 }, .layer = 0, .apply_tint = 0 };
    out[4] = .{ .pos = .{ x1, y1 }, .uv = .{ 1, 1 }, .rect_min = .{ 0, 0 }, .rect_size = .{ 1, 1 }, .layer = 0, .apply_tint = 0 };
    out[5] = .{ .pos = .{ x0, y1 }, .uv = .{ 0, 1 }, .rect_min = .{ 0, 0 }, .rect_size = .{ 1, 1 }, .layer = 0, .apply_tint = 0 };
    return 6;
}

const UvRect = struct { u0: f32, v0: f32, u1: f32, v1: f32, w: u32, h: u32 };

pub const Client = struct {
    allocator: std.mem.Allocator,
    sim: *simulation.Simulation,
    player_id: player.PlayerId,
    account_name: []const u8,
    connected: bool = false,
    // Connection/ready state
    ready: bool = false,

    // Texture/array preloading state
    textures_ready: bool = false,
    // Default layer choices for current demo material
    default_layer: LayerId = 0,
    top_layer: LayerId = 0,
    side_layer: LayerId = 0,

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
    // Atlases and bindings
    atlas_imgs: [4]sg.Image = .{ .{}, .{}, .{}, .{} },
    atlas_views: [4]sg.View = .{ .{}, .{}, .{}, .{} },
    atlas_count: u32 = 0,
    sampler: sg.Sampler = .{},
    // Debug solid-color textures (2D)
    green_img: sg.Image = .{},
    green_view: sg.View = .{},
    black_img: sg.Image = .{},
    black_view: sg.View = .{},
    white_img: sg.Image = .{},
    white_view: sg.View = .{},

    // Per-region tint cache
    region_tint_cache: std.AutoHashMap(simulation.RegionPos, RegionTintEntry) = undefined,
    // Atlas UV map per texture identifier
    atlas_uvs_by_id: std.StringHashMap(client_atlas_builder_mod.AtlasUv) = undefined,
    // Storage for owned atlas id strings kept alive for the map keys
    atlas_id_storage: std.ArrayList([]const u8) = undefined,

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
    camera: client_camera_mod.Camera = .{},

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
            .atlas_uvs_by_id = std.StringHashMap(client_atlas_builder_mod.AtlasUv).init(allocator),
            .atlas_id_storage = std.ArrayList([]const u8).initCapacity(allocator, 0) catch unreachable,
            .region_mesh_cache = std.AutoHashMap(simulation.RegionPos, RegionMesh).init(allocator),
            .last_visible_frame_chunk = std.AutoHashMap(gs.ChunkPos, u32).init(allocator),
            .last_visible_frame_region = std.AutoHashMap(simulation.RegionPos, u32).init(allocator),
            .mesher_jobs = std.ArrayList(MeshingJob).initCapacity(allocator, 0) catch unreachable,
            .mesher_results = std.ArrayList(MeshingResult).initCapacity(allocator, 0) catch unreachable,
            .mesher_threads = std.ArrayList(std.Thread).initCapacity(allocator, 0) catch unreachable,
        };
        // Initialize additional maps
        cli.region_tint_cache = std.AutoHashMap(simulation.RegionPos, RegionTintEntry).init(allocator);
        // Initialize control schemes (default to FirstPersonHorizontal)
        cli.control_schemes = &[_]ControlScheme{ firstPersonHorizontalScheme, thirdPersonIsometricScheme };
        cli.current_scheme = 0; // first person by default
        return cli;
    }

    fn mesherThreadMain(self: *Client) void {
        client_mesher_threads_mod.mesherThreadMain(self);
    }

    pub fn deinit(self: *Client) void {
        // Release GPU resources if created
        if (self.vbuf.id != 0) sg.destroyBuffer(self.vbuf);
        if (self.aabb_vbuf.id != 0) sg.destroyBuffer(self.aabb_vbuf);
        if (self.outline_vbuf.id != 0) sg.destroyBuffer(self.outline_vbuf);
        // atlas views
        var ai: usize = 0;
        while (ai < self.atlas_views.len) : (ai += 1) if (self.atlas_views[ai].id != 0) sg.destroyView(self.atlas_views[ai]);
        if (self.green_view.id != 0) sg.destroyView(self.green_view);
        if (self.black_view.id != 0) sg.destroyView(self.black_view);
        if (self.white_view.id != 0) sg.destroyView(self.white_view);
        if (self.sampler.id != 0) sg.destroySampler(self.sampler);
        ai = 0;
        while (ai < self.atlas_imgs.len) : (ai += 1) if (self.atlas_imgs[ai].id != 0) sg.destroyImage(self.atlas_imgs[ai]);
        if (self.green_img.id != 0) sg.destroyImage(self.green_img);
        if (self.black_img.id != 0) sg.destroyImage(self.black_img);
        if (self.white_img.id != 0) sg.destroyImage(self.white_img);
        if (self.pip_lines.id != 0) sg.destroyPipeline(self.pip_lines);
        if (self.pip_outline.id != 0) sg.destroyPipeline(self.pip_outline);
        if (self.pip.id != 0) sg.destroyPipeline(self.pip);
        if (self.shd.id != 0) sg.destroyShader(self.shd);

        // Stop mesher threads
        client_mesher_threads_mod.stopMesherThreads(self);

        // Deinitialize atlas map (values only; keys are borrowed from registry)
        // Destroy cached region meshes
        var it = self.region_mesh_cache.valueIterator();
        while (it.next()) |rm| {
            if (rm.vbuf.id != 0) sg.destroyBuffer(rm.vbuf);
            self.allocator.free(rm.draws);
        }
        self.region_mesh_cache.deinit();

        // Destroy per-region tint cache
        var itc = self.region_tint_cache.valueIterator();
        while (itc.next()) |entry| {
            if (entry.img.id != 0) sg.destroyImage(entry.img);
            if (entry.view.id != 0) sg.destroyView(entry.view);
        }
        self.region_tint_cache.deinit();

        // Free atlas id storage (keys)
        for (self.atlas_id_storage.items) |s| self.allocator.free(s);
        self.atlas_id_storage.deinit(self.allocator);
        self.atlas_uvs_by_id.deinit();
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
        pdesc.layout.attrs[0].format = .FLOAT3; // pos
        pdesc.layout.attrs[1].format = .FLOAT2; // uv (tile-space)
        pdesc.layout.attrs[2].format = .FLOAT2; // rect_min
        pdesc.layout.attrs[3].format = .FLOAT2; // rect_size
        pdesc.layout.attrs[4].format = .FLOAT; // layer
        pdesc.layout.attrs[5].format = .FLOAT; // apply_tint (0 or 1)
        pdesc.primitive_type = .TRIANGLES;
        pdesc.cull_mode = .BACK;
        // Front-face winding should match our emitted CCW triangles
        pdesc.face_winding = .CCW;
        pdesc.depth = .{ .compare = .LESS_EQUAL, .write_enabled = true };
        // ensure pipeline color/depth target matches swapchain format
        const desc = sg.queryDesc();
        pdesc.color_count = 1;
        pdesc.colors[0].pixel_format = desc.environment.defaults.color_format;
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
        // keep outlines render-through
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

        // Build block atlases and register atlas UVs
        client_atlas_builder_mod.buildBlockAtlases(self);
        // sampler used for all texture sampling
        self.sampler = sg.makeSampler(.{ .min_filter = .NEAREST, .mag_filter = .NEAREST, .mipmap_filter = .NEAREST, .wrap_u = .CLAMP_TO_EDGE, .wrap_v = .CLAMP_TO_EDGE });
        // shader expects atlas0 at slot 1, and atlas1..3 at 5..7; sampler at slot 2
        if (self.atlas_count > 0) self.bind.views[1] = self.atlas_views[0];
        if (self.atlas_count > 1) self.bind.views[5] = self.atlas_views[1];
        if (self.atlas_count > 2) self.bind.views[6] = self.atlas_views[2];
        if (self.atlas_count > 3) self.bind.views[7] = self.atlas_views[3];
        // Fill any missing atlas view slots with a valid fallback
        const fallback_view = if (self.atlas_count > 0) self.atlas_views[0] else self.white_view;
        if (self.bind.views[1].id == 0) self.bind.views[1] = fallback_view;
        if (self.bind.views[5].id == 0) self.bind.views[5] = fallback_view;
        if (self.bind.views[6].id == 0) self.bind.views[6] = fallback_view;
        if (self.bind.views[7].id == 0) self.bind.views[7] = fallback_view;
        self.bind.samplers[2] = self.sampler;
        // chunk tint will use view slot 3 and sampler slot 4 (reuse same sampler)
        self.bind.samplers[4] = self.sampler;

        // Create a 1x1 green texture/view for debug solid color draws (array type to match VIEW_tex_array usage)
        {
            var green = [_]u8{ 0, 255, 0, 255 };
            self.green_img = sg.makeImage(.{
                .type = ._2D,
                .width = 1,
                .height = 1,
                .num_mipmaps = 1,
                .pixel_format = .RGBA8,
                .data = .{ .subimage = .{ .{ sg.asRange(green[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} } } },
            });
            self.green_view = sg.makeView(.{ .texture = .{ .image = self.green_img } });
        }

        // Initialize local state and camera from player entity if available
        self.initLocalFromSim();
        self.control_schemes[self.current_scheme].updateCamera(self);
        // Defer mouse capture until world is ready (see checkReady)

        // Create a 1x1 white texture/view for default tint (2D type to match VIEW_chunk_tint_tex)
        {
            var white = [_]u8{ 255, 255, 255, 255 };
            self.white_img = sg.makeImage(.{
                .type = ._2D,
                .width = 1,
                .height = 1,
                .num_mipmaps = 1,
                .pixel_format = .RGBA8,
                .data = .{ .subimage = .{ .{ sg.asRange(white[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} } } },
            });
            self.white_view = sg.makeView(.{ .texture = .{ .image = self.white_img } });
            // Bind default tint view at slot 3 so pipelines expecting VIEW_chunk_tint_tex always have a valid view
            self.bind.views[3] = self.white_view;
        }

        // Create a 1x1 black texture/view for debug outlines (array type)
        {
            var black = [_]u8{ 0, 0, 0, 255 };
            self.black_img = sg.makeImage(.{
                .type = ._2D,
                .width = 1,
                .height = 1,
                .num_mipmaps = 1,
                .pixel_format = .RGBA8,
                .data = .{ .subimage = .{ .{ sg.asRange(black[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} } } },
            });
            self.black_view = sg.makeView(.{ .texture = .{ .image = self.black_img } });
        }

        // end initGfx
    }

    fn checkReady(self: *Client) void {
        if (self.ready) {
            return;
        }
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
            client_platform_mod.macosWarpCursorIntoAppWindow();
            sapp.lockMouse(true);
            client_platform_mod.setRawMouse(true);
        }
    }

    fn startMesherIfNeeded(self: *Client) void {
        client_mesher_threads_mod.startMesherIfNeeded(self);
    }

    fn pumpMesherScheduleAndAdopt(self: *Client) void {
        client_mesher_scheduler_mod.pumpMesherScheduleAndAdopt(self);
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
        client_interaction_mod.updateHighlightTarget(self);
        // Notify simulation no more than once per tick
        self.maybeSendLookToSim();
        self.maybeSendMoveToSim();

        // Prepare MVP and frustum before building the mesh (used for culling)
        const w_px: f32 = @floatFromInt(sapp.width());
        const h_px: f32 = @floatFromInt(sapp.height());
        const vp = self.control_schemes[self.current_scheme].makeViewProj(self, w_px, h_px);
        var view_adj = vp.view;
        if (self.metal_transpose_view3x3) {
            view_adj = client_camera_mod.transposeView3x3WithEye(view_adj, self.camera.pos);
        }
        // standard MVP (proj*view)
        const mvp: [16]f32 = client_camera_mod.matMul(vp.proj, view_adj);
        self.updateFrustum(mvp);
        var vs_params: shd_mod.VsParams = .{ .mvp = mvp, .region_info = .{ 0, 0, 1.0 / 512.0, 0 } };

        // Render cached meshes (build on demand)
        if (self.atlas_count > 0) {
            sg.applyPipeline(self.pip);
            sg.applyUniforms(0, sg.asRange(&vs_params));
            client_renderer_mod.renderWorldCached(self, &vs_params);
        }

        // Optional continuous probe logging

        // Cached rendering handled above

        // Draw targeted block outline (if any)
        client_interaction_mod.drawTargetOutline(self, &vs_params);

        // Draw player bounding box in solid green for debugging camera orbit (disabled in first-person)
        if (!std.mem.eql(u8, self.control_schemes[self.current_scheme].name, "FirstPersonHorizontal")) {
            client_renderer_mod.drawPlayerAabb(self, &vs_params);
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

            // Left-side debug: show client-local predicted position and current look dir
            // This reflects the camera you control this frame.
            const pos: [3]f32 = self.camera.pos;
            const look: [3]f32 = self.local_dir;

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
                    client_platform_mod.macosWarpCursorIntoAppWindow();
                    // Lock the mouse immediately when the window gains focus to avoid stray first clicks
                    sapp.lockMouse(true);
                    client_platform_mod.setRawMouse(true);
                },
                .UNFOCUSED => {
                    // Always release on focus loss
                    sapp.lockMouse(false);
                    client_platform_mod.setRawMouse(false);
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
                    client_platform_mod.setRawMouse(false);
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
                client_platform_mod.macosWarpCursorIntoAppWindow();
                // Lock the mouse when the window gains focus
                sapp.lockMouse(true);
                // Enable raw mouse input if supported
                client_platform_mod.setRawMouse(true);
            },
            .UNFOCUSED => {
                // Release the mouse when the window loses focus
                sapp.lockMouse(false);
                client_platform_mod.setRawMouse(false);
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
                    client_platform_mod.macosWarpCursorIntoAppWindow();
                    sapp.lockMouse(true);
                    client_platform_mod.setRawMouse(true);
                } else {
                    // When in first-person scheme, left-click breaks the targeted block within reach
                    if (std.mem.eql(u8, self.control_schemes[self.current_scheme].name, "FirstPersonHorizontal")) {
                        if (ev.mouse_button == .LEFT) {
                            client_interaction_mod.tryBreakBlockUnderCrosshair(self);
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
        const bid: u32 = s.palette[pidx].block_id;
        if (bid == 0) return false;
        if (bid < self.sim.reg.blocks.items.len) {
            return self.sim.reg.blocks.items[@intCast(bid)].collision;
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
            // Per-world-face materials
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
        var pal_info = std.ArrayList(PalInfo).initCapacity(self.allocator, pal_len) catch {
            return;
        };
        defer pal_info.deinit(self.allocator);
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
            if (bid < self.sim.reg.blocks.items.len) {
                pinfo.solid = self.sim.reg.blocks.items[@intCast(bid)].collision;
            }
            if (bid == 0) {
                pinfo.draw = false;
                pinfo.solid = false;
            } else {
                // Resolve per-world-face materials via Block.Def FaceConfigs and BlockState.direction
                const def = self.sim.reg.blocks.items[@intCast(bid)];
                const state = s.palette[pi];
                inline for (.{ 0, 1, 2, 3, 4, 5 }) |wf| {
                    const rel: block.Face = worldToRelativeFace(state.direction, @intCast(wf));
                    const fidx: usize = @intCast(def.face_config_index[@intFromEnum(rel)]);
                    if (fidx < def.face_configs.len) {
                        const fc = def.face_configs[fidx];
                        if (fc.layers.len > 0) {
                            switch (fc.layers[0]) {
                                .Texture => |t| {
                                    const uv_opt = self.atlas_uvs_by_id.get(t.tex);
                                    if (uv_opt) |uv| {
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
                            }
                        }
                    }
                }
            }
            pal_info.appendAssumeCapacity(pinfo);
        }

        // Greedy meshing for +Y (top) faces per Y-slice to drastically reduce vertex count
        const FaceMat = struct { uv: client_atlas_builder_mod.AtlasUv, apply_tint: bool };
        const Mat = struct {
            fn eq(a: FaceMat, b: FaceMat) bool {
                return a.apply_tint == b.apply_tint and a.uv.aidx == b.uv.aidx and a.uv.u0 == b.uv.u0 and a.uv.v0 == b.uv.v0 and a.uv.u1 == b.uv.u1 and a.uv.v1 == b.uv.v1;
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
                        mask[idx] = .{ .present = true, .m = .{ .uv = info.top_uv, .apply_tint = info.top_apply_tint } };
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
                    // Greedy top face: tile-space UVs span the rectangle size (w x h)
                    const du: f32 = mkey.uv.u1 - mkey.uv.u0;
                    const dv: f32 = mkey.uv.v1 - mkey.uv.v0;
                    const rmin = [2]f32{ mkey.uv.u0, mkey.uv.v0 };
                    const rsize = [2]f32{ du, dv };
                    const uv00: [2]f32 = .{ 0, 0 };
                    const uvh0: [2]f32 = .{ @floatFromInt(h), 0 };
                    const uvhw: [2]f32 = .{ @floatFromInt(h), @floatFromInt(w) };
                    const uv0w: [2]f32 = .{ 0, @floatFromInt(w) };
                    emitQuad(self.allocator, out, .{ wx0, wy1, wz0 }, .{ wx0, wy1, wz1 }, .{ wx1, wy1, wz1 }, .{ wx1, wy1, wz0 }, uv00, uvh0, uvhw, uv0w, rmin, rsize, @floatFromInt(mkey.uv.aidx), if (mkey.apply_tint) 1.0 else 0.0);
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

        // Greedy bottom faces per Y-slice (-Y)
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
                    if (!self.isSolidAt(ws, ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(mx)), abs_y - 1, @as(i32, @intCast(mz)))) {
                        mask[idxm] = .{ .present = true, .m = .{ .uv = info.bot_uv, .apply_tint = info.bot_apply_tint } };
                    }
                }
            }
            // rectangles over mask (x along width, z along height)
            mz = 0;
            while (mz < constants.chunk_size_z) : (mz += 1) {
                var mx: usize = 0;
                while (mx < constants.chunk_size_x) : (mx += 1) {
                    const base_idx: usize = mz * constants.chunk_size_x + mx;
                    if (!mask[base_idx].present) continue;
                    const mkey: FaceMat = mask[base_idx].m;
                    var w: usize = 1;
                    while ((mx + w) < constants.chunk_size_x) : (w += 1) {
                        const idx2 = mz * constants.chunk_size_x + (mx + w);
                        if (!(mask[idx2].present and Mat.eq(mask[idx2].m, mkey))) break;
                    }
                    var h: usize = 1;
                    outer_b: while ((mz + h) < constants.chunk_size_z) : (h += 1) {
                        var kx: usize = 0;
                        while (kx < w) : (kx += 1) {
                            const idx3 = (mz + h) * constants.chunk_size_x + (mx + kx);
                            if (!(mask[idx3].present and Mat.eq(mask[idx3].m, mkey))) break :outer_b;
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
                    emitQuad(self.allocator, out, .{ wx0, wy0, wz0 }, .{ wx1, wy0, wz0 }, .{ wx1, wy0, wz1 }, .{ wx0, wy0, wz1 }, uv00_b, uvw0, uvwh, uv0h, rmin, rsize, @floatFromInt(mkey.uv.aidx), if (mkey.apply_tint) 1.0 else 0.0);
                    var zz: usize = 0;
                    while (zz < h) : (zz += 1) {
                        var xx: usize = 0;
                        while (xx < w) : (xx += 1) mask[(mz + zz) * constants.chunk_size_x + (mx + xx)].present = false;
                    }
                }
            }
        }

        // Greedy -X and +X faces per X-slice
        var lx_face: usize = 0;
        while (lx_face < constants.chunk_size_x) : (lx_face += 1) {
            // -X
            {
                var mask_x: [constants.section_height * constants.chunk_size_z]MaskCell = undefined;
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
                        if (!self.isSolidAt(ws, ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx_face)) - 1, abs_y, @as(i32, @intCast(mz)))) {
                            mask_x[idxm] = .{ .present = true, .m = .{ .uv = info.nx_uv, .apply_tint = info.nx_apply_tint } };
                        }
                    }
                }
                // rectangles over (ly as width, mz as height)
                mz = 0;
                while (mz < constants.chunk_size_z) : (mz += 1) {
                    var ly0: usize = 0;
                    while (ly0 < constants.section_height) : (ly0 += 1) {
                        const base_idx: usize = mz * constants.section_height + ly0;
                        if (!mask_x[base_idx].present) continue;
                        const mkey: FaceMat = mask_x[base_idx].m;
                        var w: usize = 1;
                        while ((ly0 + w) < constants.section_height) : (w += 1) {
                            const idx2 = mz * constants.section_height + (ly0 + w);
                            if (!(mask_x[idx2].present and Mat.eq(mask_x[idx2].m, mkey))) break;
                        }
                        var h: usize = 1;
                        outer_xn: while ((mz + h) < constants.chunk_size_z) : (h += 1) {
                            var ky: usize = 0;
                            while (ky < w) : (ky += 1) {
                                const idx3 = (mz + h) * constants.section_height + (ly0 + ky);
                                if (!(mask_x[idx3].present and Mat.eq(mask_x[idx3].m, mkey))) break :outer_xn;
                            }
                        }
                        const x0: f32 = base_x + @as(f32, @floatFromInt(lx_face));
                        const y0: f32 = @as(f32, @floatFromInt(section_base_y)) + @as(f32, @floatFromInt(ly0));
                        const y1: f32 = y0 + @as(f32, @floatFromInt(w));
                        const z0: f32 = base_z + @as(f32, @floatFromInt(mz));
                        const z1: f32 = z0 + @as(f32, @floatFromInt(h));
                        const du: f32 = mkey.uv.u1 - mkey.uv.u0;
                        const dv: f32 = mkey.uv.v1 - mkey.uv.v0;
                        const rmin = [2]f32{ mkey.uv.u0, mkey.uv.v0 };
                        const rsize = [2]f32{ du, dv };
                        const uvs_nx = uvmap.sideTileUVs(0, w, h);
                        emitQuad(self.allocator, out, .{ x0, y0, z0 }, .{ x0, y0, z1 }, .{ x0, y1, z1 }, .{ x0, y1, z0 }, uvs_nx[0], uvs_nx[1], uvs_nx[2], uvs_nx[3], rmin, rsize, @floatFromInt(mkey.uv.aidx), if (mkey.apply_tint) 1.0 else 0.0);
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
                var mask_x: [constants.section_height * constants.chunk_size_z]MaskCell = undefined;
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
                        if (!self.isSolidAt(ws, ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx_face)) + 1, abs_y, @as(i32, @intCast(mz)))) {
                            mask_x[idxm] = .{ .present = true, .m = .{ .uv = info.px_uv, .apply_tint = info.px_apply_tint } };
                        }
                    }
                }
                mz = 0;
                while (mz < constants.chunk_size_z) : (mz += 1) {
                    var ly0: usize = 0;
                    while (ly0 < constants.section_height) : (ly0 += 1) {
                        const base_idx: usize = mz * constants.section_height + ly0;
                        if (!mask_x[base_idx].present) continue;
                        const mkey: FaceMat = mask_x[base_idx].m;
                        var w: usize = 1;
                        while ((ly0 + w) < constants.section_height) : (w += 1) {
                            const idx2 = mz * constants.section_height + (ly0 + w);
                            if (!(mask_x[idx2].present and Mat.eq(mask_x[idx2].m, mkey))) break;
                        }
                        var h: usize = 1;
                        outer_xp: while ((mz + h) < constants.chunk_size_z) : (h += 1) {
                            var ky: usize = 0;
                            while (ky < w) : (ky += 1) {
                                const idx3 = (mz + h) * constants.section_height + (ly0 + ky);
                                if (!(mask_x[idx3].present and Mat.eq(mask_x[idx3].m, mkey))) break :outer_xp;
                            }
                        }
                        const x1: f32 = base_x + @as(f32, @floatFromInt(lx_face + 1));
                        const y0: f32 = @as(f32, @floatFromInt(section_base_y)) + @as(f32, @floatFromInt(ly0));
                        const y1: f32 = y0 + @as(f32, @floatFromInt(w));
                        const z0: f32 = base_z + @as(f32, @floatFromInt(mz));
                        const z1: f32 = z0 + @as(f32, @floatFromInt(h));
                        const du: f32 = mkey.uv.u1 - mkey.uv.u0;
                        const dv: f32 = mkey.uv.v1 - mkey.uv.v0;
                        const rmin = [2]f32{ mkey.uv.u0, mkey.uv.v0 };
                        const rsize = [2]f32{ du, dv };
                        const uvs_px = uvmap.sideTileUVs(1, w, h);
                        emitQuad(self.allocator, out, .{ x1, y0, z0 }, .{ x1, y1, z0 }, .{ x1, y1, z1 }, .{ x1, y0, z1 }, uvs_px[0], uvs_px[1], uvs_px[2], uvs_px[3], rmin, rsize, @floatFromInt(mkey.uv.aidx), if (mkey.apply_tint) 1.0 else 0.0);
                        var hz: usize = 0;
                        while (hz < h) : (hz += 1) {
                            var wy: usize = 0;
                            while (wy < w) : (wy += 1) mask_x[(mz + hz) * constants.section_height + (ly0 + wy)].present = false;
                        }
                    }
                }
            }
        }

        // Greedy -Z and +Z faces per Z-slice
        var lz_face: usize = 0;
        while (lz_face < constants.chunk_size_z) : (lz_face += 1) {
            // -Z
            {
                var mask_z: [constants.section_height * constants.chunk_size_x]MaskCell = undefined;
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
                        if (!self.isSolidAt(ws, ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y, @as(i32, @intCast(lz_face)) - 1)) {
                            mask_z[idxm] = .{ .present = true, .m = .{ .uv = info.nz_uv, .apply_tint = info.nz_apply_tint } };
                        }
                    }
                }
                lx = 0;
                while (lx < constants.chunk_size_x) : (lx += 1) {
                    var ly0: usize = 0;
                    while (ly0 < constants.section_height) : (ly0 += 1) {
                        const base_idx: usize = lx * constants.section_height + ly0;
                        if (!mask_z[base_idx].present) continue;
                        const mkey: FaceMat = mask_z[base_idx].m;
                        var w: usize = 1;
                        while ((lx + w) < constants.chunk_size_x) : (w += 1) {
                            const idx2 = (lx + w) * constants.section_height + ly0;
                            if (!(mask_z[idx2].present and Mat.eq(mask_z[idx2].m, mkey))) break;
                        }
                        var h: usize = 1;
                        outer_zn: while ((ly0 + h) < constants.section_height) : (h += 1) {
                            var kx: usize = 0;
                            while (kx < w) : (kx += 1) {
                                const idx3 = (lx + kx) * constants.section_height + (ly0 + h);
                                if (!(mask_z[idx3].present and Mat.eq(mask_z[idx3].m, mkey))) break :outer_zn;
                            }
                        }
                        const z0: f32 = base_z + @as(f32, @floatFromInt(lz_face));
                        const x0: f32 = base_x + @as(f32, @floatFromInt(lx));
                        const x1: f32 = x0 + @as(f32, @floatFromInt(w));
                        const y0: f32 = @as(f32, @floatFromInt(section_base_y)) + @as(f32, @floatFromInt(ly0));
                        const y1: f32 = y0 + @as(f32, @floatFromInt(h));
                        const du: f32 = mkey.uv.u1 - mkey.uv.u0;
                        const dv: f32 = mkey.uv.v1 - mkey.uv.v0;
                        const rmin = [2]f32{ mkey.uv.u0, mkey.uv.v0 };
                        const rsize = [2]f32{ du, dv };
                        const uvs_nz = uvmap.sideTileUVs(4, w, h);
                        emitQuad(self.allocator, out, .{ x0, y0, z0 }, .{ x0, y1, z0 }, .{ x1, y1, z0 }, .{ x1, y0, z0 }, uvs_nz[0], uvs_nz[1], uvs_nz[2], uvs_nz[3], rmin, rsize, @floatFromInt(mkey.uv.aidx), if (mkey.apply_tint) 1.0 else 0.0);
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
                var mask_z: [constants.section_height * constants.chunk_size_x]MaskCell = undefined;
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
                        if (!self.isSolidAt(ws, ch, ch.pos.x, ch.pos.z, @as(i32, @intCast(lx)), abs_y, @as(i32, @intCast(lz_face)) + 1)) {
                            mask_z[idxm] = .{ .present = true, .m = .{ .uv = info.pz_uv, .apply_tint = info.pz_apply_tint } };
                        }
                    }
                }
                lx = 0;
                while (lx < constants.chunk_size_x) : (lx += 1) {
                    var ly0: usize = 0;
                    while (ly0 < constants.section_height) : (ly0 += 1) {
                        const base_idx: usize = lx * constants.section_height + ly0;
                        if (!mask_z[base_idx].present) continue;
                        const mkey: FaceMat = mask_z[base_idx].m;
                        var w: usize = 1;
                        while ((lx + w) < constants.chunk_size_x) : (w += 1) {
                            const idx2 = (lx + w) * constants.section_height + ly0;
                            if (!(mask_z[idx2].present and Mat.eq(mask_z[idx2].m, mkey))) break;
                        }
                        var h: usize = 1;
                        outer_zp: while ((ly0 + h) < constants.section_height) : (h += 1) {
                            var kx: usize = 0;
                            while (kx < w) : (kx += 1) {
                                const idx3 = (lx + kx) * constants.section_height + (ly0 + h);
                                if (!(mask_z[idx3].present and Mat.eq(mask_z[idx3].m, mkey))) break :outer_zp;
                            }
                        }
                        const z1: f32 = base_z + @as(f32, @floatFromInt(lz_face + 1));
                        const x0: f32 = base_x + @as(f32, @floatFromInt(lx));
                        const x1: f32 = x0 + @as(f32, @floatFromInt(w));
                        const y0: f32 = @as(f32, @floatFromInt(section_base_y)) + @as(f32, @floatFromInt(ly0));
                        const y1: f32 = y0 + @as(f32, @floatFromInt(h));
                        const du: f32 = mkey.uv.u1 - mkey.uv.u0;
                        const dv: f32 = mkey.uv.v1 - mkey.uv.v0;
                        const rmin = [2]f32{ mkey.uv.u0, mkey.uv.v0 };
                        const rsize = [2]f32{ du, dv };
                        const uvs_pz = uvmap.sideTileUVs(5, w, h);
                        // Use (v0,v1,v2,v3) for +Z to ensure CCW with cull BACK
                        emitQuad(self.allocator, out, .{ x0, y0, z1 }, .{ x1, y0, z1 }, .{ x1, y1, z1 }, .{ x0, y1, z1 }, uvs_pz[0], uvs_pz[3], uvs_pz[2], uvs_pz[1], rmin, rsize, @floatFromInt(mkey.uv.aidx), if (mkey.apply_tint) 1.0 else 0.0);
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
        return client_mesher_mod.buildRegionRefsUnderLock(self, alloc, ws, rs, rpos);
    }

    fn freeRegionRefs(self: *Client, alloc: std.mem.Allocator, refs: *RegionSliceRefs) void {
        client_mesher_mod.freeRegionRefs(self, alloc, refs);
    }

    // Outside world lock: copy refs into an owned RegionSnapshot
    fn copySnapshotFromRefs(self: *Client, alloc: std.mem.Allocator, refs: *const RegionSliceRefs) RegionSnapshot {
        return client_mesher_mod.copySnapshotFromRefs(self, alloc, refs);
    }

    fn freeRegionSnapshot(self: *Client, alloc: std.mem.Allocator, snap: *RegionSnapshot) void {
        client_mesher_mod.freeRegionSnapshot(self, alloc, snap);
    }

    fn snapGetChunkIndex(snap: *const RegionSnapshot, cx: i32, cz: i32) ?usize {
        return client_mesher_mod.snapGetChunkIndex(snap, cx, cz);
    }

    fn isSolidAtSnapshot(self: *Client, snap: *const RegionSnapshot, start_cx: i32, start_cz: i32, lx_in: i32, abs_y_in: i32, lz_in: i32) bool {
        return client_mesher_mod.isSolidAtSnapshot(self, snap, start_cx, start_cz, lx_in, abs_y_in, lz_in);
    }

    fn buildSectionVerticesFromSnapshot(self: *Client, snap: *const RegionSnapshot, ch: *const ChunkSnapshot, sy: usize, base_x: f32, base_z: f32, out: *std.ArrayList(Vertex), alloc: std.mem.Allocator) void {
        client_mesher_mod.buildSectionVerticesFromSnapshot(self, snap, ch, sy, base_x, base_z, out, alloc);
    }

    fn meshRegionFromSnapshot(self: *Client, alloc: std.mem.Allocator, snap: *const RegionSnapshot) MeshingResult {
        return client_mesher_mod.meshRegionFromSnapshot(self, alloc, snap);
    }

    fn adoptMesherResults(self: *Client) void {
        client_mesher_scheduler_mod.adoptMesherResults(self);
    }

    pub fn ensureRegionMesh(self: *Client, rpos: simulation.RegionPos, expected_chunk_count: usize) void {
        client_mesher_scheduler_mod.ensureRegionMesh(self, rpos, expected_chunk_count);
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

    fn updateFrustum(self: *Client, mvp: [16]f32) void {
        if (self.frustum_valid) {
            self.prev_frustum_planes = self.frustum_planes;
            self.prev_frustum_valid = true;
        }
        self.frustum_planes = client_camera_mod.extractFrustumPlanes(mvp);
        self.frustum_valid = true;
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
                const bid: u32 = s.palette[pidx].block_id;
                if (bid == 0) return false;
                if (bid < cli.sim.reg.blocks.items.len) {
                    return cli.sim.reg.blocks.items[@intCast(bid)].collision;
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

    fn makeViewNoRoll(self: *Client, eye: [3]f32, yaw: f32, pitch: f32) [16]f32 {
        // This function remains here to capture debug basis into Client fields.
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
        return client_camera_mod.makeViewFromBasis(eye, s, u, f);
    }

    pub fn regionMeshCacheCount(self: *Client) usize {
        var cnt: usize = 0;
        var it = self.region_mesh_cache.keyIterator();
        while (it.next()) |_| cnt += 1;
        return cnt;
    }
};
