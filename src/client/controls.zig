// Client control schemes and input handling (generic over caller type)
const std = @import("std");
const sokol = @import("sokol");
const sapp = sokol.app;

pub const ViewProj = struct { view: [16]f32, proj: [16]f32 };

pub const ControlScheme = struct {
    name: []const u8,
    onMouseMove: *const fn (*anyopaque, sapp.Event) void,
    onScroll: *const fn (*anyopaque, sapp.Event) void,
    applyMovement: *const fn (*anyopaque) void,
    updateCamera: *const fn (*anyopaque) void,
    makeViewProj: *const fn (*anyopaque, f32, f32) ViewProj,
};

inline fn perspective(fovy_radians: f32, aspect: f32, znear: f32, zfar: f32) [16]f32 {
    const f = 1.0 / @tan(fovy_radians / 2.0);
    var m: [16]f32 = [_]f32{0} ** 16;
    m[0] = f / (if (aspect != 0) aspect else 1.0);
    m[5] = f;
    m[10] = (zfar + znear) / (znear - zfar);
    m[11] = -1.0;
    m[14] = (2.0 * zfar * znear) / (znear - zfar);
    return m;
}

// First-person horizontal controls (generic over T that provides expected fields/methods)
pub fn fph_onMouseMove(comptime T: type, self: *T, ev: sapp.Event) void {
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

pub fn fph_onScroll(comptime T: type, self: *T, ev: sapp.Event) void {
    _ = self;
    _ = ev; // no-op for first person horizontal
}

pub fn fph_applyMovement(comptime T: type, self: *T) void {
    self.applyMovementInputsLocal();
}

pub fn fph_updateCamera(comptime T: type, self: *T) void {
    self.updateCameraFromLocal();
}

pub fn makeViewProj_fph(comptime T: type, self: *T, w_px: f32, h_px: f32) ViewProj {
    const aspect_wh: f32 = if (h_px > 0) (w_px / h_px) else 1.0;
    const proj = perspective(60.0 * std.math.pi / 180.0, aspect_wh, 0.1, 1000.0);
    const view = self.makeViewNoRoll(self.camera.pos, self.camera.yaw, self.camera.pitch);
    return .{ .view = view, .proj = proj };
}

// Third-person isometric controls
pub fn iso_onMouseMove(comptime T: type, self: *T, ev: sapp.Event) void {
    const sens: f32 = 0.002; // radians per pixel
    const dx = ev.mouse_dx * sens;
    // Horizontal orbit around the player (mouse right => turn right)
    self.iso_yaw += dx;
    // Normalize yaw to [-pi, pi]
    if (self.iso_yaw > std.math.pi) self.iso_yaw -= 2.0 * std.math.pi;
    if (self.iso_yaw < -std.math.pi) self.iso_yaw += 2.0 * std.math.pi;
}

pub fn iso_onScroll(comptime T: type, self: *T, ev: sapp.Event) void {
    const step: f32 = 1.0;
    // Positive scroll_y typically means scroll up; bring camera closer
    self.iso_dist = std.math.clamp(self.iso_dist - (ev.scroll_y * step), self.iso_min_dist, self.iso_max_dist);
}

pub fn iso_applyMovement(comptime T: type, self: *T) void {
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

pub fn iso_updateCamera(comptime T: type, self: *T) void {
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

pub fn makeViewProj_iso(comptime T: type, self: *T, w_px: f32, h_px: f32) ViewProj {
    const aspect_wh: f32 = if (h_px > 0) (w_px / h_px) else 1.0;
    const proj = perspective(60.0 * std.math.pi / 180.0, aspect_wh, 0.1, 1000.0);
    const view = self.makeViewNoRoll(self.camera.pos, self.iso_yaw, self.iso_pitch);
    return .{ .view = view, .proj = proj };
}
