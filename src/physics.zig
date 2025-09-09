const std = @import("std");

pub const PhysicsConfig = struct {
    gravity: f32 = 9.80665,
    // Small epsilon to avoid floating-point oscillations on contact
    contact_epsilon: f32 = 0.001,
};

pub const EntityKinematics = struct {
    // Position is the entity's CENTER in X/Z and CENTER in Y.
    // Note: Some rendering code may currently assume pos[1] is bottom; physics uses center consistently.
    pos: [3]f32,
    vel: [3]f32,
    half_extents: [3]f32, // {hx, hy, hz}
    // Pointer to a boolean flag stored by the caller; updated by integrateStep
    on_ground: *bool,
};

// Callback signature returning whether the integer voxel at (ix,iy,iz) is solid for collision.
// Implementations should treat out-of-bounds/unloaded as non-solid.
pub const SolidSampler = fn (ctx: *anyopaque, ix: i32, iy: i32, iz: i32) bool;

fn floor_i(v: f32) i32 {
    return @intFromFloat(@floor(v));
}

fn range_int(min_v: f32, max_v: f32, out_min: *i32, out_max: *i32) void {
    // Inclusive integer range overlapped by [min_v, max_v)
    const mn = if (min_v < max_v) min_v else max_v;
    const mx = if (min_v < max_v) max_v else min_v;
    const start = floor_i(mn);
    // subtract tiny epsilon so aabb max at integer boundary doesn't include the next block
    const end = floor_i(mx - 0.000001);
    out_min.* = start;
    out_max.* = end;
}

// Integrate one physics step with axis-swept AABB resolution against a voxel solid sampler.
// Deterministic for a given dt, inputs, and sampler results.
pub fn integrateStep(cfg: PhysicsConfig, kin: *EntityKinematics, dt: f32, isSolid: SolidSampler, ctx: *anyopaque) void {
    if (dt <= 0.0) return;

    // Apply gravity
    kin.vel[1] -= cfg.gravity * dt;

    // Proposed movement
    var next = kin.pos;
    const dx = kin.vel[0] * dt;
    const dy = kin.vel[1] * dt;
    const dz = kin.vel[2] * dt;

    var on_ground_local = false;

    const hx = kin.half_extents[0];
    const hy = kin.half_extents[1];
    const hz = kin.half_extents[2];

    // Resolve X axis
    if (dx != 0) {
        next[0] += dx;
        var y_min_i: i32 = 0;
        var y_max_i: i32 = 0;
        var z_min_i: i32 = 0;
        var z_max_i: i32 = 0;
        range_int(next[1] - hy, next[1] + hy, &y_min_i, &y_max_i);
        range_int(next[2] - hz, next[2] + hz, &z_min_i, &z_max_i);
        if (dx > 0) {
            // Leading face at maxX
            const bx = floor_i(next[0] + hx);
            // Scan the leading layer only
            var yi = y_min_i;
            while (yi <= y_max_i) : (yi += 1) {
                var zi = z_min_i;
                while (zi <= z_max_i) : (zi += 1) {
                    if (isSolid(ctx, bx, yi, zi)) {
                        const allowed_max_x: f32 = @as(f32, @floatFromInt(bx)) - cfg.contact_epsilon;
                        const new_center_x = allowed_max_x - hx;
                        if (next[0] > new_center_x) next[0] = new_center_x;
                        kin.vel[0] = 0;
                        // no need to check other cells once clamped; still continue to ensure all constraints
                    }
                }
            }
        } else {
            // dx < 0, leading face at minX
            const bx = floor_i(next[0] - hx);
            var yi = y_min_i;
            while (yi <= y_max_i) : (yi += 1) {
                var zi = z_min_i;
                while (zi <= z_max_i) : (zi += 1) {
                    if (isSolid(ctx, bx, yi, zi)) {
                        const allowed_min_x: f32 = @as(f32, @floatFromInt(bx + 1)) + cfg.contact_epsilon;
                        const new_center_x = allowed_min_x + hx;
                        if (next[0] < new_center_x) next[0] = new_center_x;
                        kin.vel[0] = 0;
                    }
                }
            }
        }
    }

    // Resolve Z axis
    if (dz != 0) {
        next[2] += dz;
        var y_min_i: i32 = 0;
        var y_max_i: i32 = 0;
        var x_min_i: i32 = 0;
        var x_max_i: i32 = 0;
        range_int(next[1] - hy, next[1] + hy, &y_min_i, &y_max_i);
        range_int(next[0] - hx, next[0] + hx, &x_min_i, &x_max_i);
        if (dz > 0) {
            const bz = floor_i(next[2] + hz);
            var yi = y_min_i;
            while (yi <= y_max_i) : (yi += 1) {
                var xi = x_min_i;
                while (xi <= x_max_i) : (xi += 1) {
                    if (isSolid(ctx, xi, yi, bz)) {
                        const allowed_max_z: f32 = @as(f32, @floatFromInt(bz)) - cfg.contact_epsilon;
                        const new_center_z = allowed_max_z - hz;
                        if (next[2] > new_center_z) next[2] = new_center_z;
                        kin.vel[2] = 0;
                    }
                }
            }
        } else {
            const bz = floor_i(next[2] - hz);
            var yi = y_min_i;
            while (yi <= y_max_i) : (yi += 1) {
                var xi = x_min_i;
                while (xi <= x_max_i) : (xi += 1) {
                    if (isSolid(ctx, xi, yi, bz)) {
                        const allowed_min_z: f32 = @as(f32, @floatFromInt(bz + 1)) + cfg.contact_epsilon;
                        const new_center_z = allowed_min_z + hz;
                        if (next[2] < new_center_z) next[2] = new_center_z;
                        kin.vel[2] = 0;
                    }
                }
            }
        }
    }

    // Resolve Y axis (sets on_ground when clamping on downward motion)
    if (dy != 0) {
        next[1] += dy;
        var x_min_i: i32 = 0;
        var x_max_i: i32 = 0;
        var z_min_i: i32 = 0;
        var z_max_i: i32 = 0;
        range_int(next[0] - hx, next[0] + hx, &x_min_i, &x_max_i);
        range_int(next[2] - hz, next[2] + hz, &z_min_i, &z_max_i);
        if (dy > 0) {
            const by = floor_i(next[1] + hy);
            var xi = x_min_i;
            while (xi <= x_max_i) : (xi += 1) {
                var zi = z_min_i;
                while (zi <= z_max_i) : (zi += 1) {
                    if (isSolid(ctx, xi, by, zi)) {
                        const allowed_max_y: f32 = @as(f32, @floatFromInt(by)) - cfg.contact_epsilon;
                        const new_center_y = allowed_max_y - hy;
                        if (next[1] > new_center_y) next[1] = new_center_y;
                        kin.vel[1] = 0;
                        // moving up into a ceiling
                    }
                }
            }
        } else {
            const by = floor_i(next[1] - hy);
            var xi = x_min_i;
            while (xi <= x_max_i) : (xi += 1) {
                var zi = z_min_i;
                while (zi <= z_max_i) : (zi += 1) {
                    if (isSolid(ctx, xi, by, zi)) {
                        const allowed_min_y: f32 = @as(f32, @floatFromInt(by + 1)) + cfg.contact_epsilon;
                        const new_center_y = allowed_min_y + hy;
                        if (next[1] < new_center_y) next[1] = new_center_y;
                        kin.vel[1] = 0;
                        on_ground_local = true;
                    }
                }
            }
        }
    }

    kin.pos = next;
    kin.on_ground.* = on_ground_local;
}
