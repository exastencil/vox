const std = @import("std");

pub const PhysicsConfig = struct {
    gravity: f32 = 9.80665,
};

pub const EntityKinematics = struct {
    pos: [3]f32,
    vel: [3]f32,
    half_extents_y: f32,
    // Pointer to a boolean flag stored by the caller; updated by integrateStep
    on_ground: *bool,
};

// Callback signature for sampling the top solid face Y at a world (x, z) column.
// Return null if the column is not available/loaded.
pub const HeightSampler = fn (ctx: *anyopaque, x: f32, z: f32) ?f32;

// Integrate one physics step: apply gravity, integrate motion, resolve vertical collision
// against the top surface sampled via the provided sampler. This routine is deterministic
// for a given dt, inputs, and sampler results.
pub fn integrateStep(cfg: PhysicsConfig, kin: *EntityKinematics, dt: f32, sampler: HeightSampler, sampler_ctx: *anyopaque) void {
    if (dt <= 0.0) return;

    // Gravity
    kin.vel[1] -= cfg.gravity * dt;

    // Integrate
    var next_pos = kin.pos;
    next_pos[0] += kin.vel[0] * dt;
    next_pos[1] += kin.vel[1] * dt;
    next_pos[2] += kin.vel[2] * dt;

    // Vertical collision against ground top from heightmap
    if (sampler(sampler_ctx, next_pos[0], next_pos[2])) |top_y| {
        const bottom_next = next_pos[1] - kin.half_extents_y;
        if (bottom_next < top_y) {
            next_pos[1] = top_y + kin.half_extents_y;
            kin.vel[1] = 0;
            kin.on_ground.* = true;
        } else {
            kin.on_ground.* = false;
        }
    }

    kin.pos = next_pos;
}
