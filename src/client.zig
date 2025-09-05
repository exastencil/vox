const std = @import("std");
const simulation = @import("simulation.zig");

pub const Client = struct {
    sim: *simulation.Simulation,
    target_tps: f64 = 20.0,
    accumulator: f64 = 0.0,
    last_time_ns: i128 = 0,
    max_accum: f64 = 0.25, // clamp to avoid spiral-of-death

    pub fn init(sim: *simulation.Simulation, target_tps: f64) Client {
        return .{
            .sim = sim,
            .target_tps = target_tps,
            .accumulator = 0.0,
            .last_time_ns = std.time.nanoTimestamp(),
            .max_accum = 0.25,
        };
    }

    pub fn update(self: *Client) void {
        const now_ns: i128 = std.time.nanoTimestamp();
        var dt_ns: i128 = now_ns - self.last_time_ns;
        if (dt_ns < 0) dt_ns = 0;
        const dt_sec: f64 = @as(f64, @floatFromInt(dt_ns)) / 1_000_000_000.0;
        self.last_time_ns = now_ns;

        // clamp frame delta to avoid huge spikes after pauses
        const clamped_dt = if (dt_sec > self.max_accum) self.max_accum else dt_sec;
        self.accumulator += clamped_dt;

        const step: f64 = 1.0 / self.target_tps;
        while (self.accumulator >= step) {
            self.sim.tick();
            self.accumulator -= step;
        }
    }
};
