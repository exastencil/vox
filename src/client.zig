const std = @import("std");
const simulation = @import("simulation.zig");

pub const Client = struct {
    sim: *simulation.Simulation,

    pub fn init(sim: *simulation.Simulation) Client {
        return .{ .sim = sim };
    }

    pub fn update(self: *Client) void {
        // delegate fixed-timestep update to the simulation
        self.sim.update();
    }
};
