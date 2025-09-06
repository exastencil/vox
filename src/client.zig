const std = @import("std");
const simulation = @import("simulation.zig");
const player = @import("player.zig");

pub const Client = struct {
    allocator: std.mem.Allocator,
    sim: *simulation.Simulation,
    player_id: player.PlayerId,
    account_name: []const u8,
    connected: bool = false,

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

    pub fn init(allocator: std.mem.Allocator, sim: *simulation.Simulation, player_id: player.PlayerId, account_name: []const u8) !Client {
        const acc = try allocator.dupe(u8, account_name);
        return .{ .allocator = allocator, .sim = sim, .player_id = player_id, .account_name = acc };
    }

    pub fn deinit(self: *Client) void {
        self.allocator.free(self.account_name);
    }

    pub fn connect(self: *Client) !void {
        if (self.connected) return;
        try self.sim.connectPlayer(self.player_id, self.account_name);
        self.connected = true;
        // Ensure server creates/associates an entity for this player
        _ = self.sim.ensurePlayerEntity(self.player_id) catch {};
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
};
