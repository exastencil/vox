const std = @import("std");

pub const PlayerId = [16]u8;

pub fn genUuidV4() PlayerId {
    var id: PlayerId = undefined;
    std.crypto.random.bytes(&id);
    // UUIDv4: set version = 4 (high nibble of byte 6), and variant = 10xx (high 2 bits of byte 8)
    id[6] = (id[6] & 0x0F) | 0x40;
    id[8] = (id[8] & 0x3F) | 0x80;
    return id;
}

pub const PlayerData = struct {
    id: PlayerId,
    account_name: []const u8, // global account name (owned)
    display_name: []const u8, // per-save display name (owned)
    // TODO: inventory, stats, etc.
};

pub const PlayerRegistry = struct {
    allocator: std.mem.Allocator,
    by_id: std.AutoHashMap(PlayerId, PlayerData),
    by_account: std.StringHashMap(PlayerId),

    pub fn init(allocator: std.mem.Allocator) PlayerRegistry {
        return .{
            .allocator = allocator,
            .by_id = std.AutoHashMap(PlayerId, PlayerData).init(allocator),
            .by_account = std.StringHashMap(PlayerId).init(allocator),
        };
    }

    pub fn deinit(self: *PlayerRegistry) void {
        var it = self.by_id.valueIterator();
        while (it.next()) |pd| {
            self.allocator.free(pd.account_name);
            self.allocator.free(pd.display_name);
        }
        self.by_id.deinit();
        // free owned keys in by_account
        var ait = self.by_account.iterator();
        while (ait.next()) |entry| {
            self.allocator.free(entry.key_ptr.*);
        }
        self.by_account.deinit();
    }

    pub fn getOrCreate(self: *PlayerRegistry, account_name: []const u8, display_name: []const u8) !PlayerId {
        if (self.by_account.get(account_name)) |existing| return existing;
        // create new player
        const id = genUuidV4();
        const acc = try self.allocator.dupe(u8, account_name);
        const disp = try self.allocator.dupe(u8, display_name);
        try self.by_id.put(id, .{ .id = id, .account_name = acc, .display_name = disp });
        const acc_key = try self.allocator.dupe(u8, account_name);
        try self.by_account.put(acc_key, id);
        return id;
    }

    pub fn get(self: *const PlayerRegistry, id: PlayerId) ?PlayerData {
        if (self.by_id.get(id)) |pd| return pd;
        return null;
    }
};
