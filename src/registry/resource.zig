const std = @import("std");

// A very small resource registry focused on block rendering resources.
// Blocks can be assigned one of three resource styles:
// - Void:     nothing rendered (e.g. core:air)
// - Uniform:  same texture on all faces
// - Facing:   one specified face uses a unique texture, all others share one texture
//
// Texture references are stored by string keys for now (e.g. "core:stone").
// This registry is independent of block definitions, allowing resource packs to
// override visuals without redefining blocks.

pub const Face = enum {
    up,
    down,
    north,
    south,
    west,
    east,
};

pub const BlockRes = union(enum) {
    Void: void,
    Uniform: struct {
        all: []const u8, // texture key for all faces
    },
    Facing: struct {
        face: Face,
        face_tex: []const u8,
        other_tex: []const u8,
    },
};

pub const Registry = struct {
    allocator: std.mem.Allocator,
    // Map block key (e.g. "core:stone") -> BlockRes
    by_block: std.StringHashMap(BlockRes),

    pub fn init(allocator: std.mem.Allocator) Registry {
        return .{ .allocator = allocator, .by_block = std.StringHashMap(BlockRes).init(allocator) };
    }

    pub fn deinit(self: *Registry) void {
        // Free any duplicated strings inside BlockRes values and keys
        var it = self.by_block.iterator();
        while (it.next()) |entry| {
            const key_owned = entry.key_ptr.*; // owned key
            self.allocator.free(key_owned);
            const res = entry.value_ptr.*;
            switch (res) {
                .Void => {},
                .Uniform => |u| self.allocator.free(u.all),
                .Facing => |f| {
                    self.allocator.free(f.face_tex);
                    self.allocator.free(f.other_tex);
                },
            }
        }
        self.by_block.deinit();
    }

    // Upsert behavior: later registrations override earlier ones.
    // This supports resource packs overriding visuals.
    pub fn setVoid(self: *Registry, block_key: []const u8) !void {
        try self.replace(block_key, .{ .Void = {} });
    }

    pub fn setUniform(self: *Registry, block_key: []const u8, all_tex: []const u8) !void {
        const all_owned = try self.allocator.dupe(u8, all_tex);
        const res = BlockRes{ .Uniform = .{ .all = all_owned } };
        try self.replace(block_key, res);
    }

    pub fn setFacing(self: *Registry, block_key: []const u8, face: Face, face_tex: []const u8, other_tex: []const u8) !void {
        const f_owned = try self.allocator.dupe(u8, face_tex);
        const o_owned = try self.allocator.dupe(u8, other_tex);
        const res = BlockRes{ .Facing = .{ .face = face, .face_tex = f_owned, .other_tex = o_owned } };
        try self.replace(block_key, res);
    }

    pub fn get(self: *const Registry, block_key: []const u8) ?BlockRes {
        if (self.by_block.get(block_key)) |r| return r;
        return null;
    }

    fn replace(self: *Registry, block_key: []const u8, res: BlockRes) !void {
        // If entry exists, replace value in-place and free old inner strings
        if (self.by_block.getPtr(block_key)) |p| {
            switch (p.*) {
                .Void => {},
                .Uniform => |u| self.allocator.free(u.all),
                .Facing => |f| {
                    self.allocator.free(f.face_tex);
                    self.allocator.free(f.other_tex);
                },
            }
            p.* = res;
            return;
        }
        // Otherwise, insert with an owned key
        const k_owned = try self.allocator.dupe(u8, block_key);
        try self.by_block.put(k_owned, res);
    }
};

const testing = std.testing;

// Basic tests for resource registry behavior

test "resource: register void, uniform, facing and retrieve" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer {
        const ok = gpa.deinit();
        testing.expect(ok == .ok) catch {};
    }
    var reg = Registry.init(gpa.allocator());
    defer reg.deinit();

    try reg.setVoid("core:air");
    try reg.setUniform("core:stone", "tex:stone");
    try reg.setFacing("core:grass", .up, "tex:grass_top", "tex:grass_side");

    const air = reg.get("core:air") orelse return error.UnexpectedNull;
    switch (air) {
        .Void => {},
        else => return error.Invalid,
    }

    const stone = reg.get("core:stone") orelse return error.UnexpectedNull;
    switch (stone) {
        .Uniform => |u| try testing.expectEqualStrings("tex:stone", u.all),
        else => return error.Invalid,
    }

    const grass = reg.get("core:grass") orelse return error.UnexpectedNull;
    switch (grass) {
        .Facing => |f| {
            try testing.expectEqual(Face.up, f.face);
            try testing.expectEqualStrings("tex:grass_top", f.face_tex);
            try testing.expectEqualStrings("tex:grass_side", f.other_tex);
        },
        else => return error.Invalid,
    }
}
