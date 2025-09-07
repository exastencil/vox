const std = @import("std");

// A very small resource registry focused on block rendering resources.
// Blocks can be assigned one of three resource styles:
// - Void:     nothing rendered (e.g. core:air)
// - Uniform:  same texture on all faces
// - Facing:   one specified face uses a unique texture, all others share one texture
//
// IMPORTANT: To keep server/headless builds free of graphics dependencies, this
// registry stores string paths to texture assets instead of GPU image handles.
// Client/full builds are responsible for loading these paths into sg.Image
// handles at runtime.
//
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
        all_path: []const u8, // path to texture for all faces
    },
    Facing: struct {
        // one face has a different texture, face selection is handled elsewhere
        face_path: []const u8,
        other_path: []const u8,
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
        // Free owned keys and any owned path strings
        var vit = self.by_block.valueIterator();
        while (vit.next()) |vptr| {
            switch (vptr.*) {
                .Uniform => |u| self.allocator.free(@constCast(u.all_path)),
                .Facing => |f| {
                    self.allocator.free(@constCast(f.face_path));
                    self.allocator.free(@constCast(f.other_path));
                },
                .Void => {},
            }
        }
        var kit = self.by_block.keyIterator();
        while (kit.next()) |kptr| self.allocator.free(kptr.*);
        self.by_block.deinit();
    }

    // Upsert behavior: later registrations override earlier ones.
    // This supports resource packs overriding visuals.
    pub fn setVoid(self: *Registry, block_key: []const u8) !void {
        try self.replace(block_key, .{ .Void = {} });
    }

    pub fn setUniformPath(self: *Registry, block_key: []const u8, all_path_in: []const u8) !void {
        const owned_path = try self.allocator.dupe(u8, all_path_in);
        const res = BlockRes{ .Uniform = .{ .all_path = owned_path } };
        try self.replace(block_key, res);
    }

    pub fn setFacingPaths(self: *Registry, block_key: []const u8, face_path_in: []const u8, other_path_in: []const u8) !void {
        const face_owned = try self.allocator.dupe(u8, face_path_in);
        const other_owned = try self.allocator.dupe(u8, other_path_in);
        const res = BlockRes{ .Facing = .{ .face_path = face_owned, .other_path = other_owned } };
        try self.replace(block_key, res);
    }

    pub fn get(self: *const Registry, block_key: []const u8) ?BlockRes {
        if (self.by_block.get(block_key)) |r| return r;
        return null;
    }

    fn replace(self: *Registry, block_key: []const u8, res: BlockRes) !void {
        // If entry exists, free any previously owned paths and replace value
        if (self.by_block.getPtr(block_key)) |p| {
            switch (p.*) {
                .Uniform => |u| self.allocator.free(@constCast(u.all_path)),
                .Facing => |f| {
                    self.allocator.free(@constCast(f.face_path));
                    self.allocator.free(@constCast(f.other_path));
                },
                .Void => {},
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

    try reg.setUniformPath("core:stone", "resources/textures/stone.png");
    try reg.setFacingPaths("core:grass", "resources/textures/grass_top.png", "resources/textures/grass_side.png");

    const air = reg.get("core:air") orelse return error.UnexpectedNull;
    switch (air) {
        .Void => {},
        else => return error.Invalid,
    }

    const stone = reg.get("core:stone") orelse return error.UnexpectedNull;
    switch (stone) {
        .Uniform => |u| {
            try testing.expectEqualStrings("resources/textures/stone.png", u.all_path);
        },
        else => return error.Invalid,
    }

    const grass = reg.get("core:grass") orelse return error.UnexpectedNull;
    switch (grass) {
        .Facing => |f| {
            try testing.expectEqualStrings("resources/textures/grass_top.png", f.face_path);
            try testing.expectEqualStrings("resources/textures/grass_side.png", f.other_path);
        },
        else => return error.Invalid,
    }
}
