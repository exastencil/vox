const std = @import("std");

// A very small resource registry focused on block rendering resources.
// Blocks can be assigned one of three resource styles:
// - Void:     nothing rendered (e.g. core:air)
// - Uniform:  same texture on all faces
// - Facing:   one specified face uses a unique texture, all others share one texture
//
// IMPORTANT: To keep server/headless builds free of graphics dependencies, this
// registry stores texture IDENTIFIERS (not file paths). Client/full builds are
// responsible for loading .txtr assets and mapping identifiers to GPU resources.
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
        all_id: []const u8, // texture identifier for all faces
    },
    Facing: struct {
        // one face has a different texture, face selection is handled elsewhere
        face_id: []const u8,
        other_id: []const u8,
    },
    /// AxisAlignedTintedPrimary: primary face on +Y (top) uses primary_face_id and is tinted
    /// using a biome tint key (e.g., "grass"); the -Y (bottom) face and the four sides use other textures.
    /// Future: allow selecting primary axis; for now only +Y is supported.
    AxisAlignedTintedPrimary: struct {
        primary_face_id: []const u8, // +Y face
        bottom_face_id: []const u8, // -Y face
        side_face_id: []const u8, // X/Z faces
        tint_key: []const u8, // biome tint key to apply on primary face
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
        // Free owned keys and any owned identifier strings
        var vit = self.by_block.valueIterator();
        while (vit.next()) |vptr| {
            switch (vptr.*) {
                .Uniform => |u| self.allocator.free(@constCast(u.all_id)),
                .Facing => |f| {
                    self.allocator.free(@constCast(f.face_id));
                    self.allocator.free(@constCast(f.other_id));
                },
                .AxisAlignedTintedPrimary => |a| {
                    self.allocator.free(@constCast(a.primary_face_id));
                    self.allocator.free(@constCast(a.bottom_face_id));
                    self.allocator.free(@constCast(a.side_face_id));
                    self.allocator.free(@constCast(a.tint_key));
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

    pub fn setUniformId(self: *Registry, block_key: []const u8, all_id_in: []const u8) !void {
        const owned = try self.allocator.dupe(u8, all_id_in);
        const res = BlockRes{ .Uniform = .{ .all_id = owned } };
        try self.replace(block_key, res);
    }

    pub fn setFacingIds(self: *Registry, block_key: []const u8, face_id_in: []const u8, other_id_in: []const u8) !void {
        const face_owned = try self.allocator.dupe(u8, face_id_in);
        const other_owned = try self.allocator.dupe(u8, other_id_in);
        const res = BlockRes{ .Facing = .{ .face_id = face_owned, .other_id = other_owned } };
        try self.replace(block_key, res);
    }

    pub fn setAxisAlignedTintedPrimaryIds(self: *Registry, block_key: []const u8, primary_face_id_in: []const u8, bottom_face_id_in: []const u8, side_face_id_in: []const u8, tint_key_in: []const u8) !void {
        const p = try self.allocator.dupe(u8, primary_face_id_in);
        const b = try self.allocator.dupe(u8, bottom_face_id_in);
        const s = try self.allocator.dupe(u8, side_face_id_in);
        const tk = try self.allocator.dupe(u8, tint_key_in);
        const res = BlockRes{ .AxisAlignedTintedPrimary = .{ .primary_face_id = p, .bottom_face_id = b, .side_face_id = s, .tint_key = tk } };
        try self.replace(block_key, res);
    }

    pub fn get(self: *const Registry, block_key: []const u8) ?BlockRes {
        if (self.by_block.get(block_key)) |r| return r;
        return null;
    }

    fn replace(self: *Registry, block_key: []const u8, res: BlockRes) !void {
        // If entry exists, free any previously owned identifiers and replace value
        if (self.by_block.getPtr(block_key)) |p| {
            switch (p.*) {
                .Uniform => |u| self.allocator.free(@constCast(u.all_id)),
                .Facing => |f| {
                    self.allocator.free(@constCast(f.face_id));
                    self.allocator.free(@constCast(f.other_id));
                },
                .AxisAlignedTintedPrimary => |a| {
                    self.allocator.free(@constCast(a.primary_face_id));
                    self.allocator.free(@constCast(a.bottom_face_id));
                    self.allocator.free(@constCast(a.side_face_id));
                    self.allocator.free(@constCast(a.tint_key));
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

    try reg.setUniformId("core:stone", "core:stone");
    try reg.setFacingIds("core:grass", "core:grass_top", "core:grass_side");

    const air = reg.get("core:air") orelse return error.UnexpectedNull;
    switch (air) {
        .Void => {},
        else => return error.Invalid,
    }

    const stone = reg.get("core:stone") orelse return error.UnexpectedNull;
    switch (stone) {
        .Uniform => |u| {
            try testing.expectEqualStrings("core:stone", u.all_id);
        },
        else => return error.Invalid,
    }

    const grass = reg.get("core:grass") orelse return error.UnexpectedNull;
    switch (grass) {
        .Facing => |f| {
            try testing.expectEqualStrings("core:grass_top", f.face_id);
            try testing.expectEqualStrings("core:grass_side", f.other_id);
        },
        else => return error.Invalid,
    }
}
