const std = @import("std");

/// 6 faces named relative to a block's facing direction. This lets directional blocks
/// specify per-face visuals declaratively without encoding world-absolute axes here.
pub const Face = enum {
    front,
    back,
    left,
    right,
    up,
    down,
};

/// A registry for Block.Defs keyed by block name (e.g., "core:stone").
/// Owns the key strings. Each stored Def borrows the map-owned key for its .name.
pub const Registry = struct {
    allocator: std.mem.Allocator,
    by_key: std.StringHashMap(Def),

    pub fn init(allocator: std.mem.Allocator) Registry {
        return .{ .allocator = allocator, .by_key = std.StringHashMap(Def).init(allocator) };
    }

    pub fn deinit(self: *Registry) void {
        // For each entry: free Def internals without freeing name; then free map-owned keys.
        var vit = self.by_key.valueIterator();
        while (vit.next()) |vptr| vptr.deinit(self.allocator, false);
        var kit = self.by_key.keyIterator();
        while (kit.next()) |kptr| self.allocator.free(kptr.*);
        self.by_key.deinit();
    }

    /// Upsert a block definition by name. The registry duplicates the key string
    /// and overwrites def.name to borrow the map-owned key. If an entry exists, the
    /// previous Def is deinitialized without freeing its borrowed name.
    pub fn set(self: *Registry, key: []const u8, def_in: Def) !void {
        if (self.by_key.getPtr(key)) |p| {
            // Existing entry: reuse existing key pointer and replace value
            const key_ptr = p.name; // the stored Def name equals the map-owned key
            var old = p.*;
            // Free old internals but keep the key string
            old.deinit(self.allocator, false);

            var new_def = def_in;
            // If incoming def allocated its own name, free it and borrow map key
            if (!std.mem.eql(u8, new_def.name, key)) self.allocator.free(@constCast(new_def.name));
            new_def.name = key_ptr;
            p.* = new_def;
            return;
        }
        // Insert new entry: own the key and make Def borrow it
        const k_owned = try self.allocator.dupe(u8, key);
        var def = def_in;
        if (!std.mem.eql(u8, def.name, key)) self.allocator.free(@constCast(def.name));
        def.name = k_owned;
        try self.by_key.put(k_owned, def);
    }

    pub fn get(self: *const Registry, key: []const u8) ?Def {
        if (self.by_key.get(key)) |d| return d;
        return null;
    }

    pub fn getPtr(self: *Registry, key: []const u8) ?*Def {
        return self.by_key.getPtr(key);
    }
};

// Global, optional Registry instance for convenient access across modules.
var g_block_registry: ?Registry = null;

pub fn initGlobal(allocator: std.mem.Allocator) void {
    if (g_block_registry == null) g_block_registry = Registry.init(allocator);
}

pub fn deinitGlobal() void {
    if (g_block_registry) |*r| {
        r.deinit();
        g_block_registry = null;
    }
}

pub fn global() *Registry {
    return &g_block_registry.?;
}

/// A single visual layer applied to a face. For now we support only Texture.
/// Identifiers are string keys (e.g., "core:stone") that are interpreted by the
/// client renderer to resolve actual GPU resources. Keeping identifiers here keeps
/// headless/server builds graphics-free.
pub const FaceLayer = union(enum) {
    Texture: struct {
        tex: []const u8, // texture identifier
        tint: ?[]const u8 = null, // optional tint identifier (e.g., biome tint key)
    },
};

/// A FaceConfig is a small stack of layers applied together. Multiple faces can
/// reference the same FaceConfig by index.
pub const FaceConfig = struct {
    layers: []FaceLayer, // owned by the block definition's allocator
};

/// Block definition: logical properties plus visual FaceConfigs.
pub const Def = struct {
    // Identity
    name: []const u8, // namespace:name (owned)

    // Physics/collision properties
    collision: bool = true,

    // Visual config
    // Between 1 and 6 face configs; faces select which config by index.
    face_configs: []FaceConfig, // owned
    face_config_index: [6]u8 = .{ 0, 0, 0, 0, 0, 0 }, // per-face index into face_configs

    /// Free owned memory (optionally including name), plus face_configs, layers, and any owned strings in layers
    pub fn deinit(self: *Def, allocator: std.mem.Allocator, free_name: bool) void {
        if (free_name) allocator.free(self.name);
        // Free layers first, then the face_configs slice itself
        for (self.face_configs) |fc| {
            // free each layer's owned strings
            for (fc.layers) |l| switch (l) {
                .Texture => |t| {
                    allocator.free(@constCast(t.tex));
                    if (t.tint) |ti| allocator.free(@constCast(ti));
                },
            };
            allocator.free(fc.layers);
        }
        allocator.free(self.face_configs);
        // Clear to safe defaults (not strictly necessary)
        self.* = .{
            .name = &[_]u8{},
            .collision = true,
            .face_configs = &[_]FaceConfig{},
            .face_config_index = .{ 0, 0, 0, 0, 0, 0 },
        };
    }

    /// Create a minimal definition with a single empty FaceConfig. Useful as a placeholder.
    pub fn initEmpty(allocator: std.mem.Allocator, name_in: []const u8) !Def {
        const name_owned = try allocator.dupe(u8, name_in);
        // allocate 0-layer array
        const layers = try allocator.alloc(FaceLayer, 0);
        // one FaceConfig referencing the empty layer list
        var fcs = try allocator.alloc(FaceConfig, 1);
        fcs[0] = .{ .layers = layers };
        return .{
            .name = name_owned,
            .collision = true,
            .face_configs = fcs,
            .face_config_index = .{ 0, 0, 0, 0, 0, 0 },
        };
    }

    /// Builder: uniform single-texture on all faces, optional tint, configurable collision.
    pub fn uniformTexture(
        allocator: std.mem.Allocator,
        name_in: []const u8,
        tex_in: []const u8,
        tint_in: ?[]const u8,
        collision: bool,
    ) !Def {
        const name_owned = try allocator.dupe(u8, name_in);
        const tex_owned = try allocator.dupe(u8, tex_in);
        errdefer allocator.free(tex_owned);
        const tint_owned = if (tint_in) |ti| try allocator.dupe(u8, ti) else null;
        errdefer if (tint_owned) |t| allocator.free(t);

        var layers = try allocator.alloc(FaceLayer, 1);
        errdefer allocator.free(layers);
        layers[0] = .{ .Texture = .{ .tex = tex_owned, .tint = tint_owned } };

        var fcs = try allocator.alloc(FaceConfig, 1);
        errdefer allocator.free(fcs);
        fcs[0] = .{ .layers = layers };

        return .{
            .name = name_owned,
            .collision = collision,
            .face_configs = fcs,
            .face_config_index = .{ 0, 0, 0, 0, 0, 0 },
        };
    }

    /// Builder: one face uses a unique texture (and optional tint), all others share another.
    pub fn facingTexture(
        allocator: std.mem.Allocator,
        name_in: []const u8,
        unique_face: Face,
        face_tex_in: []const u8,
        other_tex_in: []const u8,
        face_tint_in: ?[]const u8,
        other_tint_in: ?[]const u8,
        collision: bool,
    ) !Def {
        const name_owned = try allocator.dupe(u8, name_in);

        // Config 0: other faces
        const other_tex = try allocator.dupe(u8, other_tex_in);
        errdefer allocator.free(other_tex);
        const other_tint = if (other_tint_in) |ti| try allocator.dupe(u8, ti) else null;
        errdefer if (other_tint) |t| allocator.free(t);
        var layers0 = try allocator.alloc(FaceLayer, 1);
        errdefer allocator.free(layers0);
        layers0[0] = .{ .Texture = .{ .tex = other_tex, .tint = other_tint } };

        // Config 1: unique face
        const face_tex = try allocator.dupe(u8, face_tex_in);
        errdefer allocator.free(face_tex);
        const face_tint = if (face_tint_in) |ti| try allocator.dupe(u8, ti) else null;
        errdefer if (face_tint) |t| allocator.free(t);
        var layers1 = try allocator.alloc(FaceLayer, 1);
        errdefer allocator.free(layers1);
        layers1[0] = .{ .Texture = .{ .tex = face_tex, .tint = face_tint } };

        var fcs = try allocator.alloc(FaceConfig, 2);
        errdefer allocator.free(fcs);
        fcs[0] = .{ .layers = layers0 };
        fcs[1] = .{ .layers = layers1 };

        var idx: [6]u8 = .{ 0, 0, 0, 0, 0, 0 };
        idx[@intFromEnum(unique_face)] = 1;

        return .{
            .name = name_owned,
            .collision = collision,
            .face_configs = fcs,
            .face_config_index = idx,
        };
    }

    /// Builder: classic top/bottom/side, with optional tint on top.
    pub fn topBottomSides(
        allocator: std.mem.Allocator,
        name_in: []const u8,
        top_tex_in: []const u8,
        side_tex_in: []const u8,
        bottom_tex_in: ?[]const u8, // if null, uses side_tex
        top_tint_in: ?[]const u8,
        collision: bool,
    ) !Def {
        const name_owned = try allocator.dupe(u8, name_in);

        // Config 0: sides
        const side_tex = try allocator.dupe(u8, side_tex_in);
        errdefer allocator.free(side_tex);
        const side_tint: ?[]const u8 = null; // typically no tint on sides
        var layers_side = try allocator.alloc(FaceLayer, 1);
        errdefer allocator.free(layers_side);
        layers_side[0] = .{ .Texture = .{ .tex = side_tex, .tint = side_tint } };

        // Config 1: top
        const top_tex = try allocator.dupe(u8, top_tex_in);
        errdefer allocator.free(top_tex);
        const top_tint = if (top_tint_in) |ti| try allocator.dupe(u8, ti) else null;
        errdefer if (top_tint) |t| allocator.free(t);
        var layers_top = try allocator.alloc(FaceLayer, 1);
        errdefer allocator.free(layers_top);
        layers_top[0] = .{ .Texture = .{ .tex = top_tex, .tint = top_tint } };

        // Config 2: bottom (default to side texture if not provided)
        const bottom_tex = try allocator.dupe(u8, (bottom_tex_in orelse side_tex_in));
        errdefer allocator.free(bottom_tex);
        var layers_bottom = try allocator.alloc(FaceLayer, 1);
        errdefer allocator.free(layers_bottom);
        layers_bottom[0] = .{ .Texture = .{ .tex = bottom_tex, .tint = null } };

        var fcs = try allocator.alloc(FaceConfig, 3);
        errdefer allocator.free(fcs);
        fcs[0] = .{ .layers = layers_side };
        fcs[1] = .{ .layers = layers_top };
        fcs[2] = .{ .layers = layers_bottom };

        var idx: [6]u8 = .{ 0, 0, 0, 0, 0, 0 };
        idx[@intFromEnum(Face.up)] = 1;
        idx[@intFromEnum(Face.down)] = 2;
        // north, south, west, east already 0 (sides)

        return .{
            .name = name_owned,
            .collision = collision,
            .face_configs = fcs,
            .face_config_index = idx,
        };
    }
};
