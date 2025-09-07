const std = @import("std");
const wapi = @import("../worldgen_api.zig");

pub const Params = wapi.Params;
pub const BlockLookup = wapi.BlockLookup;

pub const Def = struct {
    key: []const u8,
    display_name: []const u8,
    // Optional phase hooks; modules should at least provide biomes and noise
    structures_starts: ?wapi.StructuresStartsFn = null,
    structures_references: ?wapi.StructuresRefsFn = null,
    biomes: ?wapi.BiomesFn = null,
    noise: ?wapi.NoiseFn = null,
    surface: ?wapi.SurfaceFn = null,
    carvers: ?wapi.CarversFn = null,
    features: ?wapi.FeaturesFn = null,
    initialize_light: ?wapi.InitLightFn = null,
    light: ?wapi.LightFn = null,
    spawn: ?wapi.SpawnFn = null,
};

pub const Registry = struct {
    allocator: std.mem.Allocator,
    by_key: std.StringHashMap(Def),

    pub fn init(allocator: std.mem.Allocator) Registry {
        return .{ .allocator = allocator, .by_key = std.StringHashMap(Def).init(allocator) };
    }

    pub fn deinit(self: *Registry) void {
        var it = self.by_key.valueIterator();
        while (it.next()) |d| {
            self.allocator.free(d.key);
            self.allocator.free(d.display_name);
        }
        self.by_key.deinit();
    }

    pub fn add(self: *Registry, key: []const u8, display_name: []const u8, hooks: Def) !void {
        if (self.by_key.get(key) != null) return;
        const k = try self.allocator.dupe(u8, key);
        const dn = try self.allocator.dupe(u8, display_name);
        var def = hooks;
        def.key = k;
        def.display_name = dn;
        try self.by_key.put(k, def);
    }

    pub fn get(self: *const Registry, key: []const u8) ?Def {
        if (self.by_key.get(key)) |d| return d;
        return null;
    }
};
