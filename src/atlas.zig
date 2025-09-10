const std = @import("std");
const Texture = @import("texture.zig").Texture;

pub const Rect = struct {
    x: u32,
    y: u32,
    w: u32,
    h: u32,
};

pub const UV = struct {
    u0: f32,
    v0: f32,
    u1: f32,
    v1: f32,
};

pub const TextureRef = struct {
    id: [:0]const u8,
    rgba8: []const u8,

    pub fn fromTexture(tex: *const Texture) TextureRef {
        return .{ .id = tex.id, .rgba8 = tex.rgba8 };
    }
};

pub const IndexEntry = struct {
    id: [:0]const u8,
    uv: UV,
    rect: Rect,
    atlas_index: u32,
};

pub const AtlasMeta = struct {
    width: u32,
    height: u32,
    // Range into entries array: [start, end) for this atlas
    start: u32,
    end: u32,
};

fn idBytes(id: [:0]const u8) []const u8 {
    return id[0 .. id.len - 1];
}

fn hash64(bytes: []const u8) u64 {
    var h: u64 = 1469598103934665603;
    for (bytes) |b| {
        h ^= b;
        h = h *% 1099511628211;
    }
    return h;
}

fn nextPow2usize(comptime n: usize) usize {
    var x: usize = if (n < 1) 1 else n - 1;
    var shift: usize = 1;
    while (shift < @bitSizeOf(usize)) : (shift <<= 1) {
        x |= (x >> shift);
    }
    return x + 1;
}

fn staticTableSize(comptime N: usize) usize {
    // target load <= 0.5 -> size >= nextPow2(N * 2)
    return nextPow2usize(N * 2);
}

pub const Config = struct {
    max_width: u32 = 4096,
    max_height: u32 = 4096,
    padding: u32 = 1,
    case_insensitive: bool = true,
};

fn asciiLower(b: u8) u8 {
    return std.ascii.toLower(b);
}

fn lessIdCaseSensitive(a: []const u8, b: []const u8) bool {
    const min_len = @min(a.len, b.len);
    var i: usize = 0;
    while (i < min_len) : (i += 1) {
        if (a[i] == b[i]) continue;
        return a[i] < b[i];
    }
    return a.len < b.len;
}

fn lessIdCaseInsensitive(a: []const u8, b: []const u8) bool {
    const min_len = @min(a.len, b.len);
    var i: usize = 0;
    while (i < min_len) : (i += 1) {
        const la = asciiLower(a[i]);
        const lb = asciiLower(b[i]);
        if (la == lb) continue;
        return la < lb;
    }
    return a.len < b.len;
}

fn lessId(cfg: Config, a: []const u8, b: []const u8) bool {
    return if (cfg.case_insensitive) lessIdCaseInsensitive(a, b) else lessIdCaseSensitive(a, b);
}

fn sortById(cfg: Config, items: []TextureRef) void {
    // Simple insertion sort to avoid std.sort API differences across Zig versions
    var i: usize = 1;
    while (i < items.len) : (i += 1) {
        const key = items[i];
        var j: isize = @as(isize, @intCast(i)) - 1;
        while (j >= 0) : (j -= 1) {
            const aj: usize = @intCast(j);
            if (!lessId(cfg, key.id[0 .. key.id.len - 1], items[aj].id[0 .. items[aj].id.len - 1])) break;
            items[aj + 1] = items[aj];
        }
        const pos: usize = @intCast(j + 1);
        items[pos] = key;
    }
}

fn computeUV(rect: Rect, atlas_w: u32, atlas_h: u32) UV {
    const fw = @as(f32, @floatFromInt(atlas_w));
    const fh = @as(f32, @floatFromInt(atlas_h));
    const x0 = @as(f32, @floatFromInt(rect.x)) / fw;
    const y0 = @as(f32, @floatFromInt(rect.y)) / fh;
    const x1 = @as(f32, @floatFromInt(rect.x + rect.w)) / fw;
    const y1 = @as(f32, @floatFromInt(rect.y + rect.h)) / fh;
    return .{ .u0 = x0, .v0 = y0, .u1 = x1, .v1 = y1 };
}

fn StaticPlanType(comptime N: usize) type {
    return struct {
        atlas_count: u32,
        entries: [N]IndexEntry,
        entry_count: u32,
        atlas_meta: [N]AtlasMeta,
        global_index: [staticTableSize(N)]u32, // open-addressing table mapping key -> entry index

        pub fn lookup(self: *const @This(), id_c: [:0]const u8) ?*const IndexEntry {
            return self.lookupSpan(idBytes(id_c));
        }

        pub fn lookupSpan(self: *const @This(), key: []const u8) ?*const IndexEntry {
            const EMPTY: u32 = 0xFFFF_FFFF;
            const mask: usize = self.global_index.len - 1;
            const h = hash64(key);
            var slot: usize = @intCast(h & @as(u64, mask));
            var steps: usize = 0;
            while (steps < self.global_index.len) : (steps += 1) {
                const idx = self.global_index[slot];
                if (idx == EMPTY) return null;
                const e = &self.entries[idx];
                if (std.mem.eql(u8, idBytes(e.id), key)) return e;
                slot = (slot + 1) & mask;
            }
            return null;
        }
    };
}

fn packShelvesStatic(comptime cfg: Config, comptime N: usize, inputs: [N]TextureRef) StaticPlanType(N) {
    // Copy and sort inputs to preserve caller order
    var sorted = inputs; // copy
    sortById(cfg, &sorted);

    var entries: [N]IndexEntry = undefined;
    var atlas_meta: [N]AtlasMeta = undefined;
    var table: [staticTableSize(N)]u32 = undefined;

    var cur_atlas: u32 = 0;
    var cur_x: u32 = 0;
    var cur_y: u32 = 0;
    var shelf_h: u32 = 0;
    var atlas_used_w: u32 = 0;
    var atlas_used_h: u32 = 0;
    var atlas_start: u32 = 0;

    var out_i: u32 = 0;

    inline for (sorted, 0..) |it, idx| {
        _ = idx;
        const side = inferSideStatic(it.rgba8.len);
        const w: u32 = side;
        const h: u32 = side;
        // New shelf if needed
        if (cur_x != 0 and cur_x + w > cfg.max_width) {
            // move to next shelf
            cur_y += shelf_h + cfg.padding;
            cur_x = 0;
            shelf_h = 0;
        }
        // New atlas if needed
        if (cur_x == 0 and (cur_y + h > cfg.max_height)) {
            // finalize previous atlas
            atlas_meta[cur_atlas] = .{
                .width = if (atlas_used_w == 0) 1 else atlas_used_w,
                .height = if (atlas_used_h == 0) 1 else atlas_used_h,
                .start = atlas_start,
                .end = out_i,
            };
            cur_atlas += 1;
            // reset shelf state
            cur_x = 0;
            cur_y = 0;
            shelf_h = 0;
            atlas_used_w = 0;
            atlas_used_h = 0;
            atlas_start = out_i;
        }
        // Place
        const rect = Rect{ .x = cur_x, .y = cur_y, .w = w, .h = h };
        // Advance
        cur_x = cur_x + w + cfg.padding;
        if (h + cfg.padding > shelf_h) shelf_h = h + cfg.padding;
        // Track used
        if (rect.x + rect.w > atlas_used_w) atlas_used_w = rect.x + rect.w;
        if (rect.y + rect.h > atlas_used_h) atlas_used_h = rect.y + rect.h;

        // Fill entry (UVs will be computed after atlas dims known)
        entries[out_i] = .{
            .id = it.id,
            .uv = .{ .u0 = 0, .v0 = 0, .u1 = 0, .v1 = 0 },
            .rect = rect,
            .atlas_index = cur_atlas,
        };
        out_i += 1;
    }

    // finalize last atlas
    atlas_meta[cur_atlas] = .{
        .width = if (atlas_used_w == 0) 1 else atlas_used_w,
        .height = if (atlas_used_h == 0) 1 else atlas_used_h,
        .start = atlas_start,
        .end = out_i,
    };

    const atlas_count = cur_atlas + 1;

    // Compute UVs now that we know atlas dims
    var i: usize = 0;
    while (i < out_i) : (i += 1) {
        const aidx = entries[i].atlas_index;
        const meta = atlas_meta[aidx];
        entries[i].uv = computeUV(entries[i].rect, meta.width, meta.height);
    }

    // Build global open-addressing index over all entries
    const EMPTY: u32 = 0xFFFF_FFFF;
    var t: usize = 0;
    while (t < table.len) : (t += 1) table[t] = EMPTY;
    const mask: usize = table.len - 1;
    var j: usize = 0;
    while (j < out_i) : (j += 1) {
        const key = idBytes(entries[j].id);
        const h = hash64(key);
        var slot: usize = @intCast(h & @as(u64, mask));
        while (true) {
            if (table[slot] == EMPTY) {
                table[slot] = @intCast(j);
                break;
            }
            slot = (slot + 1) & mask;
        }
    }

    return .{
        .atlas_count = atlas_count,
        .entries = entries,
        .entry_count = out_i,
        .atlas_meta = atlas_meta,
        .global_index = table,
    };
}

/// Static plan API suitable for comptime usage. Returns a value containing fixed-size arrays.
pub fn planStatic(comptime cfg: Config, comptime N: usize, inputs: [N]TextureRef) StaticPlanType(N) {
    return packShelvesStatic(cfg, N, inputs);
}

/// Dynamic plan API that uses an allocator and slices. Easier for runtime use.
pub const Plan = struct {
    atlas_meta: []AtlasMeta,
    entries: []IndexEntry,
    // All slices reference memory owned by this plan's arena/allocator
};

pub fn planDynamic(allocator: std.mem.Allocator, cfg: Config, inputs_slice: []const TextureRef) !Plan {
    // Copy and sort inputs
    const items = try allocator.alloc(TextureRef, inputs_slice.len);
    defer allocator.free(items);
    std.mem.copyForwards(TextureRef, items, inputs_slice);
    sortById(cfg, items);

    var metas = try allocator.alloc(AtlasMeta, items.len);
    var entries = try allocator.alloc(IndexEntry, items.len);

    var cur_atlas: u32 = 0;
    var cur_x: u32 = 0;
    var cur_y: u32 = 0;
    var shelf_h: u32 = 0;
    var atlas_used_w: u32 = 0;
    var atlas_used_h: u32 = 0;
    var atlas_start: u32 = 0;

    var out_i: u32 = 0;

    for (items) |it| {
        const side = try inferSideDynamic(it.rgba8.len);
        const w: u32 = side;
        const h: u32 = side;
        if (cur_x != 0 and cur_x + w > cfg.max_width) {
            cur_y += shelf_h + cfg.padding;
            cur_x = 0;
            shelf_h = 0;
        }
        if (cur_x == 0 and (cur_y + h > cfg.max_height)) {
            metas[cur_atlas] = .{
                .width = if (atlas_used_w == 0) 1 else atlas_used_w,
                .height = if (atlas_used_h == 0) 1 else atlas_used_h,
                .start = atlas_start,
                .end = out_i,
            };
            cur_atlas += 1;
            cur_x = 0;
            cur_y = 0;
            shelf_h = 0;
            atlas_used_w = 0;
            atlas_used_h = 0;
            atlas_start = out_i;
        }
        const rect = Rect{ .x = cur_x, .y = cur_y, .w = w, .h = h };
        cur_x = cur_x + w + cfg.padding;
        if (h + cfg.padding > shelf_h) shelf_h = h + cfg.padding;
        if (rect.x + rect.w > atlas_used_w) atlas_used_w = rect.x + rect.w;
        if (rect.y + rect.h > atlas_used_h) atlas_used_h = rect.y + rect.h;

        entries[out_i] = .{
            .id = it.id,
            .uv = .{ .u0 = 0, .v0 = 0, .u1 = 0, .v1 = 0 },
            .rect = rect,
            .atlas_index = cur_atlas,
        };
        out_i += 1;
    }

    metas[cur_atlas] = .{
        .width = if (atlas_used_w == 0) 1 else atlas_used_w,
        .height = if (atlas_used_h == 0) 1 else atlas_used_h,
        .start = atlas_start,
        .end = out_i,
    };

    const atlas_count = cur_atlas + 1;

    var i: usize = 0;
    while (i < out_i) : (i += 1) {
        const aidx = entries[i].atlas_index;
        const meta = metas[aidx];
        entries[i].uv = computeUV(entries[i].rect, meta.width, meta.height);
    }

    // Resize metas to used count
    metas = try allocator.realloc(metas, atlas_count);
    // entries length is out_i
    entries = try allocator.realloc(entries, out_i);

    return .{ .atlas_meta = metas, .entries = entries };
}

/// Compose pixels for each atlas using the provided output buffers.
/// outputs.len must equal atlas_count. Each outputs[i].pixels must be pre-allocated to width*height*4 bytes.
pub const OutAtlas = struct { pixels: []u8, width: u32, height: u32 };

pub fn compose(plan: anytype, outputs: []OutAtlas, inputs: []const TextureRef) !void {
    // plan can be either the static result or dynamic Plan
    const metas = plan.atlas_meta;
    if (outputs.len < metas.len) return error.NotEnoughOutputs;

    // Build a map from id pointer to input index for fast copy. Since ids are from inputs and preserved in entries, pointer compare is valid.
    var map = std.AutoHashMap(usize, usize).init(std.heap.page_allocator);
    defer map.deinit();
    try map.ensureTotalCapacity(@intCast(inputs.len));
    for (inputs, 0..) |t, idx| {
        try map.put(@intFromPtr(t.id.ptr), idx);
    }

    // Clear outputs
    for (outputs, 0..) |out, i| {
        const meta = metas[i];
        if (out.width != meta.width or out.height != meta.height) return error.MismatchedOutputSize;
        @memset(out.pixels, 0);
    }

    // Copy each entry's rgba into destination
    for (plan.entries) |e| {
        const idx = map.get(@intFromPtr(e.id.ptr)) orelse return error.InputNotFound;
        const src = inputs[idx];
        // Sanity: rect.w/h must match src dims
        const side = try inferSideDynamic(src.rgba8.len);
        if (e.rect.w != side or e.rect.h != side) return error.DimensionMismatch;

        const out = outputs[e.atlas_index];
        const dst_stride = out.width * 4;
        var row: u32 = 0;
        while (row < e.rect.h) : (row += 1) {
            const width_px: usize = @as(usize, side);
            const src_off: usize = (@as(usize, row) * width_px * 4);
            const dst_off: usize = (@as(usize, (e.rect.y + row)) * @as(usize, dst_stride)) + (@as(usize, e.rect.x) * 4);
            std.mem.copyForwards(u8, out.pixels[dst_off .. dst_off + width_px * 4], src.rgba8[src_off .. src_off + width_px * 4]);
        }
    }
}

fn inferSideStatic(rgba_len: usize) u32 {
    std.debug.assert(rgba_len % 4 == 0);
    const pixels: usize = rgba_len / 4;
    // Integer sqrt via increment (max 4096 steps for 4096x4096)
    var s: u32 = 1;
    while (@as(usize, s) * @as(usize, s) < pixels) : (s += 1) {}
    std.debug.assert(@as(usize, s) * @as(usize, s) == pixels);
    std.debug.assert((s & (s - 1)) == 0);
    return s;
}

fn inferSideDynamic(rgba_len: usize) !u32 {
    if (rgba_len % 4 != 0) return error.InvalidPixels;
    const pixels: usize = rgba_len / 4;
    var s: u32 = 1;
    while (@as(usize, s) * @as(usize, s) < pixels) : (s += 1) {
        // guard against overflow (not expected here)
        if (s == 0xFFFF_FFFF) return error.InvalidPixels;
    }
    if (@as(usize, s) * @as(usize, s) != pixels) return error.NonSquare;
    if ((s & (s - 1)) != 0) return error.NonPowerOfTwo;
    return s;
}

// -----------------
// Tests
// -----------------
const testing = std.testing;

test "static plan packs and groups by id; multiple atlases; static lookup works" {
    const cfg = Config{ .max_width = 8, .max_height = 8, .padding = 1, .case_insensitive = true };

    // 3 textures: 2x2, 4x4, 8x8
    const a = TextureRef{ .id = "a\x00", .rgba8 = &[_]u8{0} ** (2 * 2 * 4) };
    const b = TextureRef{ .id = "b\x00", .rgba8 = &[_]u8{1} ** (4 * 4 * 4) };
    const c = TextureRef{ .id = "c\x00", .rgba8 = &[_]u8{2} ** (8 * 8 * 4) };

    const plan = planStatic(cfg, 3, .{ a, b, c });
    try testing.expectEqual(@as(u32, 2), plan.atlas_count);

    // atlas 0 should contain ids a and b (alphabetical), atlas 1 contains c
    try testing.expect(plan.atlas_meta[0].end - plan.atlas_meta[0].start == 2);
    try testing.expect(plan.atlas_meta[1].end - plan.atlas_meta[1].start == 1);

    // Static lookup by span
    const found_b = plan.lookupSpan("b") orelse return error.Unexpected;
    try testing.expectEqualStrings("b", found_b.id[0 .. found_b.id.len - 1]);
    try testing.expectEqual(@as(u32, 0), found_b.atlas_index);

    // UVs are within 0..1
    for (plan.entries[0..plan.entry_count]) |e| {
        try testing.expect(e.uv.u0 >= 0 and e.uv.u0 <= 1);
        try testing.expect(e.uv.u1 >= 0 and e.uv.u1 <= 1);
        try testing.expect(e.uv.v0 >= 0 and e.uv.v0 <= 1);
        try testing.expect(e.uv.v1 >= 0 and e.uv.v1 <= 1);
    }
}

test "dynamic plan + compose copies pixels correctly" {
    const cfg = Config{ .max_width = 8, .max_height = 8, .padding = 1 };

    // Two 2x2 textures
    var px0: [2 * 2 * 4]u8 = undefined;
    var px1: [2 * 2 * 4]u8 = undefined;
    // fill with distinct colors
    @memset(px0[0..], 10);
    @memset(px1[0..], 20);
    const t0 = TextureRef{ .id = "a\x00", .rgba8 = &px0 };
    const t1 = TextureRef{ .id = "b\x00", .rgba8 = &px1 };

    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const alloc = gpa.allocator();

    const plan = try planDynamic(alloc, cfg, &[_]TextureRef{ t0, t1 });
    defer alloc.free(plan.atlas_meta);
    defer alloc.free(plan.entries);

    // One atlas expected
    try testing.expectEqual(@as(usize, 1), plan.atlas_meta.len);

    // Prepare output buffer
    const out_pixels = try alloc.alloc(u8, plan.atlas_meta[0].width * plan.atlas_meta[0].height * 4);
    defer alloc.free(out_pixels);

    var outs = [_]OutAtlas{.{ .pixels = out_pixels, .width = plan.atlas_meta[0].width, .height = plan.atlas_meta[0].height }};
    try compose(plan, &outs, &[_]TextureRef{ t0, t1 });

    // Check first pixel of each rect matches
    const e0 = plan.entries[0];
    const e1 = plan.entries[1];

    const stride = plan.atlas_meta[0].width * 4;

    const off0: usize = (@as(usize, e0.rect.y) * @as(usize, stride)) + (@as(usize, e0.rect.x) * 4);
    const off1: usize = (@as(usize, e1.rect.y) * @as(usize, stride)) + (@as(usize, e1.rect.x) * 4);

    try testing.expect(out_pixels[off0] == 10);
    try testing.expect(out_pixels[off1] == 20);
}
