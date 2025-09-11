// Client atlas planning and composition
const std = @import("std");
const sokol = @import("sokol");
const sg = sokol.gfx;

pub const AtlasUv = struct { aidx: u32, u0: f32, v0: f32, u1: f32, v1: f32 };

// Generic over self; expects fields used below to be present on caller (Client)
pub fn buildBlockAtlases(self: anytype) void {
    const texbin = @import("../texture.zig");
    const atlas = @import("../atlas.zig");
    // Gather inputs (ids and rgba8) from resources/*.txtr
    var id_list = std.ArrayList([]const u8).empty; // will be transferred into self.atlas_id_storage to keep keys alive
    var inputs = std.ArrayList(atlas.TextureRef).empty;
    var bufs = std.ArrayList([]u8).empty;
    var ids_cstr = std.ArrayList([]u8).empty;
    defer {
        // id_list items are transferred to self.atlas_id_storage to keep key memory alive
        // (freed in Client.deinit)
        // id_list.deinit is called after transfer below
        // free owned buffers after use below
        for (bufs.items) |b| if (b.len > 0) self.allocator.free(b);
        bufs.deinit(self.allocator);
        // free owned 0-terminated id copies
        for (ids_cstr.items) |c| self.allocator.free(c);
        ids_cstr.deinit(self.allocator);
        inputs.deinit(self.allocator);
    }

    var stack = std.ArrayList([]const u8).empty;
    defer {
        for (stack.items) |p| self.allocator.free(p);
        stack.deinit(self.allocator);
    }
    const root = self.allocator.dupe(u8, "resources") catch return;
    stack.append(self.allocator, root) catch return;
    while (stack.items.len > 0) {
        const dir_path = stack.pop().?;
        var dir = std.fs.cwd().openDir(dir_path, .{ .iterate = true }) catch {
            self.allocator.free(dir_path);
            continue;
        };
        defer dir.close();
        var it = dir.iterate();
        while (it.next() catch null) |entry| {
            if (entry.kind == .directory) {
                if (std.mem.eql(u8, entry.name, ".") or std.mem.eql(u8, entry.name, "..")) continue;
                const child = std.fmt.allocPrint(self.allocator, "{s}/{s}", .{ dir_path, entry.name }) catch continue;
                stack.append(self.allocator, child) catch {};
                continue;
            }
            if (entry.kind == .file and std.mem.endsWith(u8, entry.name, ".txtr")) {
                const file_path = std.fmt.allocPrint(self.allocator, "{s}/{s}", .{ dir_path, entry.name }) catch continue;
                defer self.allocator.free(file_path);
                var f = std.fs.cwd().openFile(file_path, .{}) catch continue;
                defer f.close();
                const st = f.stat() catch continue;
                const buf = self.allocator.alloc(u8, st.size) catch continue;
                if ((f.readAll(buf) catch 0) != st.size) {
                    self.allocator.free(buf);
                    continue;
                }
                const tx = texbin.Binary.parse(buf) catch {
                    self.allocator.free(buf);
                    continue;
                };
                // Own the buffer; tx.rgba8 references it
                bufs.append(self.allocator, buf) catch {};
                const id_bytes = tx.idSpan();
                const id_owned = self.allocator.dupe(u8, id_bytes) catch continue;
                id_list.append(self.allocator, id_owned) catch {};
                // make an owned 0-terminated copy for TextureRef.id
                const id_c = self.allocator.alloc(u8, id_bytes.len + 1) catch continue;
                @memcpy(id_c[0..id_bytes.len], id_bytes);
                id_c[id_bytes.len] = 0;
                ids_cstr.append(self.allocator, id_c) catch {};
                const id_c_view: [:0]const u8 = id_c[0..id_bytes.len :0];
                inputs.append(self.allocator, .{ .id = id_c_view, .rgba8 = tx.rgba8 }) catch {};
            }
        }
        self.allocator.free(dir_path);
    }
    if (inputs.items.len == 0) {
        // Fallback: create a 1x1 white atlas so rendering can proceed
        var white = [_]u8{ 255, 255, 255, 255 };
        self.atlas_imgs[0] = sg.makeImage(.{
            .type = ._2D,
            .width = 1,
            .height = 1,
            .num_mipmaps = 1,
            .pixel_format = .RGBA8,
            .data = .{ .subimage = .{ .{ sg.asRange(white[0..]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} } } },
        });
        self.atlas_views[0] = sg.makeView(.{ .texture = .{ .image = self.atlas_imgs[0] } });
        self.atlas_count = 1;
        self.textures_ready = true;
        return;
    }

    // Plan and compose atlases
    const cfg: atlas.Config = .{ .max_width = 4096, .max_height = 4096, .padding = 1, .case_insensitive = true };
    const plan = atlas.planDynamic(self.allocator, cfg, inputs.items) catch return;
    defer {
        self.allocator.free(plan.atlas_meta);
        self.allocator.free(plan.entries);
    }

    self.atlas_count = @min(@as(u32, @intCast(plan.atlas_meta.len)), @as(u32, 4));
    var out_pixels = std.ArrayList([]u8).empty;
    defer {
        for (out_pixels.items) |p| if (p.len > 0) self.allocator.free(p);
        out_pixels.deinit(self.allocator);
    }

    var outs = std.ArrayList(@import("../atlas.zig").OutAtlas).empty;
    defer outs.deinit(self.allocator);

    var i: usize = 0;
    while (i < self.atlas_count) : (i += 1) {
        const w = plan.atlas_meta[i].width;
        const h = plan.atlas_meta[i].height;
        const px = self.allocator.alloc(u8, @as(usize, w) * @as(usize, h) * 4) catch break;
        out_pixels.append(self.allocator, px) catch {};
        outs.append(self.allocator, .{ .pixels = px, .width = w, .height = h }) catch {};
    }
    atlas.compose(plan, outs.items, inputs.items) catch return;

    // Upload atlases and build views
    i = 0;
    while (i < self.atlas_count) : (i += 1) {
        if (self.atlas_imgs[i].id != 0) sg.destroyImage(self.atlas_imgs[i]);
        if (self.atlas_views[i].id != 0) sg.destroyView(self.atlas_views[i]);
        self.atlas_imgs[i] = sg.makeImage(.{
            .type = ._2D,
            .width = @intCast(plan.atlas_meta[i].width),
            .height = @intCast(plan.atlas_meta[i].height),
            .num_mipmaps = 1,
            .pixel_format = .RGBA8,
            .data = .{ .subimage = .{ .{ sg.asRange(out_pixels.items[i]), .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} }, .{ .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{}, .{} } } },
        });
        self.atlas_views[i] = sg.makeView(.{ .texture = .{ .image = self.atlas_imgs[i] } });
    }

    // Build atlas_uvs_by_id
    self.atlas_uvs_by_id.clearRetainingCapacity();
    inline for ([_]u32{0}) |_| {}
    var mapped: usize = 0;
    for (id_list.items, 0..) |id_bytes, idx| {
        _ = idx;
        // Find entry by matching span against entries (strip 0-terminator on entry id)
        var ei: usize = 0;
        while (ei < plan.entries.len) : (ei += 1) {
            const e = plan.entries[ei];
            const entry_id = e.id[0..e.id.len];
            var matched = std.mem.eql(u8, entry_id, id_bytes);
            if (!matched and entry_id.len <= id_bytes.len) {
                matched = std.mem.eql(u8, entry_id, id_bytes[0..entry_id.len]);
            }
            if (matched) {
                // Transfer ownership of id_bytes into atlas_id_storage so map key memory stays alive
                if (self.atlas_id_storage.append(self.allocator, id_bytes) catch null) |_| {}
                const uv = AtlasUv{ .aidx = e.atlas_index, .u0 = e.uv.u0, .v0 = e.uv.v0, .u1 = e.uv.u1, .v1 = e.uv.v1 };
                if (self.atlas_uvs_by_id.put(id_bytes, uv) catch null) |_| {}
                mapped += 1;
                break;
            }
        }
    }
    // After transfer, drop the temporary list structure but NOT the strings
    id_list.deinit(self.allocator);
    // mapped count only used in debug builds; intentionally unused in release
    // Mark textures ready after successful atlas upload and mapping
    self.textures_ready = true;
}
