const std = @import("std");
const vox = @import("vox");
const png = vox.png;
const tex_mod = vox.texture;

fn isPow2(x: u32) bool {
    return x != 0 and (x & (x - 1)) == 0;
}

fn usage() void {
    std.debug.print("Usage: texture-packer <png_path> <identifier>\n", .{});
}

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const alloc = gpa.allocator();

    var args = try std.process.argsWithAllocator(alloc);
    defer args.deinit();

    _ = args.next(); // prog name
    const png_path = args.next() orelse {
        usage();
        return error.InvalidUsage;
    };
    const identifier = args.next() orelse {
        usage();
        return error.InvalidUsage;
    };
    if (args.next()) |_| {
        usage();
        return error.InvalidUsage;
    }

    // Prevent directory traversal in output file name; keep it simple
    if (std.mem.indexOfAny(u8, identifier, "/\\")) |_| {
        std.log.err("identifier must not contain path separators", .{});
        return error.InvalidUsage;
    }

    // Load PNG as RGBA8
    const img = try png.loadFileRGBA8(alloc, png_path);
    defer alloc.free(img.pixels);

    if (img.width != img.height) {
        std.log.err("image dimensions must be square, got {d}x{d}", .{ img.width, img.height });
        return error.InvalidImage;
    }
    if (!isPow2(img.width)) {
        std.log.err("image side must be a power of two, got {d}", .{img.width});
        return error.InvalidImage;
    }

    // Encode TXTR
    const bin = try tex_mod.Binary.encode(alloc, identifier, img.pixels);
    defer alloc.free(bin);

    // Write <identifier>.txtr in current directory
    const out_name = try std.fmt.allocPrint(alloc, "{s}.txtr", .{identifier});
    defer alloc.free(out_name);

    var file = try std.fs.cwd().createFile(out_name, .{ .truncate = true });
    defer file.close();
    try file.writeAll(bin);

    std.debug.print("Wrote {s} ({d} bytes)\n", .{ out_name, bin.len });
}
