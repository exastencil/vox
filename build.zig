const std = @import("std");
const Build = std.Build;

fn collectModuleFiles(allocator: std.mem.Allocator, dir_path: []const u8, list: *std.ArrayList([]const u8)) !void {
    var dir = try std.fs.cwd().openDir(dir_path, .{ .iterate = true });
    defer dir.close();
    var it = dir.iterate();
    while (try it.next()) |entry| {
        switch (entry.kind) {
            .file => {
                if (std.mem.endsWith(u8, entry.name, ".zig")) {
                    const full = try std.fmt.allocPrint(allocator, "{s}/{s}", .{ dir_path, entry.name });
                    try list.append(allocator, full);
                }
            },
            .directory => {
                if (std.mem.eql(u8, entry.name, ".") or std.mem.eql(u8, entry.name, "..")) continue;
                const child = try std.fmt.allocPrint(allocator, "{s}/{s}", .{ dir_path, entry.name });
                try collectModuleFiles(allocator, child, list);
            },
            else => {},
        }
    }
}

fn generateModulesIndex(b: *Build, files: []const []const u8) !void {
    var arena = std.heap.ArenaAllocator.init(b.allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    var out = try std.ArrayList(u8).initCapacity(allocator, 0);
    const w = out.writer(allocator);
    // Emit root-owned imports of each module file using absolute paths
    var i: usize = 0;
    while (i < files.len) : (i += 1) {
        const abs = b.path(files[i]).getPath(b);
        try w.print("const m{d} = @import(\"{s}\");\n", .{ i, abs });
    }
    try w.writeAll("\n");
    try w.writeAll("pub const modules = .{\n");
    i = 0;
    while (i < files.len) : (i += 1) {
        try w.print("    m{d}.module,\n", .{i});
    }
    try w.writeAll("};\n");

    const contents = out.items;
    // Write into the source tree so main.zig can import it as part of the root module
    try std.fs.cwd().makePath("src/generated");
    var f = try std.fs.cwd().createFile("src/generated/vox_modules.zig", .{ .read = true, .truncate = true });
    defer f.close();
    try f.writeAll(contents);
}

pub fn build(b: *Build) !void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const dep_sokol = b.dependency("sokol", .{ .target = target, .optimize = optimize });

    // Discover modules and generate a compile-time index
    var arena = std.heap.ArenaAllocator.init(b.allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    var files = try std.ArrayList([]const u8).initCapacity(allocator, 0);
    collectModuleFiles(allocator, "src/modules", &files) catch |e| switch (e) {
        error.FileNotFound, error.NotDir => {},
        else => return e,
    };

    try generateModulesIndex(b, files.items);

    // Named core modules used across the build
    const mod_ids = b.createModule(.{ .root_source_file = b.path("src/ids.zig"), .target = target, .optimize = optimize });
    const mod_constants = b.createModule(.{ .root_source_file = b.path("src/constants.zig"), .target = target, .optimize = optimize });
    const mod_gs = b.createModule(.{ .root_source_file = b.path("src/gs.zig"), .target = target, .optimize = optimize, .imports = &.{ .{ .name = "ids", .module = mod_ids }, .{ .name = "constants", .module = mod_constants } } });
    const mod_noise = b.createModule(.{ .root_source_file = b.path("src/noise.zig"), .target = target, .optimize = optimize });

    // Aggregated module for src/ tree to avoid duplicate-file issues across modules
    const mod_vox = b.createModule(.{
        .root_source_file = b.path("src/vox.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "sokol", .module = dep_sokol.module("sokol") },
            .{ .name = "gs", .module = mod_gs },
            .{ .name = "ids", .module = mod_ids },
            .{ .name = "constants", .module = mod_constants },
            .{ .name = "noise", .module = mod_noise },
        },
    });

    // Root modules for the three executables
    const root_mod_full = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "sokol", .module = dep_sokol.module("sokol") },
            .{ .name = "gs", .module = mod_gs },
            .{ .name = "ids", .module = mod_ids },
            .{ .name = "constants", .module = mod_constants },
            .{ .name = "noise", .module = mod_noise },
        },
    });
    const root_mod_client = b.createModule(.{
        .root_source_file = b.path("src/bin/client_main.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "sokol", .module = dep_sokol.module("sokol") },
            .{ .name = "gs", .module = mod_gs },
            .{ .name = "ids", .module = mod_ids },
            .{ .name = "constants", .module = mod_constants },
            .{ .name = "noise", .module = mod_noise },
            // aggregate module exposing src/ tree
            .{ .name = "vox", .module = mod_vox },
        },
    });
    const root_mod_server = b.createModule(.{
        .root_source_file = b.path("src/bin/server_main.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "gs", .module = mod_gs },
            .{ .name = "ids", .module = mod_ids },
            .{ .name = "constants", .module = mod_constants },
            .{ .name = "noise", .module = mod_noise },
            .{ .name = "vox", .module = mod_vox },
        },
    });

    // Executables
    const exe_full = b.addExecutable(.{ .name = "vox-aetatum", .root_module = root_mod_full });
    const exe_client = b.addExecutable(.{ .name = "vox-client", .root_module = root_mod_client });
    const exe_server = b.addExecutable(.{ .name = "vox-server", .root_module = root_mod_server });

    // No-op: modules index is created on disk above before compiling

    // Shader generation (sokol-shdc) — required by client/full only
    const dep_tools = b.dependency("sokol-tools-bin", .{});
    const builtin = @import("builtin");
    const os_tag = builtin.target.os.tag;
    const arch_tag = builtin.target.cpu.arch;
    const tool_rel_path: []const u8 = switch (os_tag) {
        .macos => switch (arch_tag) {
            .aarch64 => "bin/osx_arm64/sokol-shdc",
            else => "bin/osx/sokol-shdc",
        },
        .linux => "bin/linux/sokol-shdc",
        .windows => "bin/win64/sokol-shdc.exe",
        else => "bin/linux/sokol-shdc",
    };
    const shdc_path = dep_tools.path(tool_rel_path);
    const shdc = b.addSystemCommand(&.{shdc_path.getPath(b)});
    shdc.addArgs(&.{
        "--input",                                b.path("shaders/chunk.glsl").getPath(b),
        "--output",                               b.path("src/shaders/chunk_shd.zig").getPath(b),
        "--slang=glsl430:hlsl5:metal_macos:wgsl", "--format=sokol_zig",
        "--bytecode",
    });
    // Ensure shader gets generated before compiling client/full
    exe_full.step.dependOn(&shdc.step);
    exe_client.step.dependOn(&shdc.step);

    // Install all artifacts
    b.installArtifact(exe_full);
    b.installArtifact(exe_client);
    b.installArtifact(exe_server);

    // Named build steps for each target
    b.step("full", "Build the full target (client + simulation in one process)").dependOn(&exe_full.step);
    b.step("client", "Build the client target (renders, connects to server)").dependOn(&exe_client.step);
    b.step("server", "Build the server target (headless, authoritative simulation)").dependOn(&exe_server.step);

    // Default run: full
    const run_full = b.addRunArtifact(exe_full);
    if (b.args) |args| run_full.addArgs(args);
    b.step("run", "Run Vox Aetatum (full)").dependOn(&run_full.step);

    // Named modules for tests

    // Unit test runner
    const test_step = b.step("test", "Run unit tests");

    // Noise tests
    const tests_noise = b.addTest(.{
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/noise/simplex.zig"),
            .target = target,
            .optimize = optimize,
        }),
    });
    test_step.dependOn(&tests_noise.step);

    // Additional registry and module tests re-enabled

    // Tests for registry and module

    // Registry tests (top-level registry)
    const tests_reg = b.addTest(.{
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/registry.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{ .{ .name = "sokol", .module = dep_sokol.module("sokol") }, .{ .name = "gs", .module = mod_gs }, .{ .name = "ids", .module = mod_ids } },
        }),
    });
    test_step.dependOn(&tests_reg.step);

    // Resource registry tests
    const tests_res = b.addTest(.{
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/registry/resource.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{ .{ .name = "sokol", .module = dep_sokol.module("sokol") }, .{ .name = "gs", .module = mod_gs }, .{ .name = "ids", .module = mod_ids } },
        }),
    });
    test_step.dependOn(&tests_res.step);

    // Module registry tests
    const tests_mod = b.addTest(.{
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/registry_module_test.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{ .{ .name = "gs", .module = mod_gs }, .{ .name = "ids", .module = mod_ids } },
        }),
    });
    test_step.dependOn(&tests_mod.step);

    // Worldgen modules (void, superflat) compile with named imports
    // Module worldgen compile tests (with named gs/ids/constants)
    const tests_wg_void = b.addTest(.{
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/worldgen_void_test.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{ .{ .name = "gs", .module = mod_gs }, .{ .name = "ids", .module = mod_ids }, .{ .name = "constants", .module = mod_constants } },
        }),
    });
    test_step.dependOn(&tests_wg_void.step);

    const tests_wg_superflat = b.addTest(.{
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/worldgen_superflat_test.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{ .{ .name = "gs", .module = mod_gs }, .{ .name = "ids", .module = mod_ids }, .{ .name = "constants", .module = mod_constants } },
        }),
    });
    test_step.dependOn(&tests_wg_superflat.step);
}
