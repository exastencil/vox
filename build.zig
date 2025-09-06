const std = @import("std");
const Build = std.Build;

pub fn build(b: *Build) !void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const dep_sokol = b.dependency("sokol", .{ .target = target, .optimize = optimize });

    // Feature flag to enable sokol-shdc (shader cross-compilation). Use:
    //   zig build -Duse_shdc=true
    const use_shdc = b.option(bool, "use_shdc", "Enable sokol-shdc shader generation") orelse false;

    // Build options module to let the code know if shdc is enabled
    const options = b.addOptions();
    options.addOption(bool, "use_shdc", use_shdc);

    const root_mod = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{.{ .name = "sokol", .module = dep_sokol.module("sokol") }},
    });
    root_mod.addOptions("build_options", options);

    const exe = b.addExecutable(.{
        .name = "vox-aetatum",
        .root_module = root_mod,
    });

    // If sokol-shdc is available, generate the Zig shader from GLSL
    if (use_shdc) {
        const shdc = b.addSystemCommand(&.{
            "sokol-shdc",
            "--input", b.path("shaders/chunk.glsl").getPath(b),
            "--output", b.path("src/shaders/chunk_shd.zig").getPath(b),
            "--slang=glsl330:glsl100:hlsl5:metal_macos:metal_ios:wgsl",
            "--format=sokol_zig",
            "--bytecode",
        });
        // Ensure shader gets generated before compiling the exe
        exe.step.dependOn(&shdc.step);
    }

    b.installArtifact(exe);

    const run = b.addRunArtifact(exe);
    if (b.args) |args| run.addArgs(args);
    b.step("run", "Run Vox Aetatum").dependOn(&run.step);

    // Unit tests for worldgen and core types
    const tests = b.addTest(.{
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/worldgen.zig"),
            .target = target,
            .optimize = optimize,
        }),
    });
    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&tests.step);

    // Registry tests
    const tests_reg = b.addTest(.{
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/registry.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{.{ .name = "sokol", .module = dep_sokol.module("sokol") }},
        }),
    });
    test_step.dependOn(&tests_reg.step);

    // Resource registry tests
    const tests_res = b.addTest(.{
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/registry/resource.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{.{ .name = "sokol", .module = dep_sokol.module("sokol") }},
        }),
    });
    test_step.dependOn(&tests_res.step);
}
