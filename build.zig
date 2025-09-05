const std = @import("std");
const Build = std.Build;

pub fn build(b: *Build) !void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const dep_sokol = b.dependency("sokol", .{ .target = target, .optimize = optimize });

    const exe = b.addExecutable(.{
        .name = "vox-aetatum",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{.{ .name = "sokol", .module = dep_sokol.module("sokol") }},
        }),
    });

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
        }),
    });
    test_step.dependOn(&tests_reg.step);
}
