const std = @import("std");

test "registry.module compiles with src package root" {
    _ = @import("registry/module.zig");
}
