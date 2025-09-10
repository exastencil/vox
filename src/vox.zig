const std = @import("std");

// Re-export core source files so consumers can import a single module
pub const png = @import("png.zig");
pub const registry = @import("registry.zig");
pub const player = @import("player.zig");
pub const client = @import("client.zig");
pub const simulation = @import("simulation.zig");
pub const worldgen = @import("worldgen.zig");
pub const texture = @import("texture.zig");
pub const atlas = @import("atlas.zig");

// Generated modules
pub const vox_modules = @import("generated/vox_modules.zig");
