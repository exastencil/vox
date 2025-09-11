const sokol = @import("sokol");
const sg = sokol.gfx;
const client_mesher_mod = @import("mesher.zig");

pub const SectionDraw = client_mesher_mod.SectionDraw;

pub const RegionMesh = struct {
    vbuf: sg.Buffer = .{},
    draws: []SectionDraw,
    built_chunk_count: usize = 0,
    dirty: bool = false,
    inflight: bool = false,
    last_built_frame: u32 = 0,
};

pub const REGION_MESH_BUDGET: usize = 128;
