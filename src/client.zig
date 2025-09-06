const std = @import("std");
const simulation = @import("simulation.zig");
const player = @import("player.zig");
const sokol = @import("sokol");
const sg = sokol.gfx;
const sapp = sokol.app;
const sglue = sokol.glue;
const sdtx = sokol.debugtext;

const Vertex = struct { pos: [2]f32, uv: [2]f32 };

fn buildQuadVerts(out: []Vertex, x0: f32, y0: f32, size: f32) usize {
    if (out.len < 6) return 0;
    const x1: f32 = x0 + size;
    const y1: f32 = y0 + size;
    out[0] = .{ .pos = .{ x0, y0 }, .uv = .{ 0, 0 } };
    out[1] = .{ .pos = .{ x1, y0 }, .uv = .{ 1, 0 } };
    out[2] = .{ .pos = .{ x1, y1 }, .uv = .{ 1, 1 } };
    out[3] = .{ .pos = .{ x0, y0 }, .uv = .{ 0, 0 } };
    out[4] = .{ .pos = .{ x1, y1 }, .uv = .{ 1, 1 } };
    out[5] = .{ .pos = .{ x0, y1 }, .uv = .{ 0, 1 } };
    return 6;
}

pub const Client = struct {
    allocator: std.mem.Allocator,
    sim: *simulation.Simulation,
    player_id: player.PlayerId,
    account_name: []const u8,
    connected: bool = false,

    // UI / rendering state
    pass_action: sg.PassAction = .{},
    ui_scale: f32 = 2.0,
    show_debug: bool = true,

    // Minimal chunk renderer state
    shd: sg.Shader = .{},
    pip: sg.Pipeline = .{},
    vbuf: sg.Buffer = .{},
    bind: sg.Bindings = .{},
    grass_img: sg.Image = .{},
    grass_view: sg.View = .{},
    sampler: sg.Sampler = .{},

    // Snapshot monitoring
    latest_snapshot: simulation.Snapshot = .{
        .tick = 0,
        .actual_tps = 0,
        .rolling_tps = 0,
        .dt_sec = 0,
        .world_seed = 0,
        .section_count_y = 0,
        .chunk_count = 0,
        .player_count = 0,
        .dynamic_entity_count = 0,
    },
    last_seen_tick: u64 = 0,

    pub fn init(allocator: std.mem.Allocator, sim: *simulation.Simulation, player_id: player.PlayerId, account_name: []const u8) !Client {
        const acc = try allocator.dupe(u8, account_name);
        return .{ .allocator = allocator, .sim = sim, .player_id = player_id, .account_name = acc };
    }

    pub fn deinit(self: *Client) void {
        self.allocator.free(self.account_name);
    }

    pub fn connect(self: *Client) !void {
        if (self.connected) return;
        try self.sim.connectPlayer(self.player_id, self.account_name);
        self.connected = true;
        // Ensure server creates/associates an entity for this player
        _ = self.sim.ensurePlayerEntity(self.player_id) catch {};
    }

    pub fn initGfx(self: *Client) void {
        // setup debug text and default pass action
        sdtx.setup(.{ .fonts = .{
            sdtx.fontKc853(),
            sdtx.fontKc854(),
            sdtx.fontZ1013(),
            sdtx.fontCpc(),
            sdtx.fontC64(),
            sdtx.fontOric(),
            .{},
            .{},
        } });
        sdtx.font(4);
        self.ui_scale = 2.0;
        self.pass_action.colors[0] = .{ .load_action = .CLEAR, .clear_value = .{ .r = 0, .g = 0, .b = 0, .a = 1 } };
        self.show_debug = true;

        // shader: use shdc-generated cross-platform shader
        const shd_mod = @import("shaders/chunk_shd.zig");
        const backend = sg.queryBackend();
        if (@hasDecl(shd_mod, "shaderDesc")) {
            self.shd = sg.makeShader(shd_mod.shaderDesc(backend));
        } else if (@hasDecl(shd_mod, "shader_desc")) {
            self.shd = sg.makeShader(shd_mod.shader_desc(backend));
        } else if (@hasDecl(shd_mod, "chunkShaderDesc")) {
            self.shd = sg.makeShader(shd_mod.chunkShaderDesc(backend));
        } else if (@hasDecl(shd_mod, "chunk_shader_desc")) {
            self.shd = sg.makeShader(shd_mod.chunk_shader_desc(backend));
        } else {
            @compileError("shdc-generated shader module does not expose a known descriptor function");
        }

        var pdesc: sg.PipelineDesc = .{};
        pdesc.label = "chunk-pipeline";
        pdesc.shader = self.shd;
        pdesc.layout.attrs[0].format = .FLOAT2;
        pdesc.layout.attrs[1].format = .FLOAT2;
        pdesc.primitive_type = .TRIANGLES;
        // ensure pipeline color target matches swapchain format
        const desc = sg.queryDesc();
        pdesc.color_count = 1;
        pdesc.colors[0].pixel_format = desc.environment.defaults.color_format;
        self.pip = sg.makePipeline(pdesc);
        // dynamic vertex buffer for a single chunk top surface (16*16 quads * 6 verts)
        const vcount_max: usize = (16 * 16 * 6);
        self.vbuf = sg.makeBuffer(.{ .usage = .{ .vertex_buffer = true, .stream_update = true }, .size = @intCast(vcount_max * @sizeOf(Vertex)) });
        self.bind.vertex_buffers[0] = self.vbuf;

        // pick grass top texture from registry
        if (self.sim.reg.resources.get("vox:grass")) |res| {
            switch (res) {
                .Facing => |f| self.grass_img = f.face_tex,
                .Uniform => |u| self.grass_img = u.all,
                .Void => self.grass_img = .{},
            }
        }
        // create a view and sampler for the texture
        self.grass_view = sg.makeView(.{ .texture = .{ .image = self.grass_img } });
        self.sampler = sg.makeSampler(.{ .min_filter = .NEAREST, .mag_filter = .NEAREST, .mipmap_filter = .NEAREST, .wrap_u = .REPEAT, .wrap_v = .REPEAT });
        self.bind.views[0] = self.grass_view;
        self.bind.samplers[0] = self.sampler;
    }

    fn buildTopSurfaceVerts(self: *Client, out: []Vertex) usize {
        // render chunk (0,0) from world "vox:overworld" if present
        const wk = "vox:overworld";
        self.sim.worlds_mutex.lock();
        defer self.sim.worlds_mutex.unlock();
        const ws = self.sim.worlds_state.getPtr(wk) orelse return 0;
        const rp = simulation.RegionPos{ .x = 0, .z = 0 };
        const rs = ws.regions.getPtr(rp) orelse return 0;
        const idx = rs.chunk_index.get(.{ .x = 0, .z = 0 }) orelse return 0;
        const ch = rs.chunks.items[idx];
        // map 16x16 grid to NDC square [-0.8,0.8]
        const start: f32 = -0.8;
        const size: f32 = 1.6 / 16.0;
        var n: usize = 0;
        for (0..16) |z| {
            for (0..16) |x| {
                const x0: f32 = start + @as(f32, @floatFromInt(x)) * size;
                const y0: f32 = start + @as(f32, @floatFromInt(z)) * size;
                const x1: f32 = x0 + size;
                const y1: f32 = y0 + size;
                // 2 triangles
                if (n + 6 > out.len) return n;
                out[n + 0] = .{ .pos = .{ x0, y0 }, .uv = .{ 0, 0 } };
                out[n + 1] = .{ .pos = .{ x1, y0 }, .uv = .{ 1, 0 } };
                out[n + 2] = .{ .pos = .{ x1, y1 }, .uv = .{ 1, 1 } };
                out[n + 3] = .{ .pos = .{ x0, y0 }, .uv = .{ 0, 0 } };
                out[n + 4] = .{ .pos = .{ x1, y1 }, .uv = .{ 1, 1 } };
                out[n + 5] = .{ .pos = .{ x0, y1 }, .uv = .{ 0, 1 } };
                n += 6;
            }
        }
        _ = ch; // currently unused heightmap
        return n;
    }

    pub fn frame(self: *Client) void {
        sg.beginPass(.{ .action = self.pass_action, .swapchain = sglue.swapchain() });

        // draw minimal top-surface for chunk (0,0)
        // Reserve space for 16*16*6 main verts
        var verts: [16 * 16 * 6]Vertex = undefined;
        // aspect-correction uniforms
        const w_px: f32 = @floatFromInt(sapp.width());
        const h_px: f32 = @floatFromInt(sapp.height());
        const aspect: f32 = h_px / w_px; // scale x by h/w to keep squares square
        const vs_params = struct { aspect: [2]f32 }{ .aspect = .{ aspect, 1.0 } };

        // Build main top-surface verts
        const vcount = self.buildTopSurfaceVerts(verts[0..]);

        if (vcount > 0 and self.grass_img.id != 0) {
            sg.applyPipeline(self.pip);
            sg.applyBindings(self.bind);
            sg.applyUniforms(0, sg.asRange(&vs_params));

            // Single buffer update per frame containing main surface
            sg.updateBuffer(self.vbuf, sg.asRange(verts[0..vcount]));

            // Draw main surface
            sg.draw(0, @intCast(vcount), 1);
        }

        if (self.show_debug) {
            const scale: f32 = self.ui_scale;
            const w_px_dbg: f32 = @floatFromInt(sapp.width());
            const h_px_dbg: f32 = @floatFromInt(sapp.height());
            sdtx.canvas(w_px_dbg / scale, h_px_dbg / scale);
            sdtx.origin(0, 0);
            sdtx.font(4);

            const snap = self.pollSnapshot().*;
            var buf: [96:0]u8 = undefined;
            const text_slice = std.fmt.bufPrint(buf[0 .. buf.len - 1], "tick: {d}  tps: {d:.2}  players: {d}", .{ snap.tick, snap.rolling_tps, snap.player_count }) catch "tick: ?";
            buf[text_slice.len] = 0;
            const text: [:0]const u8 = buf[0..text_slice.len :0];
            const cols_f: f32 = (w_px / scale) / 8.0;
            const text_cols_f: f32 = @floatFromInt(text.len);
            const col_start_f: f32 = @max(0.0, cols_f - text_cols_f - 1.0);
            sdtx.pos(col_start_f, 1.0);
            sdtx.color3b(255, 255, 255);
            sdtx.puts(text);

            sdtx.draw();
        }

        sg.endPass();
        sg.commit();
    }

    pub fn event(self: *Client, ev: sapp.Event) void {
        switch (ev.type) {
            .KEY_UP => {
                if (ev.key_code == .F3) self.show_debug = !self.show_debug;
            },
            else => {},
        }
    }

    pub fn shutdownGfx(self: *Client) void {
        _ = self; // currently nothing to do except sdtx, which main will shut down
    }

    // Pull the most recent snapshot from the simulation. Returns a pointer to the
    // internal latest_snapshot for convenience.
    pub fn pollSnapshot(self: *Client) *const simulation.Snapshot {
        const snap = self.sim.getSnapshot();
        if (snap.tick != self.last_seen_tick) {
            self.latest_snapshot = snap;
            self.last_seen_tick = snap.tick;
        }
        return &self.latest_snapshot;
    }
};
