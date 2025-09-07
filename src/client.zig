const std = @import("std");
const simulation = @import("simulation.zig");
const player = @import("player.zig");
const sokol = @import("sokol");
const sg = sokol.gfx;
const sapp = sokol.app;
const sglue = sokol.glue;
const sdtx = sokol.debugtext;
const shd_mod = @import("shaders/chunk_shd.zig");

const Camera = struct {
    pos: [3]f32 = .{ 0, 0, 0 },
    yaw: f32 = 0.0,
    pitch: f32 = 0.0,
    roll: f32 = 0.0,
    // Derived forward direction from yaw/pitch
    dir: [3]f32 = .{ 0, 0, -1 },

    fn computeDir(yaw: f32, pitch: f32) [3]f32 {
        // Assuming yaw around +Y, pitch around +X, radians
        const cy = @cos(yaw);
        const sy = @sin(yaw);
        const cp = @cos(pitch);
        const sp = @sin(pitch);
        // Forward in right-handed Y-up: x = cp*cy, y = sp, z = cp*sy
        return .{ cp * cy, sp, cp * sy };
    }

    fn setYawPitch(self: *Camera, yaw: f32, pitch: f32) void {
        self.yaw = yaw;
        self.pitch = pitch;
        self.dir = computeDir(yaw, pitch);
    }

    fn setDir(self: *Camera, dir_in: [3]f32) void {
        // normalize and update yaw/pitch to match
        const dx = dir_in[0];
        const dy = dir_in[1];
        const dz = dir_in[2];
        const len: f32 = @sqrt(dx * dx + dy * dy + dz * dz);
        if (len > 0.000001) {
            const nx = dx / len;
            const ny = dy / len;
            const nz = dz / len;
            self.dir = .{ nx, ny, nz };
            self.pitch = std.math.asin(std.math.clamp(ny, -1.0, 1.0));
            self.yaw = std.math.atan2(nz, nx);
        }
    }
};

const Vertex = struct { pos: [3]f32, uv: [2]f32 };

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

    // GPU buffer capacity tracking
    vbuf_capacity_bytes: usize = 0,

    // Camera mirrored from player entity
    camera: Camera = .{},

    // Input state
    move_forward: bool = false,
    move_back: bool = false,
    move_left: bool = false,
    move_right: bool = false,
    jump_pressed: bool = false, // edge-triggered

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

    // Enable/disable raw mouse input if supported by the sokol app wrapper
    fn setRawMouse(on: bool) void {
        if (@hasDecl(sapp, "rawMouseSupported") and @hasDecl(sapp, "enableRawMouse")) {
            if (sapp.rawMouseSupported()) {
                sapp.enableRawMouse(on);
            }
        } else {}
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
        pdesc.layout.attrs[0].format = .FLOAT3;
        pdesc.layout.attrs[1].format = .FLOAT2;
        pdesc.primitive_type = .TRIANGLES;
        pdesc.cull_mode = .BACK;
        // Ensure we define a consistent front-face winding across backends
        pdesc.face_winding = .CCW;
        pdesc.depth = .{ .compare = .LESS_EQUAL, .write_enabled = true };
        // ensure pipeline color/depth target matches swapchain format
        const desc = sg.queryDesc();
        pdesc.color_count = 1;
        pdesc.colors[0].pixel_format = desc.environment.defaults.color_format;
        self.pip = sg.makePipeline(pdesc);
        // dynamic vertex buffer; start with a generous capacity (e.g., 16384 verts)
        const initial_vcount: usize = 16 * 16 * 16; // plenty for top + sides on flat worlds
        const initial_bytes: usize = initial_vcount * @sizeOf(Vertex);
        self.vbuf = sg.makeBuffer(.{ .usage = .{ .vertex_buffer = true, .stream_update = true }, .size = @intCast(initial_bytes) });
        self.vbuf_capacity_bytes = initial_bytes;
        self.bind.vertex_buffers[0] = self.vbuf;
        self.pass_action.depth = .{ .load_action = .CLEAR, .clear_value = 1.0 };

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
        // shader expects VIEW_tex_texture at slot 1 and SMP_tex_sampler at slot 2
        self.bind.views[1] = self.grass_view;
        self.bind.samplers[2] = self.sampler;

        // Initialize camera from player entity if available
        self.updateCameraFromPlayer();
        // Lock and enable raw mouse input on startup
        sapp.lockMouse(true);
        setRawMouse(true);
    }

    fn buildChunkSurfaceMesh(self: *Client, out: *std.ArrayList(Vertex)) usize {
        // Render all loaded chunks in the overworld
        const wk = "vox:overworld";
        self.sim.worlds_mutex.lock();
        defer self.sim.worlds_mutex.unlock();
        const ws = self.sim.worlds_state.getPtr(wk) orelse return 0;

        // Helpers to append a quad (two triangles) with CCW winding
        const addQuad = struct {
            fn call(list: *std.ArrayList(Vertex), v0: [3]f32, v1: [3]f32, v2: [3]f32, v3: [3]f32, uv0: [2]f32, uv1: [2]f32, uv2: [2]f32, uv3: [2]f32) void {
                // Emit triangles with flipped winding so CCW remains front-facing under the chosen view handedness
                list.appendAssumeCapacity(.{ .pos = v0, .uv = uv0 });
                list.appendAssumeCapacity(.{ .pos = v2, .uv = uv2 });
                list.appendAssumeCapacity(.{ .pos = v1, .uv = uv1 });
                list.appendAssumeCapacity(.{ .pos = v0, .uv = uv0 });
                list.appendAssumeCapacity(.{ .pos = v3, .uv = uv3 });
                list.appendAssumeCapacity(.{ .pos = v2, .uv = uv2 });
            }
        }.call;

        // convenience to get heightmap value with bounds check (outside => -1)
        const getH = struct {
            fn call(hm: *const [16 * 16]i32, x: i32, z: i32) i32 {
                if (x < 0 or x >= 16 or z < 0 or z >= 16) return -1;
                return hm[@as(usize, @intCast(z)) * 16 + @as(usize, @intCast(x))];
            }
        }.call;

        const verts_before: usize = out.items.len;

        // Iterate all regions and their chunks
        var reg_it = ws.regions.iterator();
        while (reg_it.next()) |rentry| {
            const rs_val = rentry.value_ptr.*; // copy for read-only access
            for (rs_val.chunks.items) |ch| {
                const base_x: f32 = @floatFromInt(ch.pos.x * 16);
                const base_z: f32 = @floatFromInt(ch.pos.z * 16);

                // Upper bound capacity per chunk (36 verts per column)
                const cur_len: usize = out.items.len;
                out.ensureTotalCapacity(self.allocator, cur_len + 16 * 16 * 6 * 6) catch {};

                // Build top faces and simple side faces from heightmap
                const hm = ch.heightmaps.world_surface;
                for (0..16) |z| {
                    for (0..16) |x| {
                        const xi: i32 = @intCast(x);
                        const zi: i32 = @intCast(z);
                        const h = hm[z * 16 + x];
                        if (h < 0) continue; // empty column
                        const y_top: f32 = @floatFromInt(h + 1);
                        const x0: f32 = base_x + @as(f32, @floatFromInt(x));
                        const x1: f32 = x0 + 1.0;
                        const z0: f32 = base_z + @as(f32, @floatFromInt(z));
                        const z1: f32 = z0 + 1.0;
                        // Top face (upward normal), CCW seen from +Y
                        addQuad(
                            out,
                            .{ x0, y_top, z0 },
                            .{ x0, y_top, z1 },
                            .{ x1, y_top, z1 },
                            .{ x1, y_top, z0 },
                            .{ 0, 0 },
                            .{ 0, 1 },
                            .{ 1, 1 },
                            .{ 1, 0 },
                        );
                        // Sides if neighbor lower (within the same chunk). For inter-chunk gaps, we'll
                        // handle neighbors later — this renders a simple "wall" at chunk borders for now.
                        const h_w = getH(&hm, xi - 1, zi);
                        if (h_w < h) {
                            const y0: f32 = @floatFromInt(h_w + 1);
                            addQuad(
                                out,
                                .{ x0, y0, z1 },
                                .{ x0, y_top, z1 },
                                .{ x0, y_top, z0 },
                                .{ x0, y0, z0 }, // -X face
                                .{ 0, 0 },
                                .{ 0, (y_top - y0) },
                                .{ 1, (y_top - y0) },
                                .{ 1, 0 },
                            );
                        }
                        const h_e = getH(&hm, xi + 1, zi);
                        if (h_e < h) {
                            const y0: f32 = @floatFromInt(h_e + 1);
                            addQuad(
                                out,
                                .{ x1, y0, z0 },
                                .{ x1, y_top, z0 },
                                .{ x1, y_top, z1 },
                                .{ x1, y0, z1 }, // +X face
                                .{ 0, 0 },
                                .{ 0, (y_top - y0) },
                                .{ 1, (y_top - y0) },
                                .{ 1, 0 },
                            );
                        }
                        const h_n = getH(&hm, xi, zi - 1);
                        if (h_n < h) {
                            const y0: f32 = @floatFromInt(h_n + 1);
                            addQuad(
                                out,
                                .{ x0, y0, z0 },
                                .{ x0, y_top, z0 },
                                .{ x1, y_top, z0 },
                                .{ x1, y0, z0 }, // -Z face (outward -Z)
                                .{ 0, 0 },
                                .{ 0, (y_top - y0) },
                                .{ 1, (y_top - y0) },
                                .{ 1, 0 },
                            );
                        }
                        const h_s = getH(&hm, xi, zi + 1);
                        if (h_s < h) {
                            const y0: f32 = @floatFromInt(h_s + 1);
                            addQuad(
                                out,
                                .{ x1, y0, z1 },
                                .{ x1, y_top, z1 },
                                .{ x0, y_top, z1 },
                                .{ x0, y0, z1 }, // +Z face (outward +Z)
                                .{ 0, 0 },
                                .{ 0, (y_top - y0) },
                                .{ 1, (y_top - y0) },
                                .{ 1, 0 },
                            );
                        }
                    }
                }
            }
        }
        return out.items.len - verts_before;
    }

    pub fn frame(self: *Client) void {
        // Keep camera mirrored to player each frame for now
        self.updateCameraFromPlayer();

        sg.beginPass(.{ .action = self.pass_action, .swapchain = sglue.swapchain() });

        // Apply movement inputs to player entity (client-side write into sim)
        self.applyMovementInputs();

        // Build chunk surface mesh into a dynamic list
        var vert_list = std.ArrayList(Vertex).initCapacity(self.allocator, 0) catch return;
        defer vert_list.deinit(self.allocator);
        _ = self.buildChunkSurfaceMesh(&vert_list);

        // Prepare MVP
        const w_px: f32 = @floatFromInt(sapp.width());
        const h_px: f32 = @floatFromInt(sapp.height());
        const aspect_wh: f32 = if (h_px > 0) (w_px / h_px) else 1.0;
        const proj = makePerspective(60.0 * std.math.pi / 180.0, aspect_wh, 0.1, 1000.0);
        const view = makeView(self.camera.pos, self.camera.dir, .{ 0, 1, 0 });
        const mvp = matMul(proj, view);
        var vs_params: shd_mod.VsParams = .{ .mvp = mvp };

        if (vert_list.items.len > 0 and self.grass_img.id != 0) {
            sg.applyPipeline(self.pip);
            sg.applyBindings(self.bind);
            sg.applyUniforms(0, sg.asRange(&vs_params));

            // Ensure vertex buffer capacity
            const needed_bytes: usize = vert_list.items.len * @sizeOf(Vertex);
            if (self.vbuf_capacity_bytes < needed_bytes) {
                if (self.vbuf.id != 0) sg.destroyBuffer(self.vbuf);
                self.vbuf = sg.makeBuffer(.{ .usage = .{ .vertex_buffer = true, .stream_update = true }, .size = @intCast(needed_bytes) });
                self.vbuf_capacity_bytes = needed_bytes;
                self.bind.vertex_buffers[0] = self.vbuf;
            }

            // Upload and draw
            sg.updateBuffer(self.vbuf, sg.asRange(vert_list.items));
            sg.draw(0, @intCast(vert_list.items.len), 1);
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

            // Left-side debug: player's position and look vector
            // Gather current player entity safely
            self.sim.connections_mutex.lock();
            var pos: [3]f32 = .{ 0, 0, 0 };
            var look: [3]f32 = .{ 0, 0, 0 };
            const idx_opt_dbg = self.sim.entity_by_player.get(self.player_id);
            if (idx_opt_dbg) |idx_dbg| {
                if (idx_dbg < self.sim.dynamic_entities.items.len) {
                    const e_dbg = self.sim.dynamic_entities.items[idx_dbg];
                    pos = e_dbg.pos;
                    look = e_dbg.look_dir;
                }
            }
            self.sim.connections_mutex.unlock();

            var pbuf: [96:0]u8 = undefined;
            const ptxt_slice = std.fmt.bufPrint(pbuf[0 .. pbuf.len - 1], "pos: {d:.2} {d:.2} {d:.2}", .{ pos[0], pos[1], pos[2] }) catch "pos:?";
            pbuf[ptxt_slice.len] = 0;
            const ptxt: [:0]const u8 = pbuf[0..ptxt_slice.len :0];
            sdtx.pos(1.0, 1.0);
            sdtx.color3b(255, 255, 255);
            sdtx.puts(ptxt);

            var lbuf: [96:0]u8 = undefined;
            const ltxt_slice = std.fmt.bufPrint(lbuf[0 .. lbuf.len - 1], "look: {d:.2} {d:.2} {d:.2}", .{ look[0], look[1], look[2] }) catch "look:?";
            lbuf[ltxt_slice.len] = 0;
            const ltxt: [:0]const u8 = lbuf[0..ltxt_slice.len :0];
            sdtx.pos(1.0, 2.0);
            sdtx.color3b(255, 255, 255);
            sdtx.puts(ltxt);

            sdtx.draw();
        }

        sg.endPass();
        sg.commit();
    }

    pub fn event(self: *Client, ev: sapp.Event) void {
        switch (ev.type) {
            .KEY_UP => {
                if (ev.key_code == .F3) self.show_debug = !self.show_debug;
                if (ev.key_code == .ESCAPE) {
                    // Release mouse when ESC is pressed
                    sapp.lockMouse(false);
                    setRawMouse(false);
                    self.clearMovementInputs();
                }
                switch (ev.key_code) {
                    .W => self.move_forward = false,
                    .S => self.move_back = false,
                    .A => self.move_left = false,
                    .D => self.move_right = false,
                    else => {},
                }
            },
            .FOCUSED => {
                // Lock the mouse when the window gains focus
                sapp.lockMouse(true);
                // Enable raw mouse input if supported
                setRawMouse(true);
            },
            .UNFOCUSED => {
                // Release the mouse when the window loses focus
                sapp.lockMouse(false);
                setRawMouse(false);
                self.clearMovementInputs();
            },
            .KEY_DOWN => {
                switch (ev.key_code) {
                    .W => self.move_forward = true,
                    .S => self.move_back = true,
                    .A => self.move_left = true,
                    .D => self.move_right = true,
                    .SPACE => self.jump_pressed = true,
                    else => {},
                }
            },
            .MOUSE_DOWN => {
                // If the mouse is unlocked (e.g., after ESC), clicking the window should re-lock it
                if (!sapp.mouseLocked()) {
                    sapp.lockMouse(true);
                    setRawMouse(true);
                }
            },
            .MOUSE_MOVE => {
                if (sapp.mouseLocked()) {
                    const sens: f32 = 0.002; // radians per pixel
                    const dx = ev.mouse_dx * sens;
                    self.sim.connections_mutex.lock();
                    const idx_opt = self.sim.entity_by_player.get(self.player_id);
                    if (idx_opt) |idx| {
                        if (idx < self.sim.dynamic_entities.items.len) {
                            var e = &self.sim.dynamic_entities.items[idx];
                            // rotate look_dir around Y by -dx so right-drag turns right (preferred feel)
                            const cur_yaw: f32 = std.math.atan2(e.look_dir[2], e.look_dir[0]);
                            const two_pi: f32 = 6.283185307179586;
                            var yaw = cur_yaw - dx;
                            if (yaw > std.math.pi) yaw -= two_pi;
                            if (yaw < -std.math.pi) yaw += two_pi;
                            const cy = @cos(yaw);
                            const sy = @sin(yaw);
                            e.look_dir[0] = cy;
                            e.look_dir[2] = sy;
                            // preserve vertical component (no pitch changes via mouse for now)
                        }
                    }
                    self.sim.connections_mutex.unlock();
                    // keep camera in sync immediately
                    self.updateCameraFromPlayer();
                }
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

    // Update camera to mirror the player's entity position and yaw/pitch
    fn updateCameraFromPlayer(self: *Client) void {
        // Use simulation's connections mutex to safely inspect player->entity mapping
        self.sim.connections_mutex.lock();
        defer self.sim.connections_mutex.unlock();

        const idx_opt = self.sim.entity_by_player.get(self.player_id);
        if (idx_opt) |idx| {
            if (idx < self.sim.dynamic_entities.items.len) {
                const e = self.sim.dynamic_entities.items[idx];
                // Position camera at the top-center of the player's bounding box (eye-like)
                const eye_y: f32 = e.pos[1] + e.aabb_half_extents[1];
                self.camera.pos = .{ e.pos[0], eye_y, e.pos[2] };
                self.camera.setDir(e.look_dir);
                self.camera.roll = e.yaw_pitch_roll[2];
            }
        }
    }

    // Apply currently-pressed movement keys to the player entity
    fn applyMovementInputs(self: *Client) void {
        // Compute desired horizontal velocity from inputs relative to yaw
        self.sim.connections_mutex.lock();
        defer self.sim.connections_mutex.unlock();
        const idx_opt = self.sim.entity_by_player.get(self.player_id);
        if (idx_opt) |idx| {
            if (idx < self.sim.dynamic_entities.items.len) {
                var e = &self.sim.dynamic_entities.items[idx];
                // Derive forward directly from the entity's look vector so movement matches camera
                var fwd_x: f32 = e.look_dir[0];
                var fwd_z: f32 = e.look_dir[2];
                var flen: f32 = @sqrt(fwd_x * fwd_x + fwd_z * fwd_z);
                if (flen < 0.0001) {
                    // Fallback to yaw projection if degenerate
                    const yaw = std.math.atan2(e.look_dir[2], e.look_dir[0]);
                    fwd_x = @cos(yaw);
                    fwd_z = @sin(yaw);
                    flen = @sqrt(fwd_x * fwd_x + fwd_z * fwd_z);
                }
                fwd_x /= flen;
                fwd_z /= flen;
                const right_x: f32 = fwd_z; // rotate forward 90° CCW for right (corrected)
                const right_z: f32 = -fwd_x;
                var vx: f32 = 0;
                var vz: f32 = 0;
                if (self.move_forward) {
                    vx += fwd_x;
                    vz += fwd_z;
                }
                if (self.move_back) {
                    vx -= fwd_x;
                    vz -= fwd_z;
                }
                if (self.move_right) {
                    vx += right_x;
                    vz += right_z;
                }
                if (self.move_left) {
                    vx -= right_x;
                    vz -= right_z;
                }
                // Normalize if any input
                const mag: f32 = @sqrt(vx * vx + vz * vz);
                if (mag > 0.0001) {
                    vx /= mag;
                    vz /= mag;
                }
                const speed: f32 = 4.5; // m/s walking speed
                e.vel[0] = vx * speed;
                e.vel[2] = vz * speed;
                // While moving, align yaw with look vector and zero pitch/roll to stay upright
                if (self.move_forward or self.move_back or self.move_left or self.move_right) {
                    const look_yaw: f32 = std.math.atan2(e.look_dir[2], e.look_dir[0]);
                    e.yaw_pitch_roll = .{ look_yaw, 0, 0 };
                }
                // Jump impulse on edge press if on ground
                if (self.jump_pressed and e.flags.on_ground) {
                    const g: f32 = 9.80665;
                    const target_h: f32 = 1.1; // slightly over 1 block
                    const v0: f32 = @sqrt(2.0 * g * target_h);
                    e.vel[1] = v0;
                    e.flags.on_ground = false;
                }
            }
        }
        // consume jump edge
        self.jump_pressed = false;
    }

    fn clearMovementInputs(self: *Client) void {
        self.move_forward = false;
        self.move_back = false;
        self.move_left = false;
        self.move_right = false;
        // zero horizontal velocity
        self.sim.connections_mutex.lock();
        const idx_opt = self.sim.entity_by_player.get(self.player_id);
        if (idx_opt) |idx| {
            if (idx < self.sim.dynamic_entities.items.len) {
                var e = &self.sim.dynamic_entities.items[idx];
                e.vel[0] = 0;
                e.vel[2] = 0;
            }
        }
        self.sim.connections_mutex.unlock();
    }

    // Basic 4x4 column-major matrix helpers
    fn matMul(a: [16]f32, b: [16]f32) [16]f32 {
        var r: [16]f32 = undefined;
        var i: usize = 0;
        while (i < 4) : (i += 1) {
            var j: usize = 0;
            while (j < 4) : (j += 1) {
                // column-major: r[col*4+row]
                r[i * 4 + j] =
                    a[0 * 4 + j] * b[i * 4 + 0] +
                    a[1 * 4 + j] * b[i * 4 + 1] +
                    a[2 * 4 + j] * b[i * 4 + 2] +
                    a[3 * 4 + j] * b[i * 4 + 3];
            }
        }
        return r;
    }

    fn makePerspective(fovy_radians: f32, aspect: f32, znear: f32, zfar: f32) [16]f32 {
        const f: f32 = 1.0 / @tan(fovy_radians / 2.0);
        var m: [16]f32 = [_]f32{0} ** 16;
        m[0] = f / aspect;
        m[5] = f;
        m[10] = (zfar + znear) / (znear - zfar);
        m[11] = -1.0;
        m[14] = (2.0 * zfar * znear) / (znear - zfar);
        return m;
    }

    fn makeView(eye: [3]f32, forward: [3]f32, up_in: [3]f32) [16]f32 {
        const f0 = forward[0];
        const f1 = forward[1];
        const f2 = forward[2];
        // normalize f
        const flen: f32 = @sqrt(f0 * f0 + f1 * f1 + f2 * f2);
        const fx = f0 / flen;
        const fy = f1 / flen;
        const fz = f2 / flen;
        // s = normalize(cross(up, f)) to match input/movement handedness
        const sx0 = up_in[1] * fz - up_in[2] * fy;
        const sy0 = up_in[2] * fx - up_in[0] * fz;
        const sz0 = up_in[0] * fy - up_in[1] * fx;
        const slen: f32 = @sqrt(sx0 * sx0 + sy0 * sy0 + sz0 * sz0);
        const sx = sx0 / slen;
        const sy = sy0 / slen;
        const sz = sz0 / slen;
        // u = cross(f, s)
        const ux = fy * sz - fz * sy;
        const uy = fz * sx - fx * sz;
        const uz = fx * sy - fy * sx;
        var m: [16]f32 = undefined;
        m[0] = sx;
        m[4] = ux;
        m[8] = -fx;
        m[12] = 0;
        m[1] = sy;
        m[5] = uy;
        m[9] = -fy;
        m[13] = 0;
        m[2] = sz;
        m[6] = uz;
        m[10] = -fz;
        m[14] = 0;
        m[3] = 0;
        m[7] = 0;
        m[11] = 0;
        m[15] = 1;
        // translation
        var t: [16]f32 = [_]f32{0} ** 16;
        t[0] = 1;
        t[5] = 1;
        t[10] = 1;
        t[15] = 1;
        t[12] = -eye[0];
        t[13] = -eye[1];
        t[14] = -eye[2];
        return matMul(m, t);
    }
};
