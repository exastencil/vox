const std = @import("std");
const sokol = @import("sokol");
const sg = sokol.gfx;
const simulation = @import("../simulation.zig");
const client_mesher_mod = @import("mesher.zig");
const cache_mod = @import("mesher_cache.zig");

const SectionDraw = client_mesher_mod.SectionDraw;
const RegionMesh = cache_mod.RegionMesh;
const REGION_MESH_BUDGET = cache_mod.REGION_MESH_BUDGET;

// Adopt one or more mesher results into the region mesh cache, respecting budget
pub fn adoptMesherResults(self: anytype) void {
    var processed: u32 = 0;
    while (processed < self.adopt_budget_per_frame) {
        // fetch one result
        self.mesher_mutex.lock();
        if (self.mesher_results.items.len == 0) {
            self.mesher_mutex.unlock();
            break;
        }
        const mr = self.mesher_results.items[self.mesher_results.items.len - 1];
        _ = self.mesher_results.pop();
        self.mesher_mutex.unlock();

        // Upload GPU buffer
        var buf: sg.Buffer = .{};
        if (mr.verts.len > 0) {
            buf = sg.makeBuffer(.{ .usage = .{ .vertex_buffer = true }, .data = sg.asRange(mr.verts) });
        }

        // Swap into cache
        if (self.region_mesh_cache.getPtr(mr.rpos)) |existing| {
            const old = existing.*;
            existing.* = .{ .vbuf = buf, .draws = mr.draws, .built_chunk_count = mr.built_chunk_count, .dirty = false, .inflight = false, .last_built_frame = self.frame_index };
            if (old.vbuf.id != 0) sg.destroyBuffer(old.vbuf);
            self.allocator.free(old.draws);
        } else {
            if (self.regionMeshCacheCount() < REGION_MESH_BUDGET) {
                const rm: RegionMesh = .{ .vbuf = buf, .draws = mr.draws, .built_chunk_count = mr.built_chunk_count, .dirty = false, .inflight = false, .last_built_frame = self.frame_index };
                _ = self.region_mesh_cache.put(mr.rpos, rm) catch {
                    if (rm.vbuf.id != 0) sg.destroyBuffer(rm.vbuf);
                    self.allocator.free(mr.draws);
                };
            } else {
                // over budget: drop result
                if (buf.id != 0) sg.destroyBuffer(buf);
                self.allocator.free(mr.draws);
            }
        }

        // Mark region tint dirty (biome tints may have changed in updated chunks)
        if (self.region_tint_cache.getPtr(mr.rpos)) |entry| entry.dirty = true;

        // free CPU verts
        if (mr.verts.len > 0) std.heap.page_allocator.free(mr.verts);
        processed += 1;
    }
    self.dbg_last_adopted = processed;
}

// Ensure a region mesh exists and schedule a rebuild if needed
pub fn ensureRegionMesh(self: anytype, rpos: simulation.RegionPos, expected_chunk_count: usize) void {
    // If nothing to build yet, avoid scheduling and clear any stale inflight
    if (expected_chunk_count == 0) {
        if (self.region_mesh_cache.getPtr(rpos)) |existing0| {
            existing0.built_chunk_count = 0;
            existing0.inflight = false;
        }
        return;
    }
    // If cache entry exists and is up-to-date, nothing to do
    if (self.region_mesh_cache.getPtr(rpos)) |existing| {
        // If marked dirty, schedule a rebuild immediately (ignoring cooldown), unless already inflight or out of budget
        if (existing.dirty and !existing.inflight and (self.rebuilds_issued_this_frame < self.rebuild_budget_per_frame)) {
            existing.inflight = true;
            // leave existing.dirty set; it will be cleared when adopting results
            self.mesher_mutex.lock();
            self.mesher_jobs.append(self.allocator, .{ .rpos = rpos }) catch {};
            self.mesher_cv.broadcast();
            self.mesher_mutex.unlock();
            self.rebuilds_issued_this_frame += 1;
            return;
        }
        if (existing.built_chunk_count == expected_chunk_count and existing.vbuf.id != 0) return;
        // If a rebuild is already in-flight but appears stuck, allow reschedule after cooldown
        if (existing.inflight) {
            if ((self.frame_index - existing.last_built_frame) < self.rebuild_cooldown_frames) return;
            // Reset and fall through to schedule a fresh job
            existing.inflight = false;
        }
        // Throttle job issuance
        if (self.rebuilds_issued_this_frame >= self.rebuild_budget_per_frame) return;
        if ((self.frame_index - existing.last_built_frame) < self.rebuild_cooldown_frames) return;
        // Mark inflight and enqueue job
        existing.inflight = true;
        self.mesher_mutex.lock();
        self.mesher_jobs.append(self.allocator, .{ .rpos = rpos }) catch {};
        self.mesher_cv.broadcast();
        self.mesher_mutex.unlock();
        self.rebuilds_issued_this_frame += 1;
        return;
    }
    // No cache entry: if under budget, insert placeholder and schedule build
    if (self.rebuilds_issued_this_frame >= self.rebuild_budget_per_frame) return;
    if (self.regionMeshCacheCount() >= REGION_MESH_BUDGET) return;
    const placeholder: RegionMesh = .{ .vbuf = .{}, .draws = &[_]SectionDraw{}, .built_chunk_count = 0, .dirty = false, .inflight = true, .last_built_frame = self.frame_index };
    _ = self.region_mesh_cache.put(rpos, placeholder) catch return;
    self.mesher_mutex.lock();
    self.mesher_jobs.append(self.allocator, .{ .rpos = rpos }) catch {};
    self.mesher_cv.broadcast();
    self.mesher_mutex.unlock();
    self.rebuilds_issued_this_frame += 1;
}

// Scan regions, schedule work under budget, and adopt results each frame
pub fn pumpMesherScheduleAndAdopt(self: anytype) void {
    // Snapshot regions and chunk counts, schedule jobs, and adopt results without drawing
    const wk = "minecraft:overworld";
    var regions = std.ArrayList(struct { rpos: simulation.RegionPos, chunk_count: usize }).initCapacity(self.allocator, 0) catch return;
    defer regions.deinit(self.allocator);
    self.sim.worlds_mutex.lock();
    const ws = self.sim.worlds_state.getPtr(wk);
    if (ws) |ws_ptr| {
        var reg_it = ws_ptr.regions.iterator();
        while (reg_it.next()) |rentry| {
            regions.append(self.allocator, .{ .rpos = rentry.key_ptr.*, .chunk_count = rentry.value_ptr.chunks.items.len }) catch {};
        }
    }
    self.sim.worlds_mutex.unlock();
    // reset issuance counter for this prepass
    self.rebuilds_issued_this_frame = 0;
    var i: usize = 0;
    while (i < regions.items.len) : (i += 1) ensureRegionMesh(self, regions.items[i].rpos, regions.items[i].chunk_count);
    self.dbg_last_scheduled = self.rebuilds_issued_this_frame;
    // adopt after scheduling
    adoptMesherResults(self);
}
