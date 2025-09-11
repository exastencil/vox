const std = @import("std");
const client_mesher_mod = @import("mesher.zig");

// Threading and scheduling glue for the background mesher
// These functions operate on any client-like type that provides the
// required fields (allocator, sim, mesher_* queues/mutex/cond, etc.).

const MeshingJob = client_mesher_mod.MeshingJob;
const MeshingResult = client_mesher_mod.MeshingResult;
const RegionSliceRefs = client_mesher_mod.RegionSliceRefs;

pub fn startMesherIfNeeded(self: anytype) void {
    if (!self.mesher_started) {
        self.mesher_running = true;
        const thread_count: usize = 4;
        self.mesher_threads.ensureTotalCapacity(self.allocator, thread_count) catch return;
        var ti: usize = 0;
        while (ti < thread_count) : (ti += 1) {
            const th = std.Thread.spawn(.{}, mesherThreadMain, .{self}) catch break;
            self.mesher_threads.appendAssumeCapacity(th);
        }
        self.mesher_started = true;
    }
}

pub fn stopMesherThreads(self: anytype) void {
    if (self.mesher_running) {
        self.mesher_mutex.lock();
        self.mesher_running = false;
        self.mesher_cv.broadcast();
        self.mesher_mutex.unlock();
        var i: usize = 0;
        while (i < self.mesher_threads.items.len) : (i += 1) self.mesher_threads.items[i].join();
        self.mesher_threads.deinit(self.allocator);
        // Free any pending results
        var r: usize = 0;
        while (r < self.mesher_results.items.len) : (r += 1) {
            const mr = self.mesher_results.items[r];
            std.heap.page_allocator.free(mr.verts);
            self.allocator.free(mr.draws);
        }
        self.mesher_results.deinit(self.allocator);
        self.mesher_jobs.deinit(self.allocator);
    }
}

pub fn mesherThreadMain(self: anytype) void {
    // mark started (debug)
    self.mesher_started = true;
    const alloc = std.heap.page_allocator;
    while (true) {
        // Wait for a job
        self.mesher_mutex.lock();
        while (self.mesher_running and self.mesher_jobs.items.len == 0) self.mesher_cv.wait(&self.mesher_mutex);
        if (!self.mesher_running) {
            self.mesher_mutex.unlock();
            return;
        }
        const job = self.mesher_jobs.items[self.mesher_jobs.items.len - 1];
        _ = self.mesher_jobs.pop();
        self.mesher_mutex.unlock();

        // Under world lock: gather slice refs only (no large allocations or copies)
        self.sim.worlds_mutex.lock();
        const ws_ptr = self.sim.worlds_state.getPtr("minecraft:overworld") orelse {
            self.sim.worlds_mutex.unlock();
            continue;
        };
        const rs_ptr = ws_ptr.regions.getPtr(job.rpos) orelse {
            self.sim.worlds_mutex.unlock();
            continue;
        };
        var refs: RegionSliceRefs = client_mesher_mod.buildRegionRefsUnderLock(self, alloc, ws_ptr, rs_ptr, job.rpos);
        self.sim.worlds_mutex.unlock();

        if (refs.chunks.len == 0) {
            client_mesher_mod.freeRegionRefs(self, alloc, &refs);
            continue;
        }

        // Outside the world lock: copy data into an owned snapshot and free refs
        var snap = client_mesher_mod.copySnapshotFromRefs(self, alloc, &refs);
        client_mesher_mod.freeRegionRefs(self, alloc, &refs);

        if (snap.chunks.len == 0) {
            client_mesher_mod.freeRegionSnapshot(self, alloc, &snap);
            continue;
        }

        // Mesh from snapshot (no locks)
        const result = client_mesher_mod.meshRegionFromSnapshot(self, alloc, &snap);
        client_mesher_mod.freeRegionSnapshot(self, alloc, &snap);

        // Publish result
        self.mesher_mutex.lock();
        self.mesher_results.append(self.allocator, result) catch {
            if (result.verts.len > 0) alloc.free(result.verts);
            if (result.draws.len > 0) self.allocator.free(result.draws);
        };
        self.mesher_mutex.unlock();
    }
}
