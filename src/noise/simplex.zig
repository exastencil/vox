const std = @import("std");

pub const SimplexNoise = struct {
    seed: u64,
    perm: [512]u8,
    permMod12: [512]u8,

    pub fn init(seed: u64) SimplexNoise {
        var perm256: [256]u8 = undefined;
        var i: usize = 0;
        while (i < 256) : (i += 1) perm256[i] = @intCast(i);
        var rng = SplitMix64.init(seed);
        // Fisher-Yates shuffle for a deterministic permutation table
        var j: usize = 255;
        while (j > 0) : (j -= 1) {
            const r = rng.next();
            const k = @as(usize, @intCast(r % (j + 1)));
            const tmp = perm256[j];
            perm256[j] = perm256[k];
            perm256[k] = tmp;
        }
        var p512: [512]u8 = undefined;
        var pm12: [512]u8 = undefined;
        i = 0;
        while (i < 512) : (i += 1) {
            const v: u8 = perm256[i & 255];
            p512[i] = v;
            pm12[i] = v % 12;
        }
        return .{ .seed = seed, .perm = p512, .permMod12 = pm12 };
    }

    // Skewing/Unskewing factors for 2D and 3D simplex grids
    const F2: f64 = 0.36602540378443864676; // 0.5*(sqrt(3) - 1)
    const G2: f64 = 0.21132486540518711775; // (3 - sqrt(3))/6
    const F3: f64 = 0.33333333333333333333; // 1/3
    const G3: f64 = 0.16666666666666666667; // 1/6

    fn fastfloor(x: f64) i64 {
        const xi: i64 = @intFromFloat(x);
        return if (@as(f64, @floatFromInt(xi)) > x) xi - 1 else xi;
    }

    fn dot2(gx: f64, gy: f64, x: f64, y: f64) f64 {
        return gx * x + gy * y;
    }

    fn dot3(g: [3]i32, x: f64, y: f64, z: f64) f64 {
        return @as(f64, @floatFromInt(g[0])) * x + @as(f64, @floatFromInt(g[1])) * y + @as(f64, @floatFromInt(g[2])) * z;
    }

    // 12 gradient directions for 3D simplex
    const grad3: [12][3]i32 = [_][3]i32{
        .{ 1, 1, 0 }, .{ -1, 1, 0 }, .{ 1, -1, 0 }, .{ -1, -1, 0 },
        .{ 1, 0, 1 }, .{ -1, 0, 1 }, .{ 1, 0, -1 }, .{ -1, 0, -1 },
        .{ 0, 1, 1 }, .{ 0, -1, 1 }, .{ 0, 1, -1 }, .{ 0, -1, -1 },
    };

    /// 2D Simplex noise, returns approximately in [-1, 1]
    pub fn noise2(self: *const SimplexNoise, x_in: f64, y_in: f64) f64 {
        const x = x_in;
        const y = y_in;
        const s = (x + y) * F2;
        const i = fastfloor(x + s);
        const j = fastfloor(y + s);
        const t = @as(f64, @floatFromInt(i + j)) * G2;
        const X0 = @as(f64, @floatFromInt(i)) - t;
        const Y0 = @as(f64, @floatFromInt(j)) - t;
        const x0 = x - X0;
        const y0 = y - Y0;

        var i1_val: i32 = 0;
        var j1_val: i32 = 0;
        if (x0 > y0) {
            i1_val = 1;
            j1_val = 0;
        } else {
            i1_val = 0;
            j1_val = 1;
        }

        const x1 = x0 - @as(f64, @floatFromInt(i1_val)) + G2;
        const y1 = y0 - @as(f64, @floatFromInt(j1_val)) + G2;
        const x2 = x0 - 1.0 + 2.0 * G2;
        const y2 = y0 - 1.0 + 2.0 * G2;

        const ii = @as(u32, @bitCast(@as(i32, @intCast(i)) & 255));
        const jj = @as(u32, @bitCast(@as(i32, @intCast(j)) & 255));

        var n0: f64 = 0.0;
        var n1: f64 = 0.0;
        var n2: f64 = 0.0;

        // Corner 0
        var t0 = 0.5 - x0 * x0 - y0 * y0;
        if (t0 > 0.0) {
            const gi0h = self.perm[@intCast(ii + self.perm[@intCast(jj)])] & 7;
            const gx0: f64 = switch (gi0h) {
                0, 1 => 1.0,
                2, 3 => -1.0,
                4, 5 => 1.0,
                6, 7 => 0.0,
                else => 0.0,
            };
            const gy0: f64 = switch (gi0h) {
                0, 1, 2, 3 => 1.0,
                4, 5 => 0.0,
                6 => 1.0,
                7 => -1.0,
                else => 0.0,
            };
            t0 *= t0;
            n0 = t0 * t0 * dot2(gx0, gy0, x0, y0);
        }
        // Corner 1
        var t1 = 0.5 - x1 * x1 - y1 * y1;
        if (t1 > 0.0) {
            const gi1h = self.perm[@intCast(ii + @as(u32, @intCast(i1_val)) + self.perm[@intCast(jj + @as(u32, @intCast(j1_val)))])] & 7;
            const gx1: f64 = switch (gi1h) {
                0, 1 => 1.0,
                2, 3 => -1.0,
                4, 5 => 1.0,
                6, 7 => 0.0,
                else => 0.0,
            };
            const gy1: f64 = switch (gi1h) {
                0, 1, 2, 3 => 1.0,
                4, 5 => 0.0,
                6 => 1.0,
                7 => -1.0,
                else => 0.0,
            };
            t1 *= t1;
            n1 = t1 * t1 * dot2(gx1, gy1, x1, y1);
        }
        // Corner 2
        var t2 = 0.5 - x2 * x2 - y2 * y2;
        if (t2 > 0.0) {
            const gi2h = self.perm[@intCast(ii + 1 + self.perm[@intCast(jj + 1)])] & 7;
            const gx2: f64 = switch (gi2h) {
                0, 1 => 1.0,
                2, 3 => -1.0,
                4, 5 => 1.0,
                6, 7 => 0.0,
                else => 0.0,
            };
            const gy2: f64 = switch (gi2h) {
                0, 1, 2, 3 => 1.0,
                4, 5 => 0.0,
                6 => 1.0,
                7 => -1.0,
                else => 0.0,
            };
            t2 *= t2;
            n2 = t2 * t2 * dot2(gx2, gy2, x2, y2);
        }

        // Scale factor brings result to ~[-1,1]
        return 70.0 * (n0 + n1 + n2);
    }

    /// 3D Simplex noise, returns approximately in [-1, 1]
    pub fn noise3(self: *const SimplexNoise, x_in: f64, y_in: f64, z_in: f64) f64 {
        const x = x_in;
        const y = y_in;
        const z = z_in;

        const s = (x + y + z) * F3;
        const i = fastfloor(x + s);
        const j = fastfloor(y + s);
        const k = fastfloor(z + s);
        const t = @as(f64, @floatFromInt(i + j + k)) * G3;
        const X0 = @as(f64, @floatFromInt(i)) - t;
        const Y0 = @as(f64, @floatFromInt(j)) - t;
        const Z0 = @as(f64, @floatFromInt(k)) - t;
        const x0 = x - X0;
        const y0 = y - Y0;
        const z0 = z - Z0;

        var i1_val: i32 = 0;
        var j1_val: i32 = 0;
        var k1_val: i32 = 0;
        var i2_val: i32 = 0;
        var j2_val: i32 = 0;
        var k2_val: i32 = 0;

        if (x0 >= y0) {
            if (y0 >= z0) {
                i1_val = 1;
                j1_val = 0;
                k1_val = 0;
                i2_val = 1;
                j2_val = 1;
                k2_val = 0;
            } else if (x0 >= z0) {
                i1_val = 1;
                j1_val = 0;
                k1_val = 0;
                i2_val = 1;
                j2_val = 0;
                k2_val = 1;
            } else {
                i1_val = 0;
                j1_val = 0;
                k1_val = 1;
                i2_val = 1;
                j2_val = 0;
                k2_val = 1;
            }
        } else {
            if (y0 < z0) {
                i1_val = 0;
                j1_val = 0;
                k1_val = 1;
                i2_val = 0;
                j2_val = 1;
                k2_val = 1;
            } else if (x0 < z0) {
                i1_val = 0;
                j1_val = 1;
                k1_val = 0;
                i2_val = 0;
                j2_val = 1;
                k2_val = 1;
            } else {
                i1_val = 0;
                j1_val = 1;
                k1_val = 0;
                i2_val = 1;
                j2_val = 1;
                k2_val = 0;
            }
        }

        const x1 = x0 - @as(f64, @floatFromInt(i1_val)) + G3;
        const y1 = y0 - @as(f64, @floatFromInt(j1_val)) + G3;
        const z1 = z0 - @as(f64, @floatFromInt(k1_val)) + G3;
        const x2 = x0 - @as(f64, @floatFromInt(i2_val)) + 2.0 * G3;
        const y2 = y0 - @as(f64, @floatFromInt(j2_val)) + 2.0 * G3;
        const z2 = z0 - @as(f64, @floatFromInt(k2_val)) + 2.0 * G3;
        const x3 = x0 - 1.0 + 3.0 * G3;
        const y3 = y0 - 1.0 + 3.0 * G3;
        const z3 = z0 - 1.0 + 3.0 * G3;

        const ii = @as(u32, @bitCast(@as(i32, @intCast(i)) & 255));
        const jj = @as(u32, @bitCast(@as(i32, @intCast(j)) & 255));
        const kk = @as(u32, @bitCast(@as(i32, @intCast(k)) & 255));

        var n0: f64 = 0.0;
        var n1: f64 = 0.0;
        var n2: f64 = 0.0;
        var n3: f64 = 0.0;

        var t0 = 0.6 - x0 * x0 - y0 * y0 - z0 * z0;
        if (t0 > 0.0) {
            t0 *= t0;
            const gi0 = self.permMod12[@intCast(ii + self.perm[@intCast(jj + self.perm[@intCast(kk)])])];
            n0 = t0 * t0 * dot3(grad3[gi0], x0, y0, z0);
        }

        var t1 = 0.6 - x1 * x1 - y1 * y1 - z1 * z1;
        if (t1 > 0.0) {
            t1 *= t1;
            const gi1 = self.permMod12[@intCast(ii + @as(u32, @intCast(i1_val)) + self.perm[@intCast(jj + @as(u32, @intCast(j1_val)) + self.perm[@intCast(kk + @as(u32, @intCast(k1_val)))])])];
            n1 = t1 * t1 * dot3(grad3[gi1], x1, y1, z1);
        }

        var t2 = 0.6 - x2 * x2 - y2 * y2 - z2 * z2;
        if (t2 > 0.0) {
            t2 *= t2;
            const gi2 = self.permMod12[@intCast(ii + @as(u32, @intCast(i2_val)) + self.perm[@intCast(jj + @as(u32, @intCast(j2_val)) + self.perm[@intCast(kk + @as(u32, @intCast(k2_val)))])])];
            n2 = t2 * t2 * dot3(grad3[gi2], x2, y2, z2);
        }

        var t3 = 0.6 - x3 * x3 - y3 * y3 - z3 * z3;
        if (t3 > 0.0) {
            t3 *= t3;
            const gi3 = self.permMod12[@intCast(ii + 1 + self.perm[@intCast(jj + 1 + self.perm[@intCast(kk + 1)])])];
            n3 = t3 * t3 * dot3(grad3[gi3], x3, y3, z3);
        }

        return 32.0 * (n0 + n1 + n2 + n3);
    }

    /// Fractal Brownian motion sum of 2D simplex noise
    pub fn fbm2(self: *const SimplexNoise, x: f64, y: f64, octaves: u32, lacunarity: f64, gain: f64) f64 {
        var sum: f64 = 0.0;
        var amp: f64 = 1.0;
        var freq: f64 = 1.0;
        var maxamp: f64 = 0.0;
        var o: u32 = 0;
        while (o < octaves) : (o += 1) {
            sum += amp * self.noise2(x * freq, y * freq);
            maxamp += amp;
            amp *= gain;
            freq *= lacunarity;
        }
        return sum / (if (maxamp == 0.0) 1.0 else maxamp);
    }

    /// Fractal Brownian motion sum of 3D simplex noise
    pub fn fbm3(self: *const SimplexNoise, x: f64, y: f64, z: f64, octaves: u32, lacunarity: f64, gain: f64) f64 {
        var sum: f64 = 0.0;
        var amp: f64 = 1.0;
        var freq: f64 = 1.0;
        var maxamp: f64 = 0.0;
        var o: u32 = 0;
        while (o < octaves) : (o += 1) {
            sum += amp * self.noise3(x * freq, y * freq, z * freq);
            maxamp += amp;
            amp *= gain;
            freq *= lacunarity;
        }
        return sum / (if (maxamp == 0.0) 1.0 else maxamp);
    }

    // Convenience wrappers for f32
    pub fn noise2f(self: *const SimplexNoise, x: f32, y: f32) f32 {
        return @floatCast(self.noise2(@floatCast(x), @floatCast(y)));
    }
    pub fn noise3f(self: *const SimplexNoise, x: f32, y: f32, z: f32) f32 {
        return @floatCast(self.noise3(@floatCast(x), @floatCast(y), @floatCast(z)));
    }

    // SplitMix64 PRNG used to shuffle the permutation deterministically from a 64-bit seed
    const SplitMix64 = struct {
        x: u64,
        fn init(seed: u64) SplitMix64 {
            return .{ .x = seed };
        }
        fn next(self: *SplitMix64) u64 {
            self.x +%= 0x9E3779B97F4A7C15;
            var z = self.x;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EB;
            return z ^ (z >> 31);
        }
    };
};

const testing = std.testing;

test "simplex: deterministic same-seed" {
    const n1 = SimplexNoise.init(123456789);
    const n2 = SimplexNoise.init(123456789);
    try testing.expectEqual(@as(f64, 0.0), @abs(n1.noise2(0.1, -0.2) - n2.noise2(0.1, -0.2)));
    try testing.expectEqual(@as(f64, 0.0), @abs(n1.noise3(1.25, 2.5, -3.75) - n2.noise3(1.25, 2.5, -3.75)));
}

test "simplex: value range reasonable [-1,1]" {
    const n = SimplexNoise.init(42);
    const a = n.noise2(0.0, 0.0);
    const b = n.noise3(0.0, 0.0, 0.0);
    try testing.expect(a >= -1.2 and a <= 1.2);
    try testing.expect(b >= -1.2 and b <= 1.2);
}
