// Client camera and view/projection math
const std = @import("std");

pub const Camera = struct {
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

    pub fn setYawPitch(self: *Camera, yaw: f32, pitch: f32) void {
        self.yaw = yaw;
        self.pitch = pitch;
        self.dir = computeDir(yaw, pitch);
    }

    pub fn setDir(self: *Camera, dir_in: [3]f32) void {
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

// Basic 4x4 column-major matrix helpers
pub fn matMul(a: [16]f32, b: [16]f32) [16]f32 {
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

pub fn transpose4(m: [16]f32) [16]f32 {
    var r: [16]f32 = undefined;
    var c: usize = 0;
    while (c < 4) : (c += 1) {
        var rrow: usize = 0;
        while (rrow < 4) : (rrow += 1) {
            r[c * 4 + rrow] = m[rrow * 4 + c];
        }
    }
    return r;
}

// Extract 6 view-frustum planes from a column-major MVP matrix.
// Plane order: left, right, bottom, top, near, far. Each plane is (a,b,c,d) with outward normal.
pub fn extractFrustumPlanes(m: [16]f32) [6][4]f32 {
    // rows in column-major storage
    const r0 = [4]f32{ m[0], m[4], m[8], m[12] };
    const r1 = [4]f32{ m[1], m[5], m[9], m[13] };
    const r2 = [4]f32{ m[2], m[6], m[10], m[14] };
    const r3 = [4]f32{ m[3], m[7], m[11], m[15] };
    var planes: [6][4]f32 = undefined;
    // left = r3 + r0
    planes[0] = .{ r3[0] + r0[0], r3[1] + r0[1], r3[2] + r0[2], r3[3] + r0[3] };
    // right = r3 - r0
    planes[1] = .{ r3[0] - r0[0], r3[1] - r0[1], r3[2] - r0[2], r3[3] - r0[3] };
    // bottom = r3 + r1
    planes[2] = .{ r3[0] + r1[0], r3[1] + r1[1], r3[2] + r1[2], r3[3] + r1[3] };
    // top = r3 - r1
    planes[3] = .{ r3[0] - r1[0], r3[1] - r1[1], r3[2] - r1[2], r3[3] - r1[3] };
    // near = r3 + r2
    planes[4] = .{ r3[0] + r2[0], r3[1] + r2[1], r3[2] + r2[2], r3[3] + r2[3] };
    // far = r3 - r2
    planes[5] = .{ r3[0] - r2[0], r3[1] - r2[1], r3[2] - r2[2], r3[3] - r2[3] };
    // normalize planes
    var pi: usize = 0;
    while (pi < 6) : (pi += 1) {
        const a = planes[pi][0];
        const b = planes[pi][1];
        const c = planes[pi][2];
        const inv_len = 1.0 / @max(0.000001, @sqrt(a * a + b * b + c * c));
        planes[pi][0] = a * inv_len;
        planes[pi][1] = b * inv_len;
        planes[pi][2] = c * inv_len;
        planes[pi][3] = planes[pi][3] * inv_len;
    }
    return planes;
}

pub fn aabbInFrustum(planes: [6][4]f32, min: [3]f32, max: [3]f32, margin: f32) bool {
    var p: [3]f32 = undefined;
    var i: usize = 0;
    while (i < 6) : (i += 1) {
        const a = planes[i][0];
        const b = planes[i][1];
        const c = planes[i][2];
        const d = planes[i][3];
        p[0] = if (a >= 0) max[0] else min[0];
        p[1] = if (b >= 0) max[1] else min[1];
        p[2] = if (c >= 0) max[2] else min[2];
        const dist = a * p[0] + b * p[1] + c * p[2] + d;
        if (dist < -margin) return false;
    }
    return true;
}

pub fn aabbContainsPoint(min: [3]f32, max: [3]f32, p: [3]f32) bool {
    return p[0] >= min[0] and p[0] <= max[0] and p[1] >= min[1] and p[1] <= max[1] and p[2] >= min[2] and p[2] <= max[2];
}

pub fn makePerspective(fovy_radians: f32, aspect: f32, znear: f32, zfar: f32) [16]f32 {
    const f = 1.0 / @tan(fovy_radians / 2.0);
    var m: [16]f32 = [_]f32{0} ** 16;
    m[0] = f / (if (aspect != 0) aspect else 1.0);
    m[5] = f;
    m[10] = (zfar + znear) / (znear - zfar);
    m[11] = -1.0;
    m[14] = (2.0 * zfar * znear) / (znear - zfar);
    return m;
}

pub fn makeOrthoCentered(half_width: f32, half_height: f32, znear: f32, zfar: f32) [16]f32 {
    var m: [16]f32 = [_]f32{0} ** 16;
    // column-major
    m[0] = if (half_width != 0) (1.0 / half_width) else 0.0;
    m[5] = if (half_height != 0) (1.0 / half_height) else 0.0;
    m[10] = if ((zfar - znear) != 0) (-2.0 / (zfar - znear)) else 0.0;
    m[15] = 1.0;
    // translate is zero for symmetric bounds
    m[12] = 0.0;
    m[13] = 0.0;
    m[14] = -(zfar + znear) / (zfar - znear);
    return m;
}

pub fn makeViewFromBasis(eye: [3]f32, s: [3]f32, u: [3]f32, f: [3]f32) [16]f32 {
    var m: [16]f32 = undefined;
    m[0] = s[0];
    m[4] = u[0];
    // OpenGL-style view with -f in the third column (camera looks down -Z in view space)
    m[8] = -f[0];
    m[12] = 0;
    m[1] = s[1];
    m[5] = u[1];
    m[9] = -f[1];
    m[13] = 0;
    m[2] = s[2];
    m[6] = u[2];
    m[10] = -f[2];
    m[14] = 0;
    m[3] = 0;
    m[7] = 0;
    m[11] = 0;
    m[15] = 1;
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

pub fn makeViewConv(eye: [3]f32, forward: [3]f32, up_in: [3]f32, conventional: bool) [16]f32 {
    // normalize forward
    const f0 = forward[0];
    const f1 = forward[1];
    const f2 = forward[2];
    const flen: f32 = @sqrt(f0 * f0 + f1 * f1 + f2 * f2);
    const fx = if (flen > 0.000001) (f0 / flen) else 1.0;
    const fy = if (flen > 0.000001) (f1 / flen) else 0.0;
    const fz = if (flen > 0.000001) (f2 / flen) else 0.0;
    // s = normalize(cross(f, up)) (conventional) or cross(up, f)
    var sx0: f32 = undefined;
    var sy0: f32 = undefined;
    var sz0: f32 = undefined;
    if (conventional) {
        // cross(f, up)
        sx0 = fy * up_in[2] - fz * up_in[1];
        sy0 = fz * up_in[0] - fx * up_in[2];
        sz0 = fx * up_in[1] - fy * up_in[0];
    } else {
        // cross(up, f)
        sx0 = up_in[1] * fz - up_in[2] * fy;
        sy0 = up_in[2] * fx - up_in[0] * fz;
        sz0 = up_in[0] * fy - up_in[1] * fx;
    }
    const slen: f32 = @sqrt(sx0 * sx0 + sy0 * sy0 + sz0 * sz0);
    const sx = sx0 / slen;
    const sy = sy0 / slen;
    const sz = sz0 / slen;
    // u = cross(s, f)
    const ux = sy * fz - sz * fy;
    const uy = sz * fx - sx * fz;
    const uz = sx * fy - sy * fx;
    return makeViewFromBasis(eye, .{ sx, sy, sz }, .{ ux, uy, uz }, .{ fx, fy, fz });
}

pub fn transposeView3x3WithEye(v: [16]f32, eye: [3]f32) [16]f32 {
    var m = v;
    // transpose rotation 3x3 in upper-left
    const m00 = m[0];
    const m01 = m[4];
    const m02 = m[8];
    const m10 = m[1];
    const m11 = m[5];
    const m12 = m[9];
    const m20 = m[2];
    const m21 = m[6];
    const m22 = m[10];
    m[0] = m00;
    m[4] = m10;
    m[8] = m20;
    m[1] = m01;
    m[5] = m11;
    m[9] = m21;
    m[2] = m02;
    m[6] = m12;
    m[10] = m22;
    // recompute translation column using new rotation and current camera eye
    const neg_eye0 = -eye[0];
    const neg_eye1 = -eye[1];
    const neg_eye2 = -eye[2];
    m[12] = m[0] * neg_eye0 + m[4] * neg_eye1 + m[8] * neg_eye2;
    m[13] = m[1] * neg_eye0 + m[5] * neg_eye1 + m[9] * neg_eye2;
    m[14] = m[2] * neg_eye0 + m[6] * neg_eye1 + m[10] * neg_eye2;
    return m;
}
