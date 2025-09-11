// sokol-shdc cross-platform shader source for the minimal chunk renderer

@ctype mat4 [16]f32

@vs vs
in vec3 pos;
// tile-space UVs (may be >1.0 to indicate repeats)
in vec2 uv0;
// atlas rect (min and size) used to convert tile-space to atlas-space
in vec2 rect_min;
in vec2 rect_size;
// per-vertex atlas index (as float; integer layer = floor(layer_f))
in float layer_f;
// whether this face should apply biome tint (0.0 or 1.0)
in float apply_tint_f;

layout(binding=0) uniform vs_params {
    mat4 mvp;
    // region_info.xy = region origin in world blocks (xz)
    // region_info.z = inverse of region dimension in blocks (1.0/512.0)
    // region_info.w = unused
    vec4 region_info;
};

out vec2 v_uv_tile;
out vec2 v_rect_min;
out vec2 v_rect_size;
out float v_layer;
out float v_apply_tint;
out vec2 v_chunk_uv;

void main() {
    gl_Position = mvp * vec4(pos, 1.0);
    v_uv_tile = uv0;
    v_rect_min = rect_min;
    v_rect_size = rect_size;
    v_layer = layer_f;
    v_apply_tint = apply_tint_f;
    // Compute region-local UVs in VS for simplicity
    vec2 pxz = vec2(pos.x, pos.z);
    vec2 world_xz = floor(pxz);
    v_chunk_uv = (world_xz - region_info.xy) * region_info.z;
}
@end

@fs fs
in vec2 v_uv_tile;        // tile-space UVs (may be >1.0)
in vec2 v_rect_min;        // atlas rect min
in vec2 v_rect_size;       // atlas rect size
in float v_layer;          // atlas index (0..N-1)
in float v_apply_tint;
in vec2 v_chunk_uv;

// Up to 4 atlases bound at fixed slots; expand if needed
layout(binding=1) uniform texture2D atlas0;
layout(binding=5) uniform texture2D atlas1;
layout(binding=6) uniform texture2D atlas2;
layout(binding=7) uniform texture2D atlas3;
layout(binding=2) uniform sampler tex_sampler;
// Per-region tint texture (RGBA8)
layout(binding=3) uniform texture2D chunk_tint_tex;
layout(binding=4) uniform sampler chunk_tint_smp;

out vec4 frag_color;

vec3 sample_atlas(float idx, vec2 uv) {
    // Choose atlas by index; keep small and predictable branching
    if (idx < 0.5) {
        return texture(sampler2D(atlas0, tex_sampler), uv).rgb;
    } else if (idx < 1.5) {
        return texture(sampler2D(atlas1, tex_sampler), uv).rgb;
    } else if (idx < 2.5) {
        return texture(sampler2D(atlas2, tex_sampler), uv).rgb;
    } else {
        return texture(sampler2D(atlas3, tex_sampler), uv).rgb;
    }
}

void main() {
    // Convert tile-space to atlas-space, but bias away from tile boundaries to avoid sampling
    // the padded atlas border twice at shared edges (prevents visible "double lines").
    // We clamp the tile-space UV to [eps, 1-eps] where eps ~= half a texel for 32px tiles.
    // This is a small, data-independent bias that works well for 16–32px textures typical
    // of our resources and keeps seams clean even after face flips/rotations.
    const float EPS_T = 1.0 / 32.0; // ~0.5 texel for 32px tiles, 0.25 texel for 16px tiles
    vec2 uv_tile = fract(v_uv_tile);
    uv_tile = clamp(uv_tile, vec2(EPS_T), vec2(1.0 - EPS_T));
    vec2 uv_atlas = v_rect_min + uv_tile * v_rect_size;
    vec3 c = sample_atlas(floor(v_layer + 0.5), uv_atlas);
    if (v_apply_tint > 0.5) {
        vec3 tint = texture(sampler2D(chunk_tint_tex, chunk_tint_smp), v_chunk_uv).rgb;
        c *= tint;
    }
    frag_color = vec4(c, 1.0);
}
@end

@program chunk vs fs

