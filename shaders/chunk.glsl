// sokol-shdc cross-platform shader source for the minimal chunk renderer

@ctype mat4 [16]f32

@vs vs
in vec3 pos;
// tile-local UVs (may be >1.0 to indicate repeats)
in vec2 uv0;
// per-vertex texture array layer index (as float; integer layer = floor(layer_f))
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
out float v_layer;
out float v_apply_tint;
out vec2 v_chunk_uv;

void main() {
    gl_Position = mvp * vec4(pos, 1.0);
    v_uv_tile = uv0;
    v_layer = layer_f;
    v_apply_tint = apply_tint_f;
    // Compute region-local UVs in VS for simplicity
    vec2 pxz = vec2(pos.x, pos.z);
    vec2 world_xz = floor(pxz);
    v_chunk_uv = (world_xz - region_info.xy) * region_info.z;
}
@end

@fs fs
in vec2 v_uv_tile;
in float v_layer;
in float v_apply_tint;
in vec2 v_chunk_uv;

layout(binding=1) uniform texture2DArray tex_array;
layout(binding=2) uniform sampler tex_sampler;
// Per-region tint texture (RGBA8)
layout(binding=3) uniform texture2D chunk_tint_tex;
layout(binding=4) uniform sampler chunk_tint_smp;

out vec4 frag_color;

void main() {
    // Repeat the tile using fract in tile-local space and sample the selected array layer
    vec2 tiled = fract(v_uv_tile);
    float layer = floor(v_layer + 0.5);
    vec3 c = texture(sampler2DArray(tex_array, tex_sampler), vec3(tiled, layer)).rgb;
    if (v_apply_tint > 0.5) {
        vec3 tint = texture(sampler2D(chunk_tint_tex, chunk_tint_smp), v_chunk_uv).rgb;
        c *= tint;
    }
    frag_color = vec4(c, 1.0);
}
@end

@program chunk vs fs

