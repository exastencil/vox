// sokol-shdc cross-platform shader source for the minimal chunk renderer

@ctype mat4 [16]f32

@vs vs
in vec3 pos;
// tile-local UVs (may be >1.0 to indicate repeats)
in vec2 uv0;
// atlas transform for this face (constant across the face)
in vec2 uv_off;
in vec2 uv_scale;

layout(binding=0) uniform vs_params {
    mat4 mvp;
    vec2 atlas_pad; // per-texel padding in atlas space
};

out vec2 v_uv_tile;
out vec2 v_uv_off;
out vec2 v_uv_scale;
out vec2 v_atlas_pad;

void main() {
    gl_Position = mvp * vec4(pos, 1.0);
    v_uv_tile = uv0;
    v_uv_off = uv_off;
    v_uv_scale = uv_scale;
    v_atlas_pad = atlas_pad;
}
@end

@fs fs
in vec2 v_uv_tile;
in vec2 v_uv_off;
in vec2 v_uv_scale;
in vec2 v_atlas_pad;

layout(binding=1) uniform texture2D tex_texture;
layout(binding=2) uniform sampler tex_sampler;

out vec4 frag_color;

void main() {
    // Repeat the tile using fract in tile-local space, then transform into atlas space with padding
    vec2 min_uv = v_uv_off + v_atlas_pad;
    vec2 range_uv = max(vec2(0.0), v_uv_scale - 2.0 * v_atlas_pad);
    vec2 tiled = fract(v_uv_tile);
    vec2 at_uv = min_uv + tiled * range_uv;
    vec3 c = texture(sampler2D(tex_texture, tex_sampler), at_uv).rgb;
    frag_color = vec4(c, 1.0);
}
@end

@program chunk vs fs

