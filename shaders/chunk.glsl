// sokol-shdc cross-platform shader source for the minimal chunk renderer

@ctype mat4 [16]f32

@vs vs
in vec3 pos;
// tile-local UVs (may be >1.0 to indicate repeats)
in vec2 uv0;
// per-vertex texture array layer index (as float; integer layer = floor(layer_f))
in float layer_f;

layout(binding=0) uniform vs_params {
    mat4 mvp;
};

out vec2 v_uv_tile;
out float v_layer;

void main() {
    gl_Position = mvp * vec4(pos, 1.0);
    v_uv_tile = uv0;
    v_layer = layer_f;
}
@end

@fs fs
in vec2 v_uv_tile;
in float v_layer;

layout(binding=1) uniform texture2DArray tex_array;
layout(binding=2) uniform sampler tex_sampler;

out vec4 frag_color;

void main() {
    // Repeat the tile using fract in tile-local space and sample the selected array layer
    vec2 tiled = fract(v_uv_tile);
    float layer = floor(v_layer + 0.5);
    vec3 c = texture(sampler2DArray(tex_array, tex_sampler), vec3(tiled, layer)).rgb;
    frag_color = vec4(c, 1.0);
}
@end

@program chunk vs fs

