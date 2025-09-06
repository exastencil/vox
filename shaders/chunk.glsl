// sokol-shdc cross-platform shader source for the minimal chunk renderer

@ctype mat4 [16]f32

@vs vs
in vec3 pos;
in vec2 uv0;

layout(binding=0) uniform vs_params {
    mat4 mvp;
};

out vec2 uv;

void main() {
    gl_Position = mvp * vec4(pos, 1.0);
    uv = uv0;
}
@end

@fs fs
in vec2 uv;

layout(binding=1) uniform texture2D tex_texture;
layout(binding=2) uniform sampler tex_sampler;

out vec4 frag_color;

void main() {
    vec3 c = texture(sampler2D(tex_texture, tex_sampler), uv).rgb;
    frag_color = vec4(c, 1.0);
}
@end

@program chunk vs fs

