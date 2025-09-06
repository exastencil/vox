// sokol-shdc cross-platform shader source for the minimal chunk renderer

@ctype mat4 float4x4

@vs vs
in vec2 pos;
in vec2 uv0;

uniform vs_params {
    vec2 aspect;
};

out vec2 uv;

void main() {
    vec2 p = pos;
    p.x *= aspect.x;
    p.y *= aspect.y;
    gl_Position = vec4(p, 0.0, 1.0);
    uv = uv0;
}
@end

@fs fs
in vec2 uv;

uniform sampler2D tex;

out vec4 frag_color;

void main() {
    vec3 c = texture(tex, uv).rgb;
    frag_color = vec4(c, 1.0);
}
@end

@program chunk vs fs

