// Placeholder shim to allow building without sokol-shdc.
// When shdc runs, it will overwrite this file with the generated shader.
const sokol = @import("sokol");
const sg = sokol.gfx;

pub fn shaderDesc(backend: sg.Backend) sg.ShaderDesc {
    _ = backend; // unused in fallback
    var s: sg.ShaderDesc = .{};
    s.label = "chunk-shader-fallback";
    s.vertex_func.source = "#include <metal_stdlib>\nusing namespace metal;\nstruct VSIn { float2 pos [[attribute(0)]]; float2 uv [[attribute(1)]]; };\nstruct VSOut { float4 pos [[position]]; float2 uv; };\nstruct VSParams { float2 aspect; };\nvertex VSOut vs_main(VSIn in [[stage_in]], constant VSParams& params [[buffer(0)]]) { VSOut o; float2 p = in.pos; p.x *= params.aspect.x; p.y *= params.aspect.y; o.pos = float4(p, 0.0, 1.0); o.uv = in.uv; return o; }\n";
    s.vertex_func.entry = "vs_main";
    s.fragment_func.source = "#include <metal_stdlib>\nusing namespace metal;\nstruct VSOut { float4 pos [[position]]; float2 uv; };\nfragment float4 fs_main(VSOut in [[stage_in]], texture2d<float, access::sample> tex0 [[texture(0)]], sampler smp [[sampler(0)]]) {\n    float3 c = tex0.sample(smp, in.uv).rgb;\n    return float4(c, 1.0);\n}\n";
    s.fragment_func.entry = "fs_main";
    s.uniform_blocks[0].stage = .VERTEX;
    s.uniform_blocks[0].size = 8; // float2
    s.uniform_blocks[0].msl_buffer_n = 0;
    s.views[0].texture.stage = .FRAGMENT;
    s.views[0].texture.image_type = ._2D;
    s.views[0].texture.sample_type = .FLOAT;
    s.views[0].texture.msl_texture_n = 0;
    s.samplers[0].stage = .FRAGMENT;
    s.samplers[0].sampler_type = .FILTERING;
    s.samplers[0].msl_sampler_n = 0;
    s.texture_sampler_pairs[0].stage = .FRAGMENT;
    s.texture_sampler_pairs[0].view_slot = 0;
    s.texture_sampler_pairs[0].sampler_slot = 0;
    return s;
}

