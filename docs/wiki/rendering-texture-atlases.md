# Rendering: Texture Atlases and Sampling

This page documents the current texture pipeline used by the minimal chunk renderer.

Summary

- The renderer uses one or more 2D texture atlases (not a texture array).
- Block faces provide tile‑space UVs which may exceed 1.0 (for repeats on greedy quads).
- For each vertex we pass the atlas rect for that face: rect_min (u0,v0) and rect_size (du,dv).
- The fragment shader computes atlas-space UV as rect_min + fract(tile_uv) \* rect_size.
- Up to 4 atlases are bound simultaneously; the per‑vertex layer selects which atlas to sample.

Shader IO (chunk.glsl)

- Vertex inputs
  - pos: vec3
  - uv0: vec2 // tile-space UVs (may be > 1.0)
  - rect_min: vec2 // atlas rect min (u0,v0)
  - rect_size: vec2 // atlas rect size (du,dv)
  - layer_f: float // atlas index as float (rounded in FS)
  - apply_tint_f: float // 0.0 or 1.0; biome tint on primary faces
- Varyings
  - v_uv_tile: vec2
  - v_rect_min: vec2
  - v_rect_size: vec2
  - v_layer: float
  - v_apply_tint: float
  - v_chunk_uv: vec2 // region-local UV for biome tint texture
- Fragment sampling
  - uv_atlas = v_rect_min + fract(v_uv_tile) \* v_rect_size
  - Sample atlasN selected by floor(v_layer + 0.5)
  - Optional multiply by biome tint from chunk_tint_tex when v_apply_tint > 0.5

Bindings (fixed)

- Atlas textures:
  - binding=1: texture2D atlas0
  - binding=5: texture2D atlas1
  - binding=6: texture2D atlas2
  - binding=7: texture2D atlas3
- Samplers:
  - binding=2: sampler tex_sampler (used for atlases)
  - binding=4: sampler chunk_tint_smp (reuses same sampler in code)
- Chunk tint texture:
  - binding=3: texture2D chunk_tint_tex

Vertex layout (engine pipeline)

- Attribute indices map to:
  - 0: FLOAT3 pos
  - 1: FLOAT2 uv (tile-space)
  - 2: FLOAT2 rect_min
  - 3: FLOAT2 rect_size
  - 4: FLOAT layer
  - 5: FLOAT apply_tint

Greedy meshing and tiling

- Top faces are greedily merged per Y slice.
- For a greedy quad spanning w×h tiles, the mesh emits tile-space UV corners:
  - (0,0), (0,h), (w,h), (w,0)
- The shader’s fract() restores per‑tile repetition inside the atlas rect.

Orientation details

- UI/screen coordinates are top‑left origin (0,0) consistent with Sokol.
- Atlas UVs follow the conventional texture space with v increasing downward.
- For side/bottom faces we flip the tile‑space V so the top of the atlas rect maps to the top edge in world space.

Atlas construction

- Inputs are .txtr files in resources/ containing:
  - A null‑terminated identifier (e.g., "minecraft:dirt")
  - RGBA8 pixel data (square, power‑of‑two)
- Atlases are planned and composed at runtime using atlas.planDynamic + atlas.compose.
- Each atlas entry records its rect and derived UVs (u0,v0,u1,v1) and its atlas_index.
- The client builds a map from identifier -> { atlas_index, u0,v0,u1,v1 } and uses it during meshing.

Notes

- We currently bind at most 4 atlases; increase bindings and arrays if the content set grows.
- Atlases use padding=1 by default; if you ever see seams in repeats, increase padding or shrink rect_size by a half‑texel inset.
