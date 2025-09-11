// Registry and content ID type aliases used in Game State
pub const BlockId = u32;
pub const BiomeId = u16;
pub const BlockEntityTypeId = u16;
pub const EntityTypeId = u16;
pub const SkyboxId = u16;

// Shared directional enum for block-facing and orientation. This lives in ids so it can
// be referenced from multiple modules (e.g., gs and root) without causing module overlap.
pub const Direction = enum {
    front,
    back,
    left,
    right,
    up,
    down,
};
