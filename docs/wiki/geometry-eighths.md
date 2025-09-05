# Eighth-Subdivision Geometry (Octant Bitmask)

Vox Aetatum represents partial-block geometry using an 8-bit occupancy mask over the block’s eight octants. This enables consistent handling of slabs, stairs-like shapes, vertical slabs, and other composite geometries.

Core definition:
- sub_mask: u8 — each bit represents one octant (eighth) of a block.
- Bit index mapping (xyz → bit):
  - Let x_bit, y_bit, z_bit ∈ {0,1} indicate which half along each axis:
    - 0 = negative half (e.g., x∈[0, 0.5), y∈[0, 0.5), z∈[0, 0.5))
    - 1 = positive half (e.g., x∈[0.5, 1), y∈[0.5, 1), z∈[0.5, 1))
  - Bit index i = (x_bit << 0) | (y_bit << 1) | (z_bit << 2)
  - Then sub_mask has bit i set if that octant is occupied.

Octant index table (i = x | y<<1 | z<<2):
- 0b000 = 0 → (-x, -y, -z)
- 0b001 = 1 → (+x, -y, -z)
- 0b010 = 2 → (-x, +y, -z)
- 0b011 = 3 → (+x, +y, -z)
- 0b100 = 4 → (-x, -y, +z)
- 0b101 = 5 → (+x, -y, +z)
- 0b110 = 6 → (-x, +y, +z)
- 0b111 = 7 → (+x, +y, +z)

Common shapes:
- Bottom slab (y=0): bits {0,1,4,5} → sub_mask = 0b00110011
- Top slab (y=1): bits {2,3,6,7} → sub_mask = 0b11001100
- Negative X half: bits {0,2,4,6} → sub_mask = 0b01010101
- Positive X half: bits {1,3,5,7} → sub_mask = 0b10101010
- Negative Z half: bits {0,1,2,3} → sub_mask = 0b00001111
- Positive Z half: bits {4,5,6,7} → sub_mask = 0b11110000

Notes:
- Local orientation: The mapping is defined in the block’s local axis frame aligned to world axes. Rotations/mirroring transform sub_mask by permuting/flipping xyz bits.
- Collision/meshing: Geometry and collision are assembled from the occupied octants. Greedy meshing can merge contiguous octants into larger faces to reduce polygons.
- Storage: sub_mask is part of the block/state definition, not stored per-voxel unless the block type requires dynamic geometry.
- Compatibility: Full cubes can use sub_mask = 0xFF. Empty space uses 0x00. Most blocks won’t need all combinations; registries can constrain allowed masks per block type.

