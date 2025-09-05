# Client/Server Split and Runtime Topology

This document defines the runtime split for Vox Aetatum and how single-player and multiplayer modes compose the same three core components:

- World/Game State (GS): Canonical data model for the world and entities.
- Simulation (SIM): Deterministic tick-based logic that mutates GS.
- Interface/Client (CLI): Rendering, input, UI. Reads state, never mutates authoritative state.

Zig’s explicit memory model pushes us to clearly separate who owns and mutates memory. This design isolates mutation within SIM, presents stable read-only snapshots to the CLI, and allows the GS to live either in-process (single-player) or remotely (multiplayer) with the same APIs.

## Components

1. World/Game State (GS)

- Responsibility: Authoritative representation of blocks, block states, block entities, lighting, entities, scheduled/random ticks, and world metadata (time, weather, dimension info).
- Ownership: Mutated only by SIM. CLI observes via immutable snapshots.
- Serialization: Versioned, forward-compatible chunk/section formats; delta-friendly entity and block-entity records.
- Snapshots: Double/triple-buffered snapshots for rendering and UI. CLI reads from a stable snapshot while SIM advances the next state.
- Determinism anchors: Stores tick counter and seeds for subsystems to keep SIM deterministic.

2. Simulation (SIM)

- Responsibility: Deterministic fixed-step updates (target 20 TPS) that apply player commands, AI, physics, block updates, lighting updates, and worldgen streaming.
- Extensibility: All mod hooks execute in SIM. Mods must be deterministic with respect to provided seeds and inputs.
- Scheduling: Processes scheduled block updates, random ticks, and event queues in a defined order for reproducibility.
- Concurrency: Internally parallelizable via jobs, but presents atomic world-step boundaries. Writes to a new GS buffer each tick, then publishes a snapshot.

3. Interface/Client (CLI)

- Responsibility: Input capture, prediction, interpolation, rendering, audio, and UI.
- State access: Read-only access to GS snapshots via an immutable view. No direct mutation of authoritative GS.
- Networking: Encodes player intents/commands to send to SIM (local or remote) and consumes authoritative updates.

## Single-Player Topology (In-Process)

- Placement: GS and SIM live in the same process as CLI.
- Memory model:
  - SIM owns the mutable GS buffer for the current tick.
  - After a tick, SIM publishes a read-only snapshot (double/triple buffer) that CLI reads.
- Data flow:
  1. CLI collects input → enqueues commands for next tick.
  2. SIM consumes commands → advances tick → writes to GS → publishes snapshot.
  3. CLI renders from the latest available snapshot and interpolates if desired.
- Benefits: Zero network latency, minimal serialization, identical APIs to multiplayer via an internal transport.

## Multiplayer Topology (Client/Server)

- Authoritative SIM: Lives on the server alongside the authoritative GS.
- Client-side GS: CLI maintains read-only snapshots received from the server. Optionally, it runs a non-authoritative SIM for prediction.
- Transport abstraction: Same command/update API as single-player, but marshaled over the network.
- Data flow:
  1. CLI encodes commands (inputs) with local tick indices.
  2. Server SIM applies commands deterministically, advances tick, and sends deltas/snapshots back.
  3. Client reconciles: Applies authoritative updates; if running prediction, rewinds and replays unacknowledged inputs.

## Prediction and Reconciliation (Optional but Recommended)

- Prediction: Client runs a non-authoritative SIM with the same mods and deterministic seeds to mask latency.
- Reconciliation:
  - Maintain a history buffer of inputs and predicted states keyed by tick.
  - On receiving authoritative state S_authoritative at tick T, correct the local state and re-simulate inputs (T+1…now).
  - Visual smoothing: Interpolate between last-confirmed and current-predicted for rendering.
- Divergence control:
  - Strict determinism in SIM and mod hooks (seeded RNG, no wall-clock access, no non-deterministic iteration orders).
  - Floating-point care (consistent math flags) or fixed-point where needed for critical systems.

## Mods and Consistency Requirements

- Symmetry: All mods that influence SIM must be present on both client and server with compatible versions.
- Manifest & capability negotiation:
  - Each mod declares an ID, semantic version, and capability set (e.g., registers block types, entities, recipes, network messages).
  - Connection handshake validates exact content ID registry and capability compatibility.
- Content registry stability:
  - Stable numeric IDs derived from a hashed, ordered registry (namespace:name) to align block/entity IDs between peers.
  - Wire format uses stable IDs; debug paths include human-readable names for diagnostics.
- Scripted mods: Sandboxed and versioned; no access to non-deterministic sources unless explicitly provided as seeds.

## Wire Protocol and State Sync

- Commands: Compact player-intent messages (movement, interact, use, inventory actions), tagged by local tick.
- Updates:
  - Chunk streaming: Section-level palette + bit-packed blocks; optional compression.
  - Entity deltas: Position/velocity/flags/attributes; authority sets the canonical values.
  - Block-entity/state diffs: Minimal updates to NBT-like or schema’d data.
  - Lighting deltas: Incremental updates to skylight/block light fields.
- Snapshots vs deltas:
  - Initial area: Full snapshots.
  - Steady state: Deltas keyed by authoritative tick; periodic keyframes to limit drift and recovery cost.

## Tick, Time, and Ordering

- Fixed tick rate (target 20 TPS). Network packets reference authoritative tick numbers.
- Update order within a tick is defined and documented (player commands → physics → block updates → lighting → AI → finalization).
- Random ticks and scheduled updates use deterministic queues seeded per-chunk/section.

## Threading and Memory Safety

- Double/triple buffering GS snapshots isolates CLI reads from SIM writes.
- Job system:
  - Meshing, I/O, and lighting propagation can run in worker threads.
  - SIM orchestrates and collects results at safe barriers.
- CLI never holds raw pointers to mutable GS; snapshot views own their lifetime and are immutable.

## Persistence Integration

- Server saves authoritative GS via region files and per-entity/BE records.
- Client caches streamed chunks for fast rejoin, but treats them as ephemeral.
- Save format is versioned; migrations handled at load with schema adapters.

## Security and Trust

- Server validates all client commands; never accepts client-side state as authoritative.
- Mods can define validation hooks on the server to guard custom mechanics.
- Scripting sandboxes restrict file/network access unless granted by explicit capabilities.

## Open Decisions

- Exact snapshot cadence and interpolation buffer length on the client (e.g., 2–3 ticks of latency buffer).
- Whether to prefer fixed-point math for critical movement/physics for determinism across platforms.
- Reconciliation granularity for inventory/crafting UIs versus world physics.
- Standardized mod network channels vs auto-generated schemas from mod manifests.
