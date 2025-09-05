# Simulation (SIM) Update Order

This document defines the deterministic tick pipeline for Vox Aetatum. Most gameplay functionality (blocks, entities, crafting, circuits, fluids, etc.) is provided by mods. Therefore, the pipeline exposes explicit hook phases so mods can attach behavior consistently and deterministically.

Principles:

- Determinism first: Same inputs and seeds produce the same outputs. No wall-clock reads; stable iteration orders; seeded RNG.
- Clear phases: Ordered phases with explicit hook points. Mods must choose the correct phase for their logic.
- Bounded work: Per-tick budgets and iteration caps to prevent runaway updates. Overflow work is deferred.
- Parallel where safe: Background jobs (lighting, meshing, IO) run in workers; results are integrated at barriers.

## Fixed Tick Rate

- Target: 20 ticks per second (TPS). All SIM logic advances in fixed steps.
- Each tick T operates against authoritative GS and publishes a snapshot for CLI after finalization.

## High-Level Pipeline (Server/Authoritative)

Ordered phases within a tick T:

0. Intake and Setup

- Gather player commands for tick T (from local CLI or network).
- Close chunk load/unload operations finalized for this tick; create/destroy SIM state as needed.
- Hook: on_tick_start(world, T)

1. Apply Commands (Input → Intents)

- Apply movement, interactions, use/place/break, inventory actions.
- Mods can register command decoders and validators.
- Deterministic ordering: process by connection_id then by command sequence.
- Hook: on_apply_commands_pre, on_apply_commands_post

2. Neighbor and Structural Updates

- Resolve block placement/removal side-effects (neighbor notifications).
- Queue scheduled updates created by these changes.
- Hook: block.on_neighbor_changed, mod.on_block_changed

3. Scheduled Block Updates (Due at T)

- Process scheduled updates whose due_tick == T.
- Bounded iteration: up to N_scheduled_per_tick; overflow remains queued for T+1.
- Hook: block.on_scheduled_tick

4. Circuit Updates (Redstone-like)

- Evaluate power propagation, component logic (wires/torches/repeaters/comparators), and resulting block state changes.
- Deterministic evaluation order: stable component ordering (chunk_pos → local_pos → component_id) with bounded propagation passes.
- Hook: circuit.on_update, block.on_power_changed

5. Fluids

- Advance fluid simulation levels and flows.
- Bounded passes with deterministic order (by chunk then local coordinates).
- Hook: fluid.on_update

6. Physics and Collisions (Entities)

- Integrate velocities, apply gravity and friction, resolve collisions against voxel shapes and octant geometry.
- Handle ladders, liquids, step-up, triggers.
- Hook: entity.on_physics_pre, entity.on_physics_post

7. Entity AI and Behaviors

- Tick AI brains/goals with per-tick budgets for pathfinding.
- Resolve interactions with nearby blocks/entities.
- Hook: entity.on_ai_tick, mod.on_entity_interaction

8. Block Entities (Tile Entities)

- Tick block entities (containers, machines, signs) with bounded work.
- Hook: block_entity.on_tick

9. Random Ticks

- For blocks eligible for random updates, choose positions using deterministic per-section seeds.
- Hook: block.on_random_tick

10. Lighting Integration Barrier

- Apply incremental lighting updates arising from earlier phases.
- Background propagation jobs may run ahead; this phase commits results for T.
- Hook: mod.on_lighting_updated (read-only)

11. Crafting/Processing and Inventories

- Advance furnaces/machines, apply crafting outcomes, resolve inventory moves.
- Hook: mod.on_processing_tick, inventory.on_update

12. Health/Status and Cleanup

- Apply damage/heal, status effects, despawns.
- Hook: entity.on_status_tick, mod.on_cleanup

13. Time/Weather Progression

- Advance world time, update weather timers and transitions.
- Hook: world.on_time_weather_tick

14. Persistence and Dirty Flags

- Mark dirty chunks/sections/entities for save; enqueue IO tasks.
- Hook: mod.on_persist_mark (read-only; advisory)

15. Networking (Server → Clients)

- Generate authoritative deltas/snapshots keyed to tick T for subscribed clients.
- Include acks for last processed client command tick.

16. Snapshot Publish

- Publish an immutable GS snapshot for CLI rendering.
- Hook: on_tick_end(world, T)

## Deterministic Ordering Rules

- Chunks are processed in ascending (chunk_z, chunk_x).
- Within a chunk, blocks are processed in ascending (y, z, x) unless a subsystem specifies otherwise.
- Entities are processed in ascending eid. New entities spawned during T are queued for T+1 except when explicitly required.
- Queues (scheduled updates, circuits, fluids, random ticks) use stable priority keys and bounded passes per tick.

## Background Jobs and Barriers

- Meshing, lighting propagation, and IO run in worker threads.
- SIM coordinates job submission early in the tick and joins at defined barriers (notably lighting integration and before snapshot publish).

## Client Prediction (Optional)

- Clients may run a non-authoritative SIM using the same pipeline to predict local results.
- On receiving authoritative state for tick T, the client reconciles by rewinding to T and replaying unacknowledged inputs.

## Mod Hook Surface (Examples)

- on_tick_start(world, T)
- on_apply_commands_pre/ post
- block.on_neighbor_changed(pos, old_state, new_state)
- block.on_scheduled_tick(pos)
- circuit.on_update(region)
- fluid.on_update(region)
- entity.on_physics_pre/ post(entity)
- entity.on_ai_tick(entity)
- block_entity.on_tick(pos)
- block.on_random_tick(pos)
- mod.on_lighting_updated(region)
- mod.on_processing_tick(chunk)
- world.on_time_weather_tick(world)
- on_tick_end(world, T)

Hooks must obey determinism rules: no wall-clock, seeded RNG via SIM-provided sources, stable iteration, and no mutation outside their phase’s allowed state.

## Budgets and Caps (Initial)

- Scheduled updates: N_scheduled_per_tick per chunk (provisional).
- Circuit passes: up to N_circuit_passes per chunk (provisional).
- Fluid passes: up to N_fluid_passes per chunk (provisional).
- Random ticks: K_random_per_section per tick (target, seed-driven).
- AI/pathfinding budget: time or node cap per tick (provisional).

These values will be refined after profiling and captured in Constants and Limits when decided.
