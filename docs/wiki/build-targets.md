# Build Targets

We currently ship three separate executables that reflect the intended runtime split. The default when running `zig build run` is the full target.

- full (default at zig build run)
  - Executable: vox-aetatum
  - Purpose: Single‑process runtime for early development. Creates the window and renderer (Sokol), initializes the registry, and runs the authoritative Simulation in a separate thread in‑process. The client renders from snapshots published by the Simulation.
  - Build: zig build full
  - Run: zig build run -- [args]

- client
  - Executable: vox-client
  - Purpose: Rendering/UI only. Includes client and Simulation code but does not run the Simulation. Instead, it is intended to connect to a remote server and use the Simulation module only for prediction/latency compensation and as a container for streamed state.
  - Current status: Skeleton. It creates a local Simulation instance but does not start its thread, and attempts a TCP connect to 127.0.0.1:7777 in the background. Protocol/handshake and state sync are TODO.
  - Build: zig build client
  - Run: zig-out/bin/vox-client [args]

- server (headless)
  - Executable: vox-server
  - Purpose: Authoritative Simulation process. Headless (no Sokol). Accepts network connections and will drive the world, persistence, and mod hooks.
  - Current status: Skeleton. Initializes the registry and starts the Simulation thread. Listens on 127.0.0.1:7777 and accepts connections (no protocol yet).
  - Build: zig build server
  - Run: zig-out/bin/vox-server [args]

Notes

- UI/screen coordinates use origin (0,0) at the top‑left, consistent with Sokol.
- The shader `src/shaders/chunk_shd.zig` is generated at build time from `shaders/chunk.glsl` via sokol-shdc. It is required by the client and full targets; the server target does not include Sokol.
- As features evolve, if the wiki describes functionality not yet implemented in code, we place a TODO in code and prefer to keep the wiki up to date with intent and current status.

See also

- docs/wiki/architecture-client-server.md for the conceptual split between Client (CLI), Simulation (SIM), and Game State (GS), and how single‑player (full) composes the same components.

