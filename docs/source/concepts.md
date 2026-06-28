# Main Concepts

## Agent

`swarmkit-agent` is the per-vehicle gRPC server. It accepts client RPCs,
enforces command authority, starts backend telemetry streams, exposes health and
runtime counters, and routes messages or artifacts through the data-plane APIs.

An agent is configured with `swarmkit::agent::AgentConfig` and can use the
built-in simulator, the MAVLink backend, or a registered custom backend.

## Client

`swarmkit::client::Client` connects to one agent. It provides unary operations
such as `Ping()`, `GetHealth()`, `GetRuntimeStats()`, `GetCapabilities()`, and
`SendCommand()`, plus background streams for telemetry, authority events,
reports, and data messages.

The client owns its gRPC channel and retry/reconnect policies. Unary RPCs are
thread-safe; stream lifecycle methods should be coordinated by the application.

## SwarmClient

`swarmkit::client::SwarmClient` manages a registry of drone IDs mapped to
per-drone `Client` instances. It routes a command by `CommandEnvelope.context`,
broadcasts commands with bounded parallelism, and can subscribe to telemetry or
reports from all registered drones.

Use `SwarmClient` when ground-control code needs fleet-level fanout or a single
place to load `swarm.drones` topology from YAML.

## Backend

`swarmkit::agent::IDroneBackend` is the agent-side vehicle interface. Backends
execute typed command envelopes, start and stop telemetry, and report health and
capabilities. The simulator backend accepts commands and produces synthetic
telemetry. The MAVLink backend speaks direct UDP MAVLink to SITL or autopilot
traffic.

Custom backends can be registered through `BackendRegistry` with a
`BackendCreator`.

## Commands

Commands live under `swarmkit::commands` and are represented as nested
`std::variant` types:

- `FlightCmd`: arm, disarm, takeoff, land, mode changes, emergency actions.
- `NavCmd`: waypoint, goto, return home, hold, speed, yaw, velocity, home.
- `PayloadCmd`: cameras, video, gimbal, ROI, servo, relay, gripper.
- `BackendCmd`: a namespaced escape hatch for backend-specific actions.

Every command is wrapped in a `CommandEnvelope` with routing and arbitration
metadata: drone ID, client ID, priority, optional deadline, and correlation ID.

## Authority

Command authority is controlled by `CommandArbiter`. Priorities are:

- `kOperator`: normal CLI and SDK callers.
- `kSupervisor`: local automated systems.
- `kOverride`: higher-priority correction or safety control.
- `kEmergency`: bypasses normal arbitration.

Clients can acquire authority with an optional TTL. Authority watchers receive
events when a client is granted, preempted, resumed, or expired.

## Telemetry

Telemetry is normalized into `swarmkit::core::TelemetryFrame`. Numeric fields
include explicit units, and validity flags indicate which measurements are
meaningful. For example, check `frame.HasPosition()` before using `lat_deg` and
`lon_deg`.

Frames can carry source timestamps, coordinate-frame metadata, GPS quality,
estimator state, accuracy fields, home origin, active command IDs, active goal
IDs, and correlation IDs.

## Goals And Reports

An active goal describes a target position, acceptance radius, deviation radius,
optional timeout, and labels. The agent supervises goal state and emits typed
reports such as command acceptance, command failure, goal progress, stale
telemetry, heartbeat loss, and authority changes.

Use reports when automation needs a structured event stream instead of parsing
CLI text.

## Data Messages And Artifacts

The client API includes small topic-based data messages and chunked artifact
transfer. Artifacts are described by `ArtifactDescriptor` and can be uploaded,
downloaded, announced, listed, or routed to another configured data peer.

The default agent artifact directory is `/tmp/swarmkit-artifacts`.
