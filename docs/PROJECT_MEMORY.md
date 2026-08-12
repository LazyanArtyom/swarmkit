# SwarmKit Project Memory

Last updated: 2026-06-07

## Current Shape

- SwarmKit is a C++23 UAV swarm SDK and runtime layer.
- `swarmkit-agent` is a gRPC daemon intended to run beside a vehicle, usually on a Raspberry Pi companion computer.
- Clients connect to the agent over configurable transport security:
  - `insecure` for SITL/local lab testing;
  - `tls` for encrypted server-authenticated transport;
  - `mtls` for production mutual authentication;
  - `auto` to infer from configured certificate paths.
- The agent has an `IDroneBackend` boundary. Current backends include simulation and MAVLink.
- `swarmkit-cli` is a reference/smoke-test client, not the primary planner product.
- The SDK includes support for commands, telemetry, reports, authority, active goals, swarm fanout, messages, artifacts, health, stats, and capabilities.
- Static mission upload and timed trajectory upload/execution were removed to keep the core library lightweight for dynamic rotor-router style applications.
- Application concepts such as graph nodes, roles, routes, formations, gossip, target ownership, and task allocation are intentionally owned by higher-level controllers and translated into SwarmKit active goals, commands, data messages, or artifacts.
- The SDK owns verified command helpers:
  - `SendCommandAndWait`
  - `ArmAndWait`
  - `TakeoffAndWait`
  - `GotoAndWait`
  - `LandAndWait`
  - swarm `BroadcastCommandAndWait`
- The data plane has two generic layers:
  - message plane for small labeled topic payloads;
  - artifact plane for chunked binary transfer with hashes, progress, cancellation, peer routing, and metadata.

## MAVLink / SITL Goal

The MAVLink backend connects `IDroneBackend` to real MAVLink traffic.

Current development setup:

- SITL can run on another machine and send MAVLink UDP traffic to the development host:
  - `udp://0.0.0.0:14601`, MAVLink sysid `1`
  - `udp://0.0.0.0:14602`, MAVLink sysid `2`
  - `udp://0.0.0.0:14603`, MAVLink sysid `3`
- One SwarmKit agent can be run per drone:
  - `drone-1`: gRPC `50061`, MAVLink UDP `14601`, sysid `1`
  - `drone-2`: gRPC `50062`, MAVLink UDP `14602`, sysid `2`
  - `drone-3`: gRPC `50063`, MAVLink UDP `14603`, sysid `3`
- On the Raspberry Pi, the agent should listen locally to data from MAVLink Router / Pixhawk, so IP/port must remain configuration, not hardcoded.

## Preferred Direction

- Keep SwarmKit algorithm-agnostic.
- Use active goals as the main dynamic movement primitive.
- Keep generic labels on goals/messages/artifacts so higher-level projects can attach graph IDs, target IDs, camera IDs, frame IDs, or event IDs without SwarmKit knowing those semantics.
- Keep message and artifact exchange independent from command authority locks.
- Keep gRPC, command arbitration, telemetry manager, report hub, and data service strongly separated.
- Translate MAVLink telemetry messages into `core::TelemetryFrame`.
- Translate SwarmKit commands into MAVLink command or setpoint messages.
- MAVLink `COMMAND_LONG` commands wait for matching `COMMAND_ACK` and return ACK details to clients.
- Goto supports the ArduPilot fallback:
  - try `MAV_CMD_DO_REPOSITION`;
  - if unsupported, fall back to `SET_POSITION_TARGET_GLOBAL_INT`.
- MAVLink health decoding feeds readiness and verification policy:
  - GPS fix type and normalized quality, visible satellites, HDOP, relative altitude, landed state, estimator status, selected accuracy data, source timestamps, home origin, and failsafe indicators are decoded when present.
  - Telemetry frames use explicit validity flags so production clients can distinguish real zero values from unknown measurements.
  - Agent-side preconditions treat idempotent command states cleanly, including already armed, already airborne for takeoff, already landed for land, and safe disarm checks before normal disarm.
- Durable correctness evidence uses the single deterministic `ExecutionEventEnvelope` recorder (`execution_recorder` / `--evidence-file`). Report-only JSONL and its compatibility aliases were removed. Telemetry reconnect uses Agent session plus producer sequence; report replay remains bounded in memory.

## MAVLink Configs

- `testdata/agent_mavlink_drone1.yaml` selects `backend: "mavlink"`.
- It binds the agent gRPC API on `0.0.0.0:50061`.
- It binds MAVLink UDP on `0.0.0.0:14601`.
- It targets MAVLink `sysid=1`, `compid=1`.
- The testdata configs explicitly use `transport_security: "mtls"`.
- For local SITL without certs, use `--insecure` on both agent and CLI.
- Current production/test examples:
  - `testdata/agent_mavlink_drone_1.yaml`
  - `testdata/swarm_sitl_config.yaml`
  - `testdata/pixhawk_companion_config.yaml`

## Scientific Platform Context

- SwarmKit is the UAV-side SDK and agent layer for the broader cloud-native, multi-user UAV swarm platform.
- Higher-level systems such as rotor-router controllers, cloud drone gateways, supervisors, and analytics services should use the SwarmKit client SDK to communicate with agents.
- SwarmKit should not hard-code rotor-router, patrol, inspection, gossip, image analytics, or domain-specific coordination logic.
- The right abstraction is: commands, telemetry, authority, reports, active goals, data messages, artifacts, backend capabilities, and backend-specific extension commands.
- See:
  - `docs/SWARMKIT_PROJECT_DESCRIPTION.md`
  - `docs/SCIENTIFIC_CONTEXT.md`
