# SwarmKit Project Memory

Last updated: 2026-08-18

## Current Shape

- SwarmKit is a C++23 UAV swarm SDK, agent runtime, and Replay-Verifiable Common-Time State Acceptance framework.
- `swarmkit-agent` is a gRPC daemon intended to run beside a vehicle, usually on a Raspberry Pi companion computer.
- Clients connect to the agent over configurable transport security:
  - `insecure` for SITL/local lab testing;
  - `tls` for encrypted server-authenticated transport;
  - `mtls` for production mutual authentication;
  - `auto` to infer from configured certificate paths.
- The agent has an `IDroneBackend` boundary. Current backends include simulation and MAVLink.
- `swarmkit-cli` is a reference/smoke-test client tool.
- `swarmkit-evidence-inspect` inspects execution logs.
- `swarmkit-dissertation-experiment` runs paired-trace empirical benchmarks and outputs dissertation tables.
- The SDK includes support for:
  - commands, telemetry, reports, authority, active goals, swarm fanout, messages, artifacts, health, stats, and capabilities;
  - evidence store decomposition, clock-quality interval arithmetic, state-quality contracts, state acceptance certificates, and independent verification.
- Verified command helpers:
  - `SendCommandAndWait`
  - `ArmAndWait`
  - `TakeoffAndWait`
  - `GotoAndWait`
  - `LandAndWait`
  - swarm `BroadcastCommandAndWait`
- Four architectural planes:
  - **Control plane**: commands, priorities, authority locks, active goals, reports, backend abstraction;
  - **State acceptance plane**: common-time evidence store, uncertainty propagation, state-quality contracts, cryptographic certificates, independent verification;
  - **Message plane**: small labeled topic payloads;
  - **Artifact plane**: chunked binary transfer with hashes, progress, cancellation, peer routing, and metadata.

## Preferred Direction

- Keep SwarmKit algorithm-agnostic.
- Use active goals and verified state acceptance as the main dynamic primitives.
- Translate MAVLink telemetry messages into `core::TelemetryFrame`.
- Translate SwarmKit commands into MAVLink command or setpoint messages.
- MAVLink `COMMAND_LONG` commands wait for matching `COMMAND_ACK` and return ACK details to clients.
- Telemetry frames use explicit validity flags so production clients can distinguish real zero values from unknown measurements.
- Durable correctness evidence uses the single deterministic `ExecutionEventEnvelope` recorder (`execution_recorder` / `--evidence-file`).
- State acceptance enforces strict *no-silent-downgrade* semantics and zero containment failure under valid deterministic premises.
