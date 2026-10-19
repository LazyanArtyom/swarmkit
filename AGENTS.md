# SwarmKit Development Contract

SwarmKit is a greenfield, pre-release UAV SDK and onboard Verified Edge Coverage and Recovery runtime.

## API evolution

- Prefer one canonical API and one canonical representation for each concept.
- Make breaking source and protobuf changes when they materially improve correctness or clarity.
- Do not add legacy overloads, deprecated fields, compatibility wrappers, wire fixtures, or parallel
  old/new representations unless a future release policy explicitly requires them.
- When a model changes, update the protobuf schema, public C++ types, Agent, SDK, backends, CLI,
  tests, and documentation together.
- Absence and unknown correctness evidence must remain explicit; never replace them with zero,
  healthy, synchronized, or deterministic defaults.

## Architecture boundary

SwarmKit owns physical command execution, exact edge/attempt identity, telemetry normalization,
qualified sensor observations, verified edge-obligation accounting, bounded local recovery,
timing and uncertainty provenance, health, capabilities, logging, transport, and test infrastructure.
High-level swarm coordination algorithms, mission planning, and distributed application state
belong to external application layers.

`Command ACK` is not physical arrival. `GOAL_REACHED` is not edge fulfillment. Camera command
acknowledgement is not verified survey coverage. Backend-reported accuracy is not a deterministic
hard bound unless its declared semantics explicitly say so.
