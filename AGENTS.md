# SwarmKit Development Contract

SwarmKit is a greenfield, pre-release SDK and State Acceptance runtime.

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

SwarmKit owns physical command execution, exact goal/attempt identity, telemetry normalization,
evidence store decomposition, clock-quality interval arithmetic, state-quality contract evaluation,
state-acceptance certificates, timing and uncertainty provenance, health, capabilities, logging,
replay, and test infrastructure. High-level swarm coordination algorithms, mission planning, and
distributed application state belong to external application layers.

`Command ACK` is not physical arrival. `GOAL_REACHED` is not physical state acceptance. Backend-reported
accuracy is not a deterministic hard bound unless its declared semantics explicitly say so.
