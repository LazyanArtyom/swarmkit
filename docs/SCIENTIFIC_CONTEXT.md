# SwarmKit Scientific Context

Last updated: 2026-08-18

## Purpose

SwarmKit is the UAV-side SDK, agent runtime, and Replay-Verifiable Common-Time State Acceptance framework for experiments that need to observe and control real or simulated drone swarms without coupling higher-level swarm algorithms to flight-controller protocol mechanics or trusting raw, unaligned telemetry.

It is algorithm-agnostic. Distributed coordination algorithms, swarm mission planning, cloud supervision, and user collaboration belong in applications built on top of SwarmKit.

## Paper-Ready Description

SwarmKit is a C++23 gRPC-based multi-UAV agent, client SDK, and state acceptance engine. Each drone runs a SwarmKit agent on a companion computer. The agent exposes typed APIs for command dispatch, authority arbitration, active-goal supervision, telemetry streaming, report streaming, data messages, binary artifact transfer, health checks, and capability discovery. Vehicle communication is isolated behind a backend interface; the current production backend uses MAVLink for ArduPilot SITL and Pixhawk-compatible systems.

To resolve the fundamental distributed state exposure problem in asynchronous drone swarms, SwarmKit implements **Replay-Verifiable Common-Time State Acceptance**:

1. **Evidence Normalization**: Raw telemetry is decomposed into typed evidence records with explicit timing, identity, coordinate frame, and deterministic uncertainty provenance.
2. **Clock-Quality Interval Arithmetic**: Translates source timestamps into common-time generation intervals $[g^-, g^+]$ accounting for clock offset and uncertainty radius $\rho$.
3. **Deterministic Uncertainty Propagation**: Expands position uncertainty bounds conservatively across elapsed physical time: $\varepsilon_p(t^*) = e_p + V_{\max}\cdot(t^* - g^-)$.
4. **State-Quality Contracts**: Evaluates multi-UAV causal snapshots against explicit predicates (age, clock uncertainty, estimator health, coordinate frames, session epochs, completeness rules) with strict *no-silent-downgrade* semantics.
5. **Replay-Verifiable Certificates**: Produces cryptographic certificates ($K$) bound to contract hashes ($h_C$) and snapshot evidence, verifiable offline by an independent verifier.

## Platform Mapping

| Paper Platform Concept | SwarmKit Role |
| --- | --- |
| UAV-side agent | `swarmkit-agent` runtime |
| Drone gateway to UAV communication | SwarmKit client SDK and gRPC services |
| Vehicle command bridge | `IDroneBackend` and MAVLink backend |
| Telemetry ingestion source | Agent telemetry streams |
| Supervisory feedback | Agent report streams |
| Dynamic movement target | Active goal API |
| Multi-client control authority | Command priorities and authority arbiter |
| State quality contract & acceptance | `StateAcceptanceEngine` & `StateQualityContract` |
| Replay verification & audit | `StateAcceptanceVerifier` & `StateAcceptanceCertificate` |
| Swarm data substrate | Generic message plane |
| Image / log / map exchange | Generic artifact plane |
| Secure communication | Runtime `insecure`, `tls`, and `mtls` modes |

## Architectural Boundary

```text
Cloud / Planner / Gateway / CLI / Swarm Application
        |
        | SwarmKit Client SDK, gRPC
        v
SwarmKit Agent on Companion Computer
        |
        | Backend abstraction
        v
MAVLink Router / Pixhawk / SITL / Future Backend
```

The high-level platform can implement swarm coordination, area coverage, search logic, inspection logic, and supervision without knowing MAVLink message formats. The agent can change from MAVLink to another backend without forcing the platform to rewrite its algorithm layer.

## Dynamic Controllers and State Acceptance

Dynamic controllers should prefer active goals, command streams, and verified state acceptance over static uploaded plans. This keeps online decisions responsive and verified:

```text
1. evaluate StateQualityContract at evaluation time t*
2. receive verified AcceptedSnapshot or StructuredRejection
3. compute next desired swarm action
4. send active goal or verified command
5. publish small labeled data messages
6. transfer images/logs/maps as artifacts when needed
7. repeat
```

`Command ACK` confirms receipt by the flight controller; it is not physical arrival. `GOAL_REACHED` is an execution baseline; physical state acceptance requires contract evaluation against common-time bounds.

## Current Implementation Status

Implemented capabilities include:

- C++23 SDK and agent runtime;
- gRPC control and data services;
- optional `insecure`, `tls`, and `mtls` transport security;
- simulated backend and deterministic fault-injection harness;
- MAVLink backend using generated MAVLink C headers;
- command ACK handling and STATUSTEXT surfacing;
- telemetry streaming with explicit validity fields;
- GPS, EKF, landed-state, failsafe, home-origin, accuracy, and source-time decoding;
- command priorities and authority locks;
- verified command helpers;
- active goal set/cancel/get and goal reports;
- swarm fanout for commands, goals, health, stats, telemetry, and authority;
- generic message plane for small labeled payloads;
- generic artifact plane for chunked binary transfer with hashing, TTL, direct peer routing, progress, cancellation, and peer status;
- bounded in-memory report replay cursors;
- canonical replay/live telemetry with Agent-session and producer-sequence identity;
- deterministic, checksummed execution-evidence recording;
- State Acceptance Engine with State-Quality Contracts;
- State-Acceptance Certificate generation and serialization;
- Independent replay verifier with semantic consistency checks for specified mutated certificate fields;
- CLI reference tool for smoke testing SDK workflows;
- Paired-trace experiment CLI binary for empirical validation.

## Recommended Citation-Level Interpretation

Use this framing:

- "SwarmKit is the UAV-side agent, client SDK, and State Acceptance runtime of the proposed platform."
- "The multi-UAV coordination and cloud supervision components use SwarmKit to communicate with vehicle agents and evaluate replay-verifiable state contracts."
- "SwarmKit abstracts MAVLink and future vehicle backends behind common command, telemetry, goal, report, message, artifact, and state acceptance APIs."
