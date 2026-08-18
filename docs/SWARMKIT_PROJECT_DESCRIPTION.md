# SwarmKit Project Description

Last updated: 2026-08-18

## Overview

SwarmKit is a C++23 SDK and agent runtime for connecting UAV swarm applications to drone agents, simulated vehicles, and vehicle-control backends through a unified command, telemetry, authority, report, message, artifact, and state acceptance interface.

SwarmKit is not a mission planner and does not implement domain-specific swarm choreography, image analytics, or cloud application workflows. Those systems are built above SwarmKit.

The core design principle is separation of application intelligence from vehicle communication and state acceptance. Higher-level software decides what should happen; SwarmKit provides the typed, testable, verifiable, and secure path to execute commands and evaluate multi-agent physical state on real or simulated drones.

## Architecture

SwarmKit is organized around four independent planes.

### 1. Control Plane

The control plane is safety-critical and authority-protected:

- command dispatch;
- command priorities;
- authority lock/unlock/watch;
- active goals;
- telemetry subscriptions;
- health, readiness, preflight, capabilities, and runtime stats;
- typed reports;
- backend abstraction.

Commands and active goals use authority rules. Lower-priority clients can be rejected while high-priority supervisors or emergency clients can take control.

### 2. State Acceptance & Verification Plane

The state acceptance plane provides deterministic common-time guarantees:

- evidence store decomposition from telemetry streams;
- generation-time interval arithmetic accounting for clock uncertainty;
- deterministic uncertainty propagation bounding position error across elapsed time;
- state-quality contracts with explicit predicates (freshness, clock error, health, coordinate frames, session epochs);
- no-silent-downgrade evaluation producing structured rejections or accepted snapshots;
- tamper-evident cryptographic state-acceptance certificates;
- standalone independent verifier for offline audit and replay verification.

### 3. Message Plane

The message plane carries small low-latency application payloads:

- decentralized state updates;
- event-log updates;
- target sightings;
- key/value updates;
- compact JSON/CBOR/protobuf payloads.

Messages are labeled, topic-based, optionally targeted, TTL-aware, and independent from command locks. A client can exchange data even while another client holds command authority.

### 4. Artifact Plane

The artifact plane carries binary data:

- images;
- detection crops;
- maps;
- logs;
- arbitrary binary blobs.

Artifacts use chunked transfer, hashing, size limits, TTL, direct routing to configured peers, transfer handles, progress, cancellation, and storage metadata. Large data moves through artifacts, while messages announce availability or carry small metadata.

## Agent Runtime

`swarmkit-agent` is a gRPC daemon intended to run near the vehicle, usually on a Raspberry Pi companion computer.

The agent is responsible for:

- exposing SwarmKit gRPC services;
- enforcing command authority;
- forwarding commands to the selected backend;
- decoding telemetry into a common vehicle state model;
- publishing reports;
- supervising active goals;
- routing data messages and artifacts to directly configured peers;
- tracking backend health and capabilities;
- isolating applications from protocol-specific details.

The current MAVLink backend supports SITL and Pixhawk-compatible systems through configurable UDP endpoints or MAVLink Router output.

## Client SDK

The client SDK is the primary integration surface for application developers. It provides APIs for:

- ping, health, stats, and capabilities;
- command dispatch and verified command helpers;
- telemetry subscriptions;
- report subscriptions;
- authority lock/unlock/watch;
- active goal set/cancel/get;
- swarm fanout helpers;
- message publish/subscribe/send-to-drone;
- artifact upload/download/send/start/status/cancel;
- state quality contract evaluation and snapshot requests;
- certificate verification.

## CLI Tools

`swarmkit-cli` is a reference and smoke-test tool. It supports:

- single-agent and swarm operations;
- telemetry output to console or CSV;
- report output as text or JSONL;
- command priorities and verified command execution;
- active goals and authority locks;
- message and artifact workflows.

`swarmkit-evidence-inspect` inspects binary execution logs.
`swarmkit-dissertation-experiment` runs paired-trace empirical benchmarks across fault scenarios and outputs performance tables.

## Backend Abstraction

SwarmKit uses an `IDroneBackend` boundary so the agent runtime remains independent from vehicle protocol details.

Current backends:

- `sim`: synthetic backend for tests, development, and deterministic fault injection;
- `mavlink`: backend for SITL, ArduPilot systems, and Pixhawk through MAVLink Router.

Backends expose structured capabilities such as velocity support, payload support, backend-specific command support, supported modes, supported telemetry fields, and numeric limits.

## Active Goals

Active goals are the preferred movement primitive for dynamic controllers. A client can set a named goal with:

- goal ID and revision;
- target position (global WGS84 or local NED);
- speed and acceptance radius;
- deviation radius and timeout;
- generic labels.

The agent supervises progress from telemetry and emits goal reports. Applications remain responsible for choosing the next target and deciding swarm coordination policies.

## Security

SwarmKit supports:

- `insecure` for local SITL and isolated lab testing;
- `tls` for encrypted server-authenticated transport;
- `mtls` for production mutual authentication;
- `auto` to infer mode from configured certificate paths.

Production deployments should use `mtls` when command authority or drone gateway access crosses network boundaries.

## Boundaries

SwarmKit provides:

- vehicle abstraction and command transport;
- normalized telemetry, reports, and active goals;
- command authority arbitration;
- common-time evidence store and clock-quality arithmetic;
- State-Quality Contracts and State Acceptance Engine;
- State-Acceptance Certificates and Independent Verifier;
- generic data messages and binary artifacts;
- SDK utilities.

SwarmKit intentionally does not provide:

- static mission authoring;
- drone-show choreography;
- image analytics;
- domain-specific coordination algorithms.

This keeps the library modular, testable, and focused on robust multi-agent control and verified state acceptance.
