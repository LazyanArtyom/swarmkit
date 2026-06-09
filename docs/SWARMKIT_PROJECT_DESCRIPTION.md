# SwarmKit Project Description

Last updated: 2026-06-07

## Overview

SwarmKit is a C++23 SDK and agent runtime for connecting UAV swarm applications to drone agents, simulated vehicles, and future vehicle-control backends through a unified command, telemetry, authority, report, message, and artifact interface.

SwarmKit is not a mission planner and does not implement rotor-router, gossip merge logic, drone-show choreography, image analytics, or cloud application workflows. Those systems are built above SwarmKit.

The core design principle is separation of application intelligence from vehicle communication. Higher-level software decides what should happen; SwarmKit provides the typed, testable, secure path to make it happen on real or simulated drones.

## Architecture

SwarmKit is organized around three independent planes.

### Control Plane

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

### Message Plane

The message plane carries small low-latency application payloads:

- gossip deltas;
- graph/event-log updates;
- target sightings;
- key/value updates;
- compact JSON/CBOR/protobuf payloads.

Messages are labeled, topic-based, optionally targeted, TTL-aware, and independent from command locks. A local client can exchange data even while another client holds command authority.

### Artifact Plane

The artifact plane carries binary data:

- images;
- detection crops;
- maps;
- logs;
- arbitrary binary blobs.

Artifacts use chunked transfer, hashing, size limits, TTL, direct routing to configured peers, transfer handles, progress, cancellation, and storage metadata. Large data should move through artifacts, while messages should only announce availability or carry small metadata.

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
- peer status refresh.

Rotor-router and gossip projects should use active goals, reports, messages, and artifacts as building blocks rather than asking SwarmKit to own graph-specific state.

## CLI

`swarmkit-cli` is a reference and smoke-test tool. It supports:

- single-agent operations;
- swarm-config based operations;
- telemetry output to console or CSV;
- report output as text or JSONL;
- command priorities;
- verified command execution with `--verify`;
- active goals;
- authority locks;
- message and artifact workflows.

The CLI is not a planner and is not intended to replace an application SDK integration.

## Backend Abstraction

SwarmKit uses an `IDroneBackend` boundary so the agent runtime remains independent from vehicle protocol details.

Current backends:

- `sim`: synthetic backend for tests and development;
- `mavlink`: backend for SITL, ArduPilot systems, and Pixhawk through MAVLink Router.

Backends expose structured capabilities such as velocity support, payload support, backend-specific command support, supported modes, supported telemetry fields, and numeric limits.

## Active Goals

Active goals are the preferred movement primitive for dynamic planners. A client can set a named goal with:

- goal ID;
- revision;
- target position;
- optional local-NED target shape for future backend support;
- speed;
- acceptance radius;
- deviation radius;
- timeout;
- generic labels.

The agent supervises progress from telemetry and emits goal reports. Applications remain responsible for choosing the next target, updating graph state, and deciding correction policies.

Local-NED active goals are intentionally API-shaped for future indoor/low-GPS backends, but current MAVLink execution and goal monitoring only support global GPS targets. Current agents reject local-NED goals clearly instead of pretending to execute them.

## MAVLink and SITL Support

The MAVLink backend uses generated MAVLink C headers to parse telemetry and pack outgoing commands.

Supported command families include:

- arm, force-arm, disarm, force-disarm;
- takeoff and land;
- return-home, hold, pause, resume;
- set mode;
- set speed;
- goto and waypoint movement;
- yaw control;
- velocity commands;
- set home;
- selected payload commands;
- backend-specific command escape hatch.

The backend handles practical ArduPilot behavior. For example, `MAV_CMD_DO_REPOSITION` can be unsupported by ArduCopter, so the backend falls back to `SET_POSITION_TARGET_GLOBAL_INT` for guided goto-style movement.

MAVLink telemetry decoding populates GPS fix type and quality, satellites, HDOP, relative altitude, source timestamps, home origin, landed state, estimator status, accuracy estimates, and selected failsafe indicators when available.

## Security

SwarmKit supports:

- `insecure` for local SITL and isolated lab testing;
- `tls` for encrypted server-authenticated transport;
- `mtls` for production mutual authentication;
- `auto` to infer mode from configured certificate paths.

Production deployments should use `mtls` when command authority or drone gateway access crosses network boundaries.

## Boundaries

SwarmKit provides:

- vehicle abstraction;
- command transport;
- telemetry;
- reports;
- authority;
- active goals;
- backend protocol integration;
- generic data messages;
- generic binary artifacts;
- SDK utilities.

SwarmKit intentionally does not provide:

- static mission upload;
- timed trajectory upload/execution;
- implicit multi-hop data forwarding without an explicit routing policy;
- rotor-router state machines;
- gossip semantics;
- drone-show choreography;
- image analytics;
- cloud mission authoring.

This keeps the library lightweight, flexible, and suitable as the substrate for future rotor-router and gossip projects.
