# SwarmKit Project Description

Last updated: 2026-05-10

## Overview

SwarmKit is a C++23 software development kit and runtime framework for connecting UAV swarm applications to real drone agents, simulated vehicles, and future vehicle-control backends through a unified command, telemetry, authority, and reporting interface. It is designed as the low-level communication and execution substrate for larger UAV swarm platforms, including cloud-native mission systems, drone gateways, decentralized swarm controllers, dynamic planners, and operator tools.

In the broader swarm platform architecture, SwarmKit occupies the UAV-side connectivity and control layer. Higher-level services such as mission authoring, deterministic rotor-router planning, cloud supervision, telemetry aggregation, data analytics, and multi-user collaboration can use SwarmKit clients to communicate with onboard agents. Each agent runs near the vehicle, typically on a Raspberry Pi or similar companion computer, and translates generic SwarmKit commands into backend-specific vehicle protocol operations such as MAVLink messages for ArduPilot or PX4-compatible systems.

The central design principle is separation of mission intelligence from vehicle communication. SwarmKit does not impose one swarm algorithm, planner, or mission model. Instead, it provides robust primitives that allow developers to build those systems above it: command dispatch, authority arbitration, telemetry streams, typed reports, active goals, timed trajectories, backend capability discovery, and pluggable drone backends. This makes the library usable for rotor-router coordination, patrol, mapping, inspection, simulated combat scenarios, or custom dynamic swarm controllers.

## Role in the Cloud-Native UAV Swarm Platform

The scientific platform described in the accompanying UAV swarm architecture uses several logical layers: a multi-user client layer, a cloud-native microservice layer, a drone gateway layer, and an onboard UAV execution layer. SwarmKit is the practical SDK and runtime component used at the boundary between the drone gateway, external client tools, and onboard agents.

In this architecture:

- Cloud services and operator tools use the SwarmKit client SDK to communicate with UAV agents.
- Drone gateway services can use SwarmKit as the stable protocol-facing control interface toward drones.
- Onboard SwarmKit agents expose gRPC APIs for command, telemetry, reports, goals, and trajectory execution.
- Vehicle-specific communication is hidden behind backend implementations such as MAVLink or simulation.
- Higher-level planners remain free to implement their own algorithms and use SwarmKit only as the execution and observation layer.

This allows the cloud-side platform to remain algorithmically rich while keeping the drone-control boundary consistent, testable, and replaceable.

## Main Components

### Agent Runtime

`swarmkit-agent` is a gRPC daemon intended to run beside a vehicle. In a real deployment, it runs on a companion computer connected to the flight controller through MAVLink Router, serial MAVLink, UDP MAVLink, or a future backend protocol.

The agent is responsible for:

- exposing the SwarmKit gRPC service;
- receiving commands from clients;
- enforcing authority and priority rules;
- forwarding commands to the selected drone backend;
- decoding telemetry into a common vehicle state model;
- publishing typed reports;
- managing active goals and timed trajectory executions;
- tracking backend capabilities and health;
- isolating higher-level applications from protocol-specific details.

The current MAVLink backend listens on configurable UDP endpoints and targets a configured MAVLink system/component ID. In SITL development, one agent can listen to each simulated drone stream, for example ports `14601`, `14602`, and `14603`. In real deployments, the same backend can listen locally to MAVLink Router output from Pixhawk.

### Client SDK

The client SDK is the primary integration surface for developers. It provides C++ APIs for:

- ping, health, stats, and capability discovery;
- command dispatch with priority;
- telemetry subscriptions;
- authority lock/unlock/watch operations;
- typed report subscriptions;
- active goal set/cancel/get operations;
- trajectory upload, validation, preparation, start, pause, resume, abort, and status;
- swarm-level fanout helpers for multiple agents;
- trajectory loading from YAML, JSONL/NDJSON, and CSV planner outputs.

The SDK is intentionally more important than the CLI. The CLI is a reference and test tool that demonstrates how application developers can use the SDK correctly.

The SDK also owns verified command helpers so applications do not have to reimplement common wait-and-check behavior. Helpers such as `ArmAndWait`, `TakeoffAndWait`, `GotoAndWait`, `LandAndWait`, and `SendCommandAndWait` send the command and then use health or telemetry evidence to confirm the requested state. Swarm-level helpers can broadcast and verify commands across multiple agents while reporting accepted, already-satisfied, and failed outcomes separately.

### Command Line Tool

`swarmkit-cli` is a developer and operations utility for quickly testing agents, validating MAVLink connectivity, sending basic commands, subscribing to telemetry, monitoring reports, and exercising swarm workflows.

It supports:

- single-agent operations;
- swarm-config based multi-agent operations;
- telemetry output to console or CSV;
- report output as text or JSONL;
- command priorities;
- local sequence files;
- trajectory files in YAML, JSONL, and CSV.
- verified command execution through `--verify` and `--timeout-ms`;
- swarm command result policies such as `--continue-on-error` and `--require-all`; already-satisfied replies are treated as successful idempotent outcomes.

The CLI is not intended to replace a full mission planner. Its role is to be a practical reference implementation and smoke-test tool for the SDK.

### Backend Abstraction

SwarmKit uses an `IDroneBackend` boundary so that the agent runtime remains independent from vehicle protocol details. Existing and planned backends include:

- `sim`: synthetic backend for testing and development;
- `mavlink`: MAVLink backend for SITL, ArduPilot, PX4-style systems, and Pixhawk through MAVLink Router;
- future custom backends for vendor-specific protocols or specialized simulators.

Backends expose structured capabilities so clients can discover whether a vehicle supports mission upload, velocity control, payload commands, trajectory execution, backend-native commands, payload scheduling, supported modes, and telemetry fields.

## Control and Authority Model

SwarmKit supports multiple clients interacting with the same drone agent. This is essential for real swarm systems where a local onboard controller, a ground operator, a cloud supervisor, and emergency tools may all need access.

Commands carry a priority level:

- `operator`;
- `supervisor`;
- `override`;
- `emergency`.

The agent includes an authority arbiter that can lock control of a drone for a client, reject lower-priority commands, and notify subscribers about authority changes. This allows a cloud supervisor or emergency controller to override lower-level clients while still keeping command access explicit and auditable.

## Telemetry and Reports

Telemetry streams provide continuous vehicle state such as:

- position;
- relative and absolute altitude;
- velocity;
- attitude;
- battery;
- mode;
- armed state;
- landed state;
- GPS and health fields when available;
- failsafe and EKF indicators when decoded by the backend.

Telemetry fields use explicit validity metadata, so a zero value is not treated as a measurement unless the corresponding flag is true. Frames also carry source timestamps, coordinate-frame metadata, home origin when known, normalized GPS quality, estimator state, accuracy/covariance fields, and command/goal/execution linkage fields for monitoring tools and 3D swarm applications.

Reports are typed, event-oriented messages used for higher-level supervision. They are easier for applications to consume than raw telemetry when tracking goal or trajectory execution.

Report categories include:

- goal reports;
- trajectory reports;
- time synchronization reports;
- authority-related state changes;
- health and execution state events.

Reports can be persisted on the agent as rotated JSONL through `report_log_file`, `report_persistence.log_file`, or `--report-log-file`. The agent also persists the report sequence cursor, so monitoring tools can reconnect with `after_sequence` and replay retained reports instead of losing context after a restart.

For dynamic planners, reports provide a clean feedback loop. A user algorithm can send an active goal or trajectory segment, subscribe to reports, and then decide whether to continue, correct, cancel, or override based on `active`, `tracking`, `drifting`, `reached`, `failed`, `cancelled`, or `completed` states.

## Active Goals and Dynamic Command Streams

The active-goal API is designed for dynamic controllers. A client can set a named goal with:

- goal ID;
- revision;
- target position in the current global GPS frame, with API room for future local-NED targets;
- acceptance radius;
- deviation radius;
- timeout;
- optional metadata labels.

The agent supervises the goal using telemetry and reports progress or deviation, but it does not force one correction policy. This is intentional. Dynamic controllers should prefer `SetActiveGoal` for movement, wait for `GOAL_REACHED`, then update their own graph, event log, or task state. Rotor-router controllers, role-based tactical controllers, inspection tools, or custom planners can implement their own correction logic on top of the report stream.

This supports workflows where commands are generated dynamically while the drones are flying. For example, a swarm controller may repeatedly update goals as graph nodes are visited, reassign drones between roles, or issue corrective movement after observing deviation reports.

## Timed Trajectory and Planner Integration

SwarmKit includes generic timed trajectory primitives for use cases such as synchronized movement, generated coverage paths, and planner output execution.

A trajectory can contain:

- global latitude/longitude/altitude points;
- local position points where supported;
- optional velocity;
- optional yaw;
- time offsets or absolute Unix timestamps.

The lifecycle is:

1. upload trajectory;
2. validate trajectory;
3. prepare execution;
4. start at a given time;
5. monitor reports;
6. pause, resume, abort, clear, or query execution.

Trajectory validation reports hard errors, warnings, computed required speeds, and failing point indices. Vehicle profile values such as cruise speed, climb speed, descent speed, maximum altitude, battery reserve, minimum GPS fix type, minimum satellite count, maximum acceptable HDOP, takeoff/land timeout margins, and tracking tolerance are used during validation and execution supervision.

Planner integration is supported through SDK loaders for:

- YAML for human-written mission files;
- JSONL/NDJSON for generated planner streams;
- CSV for simple tabular route output.

This allows external tools to generate paths without linking directly against the SDK, while still allowing deeper integrations to use the C++ API directly.

## MAVLink and SITL Support

The MAVLink backend uses generated MAVLink C headers to parse telemetry and pack outgoing commands. It currently supports core flight commands such as:

- arm and disarm;
- takeoff and land;
- return home;
- hold;
- set mode;
- set speed;
- goto and waypoint movement;
- yaw control;
- velocity commands;
- pause/resume;
- set home;
- mission commands;
- selected payload and backend-specific commands.

The backend handles ArduPilot SITL behavior discovered during real tests. For example, `MAV_CMD_DO_REPOSITION` can be unsupported by ArduCopter, so the backend falls back to `SET_POSITION_TARGET_GLOBAL_INT` for guided goto-style movement.

The agent also performs protocol-independent command precondition checks before forwarding commands to the backend. For example, arming an already armed drone is treated as already satisfied, takeoff is treated as already satisfied when the vehicle is already airborne above the target altitude, landing an already landed vehicle is accepted, and normal disarm is rejected when telemetry indicates the vehicle is still airborne. Emergency force-disarm remains a separate high-authority operation.

MAVLink telemetry decoding populates production-relevant health fields when the vehicle publishes them, including GPS fix type and normalized quality, visible satellites, HDOP, relative altitude, source timestamps, home origin, landed state, estimator status, selected accuracy estimates, and selected failsafe indicators. These fields are used by validation and command verification policies instead of relying only on generic heartbeat state.

The same backend is intended to work against both:

- SITL on another machine sending UDP MAVLink streams to development ports;
- real Pixhawk data routed by MAVLink Router to local companion-computer ports.

The difference is configuration, not code.

## Security Model

SwarmKit supports runtime-selectable transport security:

- `insecure` for local development, SITL, and isolated lab networks;
- `tls` for server-authenticated encrypted transport;
- `mtls` for mutual certificate-based authentication;
- `auto` to infer mode from configured certificate paths.

Production deployments should use `mtls`, especially when command authority or drone gateway access crosses network boundaries. Local SITL tests can use `--insecure` to avoid certificate friction.

When mTLS is enabled, the agent can enforce allowed client identities. This is important for multi-user and multi-service systems where clients such as operators, supervisors, servers, and emergency tools must be distinguishable.

## Scientific Relevance

SwarmKit supports the practical implementation layer of a cloud-native UAV swarm research platform. It provides a concrete, testable communication and execution substrate for experiments in:

- decentralized UAV swarm control;
- deterministic rotor-router mission execution;
- cloud-assisted supervision;
- dynamic planner-to-drone command streams;
- fault-tolerant telemetry and report handling;
- multi-client authority arbitration;
- simulation-to-real-drone transition;
- secure drone gateway communication.

In a scientific paper, SwarmKit can be described as the UAV-side SDK and agent framework that realizes the interface between high-level cloud-native swarm services and heterogeneous drone-control backends. It allows research contributions at the algorithmic and platform level to be executed against simulated and real drones through a common abstraction.

## Typical Deployment

In a real drone deployment:

1. Pixhawk communicates with MAVLink Router on the companion computer.
2. MAVLink Router forwards vehicle traffic to a local UDP endpoint.
3. `swarmkit-agent` listens to that endpoint and exposes a gRPC API.
4. Local or remote clients connect to the agent by Raspberry Pi IP and gRPC port.
5. Cloud gateway or supervisor services subscribe to telemetry and reports.
6. Higher-level planners send commands, goals, or trajectories through the SDK.
7. The agent can optionally persist reports as JSONL for audit, replay, and flight-test diagnosis.

Example local companion flow:

```text
Pixhawk -> MAVLink Router -> udp://127.0.0.1:14601 -> swarmkit-agent -> gRPC clients
```

Example remote control flow:

```text
swarmkit-cli / cloud gateway / planner app -> <raspberry-pi-ip>:50061 -> swarmkit-agent -> MAVLink -> Pixhawk
```

Repository examples cover the main deployment modes:

- `testdata/agent_mavlink_drone_1.yaml` for a single SITL/MAVLink drone;
- `testdata/swarm_sitl_config.yaml` for multi-agent SITL testing;
- `testdata/pixhawk_companion_config.yaml` as the starting point for a Raspberry Pi companion deployment.

## Boundaries and Non-Goals

SwarmKit does not implement one fixed swarm algorithm. It does not own rotor-router mission planning, cloud mission authoring, image analytics, or user collaboration logic. Those belong to higher-level applications and services.

SwarmKit provides the lower-level, reusable control substrate:

- vehicle abstraction;
- command transport;
- telemetry;
- reports;
- authority;
- backend protocol integration;
- trajectory and goal supervision;
- SDK utilities for planner integration.

This separation keeps the library generic enough for many UAV swarm research and engineering projects.
