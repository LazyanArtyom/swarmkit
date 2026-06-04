# SwarmKit Scientific Context

Last updated: 2026-05-10

## Purpose of This Document

This document summarizes how SwarmKit fits into the broader scientific platform for cloud-native, multi-user UAV swarm operation. It is written as reusable background material for papers, proposals, reports, and implementation references.

## Short Paper-Ready Description

SwarmKit is a C++23 UAV swarm communication and execution SDK that provides the onboard agent and client-side control interface for a cloud-native UAV swarm platform. Each drone runs a SwarmKit agent on a companion computer, where it bridges high-level commands, verified command helpers, active goals, timed trajectories, telemetry, and execution reports to the vehicle flight stack through a pluggable backend such as MAVLink. External clients, cloud drone gateways, supervisor services, and local operator tools communicate with these agents through a typed gRPC API.

The library is intentionally algorithm-agnostic. It does not prescribe a particular swarm coordination method; instead, it supplies the execution substrate required by higher-level systems such as deterministic rotor-router planners, adaptive mission supervisors, patrol systems, inspection tools, or dynamic controllers. This separation allows scientific work on swarm coordination and cloud orchestration to be evaluated against simulated or real UAVs through a stable and reusable control interface.

Within the larger cloud-native UAV swarm platform, SwarmKit represents the UAV-side execution and connectivity layer. Cloud services can use it through the client SDK to send commands, subscribe to telemetry, monitor reports, coordinate authority, upload trajectories, and supervise active goals. Onboard agents retain local execution capability and continue operating even if cloud connectivity is intermittent, while the cloud layer can provide mission-level supervision, analytics, and corrective assistance when communication is available.

## Relation to the Proposed Cloud-Native Swarm Platform

The paper-level platform contains several conceptual layers:

- multi-user Qt/C++ client for mission authoring and visualization;
- cloud-native microservices for authentication, workspace state, planning, telemetry, supervision, and artifact analytics;
- drone gateway for secure UAV connectivity;
- onboard UAV agents for vehicle-facing execution;
- deterministic rotor-router and gossip mechanisms for decentralized swarm behavior.

SwarmKit implements the reusable software foundation for the drone gateway and onboard agent boundary. It allows the cloud platform to treat drones as controllable, observable agents with standard APIs, while still keeping the actual vehicle communication backend replaceable.

The mapping is:

| Paper Platform Concept | SwarmKit Implementation Role |
| --- | --- |
| UAV-side agent | `swarmkit-agent` runtime |
| Drone gateway to UAV communication | SwarmKit client SDK and gRPC service |
| Vehicle command bridge | `IDroneBackend` and MAVLink backend |
| Telemetry ingestion source | Agent telemetry streams |
| Supervisory reports | Agent report streams |
| Dynamic correction targets | Active goal API |
| Timed/synchronized execution | Trajectory execution API |
| Multi-client control authority | Command priority and authority arbiter |
| Secure communication | Runtime `insecure`, `tls`, and `mtls` modes |

## Architectural Position

SwarmKit should be described as middleware between high-level swarm intelligence and low-level flight-control protocols.

```text
Cloud / Planner / Gateway / CLI
        |
        | SwarmKit Client SDK, gRPC
        v
SwarmKit Agent on Companion Computer
        |
        | Backend abstraction
        v
MAVLink Router / Pixhawk / SITL / Future Backend
```

The high-level platform can use rotor-router planning, gossip dissemination, simulation, and supervision without coupling those mechanisms directly to MAVLink message details. Conversely, the agent can communicate with MAVLink or future protocols without forcing the cloud platform to change its command and telemetry model.

## Why This Layer Is Needed

Research platforms for UAV swarms often describe planning, coordination, simulation, or cloud services at a high level, but practical deployment requires a reliable boundary between those services and real vehicles. SwarmKit provides that boundary.

It addresses several implementation problems:

- heterogeneous vehicle protocol integration;
- command acknowledgement and error propagation;
- telemetry normalization;
- multi-client command authority;
- safe local testing against SITL;
- migration from simulation to real Pixhawk/MAVLink deployment;
- streaming reports for mission-level feedback;
- verified command completion based on telemetry and health evidence;
- planner-friendly trajectory input formats;
- real-drone profile configuration for validation and timeout policy;
- JSONL report persistence for audit and test diagnosis;
- optional production transport security.

This makes it possible to test cloud-side algorithms and mission logic against real communication patterns, not only abstract simulations.

## Relation to Deterministic Rotor-Router Coordination

Rotor-router coordination and graph traversal are higher-level mission mechanisms. They decide which graph node, edge, role, or obligation should be assigned next. SwarmKit does not implement that algorithm as a fixed policy. Instead, it provides the command and feedback primitives that allow such an algorithm to control real drones.

A rotor-router controller can use SwarmKit as follows:

1. compute the next graph node or movement target;
2. send an active goal or goto command through the client SDK;
3. subscribe to telemetry and reports;
4. detect reached, drifting, failed, or cancelled states;
5. update the rotor state and choose the next target;
6. send a correction or new goal if needed.

This design keeps the rotor-router logic independent, testable, and replaceable, while still allowing it to operate on real UAVs.

## Relation to Drone Shows and Synchronized Movement

For synchronized movement tasks, SwarmKit provides timed trajectory primitives rather than hard-coded application logic. A higher-level planner can generate a trajectory per drone, validate it against vehicle profile limits, prepare execution, start all drones at a synchronized timestamp, and subscribe to trajectory reports.

The generic primitives are:

- trajectory points;
- optional velocities;
- yaw targets;
- timestamped payload actions;
- validation policies;
- execution handles;
- report streams;
- abort, pause, and resume operations.

This allows show developers to implement choreography, collision checks, LED timelines, or music synchronization above the SDK without forcing those concepts into the core library.

## Relation to Dynamic Swarm Controllers

Dynamic controllers, including role-changing or adaptive controllers, can use SwarmKit without pre-uploading a full mission. They can stream commands or active goals as the mission evolves.

Example controller loop:

```text
observe telemetry and reports
compute next desired state
send command, active goal, or trajectory segment
wait for reports
correct, cancel, or continue
```

This supports use cases where the mission plan is generated online, such as dynamic coverage, adaptive search, target following, formation changes, or decentralized role reassignment.

## Security and Deployment Modes

The project supports several transport modes:

- `insecure` for local SITL and isolated lab testing;
- `tls` for encrypted server-authenticated transport;
- `mtls` for production mutual authentication;
- `auto` for certificate-path-based mode inference.

In a scientific platform, this enables the same SDK to be used in both laboratory simulation experiments and production-like deployments. For cloud-connected or multi-user operations, mTLS is the recommended mode because it allows the agent to identify clients and enforce allowed identities.

## Suggested Wording for a Paper Implementation Section

The following paragraph can be adapted into an implementation section:

> The UAV-side execution layer was implemented using SwarmKit, a C++23 gRPC-based SDK and agent runtime developed for this platform. Each UAV runs a SwarmKit agent on a companion computer. The agent exposes typed APIs for command execution, telemetry streaming, authority arbitration, active-goal supervision, timed trajectory execution, capability discovery, and event reports. Vehicle-specific communication is isolated behind a backend interface; in the current implementation, the MAVLink backend communicates with ArduPilot SITL and Pixhawk-compatible systems through configurable UDP endpoints or MAVLink Router. Higher-level cloud services and planner applications use the SwarmKit client SDK to interact with agents, allowing deterministic coordination algorithms and supervisory logic to remain independent from MAVLink protocol details. The same interface is used for simulation, SITL validation, and real-drone deployment.

## Suggested Wording for an Architecture Section

The following paragraph can be adapted into an architecture section:

> SwarmKit forms the middleware boundary between the cloud-native mission platform and the UAV flight-control layer. The cloud platform communicates with each drone through a drone gateway or client service that uses the SwarmKit SDK. On the vehicle side, the SwarmKit agent receives commands and trajectories, enforces authority rules, translates requests to the selected backend, and streams normalized telemetry and typed reports back to the platform. This design decouples mission-level logic from flight-controller protocol details and permits the same cloud services to operate against simulated vehicles, SITL instances, and real Pixhawk-based drones.

## Current Implementation Status

Current implemented capabilities include:

- C++23 SDK and agent runtime;
- gRPC service contracts;
- optional `insecure`, `tls`, and `mtls` transport security;
- simulated backend;
- MAVLink backend using generated MAVLink C headers;
- MAVLink SITL validation with three ArduCopter instances;
- command ACK handling;
- goto fallback from `MAV_CMD_DO_REPOSITION` to `SET_POSITION_TARGET_GLOBAL_INT`;
- MAVLink GPS quality, HDOP, source timestamp, home-origin, relative-altitude, landed-state, estimator, accuracy, and selected failsafe telemetry decoding with explicit field validity;
- telemetry subscription;
- swarm client fanout;
- verified command helpers in the client SDK;
- idempotent command preconditions for already-armed, already-airborne, and already-landed states;
- command priorities and authority locks;
- active goal reports;
- timed trajectory execution;
- JSONL, CSV, and YAML trajectory loading;
- vehicle profile configuration for real-drone validation policy;
- agent-side JSONL report persistence;
- CLI swarm partial-result policies;
- CLI reference tool.

Important ongoing work:

- reconnect/resume semantics using persisted report sequence state;
- backend extensions for future vehicle protocols.
- optional policy plugins for correction, retry, and abort decisions.

## Recommended Citation-Level Interpretation

In a scientific contribution, SwarmKit should not be presented as the whole cloud platform or the whole swarm algorithm. It should be presented as the implementation substrate that enables the platform to control and observe real UAV agents.

Recommended phrasing:

- "SwarmKit is the UAV-side agent and SDK layer of the proposed platform."
- "The deterministic coordination and cloud supervision components use SwarmKit to communicate with vehicle agents."
- "SwarmKit abstracts MAVLink and future vehicle backends behind a common command, telemetry, goal, trajectory, and report API."
- "The CLI is a reference client for testing and demonstrating SDK usage, not the primary planner."

This framing keeps the paper clean: rotor-router logic, cloud-native mission architecture, and analytics remain the scientific platform contributions, while SwarmKit is the concrete middleware used to realize those contributions on simulated and real drones.
