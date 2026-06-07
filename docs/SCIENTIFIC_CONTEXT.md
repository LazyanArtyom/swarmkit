# SwarmKit Scientific Context

Last updated: 2026-06-07

## Purpose

SwarmKit is the UAV-side SDK and agent runtime for experiments that need to control and observe real or simulated drones without coupling higher-level swarm algorithms to MAVLink details.

It is intentionally algorithm-agnostic. Rotor-router coordination, gossip/event logs, mission planning, analytics, cloud supervision, and user collaboration belong in applications built on top of SwarmKit.

## Paper-Ready Description

SwarmKit is a C++23 gRPC-based UAV agent and client SDK. Each drone runs a SwarmKit agent on a companion computer. The agent exposes typed APIs for command dispatch, authority arbitration, active-goal supervision, telemetry streaming, report streaming, data messages, binary artifact transfer, health checks, and capability discovery. Vehicle communication is isolated behind a backend interface; the current production backend uses MAVLink for ArduPilot SITL and Pixhawk-compatible systems.

The SDK lets cloud gateways, local onboard controllers, operator tools, and experimental swarm algorithms communicate with drone agents through a stable interface. This keeps scientific work on coordination algorithms separate from flight-controller protocol mechanics.

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
| Gossip/data substrate | Generic message plane |
| Image/log/map exchange | Generic artifact plane |
| Secure communication | Runtime `insecure`, `tls`, and `mtls` modes |

## Architectural Boundary

```text
Cloud / Planner / Gateway / CLI / Onboard Algorithm
        |
        | SwarmKit Client SDK, gRPC
        v
SwarmKit Agent on Companion Computer
        |
        | Backend abstraction
        v
MAVLink Router / Pixhawk / SITL / Future Backend
```

The high-level platform can implement rotor-router planning, gossip dissemination, search logic, inspection logic, and supervision without knowing MAVLink message formats. The agent can change from MAVLink to another backend without forcing the platform to rewrite its algorithm layer.

## Rotor-Router Usage

A rotor-router controller should treat SwarmKit as the control and observation substrate:

1. Compute the next graph node or movement target.
2. Send an active goal or direct command through the SDK.
3. Subscribe to telemetry and reports.
4. Wait for `GOAL_REACHED`, deviation, failure, or cancellation.
5. Update the rotor state and application event log.
6. Publish gossip/data messages as application-defined topics.
7. Send the next goal or correction.

SwarmKit does not know graph nodes, edges, roles, target ownership, or gossip semantics. It only transports labeled messages, artifacts, commands, goals, telemetry, and reports.

## Dynamic Controllers

Dynamic controllers should prefer active goals and command streams over static uploaded plans. This keeps online decisions in the application:

```text
observe telemetry and reports
compute next desired state
send active goal or command
publish small labeled data messages
transfer images/logs/maps as artifacts when needed
update application state
repeat
```

This supports adaptive search, target following, decentralized role reassignment, graph traversal, and cloud-assisted supervision.

## Current Implementation Status

Implemented capabilities include:

- C++23 SDK and agent runtime;
- gRPC control and data services;
- optional `insecure`, `tls`, and `mtls` transport security;
- simulated backend;
- MAVLink backend using generated MAVLink C headers;
- command ACK handling and STATUSTEXT surfacing;
- telemetry streaming with explicit validity fields;
- GPS, EKF, landed-state, failsafe, home-origin, accuracy, and source-time decoding when available;
- command priorities and authority locks;
- verified command helpers;
- active goal set/cancel/get and goal reports;
- swarm fanout for commands, goals, health, stats, telemetry, and authority;
- generic message plane for small labeled payloads;
- generic artifact plane for chunked binary transfer with hashing, TTL, direct peer routing, progress, cancellation, and peer status;
- agent-side JSONL report persistence and replay cursors;
- CLI reference tool for smoke testing SDK workflows.

Removed/non-goal core features:

- static mission upload;
- timed trajectory upload/execution;
- implicit multi-hop data forwarding without an explicit routing policy;
- drone-show choreography;
- rotor-router graph logic;
- gossip merge semantics;
- image analytics.

Those belong in higher-level applications when they are actually needed.

## Recommended Citation-Level Interpretation

Use this framing:

- "SwarmKit is the UAV-side agent and SDK layer of the proposed platform."
- "The deterministic coordination and cloud supervision components use SwarmKit to communicate with vehicle agents."
- "SwarmKit abstracts MAVLink and future vehicle backends behind common command, telemetry, goal, report, message, and artifact APIs."
- "Rotor-router and gossip are implemented above SwarmKit, not inside it."
