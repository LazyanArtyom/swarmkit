# Introduction

SwarmKit is a C++23 UAV control and telemetry library with three related
surfaces:

- `swarmkit-agent`, a gRPC daemon that runs beside a vehicle or simulator.
- `swarmkit-cli`, a developer/operator tool for health checks, telemetry,
  commands, goals, reports, messages, artifacts, and swarm workflows.
- The SwarmKit SDK, installed as static libraries and public headers for
  embedding the same client and agent building blocks into C++ applications.

The project is intended for ground-control software, robotics integration
tools, and companion-computer experiments that need a typed command model,
structured telemetry, command authority arbitration, and a consistent gRPC
transport boundary.

SwarmKit currently includes a built-in simulator backend and a direct MAVLink
UDP backend for ArduPilot SITL or companion-computer style deployments. The
agent backend interface is public, so additional vehicle integrations can be
registered without changing client application code.

## What SwarmKit Solves

SwarmKit separates application control logic from vehicle-specific protocol
details:

- Application code sends typed C++ command variants such as `CmdArm`,
  `CmdTakeoff`, `CmdGoto`, and `CmdVelocity`.
- The agent validates command authority and readiness before forwarding work to
  a backend.
- Backends translate commands into simulator behavior, MAVLink traffic, or a
  custom vehicle integration.
- Telemetry is normalized into `swarmkit::core::TelemetryFrame`, including
  validity flags so callers do not confuse missing measurements with zero.
- Fleet code can use `swarmkit::client::SwarmClient` to route commands to one
  drone, broadcast to all configured drones, and subscribe to all telemetry.

Use SwarmKit when you want a C++ SDK and tools around a gRPC agent model rather
than linking application code directly to a specific flight-controller protocol.

## Current Scope

Implemented in this repository:

- gRPC API and generated SDK protocol code.
- Static libraries: `swarmkit_core`, `swarmkit_proto`, `swarmkit_client`, and
  `swarmkit_agent`.
- Public CMake package exports with targets such as `swarmkit::client`.
- Agent RPCs for ping, health, runtime stats, capabilities, command dispatch,
  telemetry streaming, authority, reports, messages, peers, and artifacts.
- Single-agent `Client` and fleet-level `SwarmClient`.
- Command authority arbitration with priority and TTL support.
- Active goals and report streams.
- Typed data-plane messages and chunked artifact transfer APIs.
- Built-in simulator and direct MAVLink UDP backends.

SwarmKit is still evolving. Treat the simulator and MAVLink SITL paths as the
best-tested runtime flows, and review backend-specific behavior before using a
new integration near real hardware.
