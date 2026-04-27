# SwarmKit Project Memory

Last updated: 2026-04-27

## Current Shape

- SwarmKit is a C++23 multi-agent UAV control and telemetry project.
- `swarmkit-agent` is a gRPC daemon intended to run beside a vehicle, eventually on a Raspberry Pi companion computer.
- Clients connect to the agent over mTLS and send commands or subscribe to telemetry.
- The agent already has an `IDroneBackend` boundary. Today the app starts with `MakeSimBackend()`, which accepts commands and generates synthetic telemetry.
- Test tooling can launch three agents and a client/server workflow:
  - `swarmkit-test-agents` starts three local agents.
  - `swarmkit-test-client` sends rotating low-priority commands.
  - `swarmkit-test-server` subscribes to telemetry and can send interactive high-priority commands.

## MAVLink / SITL Goal

The next backend should connect `IDroneBackend` to real MAVLink traffic instead of simulated telemetry.

Current development setup:

- SITL runs on another machine.
- It sends MAVLink UDP traffic to the Mac on three ports:
  - `udp://0.0.0.0:14601`, MAVLink sysid `1`
  - `udp://0.0.0.0:14602`, MAVLink sysid `2`
  - `udp://0.0.0.0:14603`, MAVLink sysid `3`
- First target is one drone only: port `14601`, sysid `1`.
- Later, on the Raspberry Pi, the agent should listen locally to data from MAVLink Router / Pixhawk, so the IP/port should be configuration, not hardcoded.

## Preferred Direction

- Use the MAVLink C generated headers (`c_library_v2`) for parsing and packing MAVLink messages.
- `MavlinkBackend` now implements `IDroneBackend` and listens on configurable MAVLink UDP.
- Keep gRPC, command arbitration, client tooling, and telemetry manager unchanged.
- Translate MAVLink telemetry messages into `core::TelemetryFrame`.
- Translate SwarmKit commands into MAVLink command messages.
- MAVLink `COMMAND_LONG` commands now wait for matching `COMMAND_ACK` and
  return ACK details to clients.
- The MAVLink backend is intended to work both against SITL and against
  MAVLink Router on a companion Raspberry Pi. The UDP bind address and target
  IDs are configuration, not SITL-only code.
- The backend remembers the command-send UDP peer only after parsing traffic
  from the configured target MAVLink system. This avoids accidentally sending
  commands to unrelated UDP senders.
- ArduCopter mode handling is configurable. The current SITL profile switches
  to GUIDED (`custom_mode=4`) before arm/takeoff; this can be disabled or
  changed for other autopilot profiles.

## First MAVLink Config

- `testdata/agent_mavlink_drone1.yaml` selects `backend: "mavlink"`.
- It binds the agent gRPC API on `0.0.0.0:50061`.
- It binds MAVLink UDP on `0.0.0.0:14601`.
- It targets MAVLink `sysid=1`, `compid=1`.
