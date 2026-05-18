# SwarmKit Project Memory

Last updated: 2026-05-16

## Current Shape

- SwarmKit is a C++23 UAV swarm SDK and runtime layer.
- `swarmkit-agent` is a gRPC daemon intended to run beside a vehicle, usually on a Raspberry Pi companion computer.
- Clients connect to the agent over configurable transport security:
  - `insecure` for SITL/local lab testing;
  - `tls` for encrypted server-authenticated transport;
  - `mtls` for production mutual authentication;
  - `auto` to infer from configured certificate paths.
- The agent has an `IDroneBackend` boundary. Current backends include simulation and MAVLink.
- `swarmkit-cli` is a reference/smoke-test client, not the primary planner product.
- Test tooling can launch three agents and a client/server workflow:
  - `swarmkit-test-agents` starts three local agents.
  - `swarmkit-test-client` sends rotating low-priority commands.
  - `swarmkit-test-server` subscribes to telemetry and can send interactive high-priority commands.
- The SDK includes client support for telemetry, commands, reports, authority, active goals, timed trajectories, and swarm fanout.
- The SDK now has a real client-side swarm manager path:
  - logical role and formation-slot commands are consumed by `SwarmClient` and are not forwarded to Pixhawk/MAVLink backends;
  - formation plans are translated into per-drone `goto` commands;
  - planner-produced per-drone command plans can be executed through the same manager path;
  - synchronized swarm command execution uses a lock-step client barrier by default;
  - partial-failure policies include all-or-abort, best-effort, quorum, land-failed, and hold-failed.
- Trajectory file/stream loading supports YAML, JSONL/NDJSON, and CSV through the client SDK.
- The SDK owns verified command helpers, so planner tools and the CLI can use one shared implementation:
  - `SendCommandAndWait`
  - `ArmAndWait`
  - `TakeoffAndWait`
  - `GotoAndWait`
  - `LandAndWait`
  - swarm `BroadcastCommandAndWait`
- The CLI is a thin reference for those SDK APIs. It supports `--verify`, `--timeout-ms`, and swarm partial-failure policy flags such as `--continue-on-error` and `--require-all`; idempotent already-satisfied replies count as successful outcomes.

## MAVLink / SITL Goal

The MAVLink backend connects `IDroneBackend` to real MAVLink traffic instead of synthetic telemetry.

Current development setup:

- SITL runs on another machine.
- It sends MAVLink UDP traffic to the development host on three ports:
  - `udp://0.0.0.0:14601`, MAVLink sysid `1`
  - `udp://0.0.0.0:14602`, MAVLink sysid `2`
  - `udp://0.0.0.0:14603`, MAVLink sysid `3`
- One SwarmKit agent can be run per drone:
  - `drone-1`: gRPC `50061`, MAVLink UDP `14601`, sysid `1`
  - `drone-2`: gRPC `50062`, MAVLink UDP `14602`, sysid `2`
  - `drone-3`: gRPC `50063`, MAVLink UDP `14603`, sysid `3`
- On the Raspberry Pi, the agent should listen locally to data from MAVLink Router / Pixhawk, so IP/port must remain configuration, not hardcoded.

## Preferred Direction

- Use the MAVLink C generated headers (`c_library_v2`) for parsing and packing MAVLink messages.
- `MavlinkBackend` implements `IDroneBackend` and listens on configurable MAVLink UDP.
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
- Goto supports the important ArduPilot fallback:
  - try `MAV_CMD_DO_REPOSITION`;
  - if unsupported, fall back to `SET_POSITION_TARGET_GLOBAL_INT`.
- Timed trajectory execution now waits for telemetry to prove final target reach before reporting completion.
- Validation `ok` now reflects actual error issues.
- MAVLink health decoding now feeds real validation policy:
  - GPS fix type and normalized quality, visible satellites, HDOP, relative altitude, landed state, estimator status, selected accuracy data, source timestamps, home origin, and failsafe-ish indicators are decoded when present.
  - Telemetry frames now include explicit validity flags so production clients can distinguish real zero values from unknown measurements.
  - Agent-side preconditions treat idempotent command states cleanly, including already armed, already airborne for takeoff, already landed for land, and safe disarm checks before normal disarm.
- Agent reports can be persisted as rotated JSONL through `report_log_file`,
  `report_persistence.log_file`, or `--report-log-file`; the agent persists report sequence
  state so clients can reconnect with `after_sequence` and replay retained reports.
- Vehicle profile configuration is becoming the source of real-drone policy, including cruise/climb/descent limits, altitude limits, tracking tolerance, battery reserve, GPS fix/satellite/HDOP requirements, and takeoff/land timeout margins.

## MAVLink Configs

- `testdata/agent_mavlink_drone1.yaml` selects `backend: "mavlink"`.
- It binds the agent gRPC API on `0.0.0.0:50061`.
- It binds MAVLink UDP on `0.0.0.0:14601`.
- It targets MAVLink `sysid=1`, `compid=1`.
- The testdata configs explicitly use `transport_security: "mtls"`.
- For local SITL without certs, use `--insecure` on both agent and CLI.
- Current production/test examples:
  - `testdata/agent_mavlink_drone_1.yaml`
  - `testdata/swarm_sitl_config.yaml`
  - `testdata/pixhawk_companion_config.yaml`

## Scientific Platform Context

- SwarmKit is the UAV-side SDK and agent layer for the broader cloud-native, multi-user UAV swarm platform.
- Higher-level systems such as rotor-router controllers, cloud drone gateways, mission supervisors, data analytics, and drone-show tools should use the SwarmKit client SDK to communicate with agents.
- SwarmKit should stay algorithm-agnostic. It should not hard-code rotor-router, drone-show, or patrol logic.
- The right abstraction is: commands, telemetry, authority, reports, active goals, trajectories, backend capabilities, and backend-specific extension commands.
- See:
  - `docs/SWARMKIT_PROJECT_DESCRIPTION.md`
  - `docs/SCIENTIFIC_CONTEXT.md`
