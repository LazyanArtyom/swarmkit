# Real-drone validation

This procedure is the next gate after simulator and SITL work. It is a staged
engineering validation, not an airworthiness certification. Use the vehicle
manufacturer's safety procedure, a trained operator, a physical kill method,
and an appropriate test area.

For the first propeller-off hardware bring-up, start from the installed
`agent-mavlink-lab.example.yaml` and `client-lab.example.yaml`. They use
insecure gRPC on loopback only; this is acceptable for the isolated lab gate,
not for deployment or field flight. Replace every `CHANGE-ME` value, keep
flight termination disabled, and point the Agent at the MAVLink Router loopback
endpoint on the companion computer.

Run the CLI on the companion computer or forward the loopback gRPC port over
SSH from the operator computer:

```bash
ssh -N -L 50061:127.0.0.1:50061 USER@COMPANION_ADDRESS
```

Do not expose an insecure Agent on `0.0.0.0`, shared Wi-Fi, or a routed network.
The first controlled single-drone flight may keep this loopback-plus-SSH setup.
Move to `agent-mavlink.example.yaml` and `client.example.yaml` with
deployment-specific mTLS credentials before multi-drone or normal field
operation. Never reuse the repository test certificates.

## Gate 1: configuration review

Before powering motors:

1. Confirm the Agent ID, drone ID, MAVLink target system/component, source IDs,
   autopilot profile, GUIDED mode number, and UDP routing against the exact
   vehicle configuration.
2. Set conservative, vehicle-specific speed, climb, descent, altitude, battery,
   GPS, and HDOP limits. Do not copy the example limits without review.
3. Verify persistent evidence/log storage capacity, time synchronization,
   network isolation, and service-manager shutdown. For the later mTLS gate,
   also verify certificate identity, validity, and file permissions.
4. Keep `allow_unsafe_bench_commands` and `allow_flight_termination` false.
5. Arrange an independent RC/operator takeover and motor-kill procedure.

The production MAVLink backend rejects raw `backend-command` passthrough. Add a
typed command with explicit validation and capability reporting when a new
vehicle operation is required; do not reintroduce arbitrary `COMMAND_LONG`.

The Agent now rejects malformed or unknown MAVLink YAML values. Treat any
startup configuration error as a blocked validation rather than changing a
value until the process starts.

## Gate 2: powered, propellers removed

Start the Agent and collect baseline state without acquiring command authority:

```bash
swarmkit-agent --config /etc/swarmkit/agent-lab.yaml \
  --log-sink both --log-file /var/log/swarmkit/agent.log --log-level info

swarmkit-cli --config /etc/swarmkit/client-lab.yaml ping
swarmkit-cli --config /etc/swarmkit/client-lab.yaml capabilities
swarmkit-cli --config /etc/swarmkit/client-lab.yaml health
swarmkit-cli --config /etc/swarmkit/client-lab.yaml stats
swarmkit-cli --config /etc/swarmkit/client-lab.yaml telemetry \
  --drone CHANGE-ME-drone-1 --rate 10 --duration-ms 30000 \
  --telemetry-file /var/lib/swarmkit/validation/prop-off.csv
swarmkit-cli --config /etc/swarmkit/client-lab.yaml preflight \
  --drone CHANGE-ME-drone-1 --duration-ms 10000 --min-battery 30
```

Do not continue unless the observed system/component identity, mode, armed and
landed state, failsafe state, GPS, estimator state, battery, heartbeat age, and
telemetry age all match the physical vehicle. Unknown evidence is a failure.

With the frame restrained and propellers still removed, verify authority lock,
normal arm/disarm ACKs, physical arm-state transitions, authority release, and
emergency force-disarm. A command ACK alone is not proof of physical state.

## Gate 3: restrained propulsion

Install propellers only under the approved restraint procedure. Keep gRPC on
loopback through the restrained gate, use normal operator priority, and verify:

- loss of client connection does not leave unintended setpoints active;
- RC/operator takeover works at every point;
- failsafe, heartbeat loss, and stale telemetry prevent autonomous commands;
- arm/disarm and mode changes agree in Agent health, telemetry, autopilot UI,
  and the physical vehicle;
- Agent shutdown is clean and the evidence file passes
  `swarmkit-evidence-inspect`.

Any identity mismatch, stale/unknown state, unexpected motor response, recorder
failure, or session change ends the run.

## Gate 4: single-vehicle low-altitude flight

For the first controlled flight, the insecure Agent must remain loopback-only
and reachable through the SSH tunnel. Use a large controlled area, conservative
geofence and autopilot limits, good GPS, and a dedicated safety pilot. Validate
one vehicle before any swarm run:

1. Preflight and record a fresh Agent session ID.
2. Acquire authority, arm, and confirm physical armed state.
3. Take off to the minimum safe validation altitude with `--verify`.
4. Exercise hold, a short low-speed velocity pulse, and a nearby waypoint.
5. Land with `--verify`, confirm landed state, then disarm.
6. Preserve logs, configuration, firmware versions, telemetry, and execution
   evidence under a unique run ID.

Stop immediately on state disagreement, excessive tracking error, estimator or
GPS degradation, authority anomalies, delayed commands, or loss of the
independent safety channel. Swarm testing begins only after repeated clean
single-vehicle runs with an explicit review of the collected evidence.
