# Examples

SwarmKit does not currently have a separate `examples/` directory. The practical
examples in this repository are the installed CLI workflows and the SDK probe
programs under `apps/test_tools`.

## CLI: Ping A Local Agent

What it demonstrates:
: Basic client-to-agent connectivity over insecure local transport.

Run an agent:

```bash
./build/mac-debug/apps/swarmkit-agent --insecure --id agent-1 --bind 127.0.0.1:50061
```

Ping it:

```bash
./build/mac-debug/apps/swarmkit-cli --insecure 127.0.0.1:50061 ping
```

Expected result: the CLI prints the agent ID, version, timestamp, and a
successful status.

Common errors:

- `connection refused`: the agent is not running or the address is wrong.
- TLS/mTLS errors: add `--insecure` for local plaintext testing, or use matching
  certificate options.

## CLI: Subscribe To Telemetry

What it demonstrates:
: A background telemetry stream from one drone.

```bash
./build/mac-debug/apps/swarmkit-cli --insecure 127.0.0.1:50061 telemetry --drone default --rate 2
```

Expected result: frames are printed until Ctrl+C. With the simulator backend,
the values are synthetic. With MAVLink, fields depend on the incoming autopilot
messages and their validity flags.

Useful options:

- `--telemetry-file PATH` appends CSV output.
- `--duration-ms MS` stops after a fixed duration.
- `--no-console` suppresses frame printing when logging to a file.

## CLI: Send A Command

What it demonstrates:
: Typed command dispatch through the agent and backend.

```bash
./build/mac-debug/apps/swarmkit-cli --insecure 127.0.0.1:50061 command --drone default arm
./build/mac-debug/apps/swarmkit-cli --insecure 127.0.0.1:50061 command --drone default takeoff --alt 5
```

Expected result: the simulator backend accepts these commands. The MAVLink
backend maps supported commands to MAVLink messages and reports autopilot ACK
results when applicable.

Common errors:

- `rejected`: another client holds authority, the command is invalid for the
  current vehicle state, or backend support is missing.
- `deadline exceeded`: the agent or autopilot did not respond before the client
  deadline.

## CLI: Swarm Broadcast

What it demonstrates:
: Loading a swarm topology and sending one command to all drones.

```bash
./build/mac-debug/apps/swarmkit-cli --insecure \
  swarm --swarm-config testdata/swarm_config.yaml --address-mode local \
  broadcast takeoff --alt 5 --verify --timeout-ms 60000
```

Expected result: the CLI attempts the command against every configured drone.
Use `--continue-on-error` for partial-success workflows or `--require-all` when
every drone must reach the expected state.

## Sequence YAML

What it demonstrates:
: Repeatable command plans with verification and waits.

```yaml
steps:
  - broadcast: true
    args: [takeoff, --alt, 5]
    verify: true
  - drone: drone-1
    args: [goto, --lat, -35.363, --lon, 149.1655, --alt, 5, --speed, 3]
    verify: true
  - drone: drone-1
    wait_position: {lat: -35.363, lon: 149.1655, alt: 5, radius_m: 2}
    timeout_ms: 60000
  - broadcast: true
    args: [land]
    verify: true
```

Run it:

```bash
./build/mac-debug/apps/swarmkit-cli --insecure \
  swarm --swarm-config testdata/swarm_config.yaml --address-mode local \
  sequence --file plan.yaml
```

## SDK Probe: Ping

What it demonstrates:
: Consuming an installed SDK with `find_package(SwarmKit REQUIRED)`.

Build the probe project:

```bash
cmake -S apps/test_tools -B apps/test_tools/build-local \
  -DCMAKE_PREFIX_PATH=/tmp/swarmkit-sdk
cmake --build apps/test_tools/build-local
```

Run:

```bash
./apps/test_tools/build-local/swarmkit-sdk-ping --addr 127.0.0.1:50061 --insecure
```

Expected result:

```text
Ping OK
  agent_id : agent-1
  version  : ...
  time_ms  : ...
```

## SDK Probe: Control Shell

What it demonstrates:
: Interactive SDK control using `Client`, typed commands, telemetry, authority,
  messages, artifacts, and report APIs.

Build with the same `apps/test_tools` commands above, then run:

```bash
./apps/test_tools/build-local/swarmkit-sdk-control --addr 127.0.0.1:50061 --insecure
```

Useful commands inside the shell include:

```text
ping
health
telemetry start 5 quiet
telemetry show
mode guided
arm --wait
takeoff 1 --wait
land
quit
```
