# SwarmKit

Multi-agent UAV swarm control and telemetry platform.

- **Agent** (`swarmkit-agent`) — gRPC daemon that runs on a Raspberry Pi alongside the vehicle. Accepts commands, streams telemetry, and routes to a pluggable backend.
- **CLI** (`swarmkit-cli`) — developer tool to ping the agent, subscribe to live telemetry, and send commands.
- **SDK** — static libraries + public headers for embedding SwarmKit into other projects (no Conan required in the consumer).

The default agent backend is a built-in simulator. For ArduPilot SITL and
future companion-computer deployments, the agent can also use the official
MAVLink C headers vendored under `third_party/mavlink_c_library_v2` and talk
directly to MAVLink UDP traffic.

Telemetry is exposed as a typed vehicle-state model with explicit validity
flags, source timestamps, coordinate-frame metadata, home origin, GPS quality,
estimator state, accuracy fields, and command/goal linkage for monitoring and
3D swarm applications.

Production hardening is tracked in the
[Phase 0 production-readiness milestone](docs/source/production_readiness.md). A green software
test matrix is not an airworthiness certification.

---

## Prerequisites

| Tool | macOS ARM64 | Linux x86\_64 |
|------|-------------|----------------|
| Compiler | Xcode 26+ CLT (`xcode-select --install`) | GCC 13+ or Clang 17+ |
| CMake | ≥ 3.28 | ≥ 3.28 |
| Ninja | `brew install ninja` | `apt install ninja-build` |
| Conan 2 | `pip install conan` | `pip install conan` |
| Python | any recent | any recent |

Initialize Conan profile once after installation:

```bash
conan profile detect
```

---

## Build from source

### macOS ARM64

```bash
# Debug
conan install . -of build/conan -s build_type=Debug -s compiler.cppstd=23 \
  -o '&:with_tools=True' -o '&:with_tests=True' --build=missing
cmake --preset mac-debug
cmake --build --preset mac-debug

# Release
conan install . -of build/conan -s build_type=Release -s compiler.cppstd=23 \
  -o '&:with_tools=True' -o '&:with_tests=True' --build=missing
cmake --preset mac-release
cmake --build --preset mac-release
```

### Linux x86\_64

```bash
# Debug
conan install . -of build/conan -s build_type=Debug -s compiler.cppstd=23 \
  -o '&:with_tools=True' -o '&:with_tests=True' --build=missing
cmake --preset linux-debug
cmake --build --preset linux-debug

# Release
conan install . -of build/conan -s build_type=Release -s compiler.cppstd=23 \
  -o '&:with_tools=True' -o '&:with_tests=True' --build=missing
cmake --preset linux-release
cmake --build --preset linux-release
```

## Run

SwarmKit supports `auto`, `insecure`, `tls`, and `mtls` transport modes for
client/agent communication. Empty security config resolves to `insecure`, while
the repo test configs explicitly use development-only mTLS certificates under
`testdata/certs/`.

For local SITL experiments you can skip certificates:

```bash
./build/mac-debug/apps/swarmkit-agent --insecure --id agent-1 --bind 0.0.0.0:50061
./build/mac-debug/apps/swarmkit-cli --insecure 127.0.0.1:50061 ping
```

**Terminal 1 — start the agent:**

```bash
# macOS / Linux (debug build, using repo test certs)
./build/mac-debug/apps/swarmkit-agent \
    --config testdata/agent_config.yaml \
    --id agent-1 \
    --bind 0.0.0.0:50061 \
    --log-level info
```

Default bind address: `0.0.0.0:50061`.

### MAVLink SITL backend

For the first SITL drone, use the dedicated config:

```bash
./build/mac-debug/apps/swarmkit-agent \
    --config testdata/agent_mavlink_drone1.yaml \
    --log-level debug
```

That config starts the gRPC agent on `0.0.0.0:50061` and listens for MAVLink on
`udp://0.0.0.0:14601`, targeting MAVLink `sysid=1`, `compid=1`.
For a companion-computer deployment, keep the same backend and point
`mavlink.bind_addr` at the UDP endpoint exposed by MAVLink Router, usually a
localhost address on the Raspberry Pi.

The same settings can be overridden from the command line:

```bash
./build/mac-debug/apps/swarmkit-agent \
    --config testdata/agent_config.yaml \
    --backend mavlink \
    --mavlink-drone drone-1 \
    --mavlink-autopilot ardupilot-copter \
    --mavlink-bind 0.0.0.0:14601 \
    --mavlink-target-system 1 \
    --mavlink-target-component 1 \
    --log-level debug
```

Once SITL heartbeats are arriving, subscribe to telemetry:

```bash
./build/mac-debug/apps/swarmkit-cli --config testdata/client_config.yaml \
    127.0.0.1:50061 telemetry --drone drone-1 --rate 2
```

Basic command mapping is implemented through MAVLink:

- `arm` / `disarm` use `MAV_CMD_COMPONENT_ARM_DISARM`
- `arm` can optionally switch to a configured mode first; the ArduCopter test
  profile uses GUIDED (`custom_mode=4`)
- `takeoff` can optionally switch to a configured mode first, then uses
  `MAV_CMD_NAV_TAKEOFF`
- `land` uses `MAV_CMD_NAV_LAND`
- `return_home` uses `MAV_CMD_NAV_RETURN_TO_LAUNCH`
- `hold` uses `MAV_CMD_NAV_LOITER_UNLIM`
- `waypoint` uses `SET_POSITION_TARGET_GLOBAL_INT`
- `goto` first tries `MAV_CMD_DO_REPOSITION` and falls back to
  `SET_POSITION_TARGET_GLOBAL_INT` when the autopilot reports reposition as
  unsupported.

`COMMAND_LONG`-based commands wait for `COMMAND_ACK`; the CLI reports the
autopilot ACK result instead of only confirming that a UDP packet was sent.
ACK target fields are matched when present, while older/minimal ACK payloads
that omit those extension fields are still accepted.

For production-style automation, prefer CLI sequence files with telemetry
verification instead of fixed sleeps only:

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
  - broadcast: true
    args: [disarm]
    retries: 10
    retry_delay_ms: 2000
```

**Terminal 2 — use the CLI:**

```bash
# Ping
./build/mac-debug/apps/swarmkit-cli --config testdata/client_config.yaml ping
./build/mac-debug/apps/swarmkit-cli --config testdata/client_config.yaml 192.168.1.10:50061 ping

# Subscribe to telemetry (default drone, 1 Hz) — Ctrl+C to stop
./build/mac-debug/apps/swarmkit-cli --config testdata/client_config.yaml telemetry

# Custom drone and rate
./build/mac-debug/apps/swarmkit-cli --config testdata/client_config.yaml telemetry --drone uav-1 --rate 5

# Custom address + drone + rate
./build/mac-debug/apps/swarmkit-cli --config testdata/client_config.yaml 192.168.1.10:50061 telemetry --drone uav-1 --rate 2

# Send commands
./build/mac-debug/apps/swarmkit-cli --config testdata/client_config.yaml command --drone uav-1 arm
./build/mac-debug/apps/swarmkit-cli --config testdata/client_config.yaml command --drone uav-1 takeoff --alt 20
./build/mac-debug/apps/swarmkit-cli --config testdata/client_config.yaml 192.168.1.10:50061 command --drone uav-1 waypoint --lat 40.18 --lon 44.51 --alt 50
```

---

## Tests

```bash
#macOS
ctest --preset mac-release --output-on-failure

#Linux
ctest --preset linux-release --output-on-failure

```

---

## Documentation

The documentation site is built with Doxygen, Sphinx, Breathe, MyST, and Furo:

```bash
cd docs
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
make html
open build/html/index.html
```

Install Doxygen first (`brew install doxygen` on macOS, or
`sudo apt-get install doxygen graphviz` on Ubuntu/Debian). The full build notes
live in `docs/README.md`.

---

## Package (CI / release)

The package script runs the full pipeline: Conan install → CMake configure → build → test → produce
SDK and tools tarballs plus SHA-256 sidecars in `dist/`.
Package filenames use the current project version from the root `VERSION` file.
When `--preset` and `--platform` are omitted, `scripts/ci_package.sh`
auto-detects the current supported host platform:

```bash
chmod +x scripts/ci_package.sh
./scripts/ci_package.sh
```

On macOS ARM64 this produces:
```
dist/swarmkit-<version>-sdk-mac-arm64.tar.gz
dist/swarmkit-<version>-sdk-mac-arm64.tar.gz.sha256
dist/swarmkit-<version>-tools-mac-arm64.tar.gz
dist/swarmkit-<version>-tools-mac-arm64.tar.gz.sha256
```

On Linux x86\_64 this produces:
```
dist/swarmkit-<version>-sdk-linux-x86_64.tar.gz
dist/swarmkit-<version>-sdk-linux-x86_64.tar.gz.sha256
dist/swarmkit-<version>-tools-linux-x86_64.tar.gz
dist/swarmkit-<version>-tools-linux-x86_64.tar.gz.sha256
```

**In VSCode** press **F8** to run the full package pipeline for the current platform.

For explicit CI jobs, pass the preset and platform:

```bash
./scripts/ci_package.sh --preset mac-release --platform mac-arm64
./scripts/ci_package.sh --preset linux-release --platform linux-x86_64
```

---

## Tools package

The tools package contains three statically-linked binaries:

```
swarmkit-<version>-tools-<platform>/
└── bin/
    ├── swarmkit-agent
    ├── swarmkit-cli
    └── swarmkit-evidence-inspect
```

They depend only on system libraries and run on any machine of the same OS and architecture without additional dependencies.

---

## SDK installation

Deploy the SDK to a location of your choice, e.g. `~/swarmkit-sdk`:

```bash
# Auto-detects the newest SDK tarball for the current supported host platform
./scripts/deploy_sdk.sh --prefix ~/swarmkit-sdk

# Or choose a platform explicitly
./scripts/deploy_sdk.sh --platform mac-arm64 --prefix ~/swarmkit-sdk
./scripts/deploy_sdk.sh --platform linux-x86_64 --prefix ~/swarmkit-sdk

# Or pass an explicit tarball
./scripts/deploy_sdk.sh --prefix ~/swarmkit-sdk dist/swarmkit-<version>-sdk-mac-arm64.tar.gz
```

SDK layout:

```
<sdk-root>/
├── include/swarmkit/          # public headers
│   ├── core/                  # Logger, Result, TelemetryFrame, version
│   ├── agent/                 # IDroneBackend, CommandArbiter, server, SimBackend
│   ├── client/                # Client, SwarmClient, subscriptions, command results
│   ├── evidence/              # Strict execution-log reader and ordered replay
│   ├── experiment/            # Scripted/fault backends, manual runtime, normalized replay
│   ├── commands/              # flight/nav/swarm/payload command categories
│   └── commands.h             # aggregate Command, CommandContext, CommandEnvelope
├── lib/
│   ├── libswarmkit_core.a
│   ├── libswarmkit_agent.a
│   ├── libswarmkit_client.a
│   ├── libswarmkit_evidence.a
│   ├── libswarmkit_experiment.a
│   ├── libswarmkit_proto.a
│   └── cmake/SwarmKit/
│       ├── SwarmKitConfig.cmake
│       ├── SwarmKitConfigVersion.cmake
│       ├── SwarmKitTargets.cmake
│       └── deps/              # bundled Conan CMakeDeps find files (relocatable)
└── third_party/full_deploy/   # Conan dependencies (headers + static libs)
```

---

## Verify the SDK package with test tools

`apps/test_tools/` is a standalone SDK consumer project. It intentionally builds
outside the main SwarmKit build and links only through the installed package via
`find_package(SwarmKit REQUIRED)`. It does not use Conan directly.

It builds two binaries:

| Binary | Purpose |
|--------|---------|
| `swarmkit-sdk-ping` | Creates a `swarmkit::client::Client` and calls `Ping()` on an agent. |
| `swarmkit-sdk-control` | Interactive SDK shell for commands, telemetry, reports, authority, goals, messages, artifacts, and raw-key flight control. |

Build the tools against a deployed SDK:

```bash
cmake -S apps/test_tools \
    -B /tmp/swarmkit-sdk-probe-build \
    -DCMAKE_PREFIX_PATH=~/swarmkit-sdk \
    -DCMAKE_BUILD_TYPE=Release

cmake --build /tmp/swarmkit-sdk-probe-build
```

Run it against a local insecure agent:

```bash
# Terminal 1
./build/mac-release/apps/swarmkit-agent \
    --insecure \
    --id sdk-probe-agent \
    --bind 127.0.0.1:50061

# Terminal 2
/tmp/swarmkit-sdk-probe-build/swarmkit-sdk-ping \
    --addr 127.0.0.1:50061 \
    --insecure
```

Expected output:

```text
Ping OK
  agent_id : sdk-probe-agent
  version  : <version>
  time_ms  : <unix-ms>
```

For TLS/mTLS probes, omit `--insecure` and pass `--ca-cert`, `--client-cert`,
`--client-key`, and optionally `--server-name`.

Run the interactive control shell:

```bash
/tmp/swarmkit-sdk-probe-build/swarmkit-sdk-control \
    --addr 127.0.0.1:50061 \
    --drone drone-1 \
    --insecure
```

Inside the shell:

```text
launch 5
telemetry start 5 quiet
controller
```

For a one-command handoff into raw-key control, use `fly 5`. Payload smoke
commands are also exposed, for example `payload photo 0` or
`payload gimbal -20 0 90`.

In controller mode, use arrow keys or `W/S` for forward/back, left/right arrows
or `Q/E` for yaw, `A/D` for strafe, `R/F` for climb/descent, `Space` for stop,
`H` for hold, `1`/`0` for arm/disarm, `T` for takeoff, `L` for land, and `Esc`
to return to the shell.
