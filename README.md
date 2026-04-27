# SwarmKit

Multi-agent UAV swarm control and telemetry platform.

- **Agent** (`swarmkit-agent`) — gRPC daemon that runs on a Raspberry Pi alongside the vehicle. Accepts commands, streams telemetry, and routes to a pluggable backend.
- **CLI** (`swarmkit-cli`) — developer tool to ping the agent, subscribe to live telemetry, and send commands.
- **SDK** — static libraries + public headers for embedding SwarmKit into other projects (no Conan required in the consumer).

The default agent backend is a built-in simulator. For ArduPilot/PX4 SITL and
future companion-computer deployments, the agent can also use the official
MAVLink C headers vendored under `third_party/mavlink_c_library_v2` and talk
directly to MAVLink UDP traffic.

---

## Prerequisites

| Tool | macOS ARM64 | Linux x86\_64 | Windows x86\_64 |
|------|-------------|----------------|-----------------|
| Compiler | Xcode CLT (`xcode-select --install`) | GCC 13+ or Clang 17+ | MSVC 2022 (Desktop C++ workload) |
| CMake | ≥ 3.28 | ≥ 3.28 | ≥ 3.28 |
| Ninja | `brew install ninja` | `apt install ninja-build` | bundled with VS or `winget install Ninja-build.Ninja` |
| Conan 2 | `pip install conan` | `pip install conan` | `pip install conan` |
| Python | any recent | any recent | any recent |

Initialize Conan profile once after installation:

```bash
conan profile detect
```

---

## Build from source

### macOS ARM64

```bash
# Debug
conan install . -of build/conan -s build_type=Debug -s compiler.cppstd=23 --build=missing
cmake --preset mac-debug
cmake --build --preset mac-debug

# Release
conan install . -of build/conan -s build_type=Release -s compiler.cppstd=23 --build=missing
cmake --preset mac-release
cmake --build --preset mac-release
```

### Linux x86\_64

```bash
# Debug
conan install . -of build/conan -s build_type=Debug -s compiler.cppstd=23 --build=missing
cmake --preset linux-debug
cmake --build --preset linux-debug

# Release
conan install . -of build/conan -s build_type=Release -s compiler.cppstd=23 --build=missing
cmake --preset linux-release
cmake --build --preset linux-release
```

### Windows x86\_64

Run in a **Developer PowerShell for VS 2022** (or any shell where `cl.exe` is on the path):

```powershell
# Debug
conan install . -of build\conan -s build_type=Debug -s compiler.cppstd=23 --build=missing
cmake --preset win-debug
cmake --build --preset win-debug

# Release
conan install . -of build\conan -s build_type=Release -s compiler.cppstd=23 --build=missing
cmake --preset win-release
cmake --build --preset win-release
```

---

## Run

SwarmKit now uses mTLS for client/agent communication. The repo includes development-only
certificates under `testdata/certs/` for local testing.

**Terminal 1 — start the agent:**

```bash
# macOS / Linux (debug build, using repo test certs)
./build/mac-debug/apps/swarmkit-agent \
    --config testdata/agent_config.yaml \
    --id agent-1 \
    --bind 0.0.0.0:50061 \
    --log-level info
```

```powershell
# Windows
.\build\win-debug\apps\swarmkit-agent.exe --config testdata\agent_config.yaml --id agent-1
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

`COMMAND_LONG`-based commands wait for `COMMAND_ACK`; the CLI reports the
autopilot ACK result instead of only confirming that a UDP packet was sent.
ACK target fields are matched when present, while older/minimal ACK payloads
that omit those extension fields are still accepted.

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

#Windows
ctest --preset win-release --output-on-failure
```

---

## Package (CI / release)

Each script runs the full pipeline: Conan install → CMake configure → build → test → produce tarballs/zips in `dist/`.
Package filenames use the current project version from the root `VERSION` file.

### macOS ARM64

```bash
chmod +x scripts/ci_package_mac_arm64.sh
./scripts/ci_package_mac_arm64.sh
```

Produces:
```
dist/swarmkit-<version>-sdk-mac-arm64.tar.gz
dist/swarmkit-<version>-tools-mac-arm64.tar.gz
```

### Linux x86\_64

```bash
chmod +x scripts/ci_package_linux_x86_64.sh
./scripts/ci_package_linux_x86_64.sh
```

Produces:
```
dist/swarmkit-<version>-sdk-linux-x86_64.tar.gz
dist/swarmkit-<version>-tools-linux-x86_64.tar.gz
```

### Windows x86\_64

Run in **Developer PowerShell for VS 2022**:

```powershell
powershell -ExecutionPolicy Bypass -File scripts\ci_package_win_x86_64.ps1
```

Produces:
```
dist\swarmkit-<version>-sdk-win-x86_64.zip
dist\swarmkit-<version>-tools-win-x86_64.zip
```

**In VSCode** press **F8** to run the full package pipeline for the current platform.

---

## Tools package

The tools package contains only the two statically-linked binaries:

```
swarmkit-<version>-tools-<platform>/
└── bin/
    ├── swarmkit-agent       (swarmkit-agent.exe on Windows)
    └── swarmkit-cli         (swarmkit-cli.exe on Windows)
```

They depend only on system libraries and run on any machine of the same OS and architecture without additional dependencies.

---

## SDK installation

Extract the SDK to a location of your choice, e.g. `~/swarmkit-sdk`:

```bash
#macOS
tar xzf swarmkit-<version>-sdk-mac-arm64.tar.gz \
    -C ~/swarmkit-sdk --strip-components=1

#Linux
tar xzf swarmkit-<version>-sdk-linux-x86_64.tar.gz \
    -C ~/swarmkit-sdk --strip-components=1
```

```powershell
#Windows — the top - level folder inside the zip becomes the SDK root
Expand-Archive swarmkit-<version>-sdk-win-x86_64.zip -DestinationPath C:\swarmkit-sdk
#SDK root: the extracted top-level folder inside C:\swarmkit-sdk
```

SDK layout:

```
<sdk-root>/
├── include/swarmkit/          # public headers
│   ├── core/                  # Logger, Result, TelemetryFrame, version
│   ├── agent/                 # IDroneBackend, CommandArbiter, server, SimBackend
│   ├── client/                # Client, SwarmClient, subscriptions, command results
│   ├── commands/              # flight/nav/swarm/payload command categories
│   └── commands.h             # aggregate Command, CommandContext, CommandEnvelope
├── lib/
│   ├── libswarmkit_core.a     # (.lib on Windows)
│   ├── libswarmkit_agent.a
│   ├── libswarmkit_client.a
│   ├── libswarmkit_proto.a
│   └── cmake/SwarmKit/
│       ├── SwarmKitConfig.cmake
│       ├── SwarmKitConfigVersion.cmake
│       ├── SwarmKitTargets.cmake
│       └── deps/              # bundled Conan CMakeDeps find files (relocatable)
└── third_party/full_deploy/   # Conan dependencies (headers + static libs)
```

---

## Build the integration test tools against the SDK

`apps/test_tools/` is a standalone project that links against the installed SDK **without Conan**.
It produces three binaries for end-to-end swarm testing:

| Binary | Purpose |
|--------|---------|
| `swarmkit-test-agents` | Spawns 3 `swarmkit-agent` processes (drone-1…3 on ports 50061–50063) and muxes their output. macOS / Linux only. |
| `swarmkit-test-client` | Connects 3 low-priority (`kOperator`) clients to the agents and sends a rotating command cycle. Demonstrates preemption. |
| `swarmkit-test-server` | Connects a single `SwarmClient` at `kOverride` priority, subscribes to telemetry from all agents, writes frames to a CSV file, and accepts interactive commands from stdin. |

```bash
#macOS / Linux
cmake -B apps/test_tools/build -DCMAKE_PREFIX_PATH=~/swarmkit-sdk -DCMAKE_BUILD_TYPE=Release -S apps/test_tools
cmake --build apps/test_tools/build
```

```powershell
#Windows — adjust the path to match your actual SDK root folder
cmake -B apps\test_tools\build `
      -DCMAKE_PREFIX_PATH="C:\path\to\swarmkit-sdk" `
      -DCMAKE_BUILD_TYPE=Release `
      -GNinja `
      -S apps\test_tools
cmake --build apps\test_tools\build
```

**Typical local workflow (4 terminals):**

```bash
#Terminal 1 — start all 3 agents (uses swarmkit-agent from PATH or explicit path)
./apps/test_tools/build/swarmkit-test-agents ./build/mac-release/apps/swarmkit-agent
#wait for "SwarmKit Test Agents ready" before starting the next terminals

#Terminal 2 — low - priority operator clients(will be preempted)
./apps/test_tools/build/swarmkit-test-client \
  --swarm-config testdata/swarm_config.yaml \
  --address-mode local

#Terminal 3 — high - priority server : telemetry CSV + interactive commands
./apps/test_tools/build/swarmkit-test-server \
  --swarm-config testdata/swarm_config.yaml \
  --address-mode local
#stdin : drone - 1 arm
#stdin : all takeoff 30
#stdin : drone - 2 waypoint 40.18 44.51 50
#stdin : drone - 1 lock
#stdin : drone - 1 unlock

#Terminal 4 — CLI at Supervisor priority(overrides test - client, below test - server)
./build/mac-release/apps/swarmkit-cli --config testdata/client_config.yaml 127.0.0.1:50061 command --drone drone-1 arm
./build/mac-release/apps/swarmkit-cli --config testdata/client_config.yaml 127.0.0.1:50061 command --drone drone-1 takeoff --alt 20
```

`testdata/swarm_config.yaml` contains both remote swarm addresses and localhost
development addresses. For local testing on one machine, use `--address-mode local`
so the tools connect to `127.0.0.1:50061..50063` instead of the example
`192.168.10.x` addresses.
