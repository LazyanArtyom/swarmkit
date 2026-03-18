# SwarmKit

Multi-agent UAV swarm control and telemetry platform.

- **Agent** (`swarmkit-agent`) — gRPC daemon that runs on a Raspberry Pi alongside the vehicle. Accepts commands, streams telemetry, transfers files (chunked, CRC32-verified, resumable), and routes to a pluggable backend.
- **CLI** (`swarmkit-cli`) — developer tool to ping the agent and subscribe to live telemetry.
- **SDK** — static libraries + public headers for embedding SwarmKit into other projects (no Conan required in the consumer).

MAVSDK is not vendored. Install and run `mavsdk_server` separately on the target device; the agent will connect to it over TCP at runtime.

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

**Terminal 1 — start the agent:**

```bash
# macOS / Linux (debug build)
./build/mac-debug/apps/swarmkit-agent

# Agent options
./build/mac-debug/apps/swarmkit-agent \
    --id agent-1 \
    --bind 0.0.0.0:50061 \
    --log-level info
```

```powershell
# Windows
.\build\win-debug\apps\swarmkit-agent.exe --id agent-1
```

Default bind address: `0.0.0.0:50061`.

**Terminal 2 — use the CLI:**

```bash
# Ping
./build/mac-debug/apps/swarmkit-cli ping
./build/mac-debug/apps/swarmkit-cli 192.168.1.10:50061 ping

# Subscribe to telemetry (default drone, 1 Hz) — Ctrl+C to stop
./build/mac-debug/apps/swarmkit-cli telemetry

# Custom drone and rate
./build/mac-debug/apps/swarmkit-cli telemetry --drone uav-1 --rate 5

# Custom address + drone + rate
./build/mac-debug/apps/swarmkit-cli 192.168.1.10:50061 telemetry --drone uav-1 --rate 2
```

---

## Tests

```bash
# macOS
ctest --preset mac-release --output-on-failure

# Linux
ctest --preset linux-release --output-on-failure

# Windows
ctest --preset win-release --output-on-failure
```

---

## Package (CI / release)

Each script runs the full pipeline: Conan install → CMake configure → build → test → produce tarballs/zips in `dist/`.

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
# macOS
tar xzf swarmkit-<version>-sdk-mac-arm64.tar.gz \
    -C ~/swarmkit-sdk --strip-components=1

# Linux
tar xzf swarmkit-<version>-sdk-linux-x86_64.tar.gz \
    -C ~/swarmkit-sdk --strip-components=1
```

```powershell
# Windows — the top-level folder inside the zip becomes the SDK root
Expand-Archive swarmkit-<version>-sdk-win-x86_64.zip -DestinationPath C:\swarmkit-sdk
# SDK root: C:\swarmkit-sdk\swarmkit-<version>-sdk-win-x86_64\
```

SDK layout:

```
<sdk-root>/
├── include/swarmkit/          # public headers
│   ├── core/                  # Logger, Result, TelemetryFrame, version
│   ├── agent/                 # IDroneBackend, commands, server, SimBackend
│   └── client/                # Client, ClientConfig, PingResult, TelemetrySubscription
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

## Build the test client against the SDK

`apps/test_client/` is a minimal standalone project that links against the installed SDK **without Conan**.

```bash
# macOS / Linux
cmake -B apps/test_client/build \
      -DCMAKE_PREFIX_PATH=~/swarmkit-sdk \
      -DCMAKE_BUILD_TYPE=Release \
      -S apps/test_client
cmake --build apps/test_client/build
```

```powershell
# Windows — adjust the path to match your actual sdk-root folder
cmake -B apps\test_client\build `
      -DCMAKE_PREFIX_PATH="C:\swarmkit-sdk\swarmkit-<version>-sdk-win-x86_64" `
      -DCMAKE_BUILD_TYPE=Release `
      -S apps\test_client
cmake --build apps\test_client\build --config Release
```

Run (agent must already be running):

```bash
# macOS / Linux
./apps/test_client/build/swarmkit-test-ping
./apps/test_client/build/swarmkit-test-ping 192.168.1.10:50061

# Windows
.\apps\test_client\build\Release\swarmkit-test-ping.exe
```

Expected output:

```
SwarmKit test client — connecting to 127.0.0.1:50061
Ping OK
  agent_id : agent-1
  version  : 0.1.0
  time_ms  : 1741234567890
```
