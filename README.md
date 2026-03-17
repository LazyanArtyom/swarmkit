# SwarmKit

Multi-agent UAV swarm system.

- **Agent** — gRPC daemon that runs on a Raspberry Pi next to the vehicle. Receives commands, streams telemetry, transfers files (chunked, CRC32, resumable), and routes to a pluggable backend (MAVSDK Server).
- **CLI** — developer tool to send commands, subscribe to telemetry, and upload files.
- **SDK** — static libraries + headers for embedding SwarmKit into other projects.

MAVSDK is not vendored. Install and run `mavsdk_server` separately on the target device.

---

## Prerequisites

| Tool | macOS (ARM64) | Linux (x86\_64) | Windows (x86\_64) |
|------|--------------|-----------------|-------------------|
| Compiler | Xcode CLT (`xcode-select --install`) | GCC 13+ or Clang 17+ (`build-essential`) | MSVC 2022 (Desktop C++ workload) |
| CMake | ≥ 3.28 | ≥ 3.28 | ≥ 3.28 |
| Ninja | `brew install ninja` | `apt install ninja-build` | bundled with VS or `winget install Ninja-build.Ninja` |
| Conan | `pip install conan` | `pip install conan` | `pip install conan` |

Conan profile must be configured before first use:
```
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

Terminal 1 — start the agent:

```bash
# macOS / Linux
./build/mac-debug/apps/swarmkit-agent

# Windows
.\build\win-debug\apps\swarmkit-agent.exe
```

Default bind address is `0.0.0.0:50061`.

Terminal 2 — use the CLI:

```bash
./build/mac-debug/apps/swarmkit-cli 127.0.0.1:50061 ping
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

```powershell
powershell -ExecutionPolicy Bypass -File scripts\ci_package_win_x86_64.ps1
```

Produces:
```
dist\swarmkit-<version>-sdk-win-x86_64.zip
dist\swarmkit-<version>-tools-win-x86_64.zip
```

**In VSCode** press **F8** to run the package pipeline for the current platform.

---

## Tools tarball

The tools tarball contains only the two standalone binaries:

```
swarmkit-<version>-tools-<platform>/
└── bin/
    ├── swarmkit-agent
    └── swarmkit-cli
```

They link only against system libraries and will run on any machine of the same OS and architecture without additional dependencies.

---

## SDK installation

Extract the SDK tarball to a location of your choice, e.g. `~/swarmkit-sdk`:

```bash
# macOS / Linux
tar xzf swarmkit-<version>-sdk-mac-arm64.tar.gz -C ~/swarmkit-sdk --strip-components=1
```

```powershell
# Windows
Expand-Archive swarmkit-<version>-sdk-win-x86_64.zip -DestinationPath C:\swarmkit-sdk
# The top-level folder inside the zip becomes the SDK root:
# C:\swarmkit-sdk\swarmkit-<version>-sdk-win-x86_64\
```

SDK layout:
```
<sdk-root>/
├── include/swarmkit/      # public headers
├── lib/
│   ├── libswarmkit_*.a    # static libraries  (*.lib on Windows)
│   └── cmake/SwarmKit/
│       ├── SwarmKitConfig.cmake
│       ├── SwarmKitConfigVersion.cmake
│       ├── SwarmKitTargets.cmake
│       └── deps/          # bundled Conan CMakeDeps find files
└── third_party/full_deploy/   # bundled Conan dependencies (headers + static libs)
```

---

## Build the test client against the SDK

`apps/test_client/` is a minimal standalone project that links against the installed SDK without Conan.

```bash
# macOS / Linux
cmake -B apps/test_client/build \
      -DCMAKE_PREFIX_PATH=~/swarmkit-sdk \
      -DCMAKE_BUILD_TYPE=Release \
      -S apps/test_client
cmake --build apps/test_client/build
```

```powershell
# Windows  (adjust path to your sdk-root folder)
cmake -B apps\test_client\build `
      -DCMAKE_PREFIX_PATH="C:\swarmkit-sdk\swarmkit-<version>-sdk-win-x86_64" `
      -DCMAKE_BUILD_TYPE=Release `
      -S apps\test_client
cmake --build apps\test_client\build --config Release
```

Run it (agent must already be running):

```bash
# macOS / Linux
./apps/test_client/build/swarmkit-test-ping 127.0.0.1:50061

# Windows
.\apps\test_client\build\Release\swarmkit-test-ping.exe 127.0.0.1:50061
```

Expected output:
```
SwarmKit test client — connecting to 127.0.0.1:50061
Ping OK
  agent_id : agent-1
  version  : 0.1.0
  time_ms  : 1741234567890
```
