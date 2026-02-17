# swarmkit

Multi-agent UAV swarm system:

- **Agent** (runs on Raspberry Pi next to the vehicle):
  - receives commands (from local CLI or central server)
  - publishes telemetry via gRPC streaming
  - transfers arbitrary data (images/files) via **full-duplex** gRPC streaming with **chunking + CRC32 + resume**
  - routes commands to a **pluggable backend** (MAVSDK Server)

- **CLI** (developer tool):
  - sends commands to an agent
  - subscribes to telemetry
  - uploads files to the agent using the resumable streaming API

This repository **does not vendor MAVSDK**.
On Raspberry Pi you install/run `mavsdk_server` separately, and the agent backend will talk to it locally.

## What it builds

- `swarmkit_agent` – gRPC server (agent/daemon)
- `swarmkit_cli` – gRPC client (dev CLI)
- `swarmkit_tests` – Catch2 tests

## Build (Debug)

```bash
conan install . -of build/conan -s build_type=Debug -s compiler.cppstd=23 --build=missing
cmake --preset mac-debug-conan    # or linux-debug-conan / win-debug-conan
cmake --build --preset mac-debug-conan
```

## Run

Terminal 1 (agent):
```bash
./build/mac-debug-conan/swarmkit_agent --id agent-1 --bind 0.0.0.0:50061
```

Terminal 2 (CLI):
```bash
./build/mac-debug-conan/swarmkit_cli --addr 127.0.0.1:50061 ping
./build/mac-debug-conan/swarmkit_cli --addr 127.0.0.1:50061 arm --drone default
./build/mac-debug-conan/swarmkit_cli --addr 127.0.0.1:50061 takeoff --drone default --alt 10
./build/mac-debug-conan/swarmkit_cli --addr 127.0.0.1:50061 tele --drone default --rate 5
./build/mac-debug-conan/swarmkit_cli --addr 127.0.0.1:50061 send-file --file ./testdata/sample.bin
```

The agent stores received files into:
- `/tmp/swarmkit_inbox/<transfer_id>.bin` (configurable via `--inbox DIR`)