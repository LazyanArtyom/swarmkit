# Quick Start

This example connects to a local insecure agent, pings it, subscribes briefly to
telemetry, and sends an arm command. It is designed for the simulator backend or
local development, not production transport security.

## Start A Local Agent

From the SwarmKit build tree:

```bash
./build/mac-debug/apps/swarmkit-agent --insecure --id agent-1 --bind 127.0.0.1:50061
```

Use the matching binary path for your preset, for example
`./build/linux-debug/apps/swarmkit-agent` on Linux.

## Minimal C++ Client

Save this as `main.cpp` in a separate CMake project that links
`swarmkit::client`:

```cpp
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

#include "swarmkit/client/client.h"
#include "swarmkit/commands.h"
#include "swarmkit/core/security.h"

int main() {
    namespace client = swarmkit::client;
    namespace commands = swarmkit::commands;
    namespace core = swarmkit::core;

    client::ClientConfig config;
    config.address = "127.0.0.1:50061";
    config.client_id = "quick-start";
    config.security.transport_security = core::TransportSecurityMode::kInsecure;

    client::Client swarmkit{config};

    const client::PingResult ping = swarmkit.Ping();
    if (!ping.ok) {
        std::cerr << "Ping failed: " << ping.error_message << "\n";
        return 1;
    }
    std::cout << "Connected to " << ping.agent_id << " version " << ping.version << "\n";

    std::atomic<int> frames{0};
    auto telemetry = swarmkit.StartTelemetry(
        {.drone_id = "default", .rate_hertz = 1},
        [&](const core::TelemetryFrame& frame) {
            ++frames;
            if (frame.HasPosition()) {
                std::cout << "lat=" << frame.lat_deg << " lon=" << frame.lon_deg << "\n";
            } else {
                std::cout << "telemetry frame from " << frame.drone_id << "\n";
            }
        },
        [](const std::string& error) { std::cerr << "telemetry error: " << error << "\n"; });

    if (!telemetry.has_value()) {
        std::cerr << "Telemetry failed: " << telemetry.error().ToString() << "\n";
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::seconds{2});
    telemetry->Stop();

    commands::CommandEnvelope arm;
    arm.context.drone_id = "default";
    arm.context.client_id = config.client_id;
    arm.context.priority = commands::CommandPriority::kOperator;
    arm.command = commands::FlightCmd{commands::CmdArm{}};

    const client::CommandResult result = swarmkit.SendCommand(arm);
    if (!result.ok) {
        std::cerr << "Command rejected: " << result.message << "\n";
        return 1;
    }

    std::cout << "Arm accepted after " << frames.load() << " telemetry frame(s)\n";
    return 0;
}
```

## CMake

```cmake
cmake_minimum_required(VERSION 3.24)
project(SwarmKitQuickStart CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(SwarmKit REQUIRED)

add_executable(quick_start main.cpp)
target_link_libraries(quick_start PRIVATE swarmkit::client)
```

Build and run:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/tmp/swarmkit-sdk
cmake --build build
./build/quick_start
```

Expected output includes a successful ping, one or more telemetry frames, and an
accepted command. If the agent is using the MAVLink backend, the vehicle state
and readiness checks may reject commands that the simulator accepts.
