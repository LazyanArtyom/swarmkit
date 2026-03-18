#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "swarmkit/client/client.h"
#include "swarmkit/core/logger.h"
#include "swarmkit/core/telemetry.h"

// ---------------------------------------------------------------------------
// Signal handling for graceful Ctrl+C in telemetry mode.
// ---------------------------------------------------------------------------
namespace {

std::atomic<bool> g_running{true};

extern "C" void OnSignal(int /*sig*/) {
    g_running.store(false, std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// Argument helpers
// ---------------------------------------------------------------------------
std::string GetArg(int argc, char** argv, const std::string& key, const std::string& def) {
    for (int idx = 1; idx + 1 < argc; ++idx) {
        if (std::string(argv[idx]) == key) {
            return {argv[idx + 1]};
        }
    }
    return def;
}

bool HasFlag(int argc, char** argv, const std::string& flag) {
    for (int idx = 1; idx < argc; ++idx) {
        if (std::string(argv[idx]) == flag) {
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// Usage
// ---------------------------------------------------------------------------
void PrintUsage() {
    std::cout << "Usage: swarmkit-cli [ADDRESS] COMMAND [OPTIONS]\n"
                 "\n"
                 "  ADDRESS   Agent address, default: 127.0.0.1:50061\n"
                 "\n"
                 "Commands:\n"
                 "  ping                   Send a ping and print the agent response.\n"
                 "  telemetry              Subscribe to telemetry and print frames.\n"
                 "                         Press Ctrl+C to stop.\n"
                 "\n"
                 "Telemetry options:\n"
                 "  --drone  DRONE_ID      Drone to subscribe to (default: default)\n"
                 "  --rate   HZ            Frames per second     (default: 1)\n"
                 "\n"
                 "Examples:\n"
                 "  swarmkit-cli ping\n"
                 "  swarmkit-cli 192.168.1.10:50061 ping\n"
                 "  swarmkit-cli telemetry --drone uav-1 --rate 2\n";
}

// ---------------------------------------------------------------------------
// Command: ping
// ---------------------------------------------------------------------------
int RunPing(swarmkit::client::Client& client) {
    const swarmkit::client::PingResult kResult = client.Ping();

    if (!kResult.ok) {
        std::cerr << "Ping FAILED: " << kResult.error_message << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "Ping OK\n"
              << "  agent_id  : " << kResult.agent_id << "\n"
              << "  version   : " << kResult.version << "\n"
              << "  time_ms   : " << kResult.unix_time_ms << "\n";
    return EXIT_SUCCESS;
}

// ---------------------------------------------------------------------------
// Command: telemetry
// ---------------------------------------------------------------------------
int RunTelemetry(swarmkit::client::Client& client, const std::string& drone_id, int rate_hz) {
    static constexpr int kPollIntervalMs = 100;
    static constexpr int kCoordPrecision = 5;
    static constexpr int kValuePrecision = 1;

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    std::cout << "Subscribing to telemetry: drone=" << drone_id << " rate=" << rate_hz << " Hz\n"
              << "Press Ctrl+C to stop.\n\n";

    swarmkit::client::TelemetrySubscription sub;
    sub.drone_id = drone_id;
    sub.rate_hz  = rate_hz;

    client.SubscribeTelemetry(
        sub,
        [](const swarmkit::core::TelemetryFrame& frame) {
            std::cout << std::fixed << std::setprecision(kCoordPrecision) << "["
                      << frame.unix_time_ms << "]"
                      << " drone=" << frame.drone_id << " lat=" << frame.lat_deg
                      << " lon=" << frame.lon_deg << std::setprecision(kValuePrecision)
                      << " alt=" << frame.rel_alt_m << "m"
                      << " bat=" << frame.battery_percent << "%"
                      << " mode=" << frame.mode << "\n";
        },
        [](const std::string& error_msg) {
            std::cerr << "Telemetry stream error: " << error_msg << "\n";
        });

    while (g_running.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollIntervalMs));
    }

    client.StopTelemetry();
    std::cout << "\nStopped.\n";
    return EXIT_SUCCESS;
}

}  // namespace

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    if (HasFlag(argc, argv, "--help") || HasFlag(argc, argv, "-h")) {
        PrintUsage();
        return EXIT_SUCCESS;
    }

    // Resolve address and command.
    // Accepts:  swarmkit-cli [address] command [opts]
    //   or:     swarmkit-cli command [opts]       (address defaults)
    static constexpr const char* kDefaultAddr = "127.0.0.1:50061";
    static constexpr const char* kDefaultCommand = "ping";

    std::string addr;
    std::string command;

    if (argc >= 3 && std::string(argv[1]).find(':') != std::string::npos) {
        addr = argv[1];
        command = argv[2];
    } else if (argc >= 2) {
        const std::string kFirstArg = argv[1];
        if (kFirstArg == "ping" || kFirstArg == "telemetry") {
            addr = kDefaultAddr;
            command = kFirstArg;
        } else {
            addr = kFirstArg;
            command = kDefaultCommand;
        }
    } else {
        addr = kDefaultAddr;
        command = kDefaultCommand;
    }

    swarmkit::core::LoggerConfig log_cfg;
    log_cfg.sink_type = swarmkit::core::LogSinkType::kStdout;
    log_cfg.level = swarmkit::core::LogLevel::kWarn;  // CLI uses stdout directly
    swarmkit::core::Logger::Init(log_cfg);

    swarmkit::client::ClientConfig client_cfg;
    client_cfg.address = addr;
    client_cfg.client_id = "swarmkit-cli";

    swarmkit::client::Client client(client_cfg);

    if (command == "ping") {
        return RunPing(client);
    }

    if (command == "telemetry") {
        const std::string kDroneId = GetArg(argc, argv, "--drone", "default");
        const std::string kRateStr = GetArg(argc, argv, "--rate", "1");
        const int kRateHz = std::stoi(kRateStr);
        return RunTelemetry(client, kDroneId, kRateHz);
    }

    std::cerr << "Unknown command: " << command << "\n\n";
    PrintUsage();
    return EXIT_FAILURE;
}
