// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

/// @file test_agents.cpp
/// @brief Spawns three swarmkit-agent processes (one per simulated drone),
///        multiplexes their stdout/stderr with a per-drone prefix, and shuts
///        them all down cleanly on Ctrl+C.

#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "swarmkit/core/logger.h"

namespace {

/// Maximum number of agents that can be spawned.
constexpr int kMaxAgents = 3;
constexpr int kPipeFdCount = 2;
constexpr std::size_t kReadBufferSize = 4096;
constexpr std::string_view kDefaultCaCertPath = "testdata/certs/ca.pem";
constexpr std::string_view kDefaultServerCertPath = "testdata/certs/agent.pem";
constexpr std::string_view kDefaultServerKeyPath = "testdata/certs/agent.key";
constexpr auto kReadinessTimeout = std::chrono::seconds{5};
constexpr auto kReadinessPollInterval = std::chrono::milliseconds{100};

std::atomic<bool> g_running{true};
std::atomic<bool> g_shutdown_requested{false};
std::array<pid_t, kMaxAgents> g_child_pids{-1, -1, -1};
std::atomic<int> g_child_count{0};

extern "C" void OnSignal(int /*sig*/) {
    g_running.store(false, std::memory_order_relaxed);
    g_shutdown_requested.store(true, std::memory_order_relaxed);
    const int kCount = g_child_count.load(std::memory_order_relaxed);
    for (int index = 0; index < kCount; ++index) {
        const pid_t kChildPid = g_child_pids[static_cast<std::size_t>(index)];
        if (kChildPid > 0) {
            kill(kChildPid, SIGTERM);
        }
    }
}

void StopAllChildren() {
    const int kCount = g_child_count.load(std::memory_order_relaxed);
    for (int index = 0; index < kCount; ++index) {
        const pid_t kChildPid = g_child_pids[static_cast<std::size_t>(index)];
        if (kChildPid > 0) {
            kill(kChildPid, SIGTERM);
        }
    }
}

struct AgentSpec {
    std::string drone_id;
    std::string bind_addr;
    std::string port;
};

const std::array<AgentSpec, kMaxAgents> kAgentSpecs{{
    {.drone_id = "drone-1", .bind_addr = "0.0.0.0:50061", .port = "50061"},
    {.drone_id = "drone-2", .bind_addr = "0.0.0.0:50062", .port = "50062"},
    {.drone_id = "drone-3", .bind_addr = "0.0.0.0:50063", .port = "50063"},
}};

[[nodiscard]] std::string GetArg(int argc, char** argv, std::string_view key,
                                 std::string_view default_value) {
    for (int index = 1; index + 1 < argc; ++index) {
        if (std::string_view{argv[index]} == key) {
            return {argv[index + 1]};
        }
    }
    return std::string{default_value};
}

struct AgentSecurityPaths {
    std::string root_ca_cert_path;
    std::string server_cert_path;
    std::string server_key_path;
};

[[nodiscard]] AgentSecurityPaths LoadSecurityPaths(int argc, char** argv) {
    return AgentSecurityPaths{
        .root_ca_cert_path = GetArg(argc, argv, "--ca-cert", kDefaultCaCertPath),
        .server_cert_path = GetArg(argc, argv, "--server-cert", kDefaultServerCertPath),
        .server_key_path = GetArg(argc, argv, "--server-key", kDefaultServerKeyPath),
    };
}

[[nodiscard]] std::optional<std::string> DroneIdForPid(pid_t pid) {
    const int kCount = g_child_count.load(std::memory_order_relaxed);
    for (int index = 0; index < kCount; ++index) {
        if (g_child_pids[static_cast<std::size_t>(index)] != pid) {
            continue;
        }
        return kAgentSpecs[static_cast<std::size_t>(index)].drone_id;
    }
    return std::nullopt;
}

/// @brief Read from a file descriptor line-by-line and print each line with a prefix.
void ReadAndPrefix(int read_fd, const std::string& prefix) {
    std::array<char, kReadBufferSize> buf{};
    std::string partial_line;

    while (true) {
        const ssize_t kBytesRead = read(read_fd, buf.data(), buf.size());
        if (kBytesRead <= 0) {
            break;
        }
        for (ssize_t idx = 0; idx < kBytesRead; ++idx) {
            if (buf[static_cast<std::size_t>(idx)] == '\n') {
                std::cout << prefix << partial_line << "\n";
                std::cout.flush();
                partial_line.clear();
            } else {
                partial_line += buf[static_cast<std::size_t>(idx)];
            }
        }
    }

    if (!partial_line.empty()) {
        std::cout << prefix << partial_line << "\n";
        std::cout.flush();
    }

    close(read_fd);
}

[[nodiscard]] bool CanConnectToPort(std::string_view port_text) {
    const int kPort = std::stoi(std::string(port_text));
    const int kSocketFd = socket(AF_INET, SOCK_STREAM, 0);
    if (kSocketFd < 0) {
        return false;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<std::uint16_t>(kPort));
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    const int kConnectResult =
        connect(kSocketFd, reinterpret_cast<sockaddr*>(&address), sizeof(address));
    close(kSocketFd);
    return kConnectResult == 0;
}

[[nodiscard]] bool TryReapStartupExit(pid_t child_pid) {
    int status = 0;
    const pid_t kWaitResult = waitpid(child_pid, &status, WNOHANG);
    if (kWaitResult == 0) {
        return false;
    }
    if (kWaitResult != child_pid) {
        return false;
    }

    const std::string kDroneId = DroneIdForPid(child_pid).value_or("unknown");
    std::ostringstream message;
    message << "Agent '" << kDroneId << "' exited during startup";
    if (WIFEXITED(status)) {
        message << " with code " << WEXITSTATUS(status);
    } else if (WIFSIGNALED(status)) {
        message << " from signal " << WTERMSIG(status);
    }
    swarmkit::core::Logger::Error(message.str());
    return true;
}

[[nodiscard]] bool WaitForAgentsReady() {
    const auto kDeadline = std::chrono::steady_clock::now() + kReadinessTimeout;
    while (std::chrono::steady_clock::now() < kDeadline) {
        bool saw_exit = false;
        for (int index = 0; index < kMaxAgents; ++index) {
            if (TryReapStartupExit(g_child_pids[static_cast<std::size_t>(index)])) {
                saw_exit = true;
                break;
            }
        }
        if (saw_exit) {
            return false;
        }

        bool all_ports_ready = true;
        for (const auto& spec : kAgentSpecs) {
            if (!CanConnectToPort(spec.port)) {
                all_ports_ready = false;
                break;
            }
        }
        if (all_ports_ready) {
            return true;
        }

        std::this_thread::sleep_for(kReadinessPollInterval);
    }

    swarmkit::core::Logger::Error("Timed out waiting for all agent ports to become ready");
    return false;
}

void PrintReadySummary() {
    std::cout << "SwarmKit Test Agents ready:\n";
    for (int index = 0; index < kMaxAgents; ++index) {
        const auto& spec = kAgentSpecs[static_cast<std::size_t>(index)];
        const pid_t kChildPid = g_child_pids[static_cast<std::size_t>(index)];
        std::cout << "  " << spec.drone_id << "  pid=" << kChildPid
                  << "  addr=127.0.0.1:" << spec.port << "\n";
    }
    std::cout << "Next:\n"
              << "  ./apps/test_tools/build/swarmkit-test-client --swarm-config "
                 "testdata/swarm_config.yaml --address-mode local\n"
              << "  ./apps/test_tools/build/swarmkit-test-server --swarm-config "
                 "testdata/swarm_config.yaml --address-mode local\n";
    std::cout.flush();
}

void MonitorChildrenUntilExit() {
    int live_children = g_child_count.load(std::memory_order_relaxed);
    while (live_children > 0) {
        int status = 0;
        const pid_t kExitedPid = waitpid(-1, &status, 0);
        if (kExitedPid <= 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }

        --live_children;
        const std::string kDroneId = DroneIdForPid(kExitedPid).value_or("unknown");
        if (!g_shutdown_requested.load(std::memory_order_relaxed)) {
            std::ostringstream message;
            message << "Agent '" << kDroneId << "' exited unexpectedly";
            if (WIFEXITED(status)) {
                message << " with code " << WEXITSTATUS(status);
            } else if (WIFSIGNALED(status)) {
                message << " from signal " << WTERMSIG(status);
            }
            swarmkit::core::Logger::Error(message.str());
            g_shutdown_requested.store(true, std::memory_order_relaxed);
            g_running.store(false, std::memory_order_relaxed);
            StopAllChildren();
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    swarmkit::core::LoggerConfig log_cfg;
    log_cfg.sink_type = swarmkit::core::LogSinkType::kStdout;
    log_cfg.level = swarmkit::core::LogLevel::kInfo;
    swarmkit::core::Logger::Init(log_cfg);

    const std::string kAgentBin = (argc > 1) ? argv[1] : "swarmkit-agent";
    const AgentSecurityPaths kSecurityPaths = LoadSecurityPaths(argc, argv);

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    swarmkit::core::Logger::InfoFmt("Starting {} agents using binary: {}", kAgentSpecs.size(),
                                    kAgentBin);

    std::vector<std::thread> reader_threads;
    reader_threads.reserve(kAgentSpecs.size());

    for (const auto& spec : kAgentSpecs) {
        int pipe_fds[kPipeFdCount];
        if (pipe(pipe_fds) < 0) {
            swarmkit::core::Logger::ErrorFmt("pipe() failed for {}: {}", spec.drone_id,
                                             std::strerror(errno));
            return EXIT_FAILURE;
        }

        const pid_t kChildPid = fork();
        if (kChildPid < 0) {
            swarmkit::core::Logger::ErrorFmt("fork() failed for {}: {}", spec.drone_id,
                                             std::strerror(errno));
            return EXIT_FAILURE;
        }

        if (kChildPid == 0) {
            dup2(pipe_fds[1], STDOUT_FILENO);
            dup2(pipe_fds[1], STDERR_FILENO);
            close(pipe_fds[0]);
            close(pipe_fds[1]);

            setenv("SWARMKIT_AGENT_ROOT_CA_CERT_PATH", kSecurityPaths.root_ca_cert_path.c_str(), 1);
            setenv("SWARMKIT_AGENT_CERT_CHAIN_PATH", kSecurityPaths.server_cert_path.c_str(), 1);
            setenv("SWARMKIT_AGENT_PRIVATE_KEY_PATH", kSecurityPaths.server_key_path.c_str(), 1);

            execlp(kAgentBin.c_str(), kAgentBin.c_str(), "--id", spec.drone_id.c_str(), "--bind",
                   spec.bind_addr.c_str(), "--log-level", "info", nullptr);

            swarmkit::core::Logger::ErrorFmt("execlp({}) failed: {}", kAgentBin,
                                             std::strerror(errno));
            _exit(EXIT_FAILURE);
        }

        close(pipe_fds[1]);
        const int kCurrentCount = g_child_count.load(std::memory_order_relaxed);
        g_child_pids[static_cast<std::size_t>(kCurrentCount)] = kChildPid;
        g_child_count.store(kCurrentCount + 1, std::memory_order_relaxed);

        const int kReadFd = pipe_fds[0];
        const std::string kPrefix = "[" + spec.drone_id + "] ";

        reader_threads.emplace_back([kReadFd, kPrefix]() { ReadAndPrefix(kReadFd, kPrefix); });

        swarmkit::core::Logger::InfoFmt("Launched {} (pid={}) on {}", spec.drone_id, kChildPid,
                                        spec.bind_addr);
    }

    if (!WaitForAgentsReady()) {
        g_shutdown_requested.store(true, std::memory_order_relaxed);
        g_running.store(false, std::memory_order_relaxed);
        StopAllChildren();
    } else {
        swarmkit::core::Logger::Info("All agents running. Press Ctrl+C to stop.");
        PrintReadySummary();
    }

    MonitorChildrenUntilExit();

    for (auto& thread : reader_threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    swarmkit::core::Logger::Info("All agents stopped.");
    return EXIT_SUCCESS;
}
