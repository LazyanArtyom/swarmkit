/// @file test_agents.cpp
/// @brief Spawns three swarmkit-agent processes (one per simulated drone),
///        multiplexes their stdout/stderr with a per-drone prefix, and shuts
///        them all down cleanly on Ctrl+C.

#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "swarmkit/core/logger.h"

namespace {

/// Maximum number of agents that can be spawned.
constexpr int kMaxAgents = 3;

std::atomic<bool> g_running{true};
std::array<pid_t, kMaxAgents> g_child_pids{-1, -1, -1};
std::atomic<int> g_child_count{0};

extern "C" void OnSignal(int /*sig*/) {
    g_running.store(false, std::memory_order_relaxed);
    const int count = g_child_count.load(std::memory_order_relaxed);
    for (int idx = 0; idx < count; ++idx) {
        const pid_t child_pid = g_child_pids[static_cast<std::size_t>(idx)];
        if (child_pid > 0) {
            kill(child_pid, SIGTERM);
        }
    }
}

struct AgentSpec {
    std::string drone_id;
    std::string bind_addr;
    std::string port;
};

const std::array<AgentSpec, 3> kAgentSpecs{{
    {"drone-1", "0.0.0.0:50061", "50061"},
    {"drone-2", "0.0.0.0:50062", "50062"},
    {"drone-3", "0.0.0.0:50063", "50063"},
}};

/// @brief Read from a file descriptor line-by-line and print each line with a prefix.
void ReadAndPrefix(int read_fd, const std::string& prefix) {
    std::array<char, 4096> buf{};
    std::string partial_line;

    while (true) {
        const ssize_t bytes_read = read(read_fd, buf.data(), buf.size());
        if (bytes_read <= 0) {
            break;
        }
        for (ssize_t idx = 0; idx < bytes_read; ++idx) {
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

}  // namespace

int main(int argc, char** argv) {
    swarmkit::core::LoggerConfig log_cfg;
    log_cfg.sink_type = swarmkit::core::LogSinkType::kStdout;
    log_cfg.level = swarmkit::core::LogLevel::kInfo;
    swarmkit::core::Logger::Init(log_cfg);

    const std::string agent_bin = (argc > 1) ? argv[1] : "swarmkit-agent";

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    swarmkit::core::Logger::InfoFmt("Starting {} agents using binary: {}", kAgentSpecs.size(),
                                    agent_bin);

    std::vector<std::thread> reader_threads;
    reader_threads.reserve(kAgentSpecs.size());

    for (const auto& spec : kAgentSpecs) {
        int pipe_fds[2];
        if (pipe(pipe_fds) < 0) {
            swarmkit::core::Logger::ErrorFmt("pipe() failed for {}: {}", spec.drone_id,
                                             std::strerror(errno));
            return EXIT_FAILURE;
        }

        const pid_t child_pid = fork();
        if (child_pid < 0) {
            swarmkit::core::Logger::ErrorFmt("fork() failed for {}: {}", spec.drone_id,
                                             std::strerror(errno));
            return EXIT_FAILURE;
        }

        if (child_pid == 0) {
            dup2(pipe_fds[1], STDOUT_FILENO);
            dup2(pipe_fds[1], STDERR_FILENO);
            close(pipe_fds[0]);
            close(pipe_fds[1]);

            execlp(agent_bin.c_str(), agent_bin.c_str(), "--id", spec.drone_id.c_str(), "--bind",
                   spec.bind_addr.c_str(), "--log-level", "info", nullptr);

            swarmkit::core::Logger::ErrorFmt("execlp({}) failed: {}", agent_bin,
                                             std::strerror(errno));
            _exit(EXIT_FAILURE);
        }

        close(pipe_fds[1]);
        const int current_count = g_child_count.load(std::memory_order_relaxed);
        g_child_pids[static_cast<std::size_t>(current_count)] = child_pid;
        g_child_count.store(current_count + 1, std::memory_order_relaxed);

        const int read_fd = pipe_fds[0];
        const std::string prefix = "[" + spec.drone_id + "] ";

        reader_threads.emplace_back([read_fd, prefix]() { ReadAndPrefix(read_fd, prefix); });

        swarmkit::core::Logger::InfoFmt("Launched {} (pid={}) on {}", spec.drone_id, child_pid,
                                        spec.bind_addr);
    }

    swarmkit::core::Logger::Info("All agents running. Press Ctrl+C to stop.");

    const int kTotalChildren = g_child_count.load(std::memory_order_relaxed);
    for (int idx = 0; idx < kTotalChildren; ++idx) {
        int status = 0;
        waitpid(g_child_pids[static_cast<std::size_t>(idx)], &status, 0);
    }

    for (auto& thread : reader_threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    swarmkit::core::Logger::Info("All agents stopped.");
    return EXIT_SUCCESS;
}
