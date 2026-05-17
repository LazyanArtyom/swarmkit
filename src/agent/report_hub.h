// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "swarmkit/v1/swarmkit.pb.h"

namespace swarmkit::agent::internal {

struct ReportHubOptions {
    std::string report_log_file;
    std::string sequence_state_file;
    std::size_t max_in_memory_backlog{1000};
    std::size_t max_log_file_size_bytes{10UL * 1024UL * 1024UL};
    int max_log_files{5};
    bool flush_each_write{true};
    bool fsync_each_write{false};
    bool replay_from_log{true};
};

class ReportQueue {
   public:
    void Push(swarmkit::v1::AgentReport report);
    [[nodiscard]] bool Pop(swarmkit::v1::AgentReport* out, std::chrono::milliseconds timeout);
    void Shutdown();

   private:
    std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<swarmkit::v1::AgentReport> queue_;
    bool shutdown_{false};
};

struct ReportWatchToken {
    std::uint64_t watch_id{};
};

class ReportHub {
   public:
    ReportHub();
    explicit ReportHub(std::string report_log_file);
    explicit ReportHub(ReportHubOptions options);

    [[nodiscard]] ReportWatchToken Watch(std::string drone_id, std::uint64_t after_sequence,
                                         const std::shared_ptr<ReportQueue>& queue);
    void Unwatch(ReportWatchToken token);
    void Publish(swarmkit::v1::AgentReport report);

   private:
    struct Watcher {
        std::string drone_id;
        std::weak_ptr<ReportQueue> queue;
    };

    [[nodiscard]] static bool Matches(const Watcher& watcher,
                                      const swarmkit::v1::AgentReport& report);
    [[nodiscard]] std::vector<swarmkit::v1::AgentReport> LoadReplay(
        const Watcher& watcher, std::uint64_t after_sequence) const;
    void InitializePersistence();
    void OpenReportLog(bool append);
    void RotateReportLogIfNeeded(std::size_t pending_bytes);
    void WriteReportLogLine(const swarmkit::v1::AgentReport& report);
    void PersistSequenceState(std::uint64_t sequence);
    [[nodiscard]] std::uint64_t LoadSequenceState() const;
    [[nodiscard]] std::uint64_t LoadMaxSequenceFromLogs() const;
    [[nodiscard]] std::vector<std::string> ReportLogReadPaths() const;

    std::mutex mutex_;
    std::unordered_map<std::uint64_t, Watcher> watchers_;
    std::deque<swarmkit::v1::AgentReport> backlog_;
    std::ofstream report_log_;
    ReportHubOptions options_;
    std::size_t report_log_bytes_{0};
    std::uint64_t next_watch_id_{0};
    std::uint64_t next_sequence_{0};
};

}  // namespace swarmkit::agent::internal
