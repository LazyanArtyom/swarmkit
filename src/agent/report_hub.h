// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <unordered_map>

#include "swarmkit/v1/swarmkit.pb.h"

namespace swarmkit::agent::internal {

struct ReportHubOptions {
    std::size_t max_in_memory_backlog{1000};
    std::function<std::int64_t()> wall_time_ms;
    std::string agent_session_id;
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

/// In-memory report distribution. Durable evidence belongs exclusively to
/// ExecutionRecorder, so this hub cannot create a competing report log.
class ReportHub {
   public:
    using FinalizedReportObserver = std::function<void(const swarmkit::v1::AgentReport&)>;

    explicit ReportHub(ReportHubOptions options);

    void SetFinalizedReportObserver(FinalizedReportObserver observer);
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

    std::mutex mutex_;
    std::unordered_map<std::uint64_t, Watcher> watchers_;
    std::deque<swarmkit::v1::AgentReport> backlog_;
    ReportHubOptions options_;
    std::uint64_t next_watch_id_{0};
    std::uint64_t next_sequence_{0};
    std::function<std::int64_t()> wall_time_ms_;
    FinalizedReportObserver observer_;
};

}  // namespace swarmkit::agent::internal
