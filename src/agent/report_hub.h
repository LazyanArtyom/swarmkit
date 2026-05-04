// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <unordered_map>

#include "swarmkit/v1/swarmkit.pb.h"

namespace swarmkit::agent::internal {

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
    std::uint64_t next_watch_id_{0};
    std::uint64_t next_sequence_{0};
};

}  // namespace swarmkit::agent::internal
