// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "report_hub.h"

#include <chrono>
#include <utility>
#include <vector>

namespace swarmkit::agent::internal {
namespace {

[[nodiscard]] std::int64_t NowUnixMs() {
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using std::chrono::system_clock;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

}  // namespace

void ReportQueue::Push(swarmkit::v1::AgentReport report) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (shutdown_) {
            return;
        }
        queue_.push(std::move(report));
    }
    cv_.notify_one();
}

bool ReportQueue::Pop(swarmkit::v1::AgentReport* out, std::chrono::milliseconds timeout) {
    if (out == nullptr) {
        return false;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    const bool ready =
        cv_.wait_for(lock, timeout, [this] { return !queue_.empty() || shutdown_; });
    if (!ready || queue_.empty()) {
        return false;
    }
    *out = std::move(queue_.front());
    queue_.pop();
    return true;
}

void ReportQueue::Shutdown() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        shutdown_ = true;
    }
    cv_.notify_all();
}

ReportWatchToken ReportHub::Watch(std::string drone_id, std::uint64_t after_sequence,
                                  const std::shared_ptr<ReportQueue>& queue) {
    std::vector<swarmkit::v1::AgentReport> backlog;
    ReportWatchToken token;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        token.watch_id = ++next_watch_id_;
        watchers_.emplace(token.watch_id, Watcher{
                                              .drone_id = std::move(drone_id),
                                              .queue = queue,
                                          });
        for (const auto& report : backlog_) {
            if (report.sequence() > after_sequence && Matches(watchers_.at(token.watch_id), report)) {
                backlog.push_back(report);
            }
        }
    }
    for (auto& report : backlog) {
        if (queue) {
            queue->Push(std::move(report));
        }
    }
    return token;
}

void ReportHub::Unwatch(ReportWatchToken token) {
    std::lock_guard<std::mutex> lock(mutex_);
    watchers_.erase(token.watch_id);
}

void ReportHub::Publish(swarmkit::v1::AgentReport report) {
    std::vector<std::shared_ptr<ReportQueue>> queues;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        report.set_sequence(++next_sequence_);
        report.set_unix_time_ms(NowUnixMs());
        backlog_.push_back(report);
        constexpr std::size_t kMaxBacklog = 1000;
        while (backlog_.size() > kMaxBacklog) {
            backlog_.pop_front();
        }
        for (auto iter = watchers_.begin(); iter != watchers_.end();) {
            if (auto queue = iter->second.queue.lock()) {
                if (Matches(iter->second, report)) {
                    queues.push_back(std::move(queue));
                }
                ++iter;
            } else {
                iter = watchers_.erase(iter);
            }
        }
    }
    for (const auto& queue : queues) {
        queue->Push(report);
    }
}

bool ReportHub::Matches(const Watcher& watcher, const swarmkit::v1::AgentReport& report) {
    return watcher.drone_id.empty() || watcher.drone_id == "all" ||
           watcher.drone_id == report.drone_id();
}

}  // namespace swarmkit::agent::internal
