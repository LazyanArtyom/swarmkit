// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>

#include "report_hub.h"
#include "swarmkit/agent/server.h"
#include "swarmkit/core/result.h"
#include "swarmkit/v1/swarmkit.pb.h"
#include "telemetry_manager.h"

namespace swarmkit::agent::internal {

class ActiveGoalSupervisor {
   public:
    ActiveGoalSupervisor(TelemetryManager* telemetry, ReportHub* reports, const AgentConfig* config);
    ~ActiveGoalSupervisor() noexcept;

    ActiveGoalSupervisor(const ActiveGoalSupervisor&) = delete;
    ActiveGoalSupervisor& operator=(const ActiveGoalSupervisor&) = delete;

    [[nodiscard]] static core::Result ValidateGoal(const swarmkit::v1::ActiveGoal& goal);

    [[nodiscard]] std::int64_t StartGoal(swarmkit::v1::ActiveGoal goal, std::string correlation_id);
    [[nodiscard]] core::Result CancelGoal(const std::string& drone_id, std::string_view goal_id,
                                          std::string_view correlation_id);
    [[nodiscard]] std::optional<std::pair<swarmkit::v1::ActiveGoal, std::int64_t>> GetGoal(
        const std::string& drone_id, swarmkit::v1::GoalStatus* status) const;
    void Shutdown();

   private:
    struct ActiveGoalRuntime {
        swarmkit::v1::ActiveGoal goal;
        swarmkit::v1::GoalStatus status{swarmkit::v1::GOAL_STATUS_UNSPECIFIED};
        std::int64_t computed_timeout_ms{};
        std::int64_t started_unix_ms{};
        std::string correlation_id;
        std::shared_ptr<std::atomic<bool>> stop;
        std::thread worker;
    };

    void SetTerminalStatus(const std::string& drone_id, swarmkit::v1::GoalStatus status);
    void PublishGoalReport(const swarmkit::v1::ActiveGoal& goal, swarmkit::v1::GoalStatus status,
                           swarmkit::v1::ReportSeverity severity, double distance_to_goal_m,
                           double deviation_m, double altitude_error_m,
                           std::int64_t timeout_ms, std::int64_t started_ms,
                           std::string_view message, std::string_view correlation_id);
    void MonitorGoal(const swarmkit::v1::ActiveGoal& goal, std::int64_t configured_timeout_ms,
                     std::int64_t started_ms, const std::string& correlation_id,
                     const std::shared_ptr<std::atomic<bool>>& stop);

    TelemetryManager* telemetry_{nullptr};
    ReportHub* reports_{nullptr};
    const AgentConfig* config_{nullptr};
    mutable std::mutex mutex_;
    std::unordered_map<std::string, ActiveGoalRuntime> goals_;
};

}  // namespace swarmkit::agent::internal
