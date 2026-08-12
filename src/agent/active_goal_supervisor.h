// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>

#include "report_hub.h"
#include "runtime_providers.h"
#include "swarmkit/agent/server.h"
#include "swarmkit/core/execution.h"
#include "swarmkit/core/result.h"
#include "swarmkit/v1/swarmkit.pb.h"
#include "telemetry_manager.h"

namespace swarmkit::agent::internal {

class ActiveGoalSupervisor {
   public:
    using GoalLifecycleObserver =
        std::function<void(const swarmkit::v1::ActiveGoal&, const core::ExecutionHandle&,
                           swarmkit::v1::GoalStatus, std::string_view)>;

    struct GoalSnapshot {
        swarmkit::v1::ActiveGoal goal;
        std::int64_t computed_timeout_ms{};
        swarmkit::v1::GoalStatus status{swarmkit::v1::GOAL_STATUS_UNSPECIFIED};
        core::ExecutionHandle execution_handle;
    };

    ActiveGoalSupervisor(TelemetryManager* telemetry, ReportHub* reports, const AgentConfig* config,
                         RuntimeProviders providers);
    ~ActiveGoalSupervisor() noexcept;

    ActiveGoalSupervisor(const ActiveGoalSupervisor&) = delete;
    ActiveGoalSupervisor& operator=(const ActiveGoalSupervisor&) = delete;

    [[nodiscard]] static core::Result ValidateGoal(const swarmkit::v1::ActiveGoal& goal);

    /// Bind normalized telemetry to an attempt while its backend dispatch is in progress.
    void BindDispatchAttempt(const core::ExecutionHandle& execution_handle);
    void ClearDispatchAttempt(const core::ExecutionHandle& execution_handle);
    void SetGoalLifecycleObserver(GoalLifecycleObserver observer);

    [[nodiscard]] std::int64_t StartGoal(swarmkit::v1::ActiveGoal goal,
                                         core::ExecutionHandle execution_handle);
    [[nodiscard]] core::Result CancelGoal(const core::ExecutionHandle& expected_handle,
                                          core::ExecutionHandle* cancelled_handle = nullptr);
    [[nodiscard]] std::optional<GoalSnapshot> GetGoal(const std::string& drone_id) const;
    [[nodiscard]] std::optional<core::ExecutionHandle> GetExecutionHandle(
        const std::string& drone_id) const;
    void Shutdown();

   private:
    struct ActiveGoalRuntime {
        swarmkit::v1::ActiveGoal goal;
        swarmkit::v1::GoalStatus status{swarmkit::v1::GOAL_STATUS_UNSPECIFIED};
        std::int64_t computed_timeout_ms{};
        std::int64_t started_unix_ms{};
        std::string correlation_id;
        core::ExecutionHandle execution_handle;
        std::shared_ptr<std::atomic<bool>> stop;
        std::thread worker;
    };

    void SetTerminalStatus(const core::ExecutionHandle& handle, swarmkit::v1::GoalStatus status);
    void PublishGoalReport(const swarmkit::v1::ActiveGoal& goal, swarmkit::v1::GoalStatus status,
                           swarmkit::v1::ReportSeverity severity, double distance_to_goal_m,
                           double deviation_m, double altitude_error_m, std::int64_t timeout_ms,
                           std::int64_t started_ms, std::string_view message,
                           const core::ExecutionHandle& execution_handle);
    void MonitorGoal(const swarmkit::v1::ActiveGoal& goal, std::int64_t configured_timeout_ms,
                     std::int64_t started_ms, const core::ExecutionHandle& execution_handle,
                     const std::shared_ptr<std::atomic<bool>>& stop);

    TelemetryManager* telemetry_{nullptr};
    ReportHub* reports_{nullptr};
    const AgentConfig* config_{nullptr};
    RuntimeProviders providers_;
    mutable std::mutex observer_mutex_;
    GoalLifecycleObserver lifecycle_observer_;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, ActiveGoalRuntime> goals_;
    std::unordered_map<std::string, core::ExecutionHandle> dispatch_attempts_;
};

}  // namespace swarmkit::agent::internal
