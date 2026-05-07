// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "report_hub.h"
#include "swarmkit/agent/backend.h"
#include "swarmkit/agent/server.h"
#include "swarmkit/core/result.h"
#include "swarmkit/v1/swarmkit.pb.h"
#include "telemetry_manager.h"

namespace swarmkit::agent::internal {

class TrajectoryExecutionManager {
   public:
    TrajectoryExecutionManager(IDroneBackend* backend, TelemetryManager* telemetry,
                               ReportHub* reports, const AgentConfig* config);
    ~TrajectoryExecutionManager() noexcept;

    TrajectoryExecutionManager(const TrajectoryExecutionManager&) = delete;
    TrajectoryExecutionManager& operator=(const TrajectoryExecutionManager&) = delete;

    [[nodiscard]] swarmkit::v1::ValidateTrajectoryResult ValidatePlan(
        const swarmkit::v1::TrajectoryPlan& plan) const;
    [[nodiscard]] core::Result Upload(swarmkit::v1::TrajectoryPlan plan,
                                      std::string_view correlation_id,
                                      swarmkit::v1::ExecutionHandle* out_handle,
                                      swarmkit::v1::ValidateTrajectoryResult* out_validation);
    [[nodiscard]] core::Result Clear(const std::string& drone_id,
                                     const std::string& execution_id,
                                     std::string_view correlation_id,
                                     swarmkit::v1::ExecutionHandle* out_handle);
    [[nodiscard]] core::Result Prepare(const std::string& drone_id,
                                       const std::string& execution_id,
                                       std::string_view correlation_id,
                                       swarmkit::v1::ExecutionHandle* out_handle,
                                       swarmkit::v1::ValidateTrajectoryResult* out_validation);
    [[nodiscard]] core::Result StartAt(const std::string& drone_id,
                                       const std::string& execution_id,
                                       std::int64_t unix_time_ms,
                                       std::string_view correlation_id,
                                       swarmkit::v1::ExecutionHandle* out_handle);
    [[nodiscard]] core::Result Pause(const std::string& drone_id,
                                     const std::string& execution_id,
                                     std::string_view correlation_id,
                                     swarmkit::v1::ExecutionHandle* out_handle);
    [[nodiscard]] core::Result Resume(const std::string& drone_id,
                                      const std::string& execution_id,
                                      std::string_view correlation_id,
                                      swarmkit::v1::ExecutionHandle* out_handle);
    [[nodiscard]] core::Result Abort(const std::string& drone_id,
                                     const std::string& execution_id,
                                     std::string_view correlation_id,
                                     swarmkit::v1::ExecutionHandle* out_handle);
    [[nodiscard]] std::optional<std::pair<swarmkit::v1::ExecutionHandle,
                                          swarmkit::v1::TrajectoryPlan>>
    Get(const std::string& drone_id, const std::string& execution_id) const;
    [[nodiscard]] std::vector<swarmkit::v1::ExecutionHandle> List(
        const std::string& drone_id) const;
    [[nodiscard]] static swarmkit::v1::TimeSyncState GetTimeSyncState(
        const std::string& drone_id);

    void Shutdown();

   private:
    struct ExecutionRuntime {
        swarmkit::v1::TrajectoryPlan plan;
        swarmkit::v1::ExecutionHandle handle;
        swarmkit::v1::ValidateTrajectoryResult validation;
        std::shared_ptr<std::atomic<bool>> stop;
        std::shared_ptr<std::atomic<bool>> paused;
        std::thread worker;
    };

    [[nodiscard]] static std::string Key(std::string_view drone_id,
                                         std::string_view execution_id);
    [[nodiscard]] std::optional<core::TelemetryFrame> LatestTelemetry(
        const std::string& drone_id) const;
    void PublishReport(const swarmkit::v1::TrajectoryPlan& plan,
                       const swarmkit::v1::ExecutionHandle& handle,
                       swarmkit::v1::TrajectoryReportStatus status,
                       swarmkit::v1::ReportSeverity severity, int active_segment,
                       double distance_to_target_m, double drift_m,
                       std::int64_t schedule_error_ms, std::string_view message,
                       std::string_view correlation_id);
    void RunExecution(const std::string& key, const std::string& correlation_id);
    [[nodiscard]] std::int64_t ComputeTrajectoryReachTimeoutMs(double distance_m) const;
    [[nodiscard]] core::Result SendTrajectoryPoint(
        const swarmkit::v1::TrajectoryPlan& plan, const swarmkit::v1::TrajectoryPoint& point,
        std::string_view correlation_id) const;
    [[nodiscard]] core::Result SendPayloadAction(const swarmkit::v1::TrajectoryPlan& plan,
                                                 const swarmkit::v1::PayloadAction& action,
                                                 std::string_view correlation_id) const;
    IDroneBackend* backend_{nullptr};
    TelemetryManager* telemetry_{nullptr};
    ReportHub* reports_{nullptr};
    const AgentConfig* config_{nullptr};
    mutable std::mutex mutex_;
    std::unordered_map<std::string, ExecutionRuntime> executions_;
};

}  // namespace swarmkit::agent::internal
