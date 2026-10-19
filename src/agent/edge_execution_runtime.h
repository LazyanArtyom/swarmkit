// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>

#include "edge_attempt_journal.h"
#include "report_hub.h"
#include "runtime_providers.h"
#include "swarmkit/agent/arbiter.h"
#include "swarmkit/agent/backend.h"
#include "swarmkit/agent/server.h"
#include "swarmkit/core/execution.h"
#include "swarmkit/edge/vecr.h"
#include "swarmkit/v1/swarmkit.pb.h"
#include "telemetry_manager.h"

namespace swarmkit::agent::internal {

class EdgeExecutionRuntime {
   public:
    EdgeExecutionRuntime(TelemetryManager* telemetry, ReportHub* reports,
                         IDroneBackend* backend, CommandArbiter* arbiter,
                         const AgentConfig* config, RuntimeProviders providers);
    ~EdgeExecutionRuntime();
    EdgeExecutionRuntime(const EdgeExecutionRuntime&) = delete;
    EdgeExecutionRuntime& operator=(const EdgeExecutionRuntime&) = delete;

    [[nodiscard]] core::Result Start(const swarmkit::v1::EdgeOperation& operation,
                                     const core::ExecutionHandle& handle,
                                     const commands::CommandContext& context,
                                     swarmkit::v1::EdgeReport* status,
                                     core::BackendCommandOutcome* dispatch);
    [[nodiscard]] std::optional<swarmkit::v1::EdgeReport> Get(const std::string& drone_id,
                                                               const std::string& attempt_id) const;
    [[nodiscard]] core::Result JournalReady() const { return journal_.Ready(); }
    [[nodiscard]] bool Active(const std::string& drone_id) const;
    [[nodiscard]] core::Result SubmitCapture(const swarmkit::v1::SubmitEdgeCaptureRequest& capture,
                                              swarmkit::v1::EdgeReport* status);
    [[nodiscard]] core::Result Cancel(const core::ExecutionHandle& expected,
                                       swarmkit::v1::EdgeReport* status);
    void Preempt(const std::string& drone_id, const std::string& affected_client_id);
    void Shutdown();

   private:
    struct Attempt {
        Attempt(edge::EdgeRequest request, const swarmkit::v1::EdgeOperation& source,
                core::ExecutionHandle execution, commands::CommandContext command,
                TelemetryLease lease);
        mutable std::mutex mutex;
        edge::EdgeExecutionSupervisor supervisor;
        swarmkit::v1::EdgeOperation operation;
        core::ExecutionHandle handle;
        commands::CommandContext context;
        core::BackendCommandOutcome initial_dispatch;
        TelemetryLease lease;
        std::atomic<bool> stop{false};
        std::thread worker;
        std::uint64_t last_sequence{};
        double last_reported_covered{};
        edge::Phase last_reported_phase{edge::Phase::kUnknown};
        bool last_reported_arrival{};
        std::optional<edge::Point> reentry_target;
        std::int64_t reentry_candidate_ms{-1};
        std::optional<double> last_battery_percent;
        std::optional<edge::Point> last_bounded_position;
        std::optional<double> last_position_hard_bound_m;
        std::int64_t last_position_ms{-1};
        std::int64_t started_agent_ms{};
        std::int64_t last_frame_agent_ms{};
    };

    [[nodiscard]] static std::optional<edge::EdgeRequest> Convert(
        const swarmkit::v1::EdgeOperation& operation);
    [[nodiscard]] static swarmkit::v1::EdgeReport ToReport(const Attempt& attempt,
                                                           const edge::Status& status);
    void Publish(const Attempt& attempt, const edge::Status& status);
    void Work(const std::shared_ptr<Attempt>& attempt);
    void DispatchRecovery(const std::shared_ptr<Attempt>& attempt, edge::Point target,
                          std::int64_t now_ms);
    void DispatchReturnToB(const std::shared_ptr<Attempt>& attempt);

    TelemetryManager* telemetry_;
    ReportHub* reports_;
    IDroneBackend* backend_;
    CommandArbiter* arbiter_;
    const AgentConfig* config_;
    RuntimeProviders providers_;
    EdgeAttemptJournal journal_;
    std::mutex start_mutex_;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, std::shared_ptr<Attempt>> attempts_;
    std::map<std::pair<std::string, std::string>, std::shared_ptr<Attempt>> history_;
};

}  // namespace swarmkit::agent::internal
