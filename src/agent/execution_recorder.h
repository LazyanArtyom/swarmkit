// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#pragma once

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <unordered_map>

#include "runtime_providers.h"
#include "swarmkit/agent/server.h"
#include "swarmkit/v1/swarmkit.pb.h"

namespace swarmkit::agent::internal {

struct ExecutionRecorderOptions {
    ExecutionRecorderConfig config;
    RuntimeProviders providers;
    std::string agent_session_id;
    swarmkit::v1::ExecutionRunMetadata metadata;
};

/// Writes one globally ordered, checksummed stream of deterministic protobuf
/// envelopes. The recorder is synchronous by design: accepted evidence is
/// written and flushed according to policy (or the run is explicitly invalid)
/// before Record() returns.
class ExecutionRecorder {
   public:
    explicit ExecutionRecorder(ExecutionRecorderOptions options);
    ~ExecutionRecorder();

    ExecutionRecorder(const ExecutionRecorder&) = delete;
    ExecutionRecorder& operator=(const ExecutionRecorder&) = delete;

    [[nodiscard]] bool Enabled() const noexcept;
    [[nodiscard]] bool Valid() const;
    [[nodiscard]] std::string FailureMessage() const;

    void RecordTelemetry(const swarmkit::v1::TelemetryFrame& telemetry);
    void RecordCommandRequested(const swarmkit::v1::CommandRequest& request,
                                const swarmkit::v1::ExecutionHandle* execution_handle = nullptr);
    void RecordCommandCompleted(const swarmkit::v1::CommandRequest& request,
                                const swarmkit::v1::BackendCommandOutcome& outcome,
                                const swarmkit::v1::ExecutionHandle* execution_handle = nullptr);
    void RecordGoal(const swarmkit::v1::ActiveGoal& goal,
                    const swarmkit::v1::ExecutionHandle& execution_handle,
                    swarmkit::v1::GoalStatus status, std::string detail);
    void RecordReport(const swarmkit::v1::AgentReport& report);
    void RecordAuthority(const swarmkit::v1::AuthorityEvent& authority);
    void Close();

   private:
    void RecordEnvelope(swarmkit::v1::ExecutionEventEnvelope envelope);
    [[nodiscard]] bool WriteEnvelopeLocked(swarmkit::v1::ExecutionEventEnvelope* envelope);
    [[nodiscard]] bool SerializeDeterministically(
        const swarmkit::v1::ExecutionEventEnvelope& envelope, std::string* output) const;
    [[nodiscard]] bool EnsureCapacityLocked(std::size_t record_bytes);
    [[nodiscard]] bool OpenSegmentLocked(bool initial);
    [[nodiscard]] bool RotateLocked();
    void InvalidateLocked(std::string message);
    void SyncLocked();

    ExecutionRecorderOptions options_;
    mutable std::mutex mutex_;
    std::ofstream output_;
    std::size_t segment_bytes_{0};
    std::uint64_t next_event_sequence_{0};
    bool enabled_{false};
    bool valid_{true};
    bool closed_{false};
    std::string failure_message_;
    std::unordered_map<std::string, swarmkit::v1::VehicleHealthEvent> last_health_by_drone_;
};

}  // namespace swarmkit::agent::internal
