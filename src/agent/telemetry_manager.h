// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

/// @file telemetry_manager.h
/// @brief Manages per-drone telemetry state and backend lifecycle.
///
/// Extracted from AgentServiceImpl to separate telemetry stream management
/// from RPC handling.  This is an internal implementation detail.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "runtime_providers.h"
#include "swarmkit/agent/backend.h"
#include "swarmkit/core/logger.h"
#include "swarmkit/core/telemetry.h"

namespace swarmkit::agent::internal {

/// @brief Shared state for a single drone's telemetry stream.
///
/// @p data_mutex protects the cached frame; @p control_mutex protects
/// subscriber tracking and backend lifecycle.
struct TelemetryState {
    std::mutex control_mutex;
    std::mutex data_mutex;
    std::condition_variable data_cv;
    std::deque<core::TelemetryFrame> retained_frames;
    std::uint64_t sequence{0};
    std::unordered_map<std::uint64_t, int> subscriber_rates_hz;
    bool backend_running{false};
    int backend_rate_hz{0};
    bool shutting_down{false};
};

struct TelemetryCursor {
    std::uint64_t after_sequence{0};
    std::string expected_agent_session_id;
};

struct TelemetryReplaySnapshot {
    std::uint64_t requested_after_sequence{0};
    std::uint64_t oldest_available_sequence{0};
    std::uint64_t latest_available_sequence{0};
    bool session_mismatch{false};
    bool history_evicted{false};
    bool cursor_ahead{false};
    std::vector<core::TelemetryFrame> frames;
};

enum class TelemetryReadStatus : std::uint8_t {
    kFrame,
    kTimeout,
    kHistoryEvicted,
    kShutdown,
};

struct TelemetryReadResult {
    TelemetryReadStatus status{TelemetryReadStatus::kTimeout};
    core::TelemetryFrame frame;
    std::uint64_t oldest_available_sequence{0};
    std::uint64_t latest_available_sequence{0};
};

/// @brief Handle returned by TelemetryManager::AcquireLease().
///
/// Callers read telemetry frames via @c state and must call
/// TelemetryManager::ReleaseLease() when done.
struct TelemetryLease {
    std::shared_ptr<TelemetryState> state;
    std::string drone_id;
    std::uint64_t subscriber_id{0};
    std::uint64_t initial_sequence{0};
    TelemetryReplaySnapshot replay;
};

/// @brief Manages per-drone telemetry streams for the agent server.
///
/// Each gRPC StreamTelemetry call acquires a lease. The manager ensures
/// the backend is started at the highest requested rate and stopped when
/// the last subscriber disconnects.
class TelemetryManager {
   public:
    /// @param backend         Drone backend (owned externally; must outlive this manager).
    /// @param default_rate_hz Default rate when client requests 0 or negative.
    /// @param min_rate_hz     Floor rate; all requests are clamped to at least this.
    using ExecutionSnapshotProvider =
        std::function<std::optional<core::ExecutionHandle>(const std::string&)>;
    using NormalizedFrameObserver = std::function<void(const core::TelemetryFrame&)>;

    TelemetryManager(IDroneBackend* backend, int default_rate_hz, int min_rate_hz,
                     std::size_t retention_frames, RuntimeProviders providers,
                     std::string agent_session_id)
        : backend_(backend),
          default_rate_hz_(std::max(1, default_rate_hz)),
          min_rate_hz_(std::max(1, min_rate_hz)),
          retention_frames_(std::max<std::size_t>(1, retention_frames)),
          providers_(std::move(providers)),
          agent_session_id_(std::move(agent_session_id)) {}

    ~TelemetryManager() {
        try {
            ShutdownAll();
        } catch (...) {
            static_cast<void>(0);
        }
    }

    TelemetryManager(const TelemetryManager&) = delete;
    TelemetryManager& operator=(const TelemetryManager&) = delete;

    /// Set once during Agent construction, before telemetry is started.
    void SetExecutionSnapshotProvider(ExecutionSnapshotProvider provider) {
        execution_snapshot_provider_ = std::move(provider);
    }

    void SetNormalizedFrameObserver(NormalizedFrameObserver observer) {
        normalized_frame_observer_ = std::move(observer);
    }

    /// @brief Acquire a telemetry lease for a drone, starting the backend if needed.
    [[nodiscard]] core::Result AcquireLease(const std::string& drone_id, int requested_rate_hz,
                                            const TelemetryCursor& cursor,
                                            TelemetryLease* out_lease);

    /// @brief Release a telemetry lease, stopping the backend if this was the last subscriber.
    void ReleaseLease(const TelemetryLease& lease);

    /// @brief Wait for the first retained frame after @p last_sequence.
    [[nodiscard]] static TelemetryReadResult WaitForFrame(const TelemetryLease& lease,
                                                          std::uint64_t last_sequence,
                                                          std::chrono::milliseconds timeout);

    /// @brief Total number of backend start failures.
    [[nodiscard]] std::uint64_t BackendFailureCount() const {
        return backend_failure_count_.load(std::memory_order_relaxed);
    }

    /// @brief Number of active telemetry subscriptions across all drones.
    [[nodiscard]] std::uint64_t ActiveStreamCount() const {
        return active_stream_count_.load(std::memory_order_relaxed);
    }

    /// @brief Total number of subscriptions ever created.
    [[nodiscard]] std::uint64_t TotalSubscriptionCount() const {
        return total_subscription_count_.load(std::memory_order_relaxed);
    }

    /// @brief Total telemetry frames published across all drones (for counter tracking).
    void IncrementFramesSent() {
        frames_sent_total_.fetch_add(1, std::memory_order_relaxed);
    }

    [[nodiscard]] std::uint64_t FramesSentTotal() const {
        return frames_sent_total_.load(std::memory_order_relaxed);
    }

   private:
    [[nodiscard]] std::shared_ptr<TelemetryState> GetOrCreateState(const std::string& drone_id);

    void PublishFrame(const std::shared_ptr<TelemetryState>& state, const std::string& drone_id,
                      const core::TelemetryFrame& frame);

    void PublishFrameNoexcept(const std::shared_ptr<TelemetryState>& state,
                              const std::string& drone_id,
                              const core::TelemetryFrame& frame) noexcept;

    [[nodiscard]] int NormalizeRate(int requested_rate_hz) const;

    void ShutdownAll();

    IDroneBackend* backend_;
    int default_rate_hz_;
    int min_rate_hz_;
    std::size_t retention_frames_;
    RuntimeProviders providers_;
    std::string agent_session_id_;
    ExecutionSnapshotProvider execution_snapshot_provider_;
    NormalizedFrameObserver normalized_frame_observer_;

    std::mutex states_mutex_;
    std::unordered_map<std::string, std::shared_ptr<TelemetryState>> states_;
    std::atomic<std::uint64_t> next_subscriber_id_{1};
    std::atomic<std::uint64_t> backend_failure_count_{0};
    std::atomic<std::uint64_t> active_stream_count_{0};
    std::atomic<std::uint64_t> total_subscription_count_{0};
    std::atomic<std::uint64_t> frames_sent_total_{0};
};

}  // namespace swarmkit::agent::internal
