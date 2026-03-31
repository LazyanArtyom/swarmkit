// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

/// @file runtime_counters.h
/// @brief Lock-free runtime counters for agent server observability.
///
/// Extracted from AgentServiceImpl so they can be reused, tested, and
/// passed to sub-components without coupling to the gRPC service.

#include <atomic>
#include <cstdint>

namespace swarmkit::agent::internal {

/// @brief Point-in-time snapshot of all counters (plain struct, no atomics).
struct CounterSnapshot {
    std::uint64_t ping_requests_total{0};
    std::uint64_t health_requests_total{0};
    std::uint64_t runtime_stats_requests_total{0};
    std::uint64_t command_requests_total{0};
    std::uint64_t command_rejected_total{0};
    std::uint64_t command_failed_total{0};
    std::uint64_t lock_requests_total{0};
    std::uint64_t watch_requests_total{0};
    std::uint64_t current_authority_watchers{0};
    std::uint64_t total_telemetry_subscriptions{0};
    std::uint64_t current_telemetry_streams{0};
    std::uint64_t telemetry_frames_sent_total{0};
    std::uint64_t backend_failures_total{0};
};

/// @brief Lock-free atomic counters for the agent server runtime.
///
/// All methods use relaxed memory ordering — counters are approximate
/// by design (no cross-counter consistency guarantees in Snapshot()).
class RuntimeCounters {
   public:
    void IncrementPingRequests() {
        ping_requests_total_.fetch_add(1, std::memory_order_relaxed);
    }
    void IncrementHealthRequests() {
        health_requests_total_.fetch_add(1, std::memory_order_relaxed);
    }
    void IncrementRuntimeStatsRequests() {
        runtime_stats_requests_total_.fetch_add(1, std::memory_order_relaxed);
    }
    void IncrementCommandRequests() {
        command_requests_total_.fetch_add(1, std::memory_order_relaxed);
    }
    void IncrementCommandRejected() {
        command_rejected_total_.fetch_add(1, std::memory_order_relaxed);
    }
    void IncrementCommandFailed() {
        command_failed_total_.fetch_add(1, std::memory_order_relaxed);
    }
    void IncrementLockRequests() {
        lock_requests_total_.fetch_add(1, std::memory_order_relaxed);
    }
    void IncrementWatchRequests() {
        watch_requests_total_.fetch_add(1, std::memory_order_relaxed);
    }
    void IncrementBackendFailures() {
        backend_failures_total_.fetch_add(1, std::memory_order_relaxed);
    }

    void IncrementAuthorityWatchers() {
        current_authority_watchers_.fetch_add(1, std::memory_order_relaxed);
    }
    void DecrementAuthorityWatchers() {
        current_authority_watchers_.fetch_sub(1, std::memory_order_relaxed);
    }

    void IncrementFramesSent() {
        telemetry_frames_sent_total_.fetch_add(1, std::memory_order_relaxed);
    }

    /// @brief Set stream counters from TelemetryManager (called during snapshot).
    void SetTelemetryCounters(std::uint64_t total_subscriptions, std::uint64_t current_streams,
                              std::uint64_t frames_sent,
                              std::uint64_t telemetry_backend_failures) {
        total_telemetry_subscriptions_.store(total_subscriptions, std::memory_order_relaxed);
        current_telemetry_streams_.store(current_streams, std::memory_order_relaxed);
        telemetry_frames_sent_total_.store(frames_sent, std::memory_order_relaxed);
        telemetry_backend_failures_.store(telemetry_backend_failures, std::memory_order_relaxed);
    }

    /// @brief Take a point-in-time snapshot of all counters.
    [[nodiscard]] CounterSnapshot Snapshot() const {
        return CounterSnapshot{
            .ping_requests_total = ping_requests_total_.load(std::memory_order_relaxed),
            .health_requests_total = health_requests_total_.load(std::memory_order_relaxed),
            .runtime_stats_requests_total =
                runtime_stats_requests_total_.load(std::memory_order_relaxed),
            .command_requests_total = command_requests_total_.load(std::memory_order_relaxed),
            .command_rejected_total = command_rejected_total_.load(std::memory_order_relaxed),
            .command_failed_total = command_failed_total_.load(std::memory_order_relaxed),
            .lock_requests_total = lock_requests_total_.load(std::memory_order_relaxed),
            .watch_requests_total = watch_requests_total_.load(std::memory_order_relaxed),
            .current_authority_watchers =
                current_authority_watchers_.load(std::memory_order_relaxed),
            .total_telemetry_subscriptions =
                total_telemetry_subscriptions_.load(std::memory_order_relaxed),
            .current_telemetry_streams =
                current_telemetry_streams_.load(std::memory_order_relaxed),
            .telemetry_frames_sent_total =
                telemetry_frames_sent_total_.load(std::memory_order_relaxed),
            .backend_failures_total = backend_failures_total_.load(std::memory_order_relaxed) +
                                      telemetry_backend_failures_.load(std::memory_order_relaxed),
        };
    }

   private:
    std::atomic<std::uint64_t> ping_requests_total_{0};
    std::atomic<std::uint64_t> health_requests_total_{0};
    std::atomic<std::uint64_t> runtime_stats_requests_total_{0};
    std::atomic<std::uint64_t> command_requests_total_{0};
    std::atomic<std::uint64_t> command_rejected_total_{0};
    std::atomic<std::uint64_t> command_failed_total_{0};
    std::atomic<std::uint64_t> lock_requests_total_{0};
    std::atomic<std::uint64_t> watch_requests_total_{0};
    std::atomic<std::uint64_t> current_authority_watchers_{0};
    std::atomic<std::uint64_t> total_telemetry_subscriptions_{0};
    std::atomic<std::uint64_t> current_telemetry_streams_{0};
    std::atomic<std::uint64_t> telemetry_frames_sent_total_{0};
    std::atomic<std::uint64_t> backend_failures_total_{0};
    std::atomic<std::uint64_t> telemetry_backend_failures_{0};
};

}  // namespace swarmkit::agent::internal
