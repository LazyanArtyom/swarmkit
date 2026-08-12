// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "telemetry_manager.h"

#include <algorithm>
#include <string>
#include <vector>

namespace swarmkit::agent::internal {
namespace {

void NormalizeMeasurement(core::MeasurementProvenance* current,
                          const core::MeasurementProvenance* previous, std::int64_t receive_wall_ms,
                          std::int64_t receive_monotonic_ns) {
    if (current == nullptr) {
        return;
    }

    if (!current->updated) {
        if (previous != nullptr) {
            *current = *previous;
            current->updated = false;
        } else {
            current->updated = false;
        }
        return;
    }

    current->updated = true;
    current->generation = previous == nullptr ? 1 : previous->generation + 1;
    current->agent_receive_unix_time_ms = receive_wall_ms;
    current->agent_receive_monotonic_time_ns = receive_monotonic_ns;
    if (current->source.empty()) {
        current->source = "backend.unspecified";
    }
}

}  // namespace

std::shared_ptr<TelemetryState> TelemetryManager::GetOrCreateState(const std::string& drone_id) {
    std::lock_guard<std::mutex> lock(states_mutex_);
    auto& state = states_[drone_id];
    if (!state) {
        state = std::make_shared<TelemetryState>();
    }
    return state;
}

void TelemetryManager::PublishFrame(const std::shared_ptr<TelemetryState>& state,
                                    const std::string& drone_id,
                                    const core::TelemetryFrame& frame) {
    core::TelemetryFrame normalized = frame;
    normalized.drone_id = drone_id;
    normalized.agent_session_id = agent_session_id_;
    const std::int64_t receive_wall_ms = providers_.WallTimeMs();
    const std::int64_t receive_monotonic_ns = providers_.MonotonicTimeNs();
    normalized.agent_receive_unix_time_ms = receive_wall_ms;
    normalized.agent_receive_monotonic_time_ns = receive_monotonic_ns;
    normalized.execution_handle.reset();
    if (execution_snapshot_provider_) {
        normalized.execution_handle = execution_snapshot_provider_(drone_id);
    }
    {
        std::lock_guard<std::mutex> lock(state->data_mutex);
        const core::TelemetryFrame* previous =
            state->retained_frames.empty() ? nullptr : &state->retained_frames.back();
        NormalizeMeasurement(&normalized.provenance.position,
                             previous == nullptr ? nullptr : &previous->provenance.position,
                             receive_wall_ms, receive_monotonic_ns);
        NormalizeMeasurement(&normalized.provenance.velocity,
                             previous == nullptr ? nullptr : &previous->provenance.velocity,
                             receive_wall_ms, receive_monotonic_ns);
        NormalizeMeasurement(&normalized.provenance.accuracy,
                             previous == nullptr ? nullptr : &previous->provenance.accuracy,
                             receive_wall_ms, receive_monotonic_ns);
        NormalizeMeasurement(&normalized.provenance.estimator,
                             previous == nullptr ? nullptr : &previous->provenance.estimator,
                             receive_wall_ms, receive_monotonic_ns);
        NormalizeMeasurement(&normalized.provenance.vehicle_state,
                             previous == nullptr ? nullptr : &previous->provenance.vehicle_state,
                             receive_wall_ms, receive_monotonic_ns);
        const std::uint64_t accuracy_generation = normalized.provenance.accuracy.generation;
        auto set_generation =
            [accuracy_generation](std::optional<core::UncertaintyEstimate>* estimate) {
                if (estimate != nullptr && estimate->has_value()) {
                    (*estimate)->descriptor.measurement_generation = accuracy_generation;
                }
            };
        set_generation(&normalized.accuracy.horizontal_position);
        set_generation(&normalized.accuracy.vertical_position);
        set_generation(&normalized.accuracy.horizontal_velocity);
        set_generation(&normalized.accuracy.vertical_velocity);
        set_generation(&normalized.accuracy.speed);
        normalized.telemetry_sequence = ++state->sequence;
        state->retained_frames.push_back(normalized);
        while (state->retained_frames.size() > retention_frames_) {
            state->retained_frames.pop_front();
        }
        if (normalized_frame_observer_) {
            normalized_frame_observer_(normalized);
        }
    }
    state->data_cv.notify_all();
}

int TelemetryManager::NormalizeRate(int requested_rate_hz) const {
    if (requested_rate_hz <= 0) {
        return default_rate_hz_;
    }
    return std::max(min_rate_hz_, requested_rate_hz);
}

core::Result TelemetryManager::AcquireLease(const std::string& drone_id, int requested_rate_hz,
                                            const TelemetryCursor& cursor,
                                            TelemetryLease* out_lease) {
    if (out_lease == nullptr) {
        return core::Result::Failed("telemetry lease output is null");
    }

    const int kNormalizedRate = NormalizeRate(requested_rate_hz);
    auto state = GetOrCreateState(drone_id);
    const std::uint64_t kSubscriberId = next_subscriber_id_.fetch_add(1, std::memory_order_relaxed);
    std::lock_guard<std::mutex> lock(state->control_mutex);
    state->subscriber_rates_hz[kSubscriberId] = kNormalizedRate;

    // Determine the desired backend rate (max of all active subscribers).
    int desired_backend_rate = kNormalizedRate;
    for (const auto& [entry_id, rate_hz] : state->subscriber_rates_hz) {
        static_cast<void>(entry_id);
        desired_backend_rate = std::max(desired_backend_rate, rate_hz);
    }

    if (!state->backend_running) {
        const core::Result kStartResult =
            backend_->StartTelemetry(drone_id, desired_backend_rate,
                                     [this, state, drone_id](const core::TelemetryFrame& frame) {
                                         PublishFrame(state, drone_id, frame);
                                     });

        if (!kStartResult.IsOk()) {
            backend_failure_count_.fetch_add(1, std::memory_order_relaxed);
            state->subscriber_rates_hz.erase(kSubscriberId);
            const std::string message = "backend StartTelemetry failed: " + kStartResult.message;
            return kStartResult.code == core::StatusCode::kRejected
                       ? core::Result::Rejected(message)
                       : core::Result::Failed(message);
        }

        state->backend_running = true;
        state->backend_rate_hz = desired_backend_rate;
    } else if (desired_backend_rate > state->backend_rate_hz) {
        // Need to reconfigure backend at a higher rate.
        const int kPreviousRate = state->backend_rate_hz;

        const core::Result kStopResult = backend_->StopTelemetry(drone_id);
        if (!kStopResult.IsOk()) {
            backend_failure_count_.fetch_add(1, std::memory_order_relaxed);
            state->subscriber_rates_hz.erase(kSubscriberId);
            return core::Result::Failed("backend StopTelemetry failed during reconfigure: " +
                                        kStopResult.message);
        }

        const core::Result kStartResult =
            backend_->StartTelemetry(drone_id, desired_backend_rate,
                                     [this, state, drone_id](const core::TelemetryFrame& frame) {
                                         PublishFrame(state, drone_id, frame);
                                     });

        if (!kStartResult.IsOk()) {
            backend_failure_count_.fetch_add(1, std::memory_order_relaxed);
            core::Logger::ErrorFmt(
                "TelemetryManager: failed to raise rate for drone '{}' from {}Hz to {}Hz: {}",
                drone_id, kPreviousRate, desired_backend_rate, kStartResult.message);

            // Attempt to restore the previous rate.
            const core::Result kRestoreResult = backend_->StartTelemetry(
                drone_id, kPreviousRate,
                [this, state, drone_id](const core::TelemetryFrame& frame) {
                    PublishFrame(state, drone_id, frame);
                });

            if (kRestoreResult.IsOk()) {
                state->backend_running = true;
                state->backend_rate_hz = kPreviousRate;
            } else {
                state->backend_running = false;
                state->backend_rate_hz = 0;
            }

            state->subscriber_rates_hz.erase(kSubscriberId);
            const std::string message =
                "backend telemetry reconfigure failed: " + kStartResult.message;
            return kStartResult.code == core::StatusCode::kRejected
                       ? core::Result::Rejected(message)
                       : core::Result::Failed(message);
        }

        state->backend_running = true;
        state->backend_rate_hz = desired_backend_rate;
    }

    total_subscription_count_.fetch_add(1, std::memory_order_relaxed);
    active_stream_count_.fetch_add(1, std::memory_order_relaxed);

    TelemetryReplaySnapshot replay;
    replay.requested_after_sequence = cursor.after_sequence;
    {
        std::lock_guard<std::mutex> data_lock(state->data_mutex);
        replay.latest_available_sequence = state->sequence;
        replay.oldest_available_sequence =
            state->retained_frames.empty() ? 0 : state->retained_frames.front().telemetry_sequence;
        replay.session_mismatch = !cursor.expected_agent_session_id.empty() &&
                                  cursor.expected_agent_session_id != agent_session_id_;
        replay.cursor_ahead =
            !replay.session_mismatch && cursor.after_sequence > replay.latest_available_sequence;
        const std::uint64_t effective_after =
            replay.session_mismatch
                ? 0
                : (replay.cursor_ahead ? replay.latest_available_sequence : cursor.after_sequence);
        replay.history_evicted = !replay.session_mismatch && cursor.after_sequence > 0 &&
                                 replay.oldest_available_sequence > 1 &&
                                 cursor.after_sequence < replay.oldest_available_sequence - 1;
        for (const auto& retained : state->retained_frames) {
            if (retained.telemetry_sequence > effective_after) {
                replay.frames.push_back(retained);
            }
        }
    }

    out_lease->state = std::move(state);
    out_lease->drone_id = drone_id;
    out_lease->subscriber_id = kSubscriberId;
    out_lease->initial_sequence = replay.latest_available_sequence;
    out_lease->replay = std::move(replay);
    return core::Result::Success();
}

void TelemetryManager::ReleaseLease(const TelemetryLease& lease) {
    if (!lease.state) {
        return;
    }

    active_stream_count_.fetch_sub(1, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(lease.state->control_mutex);
        lease.state->subscriber_rates_hz.erase(lease.subscriber_id);

        if (lease.state->subscriber_rates_hz.empty()) {
            if (lease.state->backend_running) {
                const core::Result kStopResult = backend_->StopTelemetry(lease.drone_id);
                if (!kStopResult.IsOk()) {
                    backend_failure_count_.fetch_add(1, std::memory_order_relaxed);
                    core::Logger::WarnFmt("TelemetryManager: StopTelemetry('{}') failed: {}",
                                          lease.drone_id, kStopResult.message);
                }
                lease.state->backend_running = false;
                lease.state->backend_rate_hz = 0;
            }
        }
    }
}

TelemetryReadResult TelemetryManager::WaitForFrame(const TelemetryLease& lease,
                                                   std::uint64_t last_sequence,
                                                   std::chrono::milliseconds timeout) {
    TelemetryReadResult result;
    if (!lease.state) {
        result.status = TelemetryReadStatus::kShutdown;
        return result;
    }

    std::unique_lock<std::mutex> lock(lease.state->data_mutex);
    const bool kReady = lease.state->data_cv.wait_for(lock, timeout, [&] {
        return lease.state->shutting_down ||
               (!lease.state->retained_frames.empty() && lease.state->sequence > last_sequence);
    });
    if (!kReady) {
        result.status = TelemetryReadStatus::kTimeout;
        return result;
    }
    if (lease.state->shutting_down) {
        result.status = TelemetryReadStatus::kShutdown;
        return result;
    }

    result.oldest_available_sequence = lease.state->retained_frames.front().telemetry_sequence;
    result.latest_available_sequence = lease.state->retained_frames.back().telemetry_sequence;
    if (last_sequence > 0 && result.oldest_available_sequence > 1 &&
        last_sequence < result.oldest_available_sequence - 1) {
        result.status = TelemetryReadStatus::kHistoryEvicted;
        return result;
    }
    const auto iter = std::ranges::find_if(lease.state->retained_frames,
                                           [last_sequence](const core::TelemetryFrame& candidate) {
                                               return candidate.telemetry_sequence > last_sequence;
                                           });
    if (iter == lease.state->retained_frames.end()) {
        result.status = TelemetryReadStatus::kTimeout;
        return result;
    }
    result.status = TelemetryReadStatus::kFrame;
    result.frame = *iter;
    return result;
}

void TelemetryManager::ShutdownAll() {
    std::vector<std::string> drone_ids;
    {
        std::lock_guard<std::mutex> lock(states_mutex_);
        drone_ids.reserve(states_.size());
        for (const auto& [drone_id, state] : states_) {
            {
                std::lock_guard<std::mutex> data_lock(state->data_mutex);
                state->shutting_down = true;
            }
            state->data_cv.notify_all();
            drone_ids.push_back(drone_id);
        }
    }

    for (const auto& drone_id : drone_ids) {
        std::shared_ptr<TelemetryState> state;
        {
            std::lock_guard<std::mutex> lock(states_mutex_);
            const auto iter = states_.find(drone_id);
            if (iter != states_.end()) {
                state = iter->second;
            }
        }
        if (!state || !state->backend_running) {
            continue;
        }
        const core::Result kStopResult = backend_->StopTelemetry(drone_id);
        if (!kStopResult.IsOk()) {
            core::Logger::WarnFmt(
                "TelemetryManager: StopTelemetry('{}') failed during shutdown: {}", drone_id,
                kStopResult.message);
        }
    }
}

}  // namespace swarmkit::agent::internal
