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

std::shared_ptr<TelemetryState> TelemetryManager::GetOrCreateState(const std::string& drone_id) {
    std::lock_guard<std::mutex> lock(states_mutex_);
    auto& state = states_[drone_id];
    if (!state) {
        state = std::make_shared<TelemetryState>();
    }
    return state;
}

void TelemetryManager::PublishFrame(const std::shared_ptr<TelemetryState>& state,
                                    const core::TelemetryFrame& frame) {
    {
        std::lock_guard<std::mutex> lock(state->data_mutex);
        state->last_frame = frame;
        ++state->sequence;
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
                                             TelemetryLease* out_lease) {
    if (out_lease == nullptr) {
        return core::Result::Failed("telemetry lease output is null");
    }

    const int kNormalizedRate = NormalizeRate(requested_rate_hz);
    auto state = GetOrCreateState(drone_id);
    const std::uint64_t kSubscriberId =
        next_subscriber_id_.fetch_add(1, std::memory_order_relaxed);

    std::lock_guard<std::mutex> lock(state->control_mutex);
    state->subscriber_rates_hz[kSubscriberId] = kNormalizedRate;

    // Determine the desired backend rate (max of all active subscribers).
    int desired_backend_rate = kNormalizedRate;
    for (const auto& [entry_id, rate_hz] : state->subscriber_rates_hz) {
        static_cast<void>(entry_id);
        desired_backend_rate = std::max(desired_backend_rate, rate_hz);
    }

    if (!state->backend_running) {
        const core::Result kStartResult = backend_->StartTelemetry(
            drone_id, desired_backend_rate,
            [state](const core::TelemetryFrame& frame) { PublishFrame(state, frame); });

        if (!kStartResult.IsOk()) {
            backend_failure_count_.fetch_add(1, std::memory_order_relaxed);
            state->subscriber_rates_hz.erase(kSubscriberId);
            return core::Result::Failed("backend StartTelemetry failed: " + kStartResult.message);
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

        const core::Result kStartResult = backend_->StartTelemetry(
            drone_id, desired_backend_rate,
            [state](const core::TelemetryFrame& frame) { PublishFrame(state, frame); });

        if (!kStartResult.IsOk()) {
            backend_failure_count_.fetch_add(1, std::memory_order_relaxed);
            core::Logger::ErrorFmt(
                "TelemetryManager: failed to raise rate for drone '{}' from {}Hz to {}Hz: {}",
                drone_id, kPreviousRate, desired_backend_rate, kStartResult.message);

            // Attempt to restore the previous rate.
            const core::Result kRestoreResult = backend_->StartTelemetry(
                drone_id, kPreviousRate,
                [state](const core::TelemetryFrame& frame) { PublishFrame(state, frame); });

            if (kRestoreResult.IsOk()) {
                state->backend_running = true;
                state->backend_rate_hz = kPreviousRate;
            } else {
                state->backend_running = false;
                state->backend_rate_hz = 0;
            }

            state->subscriber_rates_hz.erase(kSubscriberId);
            return core::Result::Failed("backend telemetry reconfigure failed: " +
                                        kStartResult.message);
        }

        state->backend_running = true;
        state->backend_rate_hz = desired_backend_rate;
    }

    total_subscription_count_.fetch_add(1, std::memory_order_relaxed);
    active_stream_count_.fetch_add(1, std::memory_order_relaxed);

    out_lease->state = std::move(state);
    out_lease->drone_id = drone_id;
    out_lease->subscriber_id = kSubscriberId;
    return core::Result::Success();
}

void TelemetryManager::ReleaseLease(const TelemetryLease& lease) {
    if (!lease.state) {
        return;
    }

    active_stream_count_.fetch_sub(1, std::memory_order_relaxed);
    bool should_erase_state = false;

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

            should_erase_state = true;
        }
    }

    if (should_erase_state) {
        {
            std::lock_guard<std::mutex> data_lock(lease.state->data_mutex);
            lease.state->shutting_down = true;
            lease.state->last_frame.reset();
            lease.state->sequence = 0;
        }
        lease.state->data_cv.notify_all();

        std::lock_guard<std::mutex> map_lock(states_mutex_);
        auto iter = states_.find(lease.drone_id);
        if (iter != states_.end() && iter->second == lease.state) {
            states_.erase(iter);
        }
    }
}

bool TelemetryManager::ReadFrame(const TelemetryLease& lease, std::uint64_t* last_sequence,
                                  core::TelemetryFrame* out_frame) {
    if (!lease.state || last_sequence == nullptr || out_frame == nullptr) {
        return false;
    }

    std::lock_guard<std::mutex> lock(lease.state->data_mutex);
    if (lease.state->last_frame.has_value() && lease.state->sequence != *last_sequence) {
        *out_frame = *lease.state->last_frame;
        *last_sequence = lease.state->sequence;
        return true;
    }
    return false;
}

bool TelemetryManager::WaitForFrame(const TelemetryLease& lease, std::uint64_t* last_sequence,
                                    core::TelemetryFrame* out_frame,
                                    std::chrono::milliseconds timeout) {
    if (!lease.state || last_sequence == nullptr || out_frame == nullptr) {
        return false;
    }

    std::unique_lock<std::mutex> lock(lease.state->data_mutex);
    const bool kReady = lease.state->data_cv.wait_for(lock, timeout, [&] {
        return lease.state->shutting_down ||
               (lease.state->last_frame.has_value() && lease.state->sequence != *last_sequence);
    });
    if (!kReady || lease.state->shutting_down) {
        return false;
    }

    *out_frame = *lease.state->last_frame;
    *last_sequence = lease.state->sequence;
    return true;
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
        const core::Result kStopResult = backend_->StopTelemetry(drone_id);
        if (!kStopResult.IsOk()) {
            core::Logger::WarnFmt("TelemetryManager: StopTelemetry('{}') failed during shutdown: {}",
                                  drone_id, kStopResult.message);
        }
    }
}

}  // namespace swarmkit::agent::internal
