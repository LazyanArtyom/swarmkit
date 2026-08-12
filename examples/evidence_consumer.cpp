// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <future>
#include <iostream>
#include <string>
#include <variant>

#include "swarmkit/client/client.h"

namespace {

void PrintUncertainty(const char* name,
                      const std::optional<swarmkit::core::UncertaintyEstimate>& estimate) {
    if (!estimate.has_value()) {
        std::cout << name << "=unknown ";
        return;
    }
    std::cout << name << '=' << estimate->value
              << " semantics=" << static_cast<int>(estimate->descriptor.semantics) << ' ';
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 6) {
        std::cerr << "usage: " << argv[0]
                  << " AGENT_ADDRESS DRONE_ID TARGET_LAT TARGET_LON TARGET_ALT_M\n";
        return EXIT_FAILURE;
    }

    swarmkit::client::ClientConfig config;
    config.address = argv[1];
    config.client_id = "external-controller-example";
    config.security.transport_security = swarmkit::core::TransportSecurityMode::kInsecure;
    swarmkit::client::Client client(config);

    swarmkit::client::ActiveGoalRequest request;
    request.goal = {
        .drone_id = argv[2],
        .goal_id = "physical-transition-goal",
        .revision = 1,
        .target = {.lat_deg = std::stod(argv[3]),
                   .lon_deg = std::stod(argv[4]),
                   .alt_m = std::stod(argv[5])},
        .speed_mps = 2.0F,
        .acceptance_radius_m = 1.0F,
        .deviation_radius_m = 5.0F,
    };
    request.execution_context = swarmkit::core::ExecutionContext{
        .mission_id = "mission-example",
        .mission_revision = 1,
        .model_hash = "replace-with-canonical-model-hash",
        .operation_id = "operation-example",
        .operation_attempt_revision = 1,
    };

    const swarmkit::client::GoalResult goal = client.SetActiveGoal(request);
    if (!goal.ok || !goal.execution_handle.has_value()) {
        std::cerr << "goal dispatch failed: " << goal.message << '\n';
        return EXIT_FAILURE;
    }
    const swarmkit::core::ExecutionHandle expected = *goal.execution_handle;
    std::cout << "attempt=" << expected.physical_attempt_id
              << " revision=" << expected.physical_attempt_revision
              << " session=" << expected.agent_session_id << '\n';

    std::promise<void> observed_current_attempt;
    auto observed = observed_current_attempt.get_future();
    std::atomic<bool> completed{false};
    auto subscription = client.StartTelemetry(
        {.drone_id = expected.drone_id, .rate_hertz = 20},
        [expected, &observed_current_attempt,
         &completed](const swarmkit::client::TelemetryObservation& observation) {
            const auto* sample =
                std::get_if<swarmkit::client::TelemetryFrameObservation>(&observation);
            if (sample == nullptr) {
                return;
            }
            const auto& frame = sample->delivery.frame;
            if (!frame.execution_handle.has_value() || *frame.execution_handle != expected) {
                return;  // Unbound, stale, superseded, or old-session evidence.
            }

            std::cout << "sequence=" << frame.telemetry_sequence
                      << " relation=" << static_cast<int>(sample->sequence_relation)
                      << " source_time="
                      << frame.provenance.position.source_time.timestamp_ms.value_or(-1)
                      << " agent_receive=" << frame.agent_receive_unix_time_ms << ' ';
            PrintUncertainty("position_xy", frame.accuracy.horizontal_position);
            PrintUncertainty("position_z", frame.accuracy.vertical_position);
            std::cout << "estimator=" << static_cast<int>(frame.estimator_state)
                      << " failsafe=" << frame.failsafe << '\n';

            // An external TCRR layer would now evaluate its own freshness,
            // enclosure, velocity, hold-window, and guarded-commit rules.
            if (!completed.exchange(true)) {
                observed_current_attempt.set_value();
            }
        });
    if (!subscription.has_value()) {
        std::cerr << "telemetry subscription failed: " << subscription.error().user_message << '\n';
        return EXIT_FAILURE;
    }
    if (observed.wait_for(std::chrono::seconds{30}) != std::future_status::ready) {
        std::cerr << "no current-attempt telemetry observed before timeout\n";
        subscription->Stop();
        return EXIT_FAILURE;
    }
    subscription->Stop();
    return EXIT_SUCCESS;
}
