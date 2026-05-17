// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <mutex>
#include <optional>
#include <thread>

#include "swarmkit/agent/sim_backend.h"

namespace swarmkit::agent {
namespace {

constexpr auto kWaitTimeout = std::chrono::seconds{2};
constexpr auto kPollInterval = std::chrono::milliseconds{20};

}  // namespace

TEST_CASE("SimBackend supports concurrent telemetry streams per drone", "[agent][sim]") {
    auto backend = MakeSimBackend();

    std::atomic<int> drone_one_frames{0};
    std::atomic<int> drone_two_frames{0};
    std::mutex frame_mutex;
    std::optional<core::TelemetryFrame> first_frame;

    REQUIRE(backend
                ->StartTelemetry("drone-1", 5,
                                 [&drone_one_frames, &frame_mutex,
                                  &first_frame](const core::TelemetryFrame& frame) {
                                     if (frame.drone_id == "drone-1") {
                                         std::lock_guard<std::mutex> lock(frame_mutex);
                                         if (!first_frame.has_value()) {
                                             first_frame = frame;
                                         }
                                         drone_one_frames.fetch_add(1, std::memory_order_relaxed);
                                     }
                                 })
                .IsOk());
    REQUIRE(backend
                ->StartTelemetry("drone-2", 5,
                                 [&drone_two_frames](const core::TelemetryFrame& frame) {
                                     if (frame.drone_id == "drone-2") {
                                         drone_two_frames.fetch_add(1, std::memory_order_relaxed);
                                     }
                                 })
                .IsOk());

    const auto kDeadline = std::chrono::steady_clock::now() + kWaitTimeout;
    while (std::chrono::steady_clock::now() < kDeadline &&
           (drone_one_frames.load(std::memory_order_relaxed) < 2 ||
            drone_two_frames.load(std::memory_order_relaxed) < 2)) {
        std::this_thread::sleep_for(kPollInterval);
    }

    CHECK(drone_one_frames.load(std::memory_order_relaxed) >= 2);
    CHECK(drone_two_frames.load(std::memory_order_relaxed) >= 2);
    {
        std::lock_guard<std::mutex> lock(frame_mutex);
        REQUIRE(first_frame.has_value());
        const core::TelemetryFrame frame = first_frame.value_or(core::TelemetryFrame{});
        CHECK(frame.HasPosition());
        CHECK(frame.HasRelativeAltitude());
        CHECK(frame.HasGpsQuality());
        CHECK(frame.position_frame == core::CoordinateFrame::kWgs84);
        CHECK(frame.velocity_frame == core::CoordinateFrame::kLocalNed);
        CHECK(frame.gps_quality == core::GpsQuality::kFix3D);
        CHECK(frame.estimator_state == core::EstimatorState::kHealthy);
        CHECK(frame.accuracy.horizontal_position_valid);
        CHECK(frame.validity.home_origin);
    }

    CHECK(backend->StartTelemetry("drone-1", 5, [](const core::TelemetryFrame&) {}).code ==
          core::StatusCode::kRejected);

    CHECK(backend->StopTelemetry("drone-1").IsOk());
    CHECK(backend->StopTelemetry("drone-2").IsOk());
}

}  // namespace swarmkit::agent
