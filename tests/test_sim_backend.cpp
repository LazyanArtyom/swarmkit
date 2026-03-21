// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
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

    REQUIRE(backend
                ->StartTelemetry("drone-1", 5,
                                 [&drone_one_frames](const core::TelemetryFrame& frame) {
                                     if (frame.drone_id == "drone-1") {
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

    CHECK(backend->StartTelemetry("drone-1", 5, [](const core::TelemetryFrame&) {}).code ==
          core::StatusCode::kRejected);

    CHECK(backend->StopTelemetry("drone-1").IsOk());
    CHECK(backend->StopTelemetry("drone-2").IsOk());
}

}  // namespace swarmkit::agent
