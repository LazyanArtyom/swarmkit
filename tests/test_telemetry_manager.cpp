// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <stdexcept>

#include "../src/agent/telemetry_manager.h"
#include "test_support.h"

namespace swarmkit::agent::internal {
namespace {

TEST_CASE("Telemetry ingress contains observer exceptions at the backend callback boundary",
          "[agent][telemetry][shutdown][safety]") {
    testsupport::RecordingBackend backend;
    TelemetryManager manager(&backend, 5, 1, 8, RuntimeProviders::System(), "session-1");
    manager.SetNormalizedFrameObserver(
        [](const core::TelemetryFrame&) { throw std::runtime_error("observer failure"); });

    TelemetryLease lease;
    REQUIRE(manager.AcquireLease("drone-1", 5, {}, &lease).IsOk());

    core::TelemetryFrame frame;
    REQUIRE_NOTHROW(backend.EmitTelemetry("drone-1", frame));
    CHECK(manager.BackendFailureCount() == 1);

    REQUIRE_NOTHROW(backend.EmitTelemetry("drone-1", frame));
    CHECK(manager.BackendFailureCount() == 2);
    const TelemetryReadResult read =
        TelemetryManager::WaitForFrame(lease, 1, std::chrono::milliseconds{0});
    REQUIRE(read.status == TelemetryReadStatus::kFrame);
    CHECK(read.frame.telemetry_sequence == 2);

    manager.ReleaseLease(lease);
}

}  // namespace
}  // namespace swarmkit::agent::internal
