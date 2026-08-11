// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <string>

#include "../src/core/env_utils.h"
#include "../src/core/sha256.h"
#include "swarmkit/core/execution.h"
#include "swarmkit/core/result.h"
#include "swarmkit/core/version.h"

TEST_CASE("Version constants are sane", "[core]") {
    REQUIRE(swarmkit::core::kVersionMajor >= 0);
    REQUIRE(swarmkit::core::kVersionMinor >= 0);
    REQUIRE(swarmkit::core::kVersionPatch >= 0);
}

TEST_CASE("Core Result carries typed SwarmError metadata", "[core][result]") {
    const auto error = swarmkit::core::SwarmError::Make(
        swarmkit::core::ErrorDomain::kBackend, swarmkit::core::ErrorCode::kBackendFailure,
        "backend rejected command", swarmkit::core::ErrorSeverity::kError,
        swarmkit::core::ErrorRetryability::kUnknown, "inspect autopilot diagnostics");

    const auto result = swarmkit::core::Result::FromError(error);
    CHECK_FALSE(result.IsOk());
    CHECK(result.code == swarmkit::core::StatusCode::kFailed);
    CHECK(result.error.domain == swarmkit::core::ErrorDomain::kBackend);
    CHECK(result.error.code == swarmkit::core::ErrorCode::kBackendFailure);
    CHECK(result.error.severity == swarmkit::core::ErrorSeverity::kError);
    CHECK(result.error.retryability == swarmkit::core::ErrorRetryability::kUnknown);
    CHECK(result.error.remediation.find("autopilot") != std::string::npos);
}

TEST_CASE("Internal SHA-256 helper matches a known vector", "[core][crypto]") {
    CHECK(swarmkit::core::internal::Sha256Hex("abc") ==
          "ba7816bf8f01cfea414140de5dae2223"
          "b00361a396177a9cb410ff61f20015ad");
}

TEST_CASE("Correlation IDs include process entropy", "[core]") {
    const std::string first = swarmkit::core::internal::MakeCorrelationId("qa");
    const std::string second = swarmkit::core::internal::MakeCorrelationId("qa");
    CHECK(first != second);
    CHECK(first.starts_with("qa-"));
    CHECK(std::ranges::count(first, '-') >= 3);
}

TEST_CASE("Execution identity is either absent or complete", "[core][execution]") {
    swarmkit::core::ExecutionContext context{
        .mission_id = "mission-1",
        .mission_revision = 1,
        .model_hash = "sha256:model",
        .operation_id = "operation-1",
        .operation_attempt_revision = 1,
    };
    REQUIRE(context.IsComplete());

    swarmkit::core::ExecutionHandle handle{
        .agent_session_id = "session-1",
        .drone_id = "drone-1",
        .goal_id = "goal-1",
        .goal_revision = 1,
        .physical_attempt_id = "attempt-1",
        .physical_attempt_revision = 1,
        .client_id = "controller-1",
        .correlation_id = "correlation-1",
        .context = context,
    };
    REQUIRE(handle.IsComplete());

    handle.context->operation_id.clear();
    CHECK_FALSE(handle.IsComplete());
    handle.context.reset();
    CHECK(handle.IsComplete());
    handle.physical_attempt_id.clear();
    CHECK_FALSE(handle.IsComplete());
}
