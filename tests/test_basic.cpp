// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <catch2/catch_test_macros.hpp>

#include <string>

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
