// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <catch2/catch_test_macros.hpp>

#include "swarmkit/core/version.h"

TEST_CASE("Version constants are sane", "[core]") {
    REQUIRE(swarmkit::core::kVersionMajor >= 0);
    REQUIRE(swarmkit::core::kVersionMinor >= 0);
    REQUIRE(swarmkit::core::kVersionPatch >= 0);
}
