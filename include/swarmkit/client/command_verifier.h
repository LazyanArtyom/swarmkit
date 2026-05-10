// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

namespace swarmkit::client {

/// @brief Runtime policy for SDK helpers that send a command and wait for telemetry/health proof.
struct CommandWaitOptions {
    int timeout_ms{30000};
    int poll_interval_ms{250};
    int telemetry_rate_hz{5};
    float position_radius_m{2.0F};
    float altitude_tolerance_m{0.75F};
};

}  // namespace swarmkit::client
