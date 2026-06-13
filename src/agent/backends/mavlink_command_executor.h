// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "mavlink_common.h"
#include "swarmkit/agent/backend.h"
#include "swarmkit/commands/flight.h"
#include "swarmkit/core/result.h"

namespace swarmkit::agent::mavlink {

inline constexpr float kMavlinkForceArmDisarmMagic = 21196.0F;

struct MavlinkCommandLongSpec {
    std::uint16_t command{};
    std::array<float, 7> params{};
};

class MavlinkCommandExecutor {
   public:
    [[nodiscard]] static BackendCapabilities Capabilities(const MavlinkBackendConfig& config);
    [[nodiscard]] static core::Result ResolveCustomMode(const MavlinkBackendConfig& config,
                                                        const commands::CmdSetMode& mode,
                                                        int* custom_mode);
    [[nodiscard]] static MavlinkCommandLongSpec ArmDisarmCommand(bool arm, bool force);
};

}  // namespace swarmkit::agent::mavlink
