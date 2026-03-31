// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <string>
#include <string_view>

namespace swarmkit::apps::common {

[[nodiscard]] inline std::string GetOptionValue(int argc, char** argv, std::string_view key,
                                                std::string_view default_value = {}) {
    for (int index = 1; index + 1 < argc; ++index) {
        if (std::string_view{argv[index]} == key) {
            return {argv[index + 1]};
        }
    }
    return std::string{default_value};
}

[[nodiscard]] inline bool HasFlag(int argc, char** argv, std::string_view flag) {
    for (int index = 1; index < argc; ++index) {
        if (std::string_view{argv[index]} == flag) {
            return true;
        }
    }
    return false;
}

}  // namespace swarmkit::apps::common
