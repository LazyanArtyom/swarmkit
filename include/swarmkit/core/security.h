// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstdint>
#include <expected>
#include <string>
#include <string_view>

namespace swarmkit::core {

inline constexpr std::string_view kMutualTlsTransportName = "mtls";

enum class TransportSecurityMode : std::uint8_t {
    kAuto,
    kInsecure,
    kTls,
    kMutualTls,
};

[[nodiscard]] inline std::string_view ToString(TransportSecurityMode mode) {
    switch (mode) {
        case TransportSecurityMode::kAuto:
            return "auto";
        case TransportSecurityMode::kInsecure:
            return "insecure";
        case TransportSecurityMode::kTls:
            return "tls";
        case TransportSecurityMode::kMutualTls:
            return "mtls";
    }
    return "auto";
}

[[nodiscard]] inline std::expected<TransportSecurityMode, std::string> ParseTransportSecurityMode(
    std::string_view value) {
    if (value == "auto" || value.empty()) {
        return TransportSecurityMode::kAuto;
    }
    if (value == "insecure" || value == "none" || value == "off") {
        return TransportSecurityMode::kInsecure;
    }
    if (value == "tls") {
        return TransportSecurityMode::kTls;
    }
    if (value == "mtls" || value == "mTLS") {
        return TransportSecurityMode::kMutualTls;
    }
    return std::unexpected("invalid transport security mode '" + std::string(value) +
                           "': expected auto|insecure|tls|mtls");
}

}  // namespace swarmkit::core
