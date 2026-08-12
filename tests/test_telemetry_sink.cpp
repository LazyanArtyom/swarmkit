// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include <catch2/catch_test_macros.hpp>
#include <string>

#include "telemetry_sink.h"

namespace swarmkit::apps::cli::internal {

TEST_CASE("Telemetry CSV preserves metre-scale coordinate precision", "[cli][telemetry][csv]") {
    client::TelemetryDelivery delivery;
    delivery.frame.validity.position = true;
    delivery.frame.lat_deg = -35.363126;
    delivery.frame.lon_deg = 149.165237;

    const std::string line = TelemetryCsvLine(delivery);
    CHECK(line.find("-35.363126") != std::string::npos);
    CHECK(line.find("149.16523") != std::string::npos);
    CHECK(line.find("-35.3631,") == std::string::npos);
}

}  // namespace swarmkit::apps::cli::internal
