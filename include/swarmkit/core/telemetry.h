#pragma once

#include <cstdint>
#include <string>

namespace swarmkit::core {

struct TelemetryFrame {
    std::string drone_id;
    std::int64_t unix_time_ms{};

    double lat_deg{};
    double lon_deg{};
    float rel_alt_m{};
    float battery_percent{};
    std::string mode;
};

} // namespace swarmkit::core
