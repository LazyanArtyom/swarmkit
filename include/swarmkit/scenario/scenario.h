#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "swarmkit/core/commands.h"

namespace swarmkit::scenario {

struct TimedCommand {
    std::chrono::milliseconds at{0};
    swarmkit::core::CommandContext ctx;
    swarmkit::core::Command cmd;
};

struct Scenario {
    std::string name;
    std::vector<TimedCommand> timeline;
};

// Baseline YAML loader:
//
// name: demo
// timeline:
//   - at_ms: 0
//     drone_id: default
//     cmd: arm
//   - at_ms: 1000
//     drone_id: default
//     cmd: takeoff
//     alt_m: 10
Scenario LoadScenarioYaml(const std::string& path);

}  // namespace swarmkit::scenario
