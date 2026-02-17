#include <yaml-cpp/yaml.h>

#include <stdexcept>

#include "swarmkit/scenario/scenario.h"

namespace swarmkit::scenario {

Scenario LoadScenarioYaml(const std::string& path) {
    Scenario sc;

    const YAML::Node root = YAML::LoadFile(path);
    sc.name = root["name"].as<std::string>("scenario");

    const auto timeline = root["timeline"];
    if (!timeline || !timeline.IsSequence()) {
        return sc;
    }

    for (const auto& n : timeline) {
        TimedCommand tc;
        tc.at = std::chrono::milliseconds{n["at_ms"].as<long long>(0)};
        tc.ctx.drone_id = n["drone_id"].as<std::string>("default");

        const auto cmd = n["cmd"].as<std::string>("");
        if (cmd == "arm") {
            tc.cmd = swarmkit::core::CmdArm{};
        } else if (cmd == "disarm") {
            tc.cmd = swarmkit::core::CmdDisarm{};
        } else if (cmd == "land") {
            tc.cmd = swarmkit::core::CmdLand{};
        } else if (cmd == "takeoff") {
            swarmkit::core::CmdTakeoff t;
            t.alt_m = n["alt_m"].as<double>(10.0);
            tc.cmd = t;
        } else if (cmd == "set_role") {
            swarmkit::core::CmdSetRole r;
            r.role = n["role"].as<std::string>("worker");
            tc.cmd = r;
        } else {
            // Unknown commands are allowed during prototyping; keep as LAND to be safe.
            tc.cmd = swarmkit::core::CmdLand{};
        }

        sc.timeline.push_back(std::move(tc));
    }

    return sc;
}

}  // namespace swarmkit::scenario
