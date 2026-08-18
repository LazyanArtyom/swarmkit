// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "swarmkit/experiment/state_acceptance_experiment.h"

int main(int argc, char* argv[]) {
    swarmkit::experiment::ScenarioConfig config;
    std::string output_path;
    std::size_t uav_count = 3;
    std::size_t repetitions = 100;
    std::uint64_t seed = 42;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--uav-count" && i + 1 < argc) {
            uav_count = static_cast<std::size_t>(std::stoul(argv[++i]));
        } else if (arg == "--repetitions" && i + 1 < argc) {
            repetitions = static_cast<std::size_t>(std::stoul(argv[++i]));
        } else if (arg == "--seed" && i + 1 < argc) {
            seed = std::stoull(argv[++i]);
        } else if (arg == "--output" && i + 1 < argc) {
            output_path = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: swarmkit-dissertation-experiment [options]\n\n"
                      << "Options:\n"
                      << "  --uav-count <N>    Number of UAVs in swarm (default: 3)\n"
                      << "  --repetitions <N>  Step count per scenario (default: 100)\n"
                      << "  --seed <S>         Random seed (default: 42)\n"
                      << "  --output <file>    Save JSON results to file\n"
                      << "  --help, -h         Show help\n";
            return 0;
        }
    }

    config.seed = seed;
    config.steps_per_scenario = repetitions;
    config.agent_ids.clear();
    for (std::size_t i = 1; i <= uav_count; ++i) {
        config.agent_ids.push_back("uav-" + std::to_string(i));
    }

    std::cout << "=================================================================\n";
    std::cout << " SwarmKit Paired-Trace State Acceptance Dissertation Experiment\n";
    std::cout << " UAVs: " << uav_count << " | Steps/Scenario: " << repetitions
              << " | Seed: " << seed << "\n";
    std::cout << "=================================================================\n\n";

    swarmkit::experiment::StateAcceptanceExperimentRunner runner(config);
    auto results = runner.Run();

    std::cout << "### Paper Table II: Main Semantic Result\n\n";
    std::cout << results.FormatTableII() << "\n\n";

    std::cout << "### Paper Table III: Soundness, Replay, and Overhead\n\n";
    std::cout << results.FormatTableIII() << "\n\n";

    if (!output_path.empty()) {
        std::ofstream ofs(output_path);
        if (ofs.is_open()) {
            ofs << results.ToJson();
            std::cout << "Saved JSON results to " << output_path << "\n";
        } else {
            std::cerr << "Failed to open output path: " << output_path << "\n";
            return 1;
        }
    }

    return 0;
}
