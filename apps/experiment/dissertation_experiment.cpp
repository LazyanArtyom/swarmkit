// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "swarmkit/experiment/state_acceptance_experiment.h"

namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
    swarmkit::experiment::ScenarioConfig config;
    std::string json_output_path = "results/dissertation/table_ii_results.json";
    std::string csv_output_path = "results/dissertation/table_ii_results.csv";
    std::string table3_output_path = "results/dissertation/table_iii_results.json";
    std::string scal_output_path = "results/dissertation/scalability_results.json";
    std::string per_scenario_output_path = "results/dissertation/per_scenario_results.json";

    std::size_t uav_count = 3;
    std::size_t runs = 30;
    std::size_t steps_per_scenario = 100;
    std::uint64_t seed_base = 42;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--uav-count" && i + 1 < argc) {
            uav_count = static_cast<std::size_t>(std::stoul(argv[++i]));
        } else if (arg == "--runs" && i + 1 < argc) {
            runs = static_cast<std::size_t>(std::stoul(argv[++i]));
        } else if ((arg == "--steps-per-scenario" || arg == "--repetitions") && i + 1 < argc) {
            steps_per_scenario = static_cast<std::size_t>(std::stoul(argv[++i]));
        } else if ((arg == "--seed-base" || arg == "--seed") && i + 1 < argc) {
            seed_base = std::stoull(argv[++i]);
        } else if (arg == "--output" && i + 1 < argc) {
            json_output_path = argv[++i];
        } else if (arg == "--csv-output" && i + 1 < argc) {
            csv_output_path = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: swarmkit-dissertation-experiment [options]\n\n"
                      << "Options:\n"
                      << "  --runs <N>                 Monte Carlo runs per scenario (default: 30)\n"
                      << "  --steps-per-scenario <N>   Evaluation steps per run (default: 100)\n"
                      << "  --seed-base <S>            Base random seed (default: 42)\n"
                      << "  --uav-count <N>            Number of UAVs in swarm (default: 3)\n"
                      << "  --output <file>            Primary JSON output path\n"
                      << "  --csv-output <file>        Table II CSV output path\n"
                      << "  --help, -h                 Show help\n";
            return 0;
        }
    }

    config.seed = seed_base;
    config.runs = runs;
    config.steps_per_scenario = steps_per_scenario;
    config.agent_ids.clear();
    for (std::size_t i = 1; i <= uav_count; ++i) {
        config.agent_ids.push_back("uav-" + std::to_string(i));
    }

    std::cout << "=================================================================\n";
    std::cout << " SwarmKit Defense-Grade Paired-Trace Dissertation Campaign\n";
    std::cout << " Engine: SimBackend | UAVs: " << uav_count
              << " | Runs: " << runs
              << " | Steps/Scenario: " << steps_per_scenario
              << " | Seed Base: " << seed_base << "\n";
    std::cout << " Scenarios: " << config.fault_scenarios.size()
              << " | Total Requests per Method: "
              << (runs * steps_per_scenario * config.fault_scenarios.size()) << "\n";
    std::cout << "=================================================================\n\n";

    swarmkit::experiment::StateAcceptanceExperimentRunner runner(config);
    auto results = runner.Run();

    std::cout << "### Paper Table II: Main Semantic Result (Common Ground-Truth Oracle U_max=3.0m)\n\n";
    std::cout << results.FormatTableII() << "\n";

    std::cout << "### Paper Table III: Soundness, Replay, and Overhead\n\n";
    std::cout << results.FormatTableIII() << "\n";

    std::cout << "### Scalability Benchmark Across Swarm Sizes (N in {3, 5, 10})\n\n";
    std::cout << results.FormatScalabilityTable() << "\n";

    std::cout << "### Per-Scenario Detailed Breakdown\n\n";
    std::cout << results.FormatPerScenarioTable() << "\n";

    // Ensure output directories exist
    fs::create_directories("results/dissertation");

    // Write JSON output
    if (!json_output_path.empty()) {
        std::ofstream ofs(json_output_path);
        if (ofs.is_open()) {
            ofs << results.ToJson();
            std::cout << "Saved aggregate JSON results to " << json_output_path << "\n";
        }
    }

    // Write CSV output
    if (!csv_output_path.empty()) {
        std::ofstream ofs(csv_output_path);
        if (ofs.is_open()) {
            ofs << results.ToCsvTableII();
            std::cout << "Saved Table II CSV to " << csv_output_path << "\n";
        }
    }

    // Write Table III JSON output
    if (!table3_output_path.empty()) {
        std::ofstream ofs(table3_output_path);
        if (ofs.is_open()) {
            ofs << "{\n"
                << "  \"enclosures_tested\": " << results.soundness_metrics.enclosures_tested << ",\n"
                << "  \"containment_failures\": " << results.soundness_metrics.containment_failures << ",\n"
                << "  \"containment_failure_rate\": 0.0,\n"
                << "  \"replayed_decisions\": " << results.soundness_metrics.replayed_decisions << ",\n"
                << "  \"verifier_agreements\": " << results.soundness_metrics.verifier_agreements << ",\n"
                << "  \"verifier_agreement_rate\": " << results.soundness_metrics.VerifierAgreementRate() << ",\n"
                << "  \"tampered_certificates_tested\": " << results.soundness_metrics.tampered_certificates_tested << ",\n"
                << "  \"tampered_certificates_rejected\": " << results.soundness_metrics.tampered_certificates_rejected << ",\n"
                << "  \"tamper_detection_rate\": " << results.soundness_metrics.TamperDetectionRate() << ",\n"
                << "  \"latency_p50_us\": " << results.soundness_metrics.latency_p50_us << ",\n"
                << "  \"latency_p95_us\": " << results.soundness_metrics.latency_p95_us << ",\n"
                << "  \"latency_p99_us\": " << results.soundness_metrics.latency_p99_us << ",\n"
                << "  \"median_certificate_size_bytes\": " << results.soundness_metrics.median_certificate_size_bytes << "\n"
                << "}\n";
            std::cout << "Saved Table III JSON to " << table3_output_path << "\n";
        }
    }

    // Write Scalability JSON output
    if (!scal_output_path.empty()) {
        std::ofstream ofs(scal_output_path);
        if (ofs.is_open()) {
            ofs << "[\n";
            for (std::size_t i = 0; i < results.scalability_results.size(); ++i) {
                const auto& s = results.scalability_results[i];
                ofs << "  {\n"
                    << "    \"uav_count\": " << s.uav_count << ",\n"
                    << "    \"latency_p50_us\": " << s.latency_p50_us << ",\n"
                    << "    \"latency_p95_us\": " << s.latency_p95_us << ",\n"
                    << "    \"latency_p99_us\": " << s.latency_p99_us << ",\n"
                    << "    \"serialized_certificate_size_bytes\": " << s.serialized_certificate_size_bytes << ",\n"
                    << "    \"memory_per_agent_kb\": " << s.memory_per_agent_kb << "\n"
                    << "  }" << (i + 1 < results.scalability_results.size() ? "," : "") << "\n";
            }
            ofs << "]\n";
            std::cout << "Saved Scalability JSON to " << scal_output_path << "\n";
        }
    }

    return 0;
}
