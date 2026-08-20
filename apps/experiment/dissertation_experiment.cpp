// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <sys/utsname.h>

#ifdef __APPLE__
#include <sys/sysctl.h>
#endif

#include "swarmkit/core/state_acceptance_certificate.h"
#include "swarmkit/experiment/state_acceptance_experiment.h"

#ifndef SWARMKIT_EXPERIMENT_GIT_COMMIT
#define SWARMKIT_EXPERIMENT_GIT_COMMIT "unknown"
#endif
#ifndef SWARMKIT_EXPERIMENT_GIT_DIRTY
#define SWARMKIT_EXPERIMENT_GIT_DIRTY false
#endif
#ifndef SWARMKIT_EXPERIMENT_BUILD_TYPE
#define SWARMKIT_EXPERIMENT_BUILD_TYPE "unknown"
#endif
#ifndef SWARMKIT_EXPERIMENT_COMPILER_ID
#define SWARMKIT_EXPERIMENT_COMPILER_ID "unknown"
#endif
#ifndef SWARMKIT_EXPERIMENT_COMPILER_VERSION
#define SWARMKIT_EXPERIMENT_COMPILER_VERSION "unknown"
#endif

namespace fs = std::filesystem;
using swarmkit::experiment::EvaluationMethod;
using swarmkit::experiment::ExperimentResults;
using swarmkit::experiment::ReplicateRecord;
using swarmkit::experiment::ScenarioFaultKind;

namespace {

bool WriteFile(const fs::path& path, std::string_view content) {
    std::ofstream output(path);
    if (!output.is_open()) return false;
    output << content;
    return output.good();
}

std::string Iso8601UtcNow() {
    const auto now = std::chrono::system_clock::now();
    const auto value = std::chrono::system_clock::to_time_t(now);
    std::tm utc{};
    gmtime_r(&value, &utc);
    std::ostringstream output;
    output << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
    return output.str();
}

std::string CpuName() {
#ifdef __APPLE__
    std::size_t size = 0;
    if (sysctlbyname("machdep.cpu.brand_string", nullptr, &size, nullptr, 0) == 0 && size > 1) {
        std::string value(size, '\0');
        if (sysctlbyname("machdep.cpu.brand_string", value.data(), &size, nullptr, 0) == 0) {
            if (!value.empty() && value.back() == '\0') value.pop_back();
            return value;
        }
    }
    if (sysctlbyname("hw.model", nullptr, &size, nullptr, 0) == 0 && size > 1) {
        std::string value(size, '\0');
        if (sysctlbyname("hw.model", value.data(), &size, nullptr, 0) == 0) {
            if (!value.empty() && value.back() == '\0') value.pop_back();
            return value;
        }
    }
#endif
    return "unknown";
}

std::string TableIIIJson(const ExperimentResults& results) {
    std::ostringstream output;
    output << std::setprecision(17)
           << "{\n  \"deterministic_enclosures_tested\": "
           << results.soundness_metrics.enclosures_tested
           << ",\n  \"containment_failures\": " << results.soundness_metrics.containment_failures
           << ",\n  \"containment_failure_rate\": "
           << results.soundness_metrics.ContainmentFailureRate()
           << ",\n  \"max_containment_ratio\": " << results.soundness_metrics.max_containment_ratio
           << ",\n  \"containment_ratio_p50\": " << results.soundness_metrics.containment_ratio_p50
           << ",\n  \"containment_ratio_p95\": " << results.soundness_metrics.containment_ratio_p95
           << ",\n  \"containment_ratio_p99\": " << results.soundness_metrics.containment_ratio_p99
           << ",\n  \"persisted_replay_decisions\": " << results.soundness_metrics.replayed_decisions
           << ",\n  \"verifier_replay_agreements\": " << results.soundness_metrics.verifier_agreements
           << ",\n  \"replay_disagreements\": " << results.soundness_metrics.replay_disagreements
           << ",\n  \"replay_agreement_rate\": " << results.soundness_metrics.VerifierAgreementRate()
           << ",\n  \"mutation_classes_total\": " << results.soundness_metrics.mutation_classes_tested
           << ",\n  \"mutation_classes_rejected\": " << results.soundness_metrics.mutation_classes_rejected
           << ",\n  \"mutation_cases_total\": " << results.soundness_metrics.mutation_cases_tested
           << ",\n  \"mutation_cases_rejected\": " << results.soundness_metrics.mutation_cases_rejected
           << ",\n  \"mutation_rejection_rate\": " << results.soundness_metrics.MutationRejectionRate();
    const auto n10 = std::find_if(results.scalability_results.begin(),
                                  results.scalability_results.end(),
                                  [](const auto& value) { return value.uav_count == 10; });
    if (n10 != results.scalability_results.end()) {
        output << ",\n  \"N10_end_to_end_p95_latency_us\": " << n10->latency_p95_us
               << ",\n  \"N10_median_certificate_wire_bytes\": "
               << n10->certificate_size_median_bytes;
    }
    output << "\n}\n";
    return output.str();
}

std::string ManifestJson(const ExperimentResults& results,
                         const swarmkit::experiment::ScenarioConfig& config) {
    struct utsname system{};
    const bool uname_ok = uname(&system) == 0;
    std::ostringstream output;
    output << std::setprecision(17)
           << "{\n  \"git_commit\": \"" << SWARMKIT_EXPERIMENT_GIT_COMMIT << "\",\n"
           << "  \"git_dirty_state\": "
           << (SWARMKIT_EXPERIMENT_GIT_DIRTY ? "true" : "false") << ",\n"
           << "  \"experiment_schema_version\": \"2.0\",\n"
           << "  \"acceptance_semantics_version\": \""
           << swarmkit::core::kAcceptanceSemanticsVersion << "\",\n"
           << "  \"certificate_schema_version\": \""
           << swarmkit::core::kCertificateSchemaVersion << "\",\n"
           << "  \"propagation_model_id\": \"linear_bounded_vmax\",\n"
           << "  \"propagation_model_version\": \"1.0\",\n"
           << "  \"compiler\": \"" << SWARMKIT_EXPERIMENT_COMPILER_ID << "\",\n"
           << "  \"compiler_version\": \"" << SWARMKIT_EXPERIMENT_COMPILER_VERSION << "\",\n"
           << "  \"build_type\": \"" << SWARMKIT_EXPERIMENT_BUILD_TYPE << "\",\n"
           << "  \"os\": \"" << (uname_ok ? system.sysname : "unknown") << " "
           << (uname_ok ? system.release : "unknown") << "\",\n"
           << "  \"architecture\": \"" << (uname_ok ? system.machine : "unknown") << "\",\n"
           << "  \"cpu\": \"" << CpuName() << "\",\n"
           << "  \"primary_N\": " << config.agent_ids.size() << ",\n"
           << "  \"scale_N_values\": [3, 5, 10],\n"
           << "  \"replicate_count\": " << config.runs << ",\n"
           << "  \"scenario_count\": " << config.fault_scenarios.size() << ",\n"
           << "  \"scored_requests_per_scenario\": " << config.steps_per_scenario << ",\n"
           << "  \"base_seed\": " << config.seed << ",\n"
           << "  \"motion_seed_derivation\": \"HashSeed(base, replicate_id, motion)\",\n"
           << "  \"fault_seed_derivation\": \"HashSeed(base, replicate_id, scenario_name)\",\n"
           << "  \"bootstrap_seed\": " << results.bootstrap_seed << ",\n"
           << "  \"bootstrap_iterations\": " << results.bootstrap_iterations << ",\n"
           << "  \"Umax_m\": " << config.physical_error_tolerance_m << ",\n"
           << "  \"max_age_ms\": " << config.max_age_ms << ",\n"
           << "  \"max_clock_uncertainty_ms\": " << config.max_clock_unc_ms << ",\n"
           << "  \"Vmax_horizontal_mps\": " << config.max_speed_mps << ",\n"
           << "  \"Vmax_vertical_mps\": 3.0,\n"
           << "  \"Vmax_3d_mps\": " << std::hypot(config.max_speed_mps, 3.0) << ",\n"
           << "  \"high_speed_velocity_mps\": [8.5, 1.0, 0.0],\n"
           << "  \"high_speed_configured_delay_ms\": 200,\n"
           << "  \"restart_step\": 30,\n"
           << "  \"obsolete_E1_injection_step\": 31,\n"
           << "  \"E2_clock_reestablishment_step\": 33,\n"
           << "  \"estimator_degradation_onset_step\": 20,\n"
           << "  \"canonical_contract_hash\": \"" << results.canonical_contract_hash << "\",\n"
           << "  \"campaign_timestamp_utc\": \"" << Iso8601UtcNow() << "\"\n}\n";
    return output.str();
}

bool ScenarioActivationPasses(const ExperimentResults& results, std::string* detail) {
    for (const auto scenario : {ScenarioFaultKind::kNetworkDelay,
                                ScenarioFaultKind::kNetworkReorder,
                                ScenarioFaultKind::kPacketLoss,
                                ScenarioFaultKind::kClockOffsetDrift,
                                ScenarioFaultKind::kEstimatorDegradation,
                                ScenarioFaultKind::kHighSpeedMotion,
                                ScenarioFaultKind::kAgentRestartDelayedPackets,
                                ScenarioFaultKind::kFrameMismatch}) {
        std::size_t delay = 0, reorder = 0, inversions = 0, loss = 0, clock = 0;
        std::size_t estimator = 0, restart = 0, obsolete = 0, reestablished = 0, frame = 0;
        for (const ReplicateRecord& record : results.replicate_records) {
            if (record.method != static_cast<std::uint8_t>(EvaluationMethod::kProposedStateAcceptance) ||
                record.scenario_id != static_cast<std::uint8_t>(scenario)) continue;
            delay += record.realized_delay_frames;
            reorder += record.realized_reorder_events;
            inversions += record.reorder_inversions;
            loss += record.realized_packet_losses;
            clock += record.clock_offset_fault_events;
            estimator += record.estimator_degradation_events;
            restart += record.restart_events;
            obsolete += record.obsolete_epoch_packets_injected;
            reestablished += record.clock_reestablishments;
            frame += record.frame_mismatch_events;
        }
        const bool active = scenario == ScenarioFaultKind::kNetworkDelay ? delay > 0 :
            scenario == ScenarioFaultKind::kNetworkReorder ? reorder > 0 && inversions > 0 :
            scenario == ScenarioFaultKind::kPacketLoss ? loss > 0 :
            scenario == ScenarioFaultKind::kClockOffsetDrift ? clock > 0 :
            scenario == ScenarioFaultKind::kEstimatorDegradation ? estimator > 0 :
            scenario == ScenarioFaultKind::kHighSpeedMotion ? delay > 0 :
            scenario == ScenarioFaultKind::kAgentRestartDelayedPackets
                ? restart > 0 && obsolete > 0 && reestablished > 0 : frame > 0;
        if (!active) {
            if (detail != nullptr) {
                *detail = "scenario did not realize required mechanism: " +
                          swarmkit::experiment::ScenarioFaultKindToString(scenario);
            }
            return false;
        }
    }
    return true;
}

}  // namespace

int main(int argc, char* argv[]) {
    swarmkit::experiment::ScenarioConfig config;
    fs::path output_dir = "results/dissertation";
    std::size_t uav_count = 3;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--uav-count" && i + 1 < argc) {
            uav_count = static_cast<std::size_t>(std::stoul(argv[++i]));
        } else if (arg == "--runs" && i + 1 < argc) {
            config.runs = static_cast<std::size_t>(std::stoul(argv[++i]));
        } else if ((arg == "--steps-per-scenario" || arg == "--repetitions") && i + 1 < argc) {
            config.steps_per_scenario = static_cast<std::size_t>(std::stoul(argv[++i]));
        } else if ((arg == "--seed-base" || arg == "--seed") && i + 1 < argc) {
            config.seed = std::stoull(argv[++i]);
        } else if (arg == "--output-dir" && i + 1 < argc) {
            output_dir = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: swarmkit-dissertation-experiment [--runs N] "
                         "[--steps-per-scenario N] [--seed-base S] [--uav-count N] "
                         "[--output-dir DIR]\n";
            return 0;
        }
    }
    config.agent_ids.clear();
    for (std::size_t i = 1; i <= uav_count; ++i) {
        config.agent_ids.push_back("uav-" + std::to_string(i));
    }

    std::cout << "SwarmKit paired-trace campaign: N=" << uav_count
              << ", replicates=" << config.runs
              << ", scenarios=" << config.fault_scenarios.size()
              << ", requests/scenario=" << config.steps_per_scenario
              << ", seed=" << config.seed << "\n";

    const auto results = swarmkit::experiment::StateAcceptanceExperimentRunner(config).Run();
    std::cout << results.FormatTableII() << "\n" << results.FormatTableIII() << "\n"
              << results.FormatScalabilityTable() << "\n" << results.FormatPerScenarioTable() << "\n";

    std::string activation_failure;
    if (!ScenarioActivationPasses(results, &activation_failure)) {
        std::cerr << "INVALID CAMPAIGN: " << activation_failure << "\n";
        return 2;
    }
    if (results.soundness_metrics.replay_disagreements != 0) {
        std::cerr << "INVALID CAMPAIGN: persisted replay disagreement count is "
                  << results.soundness_metrics.replay_disagreements << "\n";
        return 3;
    }
    if (results.soundness_metrics.mutation_cases_tested !=
            results.soundness_metrics.mutation_cases_rejected ||
        results.soundness_metrics.mutation_classes_tested !=
            results.soundness_metrics.mutation_classes_rejected) {
        std::cerr << "INVALID CAMPAIGN: at least one mutated/inconsistent certificate verified\n";
        return 4;
    }

    fs::create_directories(output_dir);
    const std::pair<const char*, std::string> artifacts[] = {
        {"table_ii_results.json", results.ToJson()},
        {"table_ii_results.csv", results.ToCsvTableII()},
        {"table_iii_results.json", TableIIIJson(results)},
        {"scalability_results.json", results.ToScalabilityJson()},
        {"scalability_latency_samples.csv", results.ToScalabilitySamplesCsv()},
        {"per_scenario_results.json", results.ToPerScenarioJson()},
        {"replicate_results.csv", results.ToReplicateCsv()},
        {"replicate_distribution_summary.json", results.ToReplicateDistributionJson()},
        {"bootstrap_results.json", results.ToBootstrapJson()},
        {"mutation_results.json", results.ToMutationJson()},
        {"replay_results.json", results.ToReplayJson()},
        {"experiment_manifest.json", ManifestJson(results, config)},
    };
    for (const auto& [name, content] : artifacts) {
        const auto path = output_dir / name;
        if (!WriteFile(path, content)) {
            std::cerr << "failed to write " << path << "\n";
            return 5;
        }
        std::cout << "saved " << path << "\n";
    }
    return 0;
}
