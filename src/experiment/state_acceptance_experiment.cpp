// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "swarmkit/experiment/state_acceptance_experiment.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <numeric>
#include <random>
#include <sstream>

#include "swarmkit/core/replay_trace.h"

namespace swarmkit::experiment {

namespace {

constexpr double kEarthRadiusM = 6378137.0;
constexpr double kDegToRad = std::numbers::pi / 180.0;

// Exact Euclidean distance in metric LocalNED space [north_m, east_m, down_m]
double DistanceMeters(const std::array<double, 3>& p1, const std::array<double, 3>& p2) {
    const double dx = p1[0] - p2[0];
    const double dy = p1[1] - p2[1];
    const double dz = p1[2] - p2[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Convert SimBackend truth to LocalNED metric GroundTruthState
GroundTruthState ConvertSimTruth(
    const std::string& drone_id,
    const agent::SimulationTruthFrame& truth_frame,
    const std::string& session_id,
    bool healthy_flag) {

    GroundTruthState state;
    state.drone_id = drone_id;
    state.physical_time_ms = static_cast<double>(truth_frame.simulation_time_ms);
    state.session_id = session_id;
    state.healthy = healthy_flag && !truth_frame.failed;

    // Direct metric positions in LocalNED [north, east, down]
    state.position = {truth_frame.north_m, truth_frame.east_m, -truth_frame.up_m};
    state.velocity = {
        static_cast<float>(truth_frame.velocity_north_mps),
        static_cast<float>(truth_frame.velocity_east_mps),
        static_cast<float>(-truth_frame.velocity_up_mps),
    };
    state.position_frame = core::CoordinateFrame::kLocalNed;
    state.velocity_frame = core::CoordinateFrame::kLocalNed;
    return state;
}

[[nodiscard]] commands::CommandEnvelope FlightCommand(std::string drone_id, commands::FlightCmd command) {
    return {
        .context = {.drone_id = std::move(drone_id), .correlation_id = "exp-flight"},
        .command = std::move(command),
    };
}

[[nodiscard]] commands::CommandEnvelope NavCommand(std::string drone_id, commands::NavCmd command) {
    return {
        .context = {.drone_id = std::move(drone_id), .correlation_id = "exp-nav"},
        .command = std::move(command),
    };
}

std::uint64_t HashSeed(std::uint64_t base, std::size_t rep, const std::string& label) {
    std::uint64_t h = base ^ (static_cast<std::uint64_t>(rep) * 0x517cc1b727220a95ULL);
    for (char c : label) {
        h = (h ^ static_cast<std::uint64_t>(c)) * 0x9e3779b97f4a7c15ULL;
    }
    return h;
}

const std::vector<std::string>& MutationClassNames() {
    static const std::vector<std::string> names{
        "outer_consistency_hash", "evaluation_time_t_star", "evidence_freeze_r_star",
        "contract_hash", "contract_schema_version", "contract_content_version",
        "certificate_schema_version", "acceptance_semantics_version", "participant_set",
        "membership_revision", "accepted_agent_set", "evidence_sequence", "evidence_hash",
        "agent_session", "source_component", "source_time", "receive_time", "coordinate_frame",
        "uncertainty_semantics", "observation_uncertainty", "health_predicate",
        "mission_revision", "gps_quality", "theta_hat", "base_rho", "effective_rho",
        "max_drift_rate", "clock_model_update_time", "clock_model_version",
        "clock_incarnation", "clock_domain", "clock_synchronization",
        "clock_deterministic_bound", "g_minus", "g_plus", "delta_plus",
        "propagated_uncertainty", "propagation_model_id", "propagation_model_version",
        "max_horizontal_speed", "max_vertical_speed", "summary_max_clock",
        "summary_max_elapsed", "summary_max_position_uncertainty",
    };
    return names;
}

void ApplyCertificateMutation(std::size_t index, core::StateAcceptanceCertificate* cert) {
    if (cert == nullptr || cert->evidence_entries.empty()) return;
    auto& entry = cert->evidence_entries.front();
    switch (index) {
        case 0: cert->certificate_hash.assign(64, '0'); return;
        case 1: cert->evaluation_time_ms += 1.0; break;
        case 2: cert->evidence_freeze_ms += 1; break;
        case 3: cert->contract_hash.assign(64, '1'); break;
        case 4: ++cert->contract_schema_version; break;
        case 5: ++cert->contract_content_version; break;
        case 6: cert->certificate_schema_version = "CERT_V999"; break;
        case 7: cert->acceptance_semantics_version = "999.0"; break;
        case 8: cert->participants.agent_ids.push_back("uav-inconsistent"); break;
        case 9: ++cert->participants.membership_revision; break;
        case 10: cert->accepted_agents.push_back("uav-inconsistent"); break;
        case 11: ++entry.sequence; break;
        case 12: entry.evidence_hash.assign(64, '2'); break;
        case 13: entry.agent_session_id += "-inconsistent"; break;
        case 14: entry.source_component += "-inconsistent"; break;
        case 15: entry.source_time_ms = entry.source_time_ms.value_or(0) + 1; break;
        case 16: ++entry.receive_time_ms; break;
        case 17: entry.coordinate_frame = entry.coordinate_frame == core::CoordinateFrame::kLocalNed
                     ? core::CoordinateFrame::kWgs84 : core::CoordinateFrame::kLocalNed; break;
        case 18: entry.uncertainty_semantics = core::UncertaintySemantics::kUnknown; break;
        case 19: entry.observation_uncertainty += 0.25; break;
        case 20: entry.estimator_healthy = !entry.estimator_healthy; break;
        case 21: ++entry.mission_revision; break;
        case 22: entry.gps_quality = core::GpsQuality::kRtkFixed; break;
        case 23: entry.theta_hat_ms += 0.5; break;
        case 24: entry.base_rho_ms += 0.5; break;
        case 25: entry.effective_rho_ms += 0.5; break;
        case 26: entry.max_drift_rate_ppm += 1.0; break;
        case 27: ++entry.clock_model_last_update_reference_ms; break;
        case 28: entry.clock_model_version += "-inconsistent"; break;
        case 29: entry.agent_incarnation_id += "-inconsistent"; break;
        case 30: entry.clock_domain = core::ClockDomain::kUnknown; break;
        case 31: entry.clock_synchronization = core::ClockSynchronization::kUnknown; break;
        case 32: entry.clock_deterministic_bound = !entry.clock_deterministic_bound; break;
        case 33: entry.generation_interval.lower_ms -= 0.5; break;
        case 34: entry.generation_interval.upper_ms += 0.5; break;
        case 35: entry.conservative_elapsed_ms += 0.5; break;
        case 36: entry.propagated_uncertainty += 0.25; break;
        case 37: cert->propagation_model_id += "-inconsistent"; break;
        case 38: cert->propagation_model_version += "-inconsistent"; break;
        case 39: cert->max_horizontal_speed_mps += 0.5F; break;
        case 40: cert->max_vertical_speed_mps += 0.5F; break;
        case 41: cert->max_clock_uncertainty_ms += 0.5; break;
        case 42: cert->max_conservative_elapsed_ms += 0.5; break;
        case 43: cert->max_propagated_position_uncertainty_m += 0.5; break;
        default: return;
    }
    cert->certificate_hash = core::ComputeCertificateHash(*cert);
}

}  // namespace

std::string ScenarioFaultKindToString(ScenarioFaultKind kind) {
    switch (kind) {
        case ScenarioFaultKind::kNormal: return "normal";
        case ScenarioFaultKind::kNetworkDelay: return "network_delay";
        case ScenarioFaultKind::kNetworkReorder: return "network_reorder";
        case ScenarioFaultKind::kPacketLoss: return "packet_loss";
        case ScenarioFaultKind::kClockOffsetDrift: return "clock_offset_drift";
        case ScenarioFaultKind::kEstimatorDegradation: return "estimator_degradation";
        case ScenarioFaultKind::kHighSpeedMotion: return "high_speed_delayed_motion";
        case ScenarioFaultKind::kAgentRestartDelayedPackets: return "agent_restart_delayed_packets";
        case ScenarioFaultKind::kFrameMismatch: return "frame_mismatch";
    }
    return "unknown";
}

std::string EvaluationMethodToString(EvaluationMethod method) {
    switch (method) {
        case EvaluationMethod::kReceiveLatest: return "Receive-latest (B0)";
        case EvaluationMethod::kTimestampAlignedAge: return "Timestamp-aligned + age (B1)";
        case EvaluationMethod::kProposedStateAcceptance: return "Proposed state acceptance (P)";
    }
    return "unknown";
}

// ---------------------------------------------------------------------------
// BaselineEvaluator & Common Ground-Truth Oracle
// ---------------------------------------------------------------------------

OutputValidityBreakdown BaselineEvaluator::EvaluateAcceptedOutputValidity(
    const MethodEvaluationOutcome& outcome,
    const std::unordered_map<std::string, GroundTruthState>& truth,
    const std::unordered_map<std::string, std::string>& authoritative_sessions,
    const core::StateQualityContract& contract,
    double u_max_meters) {

    OutputValidityBreakdown br;
    br.complete = true;
    br.spatial_valid = true;
    br.frame_valid = true;
    br.session_valid = true;
    br.health_valid = true;
    br.mission_valid = true;
    br.provenance_valid = true;
    br.gps_valid = true;

    if (!outcome.accepted) {
        br.overall_valid = false;
        return br;
    }

    for (const auto& agent_id : contract.required_agents) {
        const auto truth_it = truth.find(agent_id);
        if (truth_it == truth.end()) {
            br.complete = false;
            break;
        }
        const auto& true_state = truth_it->second;

        const auto rec_it = outcome.selected_position_evidence.find(agent_id);
        if (rec_it == outcome.selected_position_evidence.end()) {
            br.complete = false;
            break;
        }
        const auto& rec = rec_it->second;

        if (!std::holds_alternative<std::array<double, 3>>(rec.value)) {
            br.complete = false;
            break;
        }
        const auto& est_pos = std::get<std::array<double, 3>>(rec.value);

        // Spatial check: Euclidean distance in meters
        const double dist = DistanceMeters(est_pos, true_state.position);
        if (dist > u_max_meters) {
            br.spatial_valid = false;
        }

        // Frame check
        if (contract.required_position_frame != core::CoordinateFrame::kUnknown) {
            if (rec.identity.coordinate_frame != contract.required_position_frame) {
                br.frame_valid = false;
            }
        }

        // Session/epoch check
        if (contract.require_current_epoch) {
            const auto sess_it = authoritative_sessions.find(agent_id);
            if (sess_it == authoritative_sessions.end() ||
                rec.identity.agent_session_id != sess_it->second) {
                br.session_valid = false;
            }
        }

        // Health is a predicate over the selected evidence.  Simulator truth is
        // used independently for the position-error oracle at t*; current truth
        // health is not an undeclared StateQualityContract predicate.
        if (contract.require_estimator_healthy) {
            if (!rec.quality.estimator_healthy) {
                br.health_valid = false;
            }
        }
        if (contract.require_estimator_position_ok) {
            if (!rec.quality.estimator_position_ok) {
                br.health_valid = false;
            }
        }
        if (contract.require_estimator_velocity_ok) {
            if (!rec.quality.estimator_velocity_ok) {
                br.health_valid = false;
            }
        }

        // Mission check
        if (contract.require_current_mission) {
            if (rec.identity.mission_id != contract.required_mission_id ||
                rec.identity.mission_revision != contract.required_mission_revision) {
                br.mission_valid = false;
            }
        }

        if (rec.identity.agent_id != agent_id ||
            rec.identity.field_id != core::EvidenceFieldId::kPosition ||
            rec.identity.source_component.empty()) {
            br.provenance_valid = false;
        }

        if (contract.min_gps_quality != core::GpsQuality::kUnknown) {
            const auto gps_it = outcome.selected_gps_evidence.find(agent_id);
            if (gps_it == outcome.selected_gps_evidence.end() ||
                !std::holds_alternative<core::GpsQuality>(gps_it->second.value) ||
                static_cast<std::uint8_t>(
                    std::get<core::GpsQuality>(gps_it->second.value)) <
                    static_cast<std::uint8_t>(contract.min_gps_quality)) {
                br.gps_valid = false;
            }
        }
    }

    br.overall_valid = br.complete && br.spatial_valid && br.frame_valid &&
                       br.session_valid && br.health_valid && br.mission_valid &&
                       br.provenance_valid && br.gps_valid;
    return br;
}

MethodEvaluationOutcome BaselineEvaluator::EvaluateReceiveLatest(
    const core::StateQualityContract& contract,
    const core::SnapshotRequestContext& request_ctx,
    const core::EvidenceStore& store,
    const std::unordered_map<std::string, GroundTruthState>& truth,
    const std::unordered_map<std::string, std::string>& authoritative_sessions,
    double u_max_meters) {

    const auto t_start = std::chrono::steady_clock::now();
    MethodEvaluationOutcome outcome;
    outcome.accepted = false;

    bool all_found = true;
    for (const auto& agent_id : contract.required_agents) {
        auto pos_records = store.All(agent_id, core::EvidenceFieldId::kPosition);
        if (pos_records.empty()) {
            all_found = false;
            break;
        }

        // Select newest received record with receive_time <= r*
        std::optional<core::EvidenceRecord> best_rec;
        std::int64_t best_rx = -1;

        for (const auto& rec : pos_records) {
            if (rec.receive_time_ms <= request_ctx.evidence_freeze_ms) {
                if (!best_rec.has_value() || rec.receive_time_ms > best_rx) {
                    best_rx = rec.receive_time_ms;
                    best_rec = rec;
                }
            }
        }

        if (!best_rec.has_value() || !std::holds_alternative<std::array<double, 3>>(best_rec->value)) {
            all_found = false;
            break;
        }

        outcome.selected_position_evidence[agent_id] = *best_rec;
        outcome.estimated_positions[agent_id] = std::get<std::array<double, 3>>(best_rec->value);
        outcome.position_enclosures[agent_id] = 1.0;
    }

    if (all_found) {
        outcome.accepted = true;
        outcome.validity_breakdown = EvaluateAcceptedOutputValidity(
            outcome, truth, authoritative_sessions, contract, u_max_meters);
        outcome.ground_truth_valid = outcome.validity_breakdown.overall_valid;
    } else {
        outcome.rejection_reason = "missing_recent_position_evidence";
    }

    const auto t_end = std::chrono::steady_clock::now();
    outcome.latency_us = std::chrono::duration<double, std::micro>(t_end - t_start).count();
    return outcome;
}

MethodEvaluationOutcome BaselineEvaluator::EvaluateTimestampAlignedAge(
    const core::StateQualityContract& contract,
    const core::SnapshotRequestContext& request_ctx,
    const core::EvidenceStore& store,
    double max_age_ms,
    const std::unordered_map<std::string, GroundTruthState>& truth,
    const std::unordered_map<std::string, std::string>& authoritative_sessions,
    double u_max_meters) {

    const auto t_start = std::chrono::steady_clock::now();
    MethodEvaluationOutcome outcome;
    outcome.accepted = false;

    bool all_found = true;
    for (const auto& agent_id : contract.required_agents) {
        auto pos_records = store.All(agent_id, core::EvidenceFieldId::kPosition);
        if (pos_records.empty()) {
            all_found = false;
            break;
        }

        std::optional<core::EvidenceRecord> best_rec;
        double best_source_time = -1e18;

        for (const auto& rec : pos_records) {
            if (rec.receive_time_ms > request_ctx.evidence_freeze_ms) continue;
            if (!rec.source_time.timestamp_ms.has_value()) continue;
            const double s = static_cast<double>(*rec.source_time.timestamp_ms);
            if (s <= request_ctx.evaluation_time_ms) {
                if (!best_rec.has_value() || s > best_source_time) {
                    best_source_time = s;
                    best_rec = rec;
                }
            }
        }

        if (!best_rec.has_value()) {
            all_found = false;
            break;
        }

        const double raw_age = request_ctx.evaluation_time_ms - best_source_time;
        if (raw_age > max_age_ms) {
            all_found = false;
            break;
        }

        if (!std::holds_alternative<std::array<double, 3>>(best_rec->value)) {
            all_found = false;
            break;
        }

        outcome.selected_position_evidence[agent_id] = *best_rec;
        outcome.estimated_positions[agent_id] = std::get<std::array<double, 3>>(best_rec->value);
        outcome.position_enclosures[agent_id] = 1.0;
    }

    if (all_found) {
        outcome.accepted = true;
        outcome.validity_breakdown = EvaluateAcceptedOutputValidity(
            outcome, truth, authoritative_sessions, contract, u_max_meters);
        outcome.ground_truth_valid = outcome.validity_breakdown.overall_valid;
    } else {
        outcome.rejection_reason = "age_exceeded_or_no_causal_sample";
    }

    const auto t_end = std::chrono::steady_clock::now();
    outcome.latency_us = std::chrono::duration<double, std::micro>(t_end - t_start).count();
    return outcome;
}

MethodEvaluationOutcome BaselineEvaluator::EvaluateProposed(
    const core::StateAcceptanceEngine& engine,
    const core::StateQualityContract& contract,
    const core::SnapshotRequestContext& request_ctx,
    const core::EvidenceStore& store,
    const std::unordered_map<std::string, core::ClockQualityState>& clock_states,
    const std::unordered_map<std::string, GroundTruthState>& truth,
    const std::unordered_map<std::string, std::string>& authoritative_sessions,
    double u_max_meters) {

    const auto t_start = std::chrono::steady_clock::now();
    MethodEvaluationOutcome outcome;
    outcome.accepted = false;

    auto result = engine.RequestSnapshot(contract, request_ctx, store, clock_states);

    if (std::holds_alternative<core::AcceptedSnapshot>(result)) {
        const auto& snapshot = std::get<core::AcceptedSnapshot>(result);
        outcome.accepted = true;

        for (const auto& agent_id : snapshot.accepted_agents) {
            const auto it = snapshot.agent_states.find(agent_id);
            if (it != snapshot.agent_states.end()) {
                const auto pos_it = it->second.find(static_cast<std::uint8_t>(core::EvidenceFieldId::kPosition));
                if (pos_it != it->second.end()) {
                    outcome.selected_position_evidence[agent_id] = pos_it->second.evidence;
                    if (std::holds_alternative<std::array<double, 3>>(pos_it->second.evidence.value)) {
                        outcome.estimated_positions[agent_id] =
                            std::get<std::array<double, 3>>(pos_it->second.evidence.value);
                        outcome.position_enclosures[agent_id] = pos_it->second.propagated_uncertainty;
                    }
                }
                const auto gps_it = it->second.find(
                    static_cast<std::uint8_t>(core::EvidenceFieldId::kGpsQuality));
                if (gps_it != it->second.end()) {
                    outcome.selected_gps_evidence[agent_id] = gps_it->second.evidence;
                }
            }
        }

        outcome.certificate = core::BuildCertificate(snapshot, contract, request_ctx);
        outcome.validity_breakdown = EvaluateAcceptedOutputValidity(
            outcome, truth, authoritative_sessions, contract, u_max_meters);
        outcome.ground_truth_valid = outcome.validity_breakdown.overall_valid;
    } else {
        const auto& rej = std::get<core::StructuredRejection>(result);
        outcome.rejection_reason = rej.failures.empty()
            ? "rejection_unknown"
            : ("rejection_" + std::to_string(static_cast<int>(rej.failures[0].reason)));
        outcome.ground_truth_valid = false;
    }

    const auto t_end = std::chrono::steady_clock::now();
    outcome.latency_us = std::chrono::duration<double, std::micro>(t_end - t_start).count();
    return outcome;
}

// ---------------------------------------------------------------------------
// StateAcceptanceExperimentRunner Implementation
// ---------------------------------------------------------------------------

StateAcceptanceExperimentRunner::StateAcceptanceExperimentRunner(ScenarioConfig config)
    : config_(std::move(config)) {}

ExperimentResults StateAcceptanceExperimentRunner::Run() {
    ExperimentResults results;
    results.total_runs = config_.runs;
    results.steps_per_scenario = config_.steps_per_scenario;
    results.seed_base = config_.seed;
    results.agent_ids = config_.agent_ids;

    // Set up StateQualityContract v1
    core::StateQualityContract contract;
    contract.contract_id = "contract-swarm-dissertation-v1";
    contract.schema_version = 1;
    contract.content_version = 1;
    contract.required_agents = {config_.agent_ids.begin(), config_.agent_ids.end()};
    contract.required_fields = {
        core::EvidenceFieldId::kPosition,
        core::EvidenceFieldId::kVelocity,
    };
    contract.max_evidence_age_ms = config_.max_age_ms;
    contract.max_clock_uncertainty_ms = config_.max_clock_unc_ms;
    contract.max_position_uncertainty_m = config_.max_pos_unc_m;
    contract.require_estimator_healthy = true;
    contract.required_position_frame = core::CoordinateFrame::kLocalNed;
    contract.require_current_epoch = true;
    contract.require_deterministic_bounds = true;
    contract.completeness = core::CompletenessRule::kAllRequired;
    contract.min_required_agents = config_.agent_ids.size();
    contract.max_horizontal_speed_mps = static_cast<float>(config_.max_speed_mps);
    contract.max_vertical_speed_mps = 3.0F;
    contract.propagation_model_id = "linear_bounded_vmax";
    contract.propagation_model_version = "1.0";
    results.canonical_contract_hash = core::ComputeContractHash(contract);

    core::StateAcceptanceEngine engine;
    core::StateAcceptanceVerifier verifier;

    for (auto method : {EvaluationMethod::kReceiveLatest,
                        EvaluationMethod::kTimestampAlignedAge,
                        EvaluationMethod::kProposedStateAcceptance}) {
        results.aggregate_method_metrics[static_cast<uint8_t>(method)] = MethodMetrics{.method = method};
    }
    for (const auto& name : MutationClassNames()) {
        results.mutation_results.push_back(MutationClassResult{.class_name = name});
    }
    results.soundness_metrics.mutation_classes_tested = results.mutation_results.size();

    const std::filesystem::path replay_dir = std::filesystem::temp_directory_path() / "swarmkit_replays";
    std::filesystem::create_directories(replay_dir);

    for (const auto fault_kind : config_.fault_scenarios) {
        ScenarioMetrics sc_metrics;
        sc_metrics.fault_kind = fault_kind;
        for (auto method : {EvaluationMethod::kReceiveLatest,
                            EvaluationMethod::kTimestampAlignedAge,
                            EvaluationMethod::kProposedStateAcceptance}) {
            sc_metrics.metrics_by_method[static_cast<uint8_t>(method)] = MethodMetrics{.method = method};
        }

        for (std::size_t run_idx = 0; run_idx < config_.runs; ++run_idx) {
            const std::uint64_t motion_seed = HashSeed(config_.seed, run_idx, "motion");
            const std::uint64_t fault_seed = HashSeed(config_.seed, run_idx, ScenarioFaultKindToString(fault_kind));

            // 1. Configure SimBackend
            agent::SimBackendConfig sim_config{
                .clock_mode = agent::SimulationClockMode::kManual,
                .integration_step_ms = 20,
                .initial_source_unix_time_ms = 1'700'000'000'000LL,
                .home_lat_deg = 37.7749,
                .home_lon_deg = -122.4194,
                .home_alt_m = 10.0,
                .max_horizontal_speed_mps = static_cast<float>(config_.max_speed_mps),
            };

            auto sim_inst_or = agent::MakeSimBackend(sim_config);
            if (!sim_inst_or.has_value()) {
                continue;
            }
            auto sim_backend = std::move(sim_inst_or->backend);
            auto sim_control = std::move(sim_inst_or->control);

            // 2. Configure FaultInjectingBackend
            FaultInjectionConfig fault_cfg{
                .seed = fault_seed,
            };

            if (fault_kind == ScenarioFaultKind::kNetworkDelay) {
                fault_cfg.telemetry_delay_frames = 5;  // 500ms > max_age_ms 300ms
            } else if (fault_kind == ScenarioFaultKind::kNetworkReorder) {
                fault_cfg.telemetry_reorder_probability = 0.5;
            } else if (fault_kind == ScenarioFaultKind::kPacketLoss) {
                fault_cfg.telemetry_loss_probability = 0.45;
            } else if (fault_kind == ScenarioFaultKind::kClockOffsetDrift) {
                fault_cfg.source_clock_offset_ms = 15;
                fault_cfg.source_clock_drift_ms_per_frame = 0.08;
                fault_cfg.source_clock_uncertainty_ms = 15.0;  // > max_clock_unc_ms 10ms
            } else if (fault_kind == ScenarioFaultKind::kEstimatorDegradation) {
                fault_cfg.estimator_degradation_probability = 0.0;  // Deterministic onset at step 20
            } else if (fault_kind == ScenarioFaultKind::kHighSpeedMotion) {
                fault_cfg.telemetry_delay_frames = 2;  // 200ms delay to exercise V*Delta+ propagation
            }

            auto fault_inst_or = MakeFaultInjectingBackend(std::move(sim_backend), fault_cfg);
            if (!fault_inst_or.has_value()) {
                continue;
            }
            auto fault_backend = std::move(fault_inst_or->backend);
            auto fault_control = std::move(fault_inst_or->control);

            // 3. Arm, Takeoff, and Command Horizontal Motion for all UAVs
            for (std::size_t i = 0; i < config_.agent_ids.size(); ++i) {
                const auto& agent_id = config_.agent_ids[i];
                (void)fault_backend->Execute(FlightCommand(agent_id, commands::CmdArm{}));
                (void)fault_backend->Execute(FlightCommand(agent_id, commands::CmdTakeoff{.alt_m = 5.0}));
            }

            (void)sim_control->AdvanceAll(std::chrono::seconds(1));

            // Set varied motion from motion_seed
            std::mt19937_64 motion_rng(motion_seed);
            std::uniform_real_distribution<float> speed_dist(2.0F, 5.5F);
            std::uniform_real_distribution<float> angle_dist(-std::numbers::pi_v<float>, std::numbers::pi_v<float>);

            for (std::size_t i = 0; i < config_.agent_ids.size(); ++i) {
                const auto& agent_id = config_.agent_ids[i];
                float speed = speed_dist(motion_rng);
                float angle = angle_dist(motion_rng);
                float vx = speed * std::cos(angle);
                float vy = speed * std::sin(angle);

                if (fault_kind == ScenarioFaultKind::kHighSpeedMotion) {
                    vx = 8.5F;
                    vy = 1.0F;  // speed = 8.558 m/s <= 10.0 m/s
                }

                (void)fault_backend->Execute(NavCommand(
                    agent_id, commands::CmdVelocity{.vx_mps = vx, .vy_mps = vy, .duration_ms = 600'000}));
            }

            // 4. Setup EvidenceStore, sessions, clock states, and replay trace
            core::EvidenceStore store;
            std::unordered_map<std::string, std::string> active_sessions;
            std::unordered_map<std::string, core::ClockQualityState> clock_states;

            core::ReplayTrace run_trace;
            run_trace.trace_id = "trace-rep-" + std::to_string(run_idx) + "-" + ScenarioFaultKindToString(fault_kind);

            // Record initial membership
            run_trace.events.push_back(core::MembershipChangeEvent{
                .timestamp_ms = 1'700'000'000'000LL,
                .participants = {
                    .agent_ids = config_.agent_ids,
                    .membership_revision = 1,
                },
            });

            for (const auto& id : config_.agent_ids) {
                const std::string sess = "session-" + id + "-run" + std::to_string(run_idx);
                active_sessions[id] = sess;
                store.SetCurrentSession(id, sess);

                run_trace.events.push_back(core::SessionTransitionEvent{
                    .timestamp_ms = 1'700'000'000'000LL,
                    .agent_id = id,
                    .new_session_id = sess,
                });

                clock_states[id] = core::ClockQualityState{
                    .offset_estimate_ms = 0.0,
                    .uncertainty_radius_ms = 3.0,
                    .source_domain = core::ClockDomain::kUnixEpoch,
                    .synchronization = core::ClockSynchronization::kSynchronized,
                    .last_update_ms = 1'700'000'000'000LL,
                    .deterministic_bound = true,
                    .agent_incarnation_id = sess,
                    .clock_model_version = "clock-v1",
                };

                run_trace.events.push_back(core::ClockModelUpdateEvent{
                    .timestamp_ms = 1'700'000'000'000LL,
                    .agent_id = id,
                    .clock_state = clock_states[id],
                });
            }

            std::vector<core::TelemetryFrame> delivered_batch;
            std::mutex batch_mutex;

            for (const auto& agent_id : config_.agent_ids) {
                (void)fault_backend->StartTelemetry(
                    agent_id, 10,
                    [&batch_mutex, &delivered_batch](const core::TelemetryFrame& frame) {
                        std::lock_guard<std::mutex> lock(batch_mutex);
                        delivered_batch.push_back(frame);
                    });
            }

            double physical_time_ms = 1'700'000'001'000.0;
            std::optional<core::TelemetryFrame> saved_old_session_frame;

            // Per-run replicate measurements
            ReplicateRecord rep_b0{
                .replicate_id = run_idx,
                .base_seed = config_.seed,
                .motion_seed = motion_seed,
                .fault_seed = fault_seed,
                .scenario_id = static_cast<std::uint8_t>(fault_kind),
                .method = static_cast<std::uint8_t>(EvaluationMethod::kReceiveLatest),
            };
            ReplicateRecord rep_b1{
                .replicate_id = run_idx,
                .base_seed = config_.seed,
                .motion_seed = motion_seed,
                .fault_seed = fault_seed,
                .scenario_id = static_cast<std::uint8_t>(fault_kind),
                .method = static_cast<std::uint8_t>(EvaluationMethod::kTimestampAlignedAge),
            };
            ReplicateRecord rep_p{
                .replicate_id = run_idx,
                .base_seed = config_.seed,
                .motion_seed = motion_seed,
                .fault_seed = fault_seed,
                .scenario_id = static_cast<std::uint8_t>(fault_kind),
                .method = static_cast<std::uint8_t>(EvaluationMethod::kProposedStateAcceptance),
            };

            std::unordered_map<std::string, std::uint64_t> last_delivered_sequence;
            std::size_t reorder_inversions = 0;
            std::size_t estimator_degradation_events = 0;
            std::size_t frame_mismatch_events = 0;
            std::size_t restart_events = 0;
            std::size_t obsolete_epoch_packets_injected = 0;
            std::size_t clock_invalidations = 0;
            std::size_t clock_reestablishments = 0;
            std::size_t clock_offset_fault_events = 0;
            double max_effective_rho_ms = 0.0;
            double max_truth_speed_mps = 0.0;
            double max_delayed_age_ms = 0.0;
            double max_propagated_motion_m = 0.0;

            for (std::size_t step = 0; step < config_.steps_per_scenario; ++step) {
                physical_time_ms += config_.step_dt_ms;

                (void)sim_control->AdvanceAll(
                    std::chrono::milliseconds(static_cast<long long>(config_.step_dt_ms)));

                std::vector<core::TelemetryFrame> current_delivered;
                {
                    std::lock_guard<std::mutex> lock(batch_mutex);
                    current_delivered = std::move(delivered_batch);
                    delivered_batch.clear();
                }

                // Ingest delivered telemetry frames converted to metric LocalNED
                for (auto& frame : current_delivered) {
                    const auto prior_sequence = last_delivered_sequence.find(frame.drone_id);
                    if (prior_sequence != last_delivered_sequence.end() &&
                        frame.telemetry_sequence < prior_sequence->second) {
                        ++reorder_inversions;
                    }
                    last_delivered_sequence[frame.drone_id] = frame.telemetry_sequence;
                    const double dlat_rad = (frame.lat_deg - sim_config.home_lat_deg) * kDegToRad;
                    const double dlon_rad = (frame.lon_deg - sim_config.home_lon_deg) * kDegToRad;
                    const double north_m = dlat_rad * kEarthRadiusM;
                    const double cos_home_lat = std::max(0.01, std::cos(sim_config.home_lat_deg * kDegToRad));
                    const double east_m = dlon_rad * kEarthRadiusM * cos_home_lat;
                    const double down_m = -static_cast<double>(frame.rel_alt_m);

                    frame.lat_deg = north_m;
                    frame.lon_deg = east_m;
                    frame.rel_alt_m = static_cast<float>(down_m);
                    frame.position_frame = core::CoordinateFrame::kLocalNed;
                    frame.agent_receive_unix_time_ms = static_cast<std::int64_t>(physical_time_ms);
                    frame.agent_session_id = active_sessions[frame.drone_id];

                    if (fault_kind == ScenarioFaultKind::kFrameMismatch &&
                        frame.drone_id == config_.agent_ids[0]) {
                        frame.position_frame = core::CoordinateFrame::kWgs84;
                        ++frame_mismatch_events;
                    }

                    if (fault_kind == ScenarioFaultKind::kEstimatorDegradation && step >= 20) {
                        frame.estimator_state = core::EstimatorState::kDegraded;
                        frame.estimator_position_ok = false;
                        frame.estimator_velocity_ok = false;
                        ++estimator_degradation_events;
                    }

                    // For restart scenario: capture genuine E1 frame at step 29
                    if (fault_kind == ScenarioFaultKind::kAgentRestartDelayedPackets &&
                        step == 29 && frame.drone_id == config_.agent_ids[0]) {
                        saved_old_session_frame = frame;
                    }

                    if (fault_kind == ScenarioFaultKind::kAgentRestartDelayedPackets &&
                        frame.drone_id == config_.agent_ids[0] && step >= 30 && step < 33) {
                        continue;
                    }

                    store.InsertFrame(frame);

                    // Record exact decomposed evidence into replay trace
                    for (const auto& rec : core::DecomposeToEvidence(frame)) {
                        run_trace.events.push_back(core::EvidenceReceivedEvent{
                            .receive_time_ms = rec.receive_time_ms,
                            .agent_id = frame.drone_id,
                            .record = rec,
                        });
                    }
                }

                // Restart scenario handling at step 30
                if (fault_kind == ScenarioFaultKind::kAgentRestartDelayedPackets) {
                    const std::string restarted_agent = config_.agent_ids[0];

                    if (step == 30) {
                        const std::string new_sess = "session-" + restarted_agent + "-restarted";
                        active_sessions[restarted_agent] = new_sess;
                        store.SetCurrentSession(restarted_agent, new_sess);

                        run_trace.events.push_back(core::SessionTransitionEvent{
                            .timestamp_ms = static_cast<std::int64_t>(physical_time_ms),
                            .agent_id = restarted_agent,
                            .new_session_id = new_sess,
                        });

                        // Invalidate old clock model for active agent
                        clock_states.erase(restarted_agent);
                        ++restart_events;
                        ++clock_invalidations;
                    }

                    // Deliver authentic old E1 frame at step 31
                    if (step == 31 && saved_old_session_frame.has_value()) {
                        auto delayed_frame = *saved_old_session_frame;
                        delayed_frame.agent_receive_unix_time_ms = static_cast<std::int64_t>(physical_time_ms);
                        store.InsertFrame(delayed_frame);
                        ++obsolete_epoch_packets_injected;

                        for (const auto& rec : core::DecomposeToEvidence(delayed_frame)) {
                            run_trace.events.push_back(core::EvidenceReceivedEvent{
                                .receive_time_ms = rec.receive_time_ms,
                                .agent_id = restarted_agent,
                                .record = rec,
                            });
                        }
                    }

                    // Establish new E2 clock model at step 33
                    if (step == 33) {
                        const std::string new_sess = active_sessions[restarted_agent];
                        auto new_clk = core::ClockQualityState{
                            .offset_estimate_ms = 0.0,
                            .uncertainty_radius_ms = 3.0,
                            .source_domain = core::ClockDomain::kUnixEpoch,
                            .synchronization = core::ClockSynchronization::kSynchronized,
                            .last_update_ms = static_cast<std::int64_t>(physical_time_ms),
                            .deterministic_bound = true,
                            .agent_incarnation_id = new_sess,
                            .clock_model_version = "clock-v2",
                        };
                        clock_states[restarted_agent] = new_clk;
                        ++clock_reestablishments;

                        run_trace.events.push_back(core::ClockModelUpdateEvent{
                            .timestamp_ms = static_cast<std::int64_t>(physical_time_ms),
                            .agent_id = restarted_agent,
                            .clock_state = new_clk,
                        });
                    }
                }

                // Clock drift scenario updates
                if (fault_kind == ScenarioFaultKind::kClockOffsetDrift) {
                    for (const auto& id : config_.agent_ids) {
                        const double drift_offset = 15.0 + 0.08 * static_cast<double>(step);
                        clock_states[id].offset_estimate_ms = drift_offset;
                        clock_states[id].uncertainty_radius_ms = 15.0;  // exceeds contract 10ms
                        ++clock_offset_fault_events;
                        max_effective_rho_ms = std::max(
                            max_effective_rho_ms,
                            clock_states[id].ComputeEffectiveUncertainty(
                                physical_time_ms - drift_offset));

                        run_trace.events.push_back(core::ClockModelUpdateEvent{
                            .timestamp_ms = static_cast<std::int64_t>(physical_time_ms),
                            .agent_id = id,
                            .clock_state = clock_states[id],
                        });
                    }
                }

                // 5. Read Ground-Truth State strictly from SimBackendControl::Truth()
                std::unordered_map<std::string, GroundTruthState> ground_truth;
                for (const auto& agent_id : config_.agent_ids) {
                    auto truth_frame = sim_control->Truth(agent_id);
                    if (truth_frame.has_value()) {
                        bool healthy = true;
                        if (fault_kind == ScenarioFaultKind::kEstimatorDegradation && step >= 20) {
                            healthy = false;
                        }
                        ground_truth[agent_id] = ConvertSimTruth(
                            agent_id, *truth_frame, active_sessions[agent_id], healthy);
                        const auto& velocity = ground_truth[agent_id].velocity;
                        max_truth_speed_mps = std::max(max_truth_speed_mps, std::sqrt(
                            static_cast<double>(velocity[0]) * velocity[0] +
                            static_cast<double>(velocity[1]) * velocity[1] +
                            static_cast<double>(velocity[2]) * velocity[2]));
                    }
                }

                // 6. Evaluate All Methods at Common Reference Time t* and Evidence Frontier r*
                const double t_star = physical_time_ms;
                const std::int64_t r_star = static_cast<std::int64_t>(physical_time_ms);

                core::SnapshotRequestContext request_ctx{
                    .evaluation_time_ms = t_star,
                    .evidence_freeze_ms = r_star,
                    .participants = {
                        .agent_ids = config_.agent_ids,
                        .membership_revision = 1,
                    },
                };

                auto outcome_b0 = BaselineEvaluator::EvaluateReceiveLatest(
                    contract, request_ctx, store, ground_truth, active_sessions, config_.physical_error_tolerance_m);

                auto outcome_b1 = BaselineEvaluator::EvaluateTimestampAlignedAge(
                    contract, request_ctx, store, config_.max_age_ms, ground_truth, active_sessions,
                    config_.physical_error_tolerance_m);

                auto outcome_p = BaselineEvaluator::EvaluateProposed(
                    engine, contract, request_ctx, store, clock_states, ground_truth, active_sessions,
                    config_.physical_error_tolerance_m);

                results.latencies_us.push_back(outcome_p.latency_us);

                // Record confusion matrix outcomes fairly using common oracle
                // Rejection availability is referenced to the existence of a complete
                // physical truth state, not an undeclared current-estimator-health
                // predicate. Accepted-output validity is evaluated only by the common
                // selected-evidence oracle above.
                const bool ground_truth_valid_at_reference =
                    ground_truth.size() == contract.required_agents.size();

                auto record_outcome = [&](EvaluationMethod method,
                                         const MethodEvaluationOutcome& outcome,
                                         ReplicateRecord& rep) {
                    auto& agg = results.aggregate_method_metrics[static_cast<uint8_t>(method)];
                    auto& scm = sc_metrics.metrics_by_method[static_cast<uint8_t>(method)];

                    ++agg.total_requests;
                    ++scm.total_requests;
                    ++rep.requests;

                    if (outcome.accepted) {
                        ++rep.accepted;
                        if (outcome.ground_truth_valid) {
                            ++agg.true_accepts;
                            ++scm.true_accepts;
                            ++rep.true_accepts;
                        } else {
                            ++agg.false_accepts;
                            ++scm.false_accepts;
                            ++rep.false_accepts;
                        }
                    } else {
                        if (ground_truth_valid_at_reference) {
                            ++agg.false_rejects;
                            ++scm.false_rejects;
                            ++rep.false_rejects;
                        } else {
                            ++agg.true_rejects;
                            ++scm.true_rejects;
                            ++rep.true_rejects;
                        }
                    }
                };

                record_outcome(EvaluationMethod::kReceiveLatest, outcome_b0, rep_b0);
                record_outcome(EvaluationMethod::kTimestampAlignedAge, outcome_b1, rep_b1);
                record_outcome(EvaluationMethod::kProposedStateAcceptance, outcome_p, rep_p);

                if (fault_kind == ScenarioFaultKind::kAgentRestartDelayedPackets && step >= 30) {
                    const auto selected_obsolete = [&](const MethodEvaluationOutcome& outcome) {
                        const auto it = outcome.selected_position_evidence.find(config_.agent_ids[0]);
                        return it != outcome.selected_position_evidence.end() &&
                               it->second.identity.agent_session_id != active_sessions[config_.agent_ids[0]];
                    };
                    if (selected_obsolete(outcome_b0)) ++rep_b0.obsolete_epoch_packets_selected;
                    if (selected_obsolete(outcome_b1)) ++rep_b1.obsolete_epoch_packets_selected;
                    if (selected_obsolete(outcome_p)) ++rep_p.obsolete_epoch_packets_selected;
                }

                // Enclosure Containment Soundness
                if (outcome_p.accepted && outcome_p.certificate.has_value()) {
                    const auto& cert = *outcome_p.certificate;
                    const std::string serialized_cert = core::SerializeCertificate(cert);
                    results.certificate_sizes_bytes.push_back(serialized_cert.size());
                    for (const auto& entry : cert.evidence_entries) {
                        max_effective_rho_ms = std::max(max_effective_rho_ms, entry.effective_rho_ms);
                        max_delayed_age_ms = std::max(max_delayed_age_ms, entry.conservative_elapsed_ms);
                        if (entry.field == core::EvidenceFieldId::kPosition) {
                            max_propagated_motion_m = std::max(
                                max_propagated_motion_m,
                                std::max(0.0, entry.propagated_uncertainty -
                                                  entry.observation_uncertainty));
                        }
                    }

                    for (const auto& [agent_id, enclosure_radius] : outcome_p.position_enclosures) {
                        const auto truth_it = ground_truth.find(agent_id);
                        const auto est_it = outcome_p.estimated_positions.find(agent_id);
                        if (truth_it != ground_truth.end() && est_it != outcome_p.estimated_positions.end()) {
                            ++results.soundness_metrics.enclosures_tested;
                            ++rep_p.enclosures_tested;
                            const double err = DistanceMeters(est_it->second, truth_it->second.position);
                            if (enclosure_radius > 0.0) {
                                results.containment_ratios.push_back(err / enclosure_radius);
                            }
                            if (err > enclosure_radius) {
                                ++results.soundness_metrics.containment_failures;
                                ++rep_p.containment_failures;
                            }
                        }
                    }

                    // Record snapshot request event for persisted replay
                    run_trace.events.push_back(core::SnapshotRequestEvent{
                        .request_id = "req-rep-" + std::to_string(run_idx) + "-step-" + std::to_string(step),
                        .evaluation_time_ms = t_star,
                        .evidence_freeze_ms = r_star,
                        .contract_hash = core::ComputeContractHash(contract),
                        .participants = request_ctx.participants,
                        .certificate = cert,
                    });
                }
            }

            sc_metrics.requests += config_.steps_per_scenario;

            // 7. Persisted Offline Replay Verification (§18-27)
            const std::string trace_file_path = (replay_dir / (run_trace.trace_id + ".jsonl")).string();
            if (run_trace.SaveToFile(trace_file_path)) {
                // Destroy live state and reload from file
                auto loaded_trace = core::ReplayTrace::LoadFromFile(trace_file_path);
                if (loaded_trace.has_value()) {
                    core::EvidenceStore replayed_store;
                    std::unordered_map<std::string, std::string> replayed_sessions;
                    std::unordered_map<std::string, core::ClockQualityState> replayed_clocks;

                    for (const auto& ev : loaded_trace->events) {
                        std::visit([&](const auto& e) {
                            using T = std::decay_t<decltype(e)>;
                            if constexpr (std::is_same_v<T, core::SessionTransitionEvent>) {
                                replayed_sessions[e.agent_id] = e.new_session_id;
                                replayed_store.SetCurrentSession(e.agent_id, e.new_session_id);
                            } else if constexpr (std::is_same_v<T, core::ClockModelUpdateEvent>) {
                                replayed_clocks[e.agent_id] = e.clock_state;
                            } else if constexpr (std::is_same_v<T, core::EvidenceReceivedEvent>) {
                                replayed_store.Insert(e.agent_id, e.record);
                            } else if constexpr (std::is_same_v<T, core::SnapshotRequestEvent>) {
                                if (e.certificate.has_value()) {
                                    ++results.soundness_metrics.replayed_decisions;

                                    core::SnapshotRequestContext replayed_ctx{
                                        .evaluation_time_ms = e.evaluation_time_ms,
                                        .evidence_freeze_ms = e.evidence_freeze_ms,
                                        .participants = e.participants,
                                    };

                                    const bool contract_bound =
                                        core::VerifySnapshotRequestContractBinding(e, contract);
                                    const auto ver_res = contract_bound
                                        ? verifier.Verify(*e.certificate, replayed_store, contract,
                                                          replayed_ctx, replayed_clocks)
                                        : core::VerificationResult{core::VerificationRejection{
                                              .certificate_id = e.certificate->certificate_id,
                                              .failures = {{
                                                  .reason = core::VerificationFailureReason::kContractHashMismatch,
                                                  .detail = "persisted request contract hash mismatch",
                                              }},
                                          }};
                                    if (std::holds_alternative<core::VerifiedAcceptance>(ver_res)) {
                                        ++results.soundness_metrics.verifier_agreements;
                                    }

                                    // Rehashed semantic mutation matrix plus one outer-hash case.
                                    const auto base_cert = *e.certificate;
                                    for (std::size_t mutation_class = 0;
                                         mutation_class < results.mutation_results.size();
                                         ++mutation_class) {
                                        auto mutated = base_cert;
                                        ++results.soundness_metrics.mutation_cases_tested;
                                        auto& mutation_result = results.mutation_results[mutation_class];
                                        ++mutation_result.instances;
                                        ApplyCertificateMutation(mutation_class, &mutated);

                                        auto mut_res = verifier.Verify(
                                            mutated, replayed_store, contract, replayed_ctx, replayed_clocks);
                                        if (std::holds_alternative<core::VerificationRejection>(mut_res)) {
                                            ++results.soundness_metrics.mutation_cases_rejected;
                                            ++mutation_result.rejected;
                                        }
                                    }
                                }
                            }
                        }, ev);
                    }
                }
                std::filesystem::remove(trace_file_path);
            }

            const auto decisions = fault_control->Decisions();
            std::size_t realized_delay_frames = 0;
            std::size_t realized_packet_losses = 0;
            std::size_t realized_reorder_events = 0;
            for (const auto& decision : decisions) {
                if (!decision.applied) continue;
                if (decision.kind == "telemetry-delay") ++realized_delay_frames;
                if (decision.kind == "telemetry-loss") ++realized_packet_losses;
                if (decision.kind == "telemetry-reorder") ++realized_reorder_events;
            }
            for (auto* rep : {&rep_b0, &rep_b1, &rep_p}) {
                rep->realized_delay_frames = realized_delay_frames;
                rep->realized_packet_losses = realized_packet_losses;
                rep->realized_reorder_events = realized_reorder_events;
                rep->reorder_inversions = reorder_inversions;
                rep->restart_events = restart_events;
                rep->obsolete_epoch_packets_injected = obsolete_epoch_packets_injected;
                rep->estimator_degradation_events = estimator_degradation_events;
                rep->frame_mismatch_events = frame_mismatch_events;
                rep->clock_invalidations = clock_invalidations;
                rep->clock_reestablishments = clock_reestablishments;
                rep->clock_offset_fault_events = clock_offset_fault_events;
                rep->max_effective_rho_ms = max_effective_rho_ms;
                rep->max_truth_speed_mps = max_truth_speed_mps;
                rep->max_delayed_age_ms = max_delayed_age_ms;
                rep->max_propagated_motion_m = max_propagated_motion_m;
            }
            results.replicate_records.push_back(rep_b0);
            results.replicate_records.push_back(rep_b1);
            results.replicate_records.push_back(rep_p);
        }

        results.per_scenario_metrics.push_back(sc_metrics);
    }

    // Compute Cluster Bootstrap CIs across replicate IDs (B = 10,000)
    results.bootstrap_results = ComputeClusterBootstrap(results.replicate_records, config_.runs);

    results.soundness_metrics.replay_disagreements =
        results.soundness_metrics.replayed_decisions -
        results.soundness_metrics.verifier_agreements;
    results.soundness_metrics.mutation_classes_rejected =
        static_cast<std::size_t>(std::count_if(
            results.mutation_results.begin(), results.mutation_results.end(),
            [](const MutationClassResult& result) {
                return result.instances > 0 && result.instances == result.rejected;
            }));

    // Compute Latency Percentiles & Certificate Sizes
    if (!results.latencies_us.empty()) {
        auto sorted_lat = results.latencies_us;
        std::sort(sorted_lat.begin(), sorted_lat.end());
        results.soundness_metrics.latency_p50_us = sorted_lat[sorted_lat.size() * 50 / 100];
        results.soundness_metrics.latency_p95_us = sorted_lat[sorted_lat.size() * 95 / 100];
        results.soundness_metrics.latency_p99_us = sorted_lat[sorted_lat.size() * 99 / 100];
    }

    if (!results.certificate_sizes_bytes.empty()) {
        auto sorted_sizes = results.certificate_sizes_bytes;
        std::sort(sorted_sizes.begin(), sorted_sizes.end());
        results.soundness_metrics.median_certificate_size_bytes = sorted_sizes[sorted_sizes.size() / 2];
        results.soundness_metrics.min_certificate_size_bytes = sorted_sizes.front();
        results.soundness_metrics.max_certificate_size_bytes = sorted_sizes.back();
    }

    if (!results.containment_ratios.empty()) {
        auto ratios = results.containment_ratios;
        std::sort(ratios.begin(), ratios.end());
        results.soundness_metrics.max_containment_ratio = ratios.back();
        results.soundness_metrics.containment_ratio_p50 = ratios[ratios.size() * 50 / 100];
        results.soundness_metrics.containment_ratio_p95 = ratios[ratios.size() * 95 / 100];
        results.soundness_metrics.containment_ratio_p99 = ratios[ratios.size() * 99 / 100];
    }

    results.scalability_results = RunScalabilityBenchmark();

    return results;
}

// ---------------------------------------------------------------------------
// Scalability Benchmark Implementation across N in {3, 5, 10}
// ---------------------------------------------------------------------------

std::vector<ScalabilityBenchmarkResult> StateAcceptanceExperimentRunner::RunScalabilityBenchmark() {
    std::vector<ScalabilityBenchmarkResult> scalability_results;
    core::StateAcceptanceEngine engine;

    for (const std::size_t n : {3UL, 5UL, 10UL}) {
        std::vector<std::string> agents;
        for (std::size_t i = 1; i <= n; ++i) {
            agents.push_back("uav-" + std::to_string(i));
        }

        core::StateQualityContract contract;
        contract.contract_id = "scalability-contract-v1";
        contract.required_agents = {agents.begin(), agents.end()};
        contract.required_fields = {
            core::EvidenceFieldId::kPosition,
            core::EvidenceFieldId::kVelocity,
        };
        contract.max_evidence_age_ms = 300.0;
        contract.max_clock_uncertainty_ms = 10.0;
        contract.max_position_uncertainty_m = 3.0;
        contract.max_horizontal_speed_mps = 10.0F;
        contract.max_vertical_speed_mps = 3.0F;
        contract.propagation_model_id = "linear_bounded_vmax";
        contract.propagation_model_version = "1.0";
        contract.require_estimator_healthy = true;
        contract.required_position_frame = core::CoordinateFrame::kLocalNed;
        contract.require_current_epoch = true;
        contract.require_deterministic_bounds = true;
        contract.completeness = core::CompletenessRule::kAllRequired;

        core::EvidenceStore store;
        std::unordered_map<std::string, core::ClockQualityState> clock_states;

        const double t_star = 1'700'000'000'100.0;
        const std::int64_t r_star = 1'700'000'000'100LL;

        core::SnapshotRequestContext req_ctx{
            .evaluation_time_ms = t_star,
            .evidence_freeze_ms = r_star,
            .participants = {
                .agent_ids = agents,
                .membership_revision = 1,
            },
        };

        for (const auto& id : agents) {
            store.SetCurrentSession(id, "scale-session-" + id);
            clock_states[id] = core::ClockQualityState{
                .offset_estimate_ms = 0.0,
                .uncertainty_radius_ms = 2.0,
                .source_domain = core::ClockDomain::kUnixEpoch,
                .synchronization = core::ClockSynchronization::kSynchronized,
                .last_update_ms = 1'700'000'000'000LL,
                .deterministic_bound = true,
                .agent_incarnation_id = "scale-session-" + id,
            };

            core::EvidenceRecord pos_rec{
                .value = std::array<double, 3>{10.0, 20.0, -5.0},
                .source_time = {
                    .timestamp_ms = 1'700'000'000'050LL,
                    .clock_domain = core::ClockDomain::kUnixEpoch,
                    .synchronization = core::ClockSynchronization::kSynchronized,
                    .clock_uncertainty_ms = 2.0,
                },
                .receive_time_ms = 1'700'000'000'070LL,
                .quality = {
                    .uncertainty = core::UncertaintyEstimate{
                        .value = 0.25F,
                        .descriptor = {.semantics = core::UncertaintySemantics::kDeterministicHardBound},
                    },
                    .estimator_healthy = true,
                    .estimator_position_ok = true,
                },
                .identity = {
                    .agent_id = id,
                    .agent_session_id = "scale-session-" + id,
                    .field_id = core::EvidenceFieldId::kPosition,
                    .sequence = 1,
                    .coordinate_frame = core::CoordinateFrame::kLocalNed,
                    .source_component = "sim_pos",
                },
            };

            core::EvidenceRecord vel_rec{
                .value = std::array<float, 3>{1.0F, 0.0F, 0.0F},
                .source_time = {
                    .timestamp_ms = 1'700'000'000'050LL,
                    .clock_domain = core::ClockDomain::kUnixEpoch,
                    .synchronization = core::ClockSynchronization::kSynchronized,
                    .clock_uncertainty_ms = 2.0,
                },
                .receive_time_ms = 1'700'000'000'070LL,
                .quality = {
                    .uncertainty = core::UncertaintyEstimate{
                        .value = 0.1F,
                        .descriptor = {.semantics = core::UncertaintySemantics::kDeterministicHardBound},
                    },
                    .estimator_healthy = true,
                    .estimator_velocity_ok = true,
                },
                .identity = {
                    .agent_id = id,
                    .agent_session_id = "scale-session-" + id,
                    .field_id = core::EvidenceFieldId::kVelocity,
                    .sequence = 1,
                    .coordinate_frame = core::CoordinateFrame::kLocalNed,
                    .source_component = "sim_vel",
                },
            };

            store.Insert(id, pos_rec);
            store.Insert(id, vel_rec);
        }

        constexpr std::size_t kWarmupIterations = 100;
        constexpr std::size_t kTimedIterations = 1000;

        // Warm up the exact end-to-end operation reported below.
        for (std::size_t w = 0; w < kWarmupIterations; ++w) {
            auto res = engine.RequestSnapshot(contract, req_ctx, store, clock_states);
            if (std::holds_alternative<core::AcceptedSnapshot>(res)) {
                auto cert = core::BuildCertificate(std::get<core::AcceptedSnapshot>(res), contract, req_ctx);
                (void)core::SerializeCertificate(cert);
            }
        }

        // Measured iterations
        std::vector<double> latencies;
        std::vector<std::size_t> certificate_sizes;
        latencies.reserve(kTimedIterations);
        certificate_sizes.reserve(kTimedIterations);

        for (std::size_t iter = 0; iter < kTimedIterations; ++iter) {
            const auto t0 = std::chrono::steady_clock::now();
            auto res = engine.RequestSnapshot(contract, req_ctx, store, clock_states);
            if (std::holds_alternative<core::AcceptedSnapshot>(res)) {
                auto cert = core::BuildCertificate(std::get<core::AcceptedSnapshot>(res), contract, req_ctx);
                std::string ser = core::SerializeCertificate(cert);
                certificate_sizes.push_back(ser.size());
            }
            const auto t1 = std::chrono::steady_clock::now();
            latencies.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
        }

        const auto raw_latencies = latencies;
        const auto raw_certificate_sizes = certificate_sizes;
        std::sort(latencies.begin(), latencies.end());
        std::sort(certificate_sizes.begin(), certificate_sizes.end());
        const auto percentile_index = [](std::size_t count, double fraction) {
            return static_cast<std::size_t>(
                std::floor(fraction * static_cast<double>(count - 1)));
        };
        const double mean = std::accumulate(latencies.begin(), latencies.end(), 0.0) /
                            static_cast<double>(latencies.size());
        double squared_sum = 0.0;
        for (const double sample : latencies) {
            const double residual = sample - mean;
            squared_sum += residual * residual;
        }
        const double stddev = std::sqrt(
            squared_sum / static_cast<double>(latencies.size() - 1));

        ScalabilityBenchmarkResult bench;
        bench.uav_count = n;
        bench.warmup_iterations = kWarmupIterations;
        bench.timed_iterations = latencies.size();
        bench.latency_mean_us = mean;
        bench.latency_stddev_us = stddev;
        bench.latency_min_us = latencies.front();
        bench.latency_p50_us = latencies[percentile_index(latencies.size(), 0.50)];
        bench.latency_p90_us = latencies[percentile_index(latencies.size(), 0.90)];
        bench.latency_p95_us = latencies[percentile_index(latencies.size(), 0.95)];
        bench.latency_p99_us = latencies[percentile_index(latencies.size(), 0.99)];
        bench.latency_max_us = latencies.back();
        bench.certificate_size_min_bytes = certificate_sizes.front();
        bench.certificate_size_median_bytes =
            certificate_sizes[percentile_index(certificate_sizes.size(), 0.50)];
        bench.certificate_size_p95_bytes =
            certificate_sizes[percentile_index(certificate_sizes.size(), 0.95)];
        bench.certificate_size_max_bytes = certificate_sizes.back();
        bench.latency_samples_us = raw_latencies;
        bench.certificate_size_samples_bytes = raw_certificate_sizes;

        scalability_results.push_back(bench);
    }

    return scalability_results;
}

// ---------------------------------------------------------------------------
// Report & Table Formatters
// ---------------------------------------------------------------------------

std::string ExperimentResults::FormatTableII() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "### Paper Table II: Main Semantic Result\n\n";
    oss << "| Evaluation Method | Requests | Accepted | False Valid (FV) [95% CI] | Availability [95% CI] | Unsafe per Req (UAR) [95% CI] |\n";
    oss << "| :--- | :--- | :--- | :--- | :--- | :--- |\n";

    for (auto method : {EvaluationMethod::kReceiveLatest,
                        EvaluationMethod::kTimestampAlignedAge,
                        EvaluationMethod::kProposedStateAcceptance}) {
        const auto it = aggregate_method_metrics.find(static_cast<uint8_t>(method));
        if (it != aggregate_method_metrics.end()) {
            const auto& m = it->second;
            const ConfidenceInterval* fv = nullptr;
            const ConfidenceInterval* availability = nullptr;
            const ConfidenceInterval* uar = nullptr;
            if (method == EvaluationMethod::kReceiveLatest) {
                fv = &bootstrap_results.b0_fv;
                availability = &bootstrap_results.b0_availability;
                uar = &bootstrap_results.b0_uar;
            } else if (method == EvaluationMethod::kTimestampAlignedAge) {
                fv = &bootstrap_results.b1_fv;
                availability = &bootstrap_results.b1_availability;
                uar = &bootstrap_results.b1_uar;
            } else {
                fv = &bootstrap_results.proposed_fv;
                availability = &bootstrap_results.proposed_availability;
                uar = &bootstrap_results.proposed_uar;
            }
            oss << "| " << std::left << std::setw(28) << EvaluationMethodToString(method)
                << "| " << std::setw(9) << m.total_requests
                << "| " << std::setw(9) << m.AcceptedCount()
                << "| " << fv->point_estimate * 100.0 << "% ["
                << fv->lower_95 * 100.0 << "%, " << fv->upper_95 * 100.0 << "%] "
                << "| " << availability->point_estimate * 100.0 << "% ["
                << availability->lower_95 * 100.0 << "%, " << availability->upper_95 * 100.0 << "%] "
                << "| " << uar->point_estimate * 100.0 << "% ["
                << uar->lower_95 * 100.0 << "%, " << uar->upper_95 * 100.0 << "%] |\n";
        }
    }
    const auto& delta = bootstrap_results.delta_fv_proposed_vs_b1;
    oss << "\nPaired ΔFV (P − B1): " << delta.point_estimate * 100.0
        << " percentage points [95% cluster-bootstrap CI "
        << delta.lower_95 * 100.0 << ", " << delta.upper_95 * 100.0 << "].\n";
    return oss.str();
}

std::string ExperimentResults::FormatTableIII() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1);
    oss << "### Paper Table III: Soundness, Replay, and Overhead\n\n";
    oss << "| Metric | Value |\n";
    oss << "| :--- | :--- |\n";
    oss << "| Deterministic Enclosures Tested | " << soundness_metrics.enclosures_tested << " |\n";
    oss << "| Containment Failures (CF) | " << soundness_metrics.containment_failures
        << " (" << std::setprecision(2)
        << (soundness_metrics.ContainmentFailureRate() * 100.0) << "%) |\n";
    oss << "| Persisted Offline Replay Decisions | " << soundness_metrics.replayed_decisions << " |\n";
    oss << "| Verifier Replay Agreement | " << soundness_metrics.verifier_agreements
        << " (" << std::setprecision(1) << (soundness_metrics.VerifierAgreementRate() * 100.0) << "%) |\n";
    oss << "| Mutation Classes Rejected | " << soundness_metrics.mutation_classes_rejected
        << "/" << soundness_metrics.mutation_classes_tested << " |\n";
    oss << "| Mutation Cases Tested | " << soundness_metrics.mutation_cases_tested << " |\n";
    oss << "| Mutation Cases Rejected | " << soundness_metrics.mutation_cases_rejected
        << " (" << std::setprecision(1) << (soundness_metrics.MutationRejectionRate() * 100.0) << "%) |\n";
    const auto n10 = std::find_if(scalability_results.begin(), scalability_results.end(),
                                  [](const auto& item) { return item.uav_count == 10; });
    if (n10 != scalability_results.end()) {
        oss << "| N=10 end-to-end latency p95 | " << n10->latency_p95_us << " µs |\n";
        oss << "| N=10 median serialized certificate size | "
            << n10->certificate_size_median_bytes << " bytes |\n";
    }

    return oss.str();
}

std::string ExperimentResults::FormatScalabilityTable() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1);
    oss << "### Scalability Benchmark Across Swarm Sizes (N in {3, 5, 10})\n\n";
    oss << "| Swarm Size (N) | Count | Latency p50 (µs) | Latency p95 (µs) | Latency p99 (µs) | Median Cert Size (bytes) |\n";
    oss << "| :--- | :--- | :--- | :--- | :--- | :--- |\n";

    for (const auto& res : scalability_results) {
        oss << "| " << std::right << std::setw(10) << res.uav_count << " UAVs "
            << "| " << std::setw(8) << res.timed_iterations
            << "| " << std::setw(16) << res.latency_p50_us
            << "| " << std::setw(16) << res.latency_p95_us
            << "| " << std::setw(16) << res.latency_p99_us
            << "| " << std::setw(17) << res.certificate_size_median_bytes
            << " |\n";
    }
    return oss.str();
}

std::string ExperimentResults::FormatPerScenarioTable() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1);
    oss << "### Per-Scenario Detailed Breakdown\n\n";
    oss << "| Scenario | Requests | B0 FV% | B1 FV% | Proposed FV% | B0 Avail% | B1 Avail% | Proposed Avail% |\n";
    oss << "| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |\n";

    for (const auto& sc : per_scenario_metrics) {
        const auto& b0 = sc.metrics_by_method.at(static_cast<uint8_t>(EvaluationMethod::kReceiveLatest));
        const auto& b1 = sc.metrics_by_method.at(static_cast<uint8_t>(EvaluationMethod::kTimestampAlignedAge));
        const auto& p = sc.metrics_by_method.at(static_cast<uint8_t>(EvaluationMethod::kProposedStateAcceptance));

        oss << "| " << std::left << std::setw(26) << ScenarioFaultKindToString(sc.fault_kind)
            << "| " << std::setw(9) << sc.requests
            << "| " << std::setw(5) << (b0.FalseValidRate() * 100.0) << "% "
            << "| " << std::setw(5) << (b1.FalseValidRate() * 100.0) << "% "
            << "| " << std::setw(5) << (p.FalseValidRate() * 100.0) << "% "
            << "| " << std::setw(5) << (b0.Availability() * 100.0) << "% "
            << "| " << std::setw(5) << (b1.Availability() * 100.0) << "% "
            << "| " << std::setw(5) << (p.Availability() * 100.0) << "% |\n";
    }
    return oss.str();
}

std::string ExperimentResults::ToCsvTableII() const {
    std::ostringstream oss;
    oss << "method,requests,accepted,false_accepts,FV,FV_CI_low,FV_CI_high,availability,availability_CI_low,availability_CI_high,UAR,UAR_CI_low,UAR_CI_high\n";
    for (auto method : {EvaluationMethod::kReceiveLatest,
                        EvaluationMethod::kTimestampAlignedAge,
                        EvaluationMethod::kProposedStateAcceptance}) {
        const auto it = aggregate_method_metrics.find(static_cast<uint8_t>(method));
        if (it != aggregate_method_metrics.end()) {
            const auto& m = it->second;
            const ConfidenceInterval* fv = method == EvaluationMethod::kReceiveLatest
                ? &bootstrap_results.b0_fv : method == EvaluationMethod::kTimestampAlignedAge
                ? &bootstrap_results.b1_fv : &bootstrap_results.proposed_fv;
            const ConfidenceInterval* availability = method == EvaluationMethod::kReceiveLatest
                ? &bootstrap_results.b0_availability : method == EvaluationMethod::kTimestampAlignedAge
                ? &bootstrap_results.b1_availability : &bootstrap_results.proposed_availability;
            const ConfidenceInterval* uar = method == EvaluationMethod::kReceiveLatest
                ? &bootstrap_results.b0_uar : method == EvaluationMethod::kTimestampAlignedAge
                ? &bootstrap_results.b1_uar : &bootstrap_results.proposed_uar;
            oss << "\"" << EvaluationMethodToString(method) << "\","
                << m.total_requests << ","
                << m.AcceptedCount() << ","
                << m.false_accepts << ","
                << fv->point_estimate << "," << fv->lower_95 << "," << fv->upper_95 << ","
                << availability->point_estimate << "," << availability->lower_95 << ","
                << availability->upper_95 << "," << uar->point_estimate << ","
                << uar->lower_95 << "," << uar->upper_95 << "\n";
        }
    }
    return oss.str();
}

std::string ExperimentResults::ToJson() const {
    std::ostringstream oss;
    oss << std::setprecision(std::numeric_limits<double>::max_digits10);
    oss << "{\n";
    oss << "  \"experiment_schema_version\": \"2.0\",\n";
    oss << "  \"bootstrap_iterations\": " << bootstrap_iterations << ",\n";
    oss << "  \"bootstrap_seed\": " << bootstrap_seed << ",\n";
    oss << "  \"replicate_count\": " << total_runs << ",\n";
    oss << "  \"steps_per_scenario\": " << steps_per_scenario << ",\n";
    oss << "  \"seed_base\": " << seed_base << ",\n";
    oss << "  \"agent_count\": " << agent_ids.size() << ",\n";
    oss << "  \"methods\": {\n";
    const auto emit_method = [&](std::string_view key, EvaluationMethod method,
                                 const ConfidenceInterval& fv,
                                 const ConfidenceInterval& availability,
                                 const ConfidenceInterval& uar, bool trailing) {
        const auto& m = aggregate_method_metrics.at(static_cast<std::uint8_t>(method));
        oss << "    \"" << key << "\": {\"requests\": " << m.total_requests
            << ", \"accepted\": " << m.AcceptedCount()
            << ", \"false_accepts\": " << m.false_accepts
            << ", \"FV\": " << fv.point_estimate
            << ", \"FV_CI_low\": " << fv.lower_95
            << ", \"FV_CI_high\": " << fv.upper_95
            << ", \"availability\": " << availability.point_estimate
            << ", \"availability_CI_low\": " << availability.lower_95
            << ", \"availability_CI_high\": " << availability.upper_95
            << ", \"UAR\": " << uar.point_estimate
            << ", \"UAR_CI_low\": " << uar.lower_95
            << ", \"UAR_CI_high\": " << uar.upper_95 << "}"
            << (trailing ? "," : "") << "\n";
    };
    emit_method("B0", EvaluationMethod::kReceiveLatest, bootstrap_results.b0_fv,
                bootstrap_results.b0_availability, bootstrap_results.b0_uar, true);
    emit_method("B1", EvaluationMethod::kTimestampAlignedAge, bootstrap_results.b1_fv,
                bootstrap_results.b1_availability, bootstrap_results.b1_uar, true);
    emit_method("P", EvaluationMethod::kProposedStateAcceptance, bootstrap_results.proposed_fv,
                bootstrap_results.proposed_availability, bootstrap_results.proposed_uar, false);
    const auto& delta = bootstrap_results.delta_fv_proposed_vs_b1;
    const double b1_fv = bootstrap_results.b1_fv.point_estimate;
    const double relative_reduction = b1_fv > 0.0
        ? (b1_fv - bootstrap_results.proposed_fv.point_estimate) / b1_fv : 0.0;
    oss << "  },\n";
    oss << "  \"paired_P_minus_B1\": {\"delta_FV\": " << delta.point_estimate
        << ", \"CI_low\": " << delta.lower_95 << ", \"CI_high\": "
        << delta.upper_95 << ", \"descriptive_relative_reduction\": "
        << relative_reduction << "}\n";
    oss << "}\n";

    return oss.str();
}

std::string ExperimentResults::ToBootstrapJson() const {
    return ToJson();
}

std::string ExperimentResults::ToPerScenarioJson() const {
    std::ostringstream oss;
    oss << std::setprecision(std::numeric_limits<double>::max_digits10);
    oss << "{\n  \"scenarios\": [\n";
    for (std::size_t i = 0; i < per_scenario_metrics.size(); ++i) {
        const auto& scenario = per_scenario_metrics[i];
        oss << "    {\"scenario\": \"" << ScenarioFaultKindToString(scenario.fault_kind)
            << "\", \"methods\": {";
        std::size_t method_index = 0;
        for (const auto method : {EvaluationMethod::kReceiveLatest,
                                  EvaluationMethod::kTimestampAlignedAge,
                                  EvaluationMethod::kProposedStateAcceptance}) {
            const auto& metrics = scenario.metrics_by_method.at(static_cast<std::uint8_t>(method));
            const char* key = method == EvaluationMethod::kReceiveLatest ? "B0" :
                              method == EvaluationMethod::kTimestampAlignedAge ? "B1" : "P";
            if (method_index++ > 0) oss << ", ";
            oss << "\"" << key << "\": {\"requests\": " << metrics.total_requests
                << ", \"accepted\": " << metrics.AcceptedCount()
                << ", \"false_accepts\": " << metrics.false_accepts
                << ", \"FV\": " << metrics.FalseValidRate()
                << ", \"availability\": " << metrics.Availability()
                << ", \"UAR\": " << metrics.UnsafeAcceptancePerRequest() << "}";
        }
        oss << "}}" << (i + 1 < per_scenario_metrics.size() ? "," : "") << "\n";
    }
    oss << "  ]\n}\n";
    return oss.str();
}

std::string ExperimentResults::ToReplicateCsv() const {
    std::ostringstream oss;
    oss << std::setprecision(std::numeric_limits<double>::max_digits10);
    oss << "replicate_id,scenario,method,base_seed,motion_seed,fault_seed,requests,accepted,true_accepts,false_accepts,FV,availability,UAR,enclosures_tested,containment_failures,realized_delay_frames,realized_packet_losses,realized_reorder_events,reorder_inversions,restart_events,obsolete_epoch_packets_injected,obsolete_epoch_packets_selected,estimator_degradation_events,frame_mismatch_events,clock_invalidations,clock_reestablishments,clock_offset_fault_events,max_effective_rho_ms,max_truth_speed_mps,max_delayed_age_ms,max_propagated_motion_m\n";
    for (const auto& rep : replicate_records) {
        oss << rep.replicate_id << ","
            << ScenarioFaultKindToString(static_cast<ScenarioFaultKind>(rep.scenario_id)) << ","
            << (rep.method == 0 ? "B0" : rep.method == 1 ? "B1" : "P") << ","
            << rep.base_seed << "," << rep.motion_seed << "," << rep.fault_seed << ","
            << rep.requests << "," << rep.accepted << "," << rep.true_accepts << ","
            << rep.false_accepts << "," << rep.FalseValidRate() << ","
            << rep.Availability() << "," << rep.UnsafeAcceptanceRate() << ","
            << rep.enclosures_tested << "," << rep.containment_failures << ","
            << rep.realized_delay_frames << "," << rep.realized_packet_losses << ","
            << rep.realized_reorder_events << "," << rep.reorder_inversions << ","
            << rep.restart_events << "," << rep.obsolete_epoch_packets_injected << ","
            << rep.obsolete_epoch_packets_selected << "," << rep.estimator_degradation_events << ","
            << rep.frame_mismatch_events << "," << rep.clock_invalidations << ","
            << rep.clock_reestablishments << "," << rep.clock_offset_fault_events << ","
            << rep.max_effective_rho_ms << "," << rep.max_truth_speed_mps << ","
            << rep.max_delayed_age_ms << "," << rep.max_propagated_motion_m << "\n";
    }
    return oss.str();
}

std::string ExperimentResults::ToReplicateDistributionJson() const {
    struct Totals {
        std::size_t requests{};
        std::size_t accepted{};
        std::size_t false_accepts{};
    };
    struct Summary {
        double mean{};
        double median{};
        double stddev{};
        double minimum{};
        double maximum{};
        double q1{};
        double q3{};
    };
    std::array<std::unordered_map<std::size_t, Totals>, 3> totals;
    for (const auto& record : replicate_records) {
        auto& total = totals.at(record.method)[record.replicate_id];
        total.requests += record.requests;
        total.accepted += record.accepted;
        total.false_accepts += record.false_accepts;
    }
    const auto summarize = [](std::vector<double> values) {
        Summary summary;
        if (values.empty()) return summary;
        std::sort(values.begin(), values.end());
        const auto quantile = [&](double fraction) {
            return values[static_cast<std::size_t>(std::floor(
                fraction * static_cast<double>(values.size() - 1)))];
        };
        summary.mean = std::accumulate(values.begin(), values.end(), 0.0) /
                       static_cast<double>(values.size());
        summary.median = quantile(0.50);
        summary.minimum = values.front();
        summary.maximum = values.back();
        summary.q1 = quantile(0.25);
        summary.q3 = quantile(0.75);
        if (values.size() > 1) {
            double sum = 0.0;
            for (const auto value : values) {
                const auto residual = value - summary.mean;
                sum += residual * residual;
            }
            summary.stddev = std::sqrt(sum / static_cast<double>(values.size() - 1));
        }
        return summary;
    };
    const auto values_for = [&](std::size_t method, std::string_view metric) {
        std::vector<double> values;
        for (std::size_t replicate = 0; replicate < total_runs; ++replicate) {
            const auto it = totals[method].find(replicate);
            if (it == totals[method].end()) continue;
            const auto& value = it->second;
            if (metric == "FV") {
                values.push_back(value.accepted > 0
                    ? static_cast<double>(value.false_accepts) / static_cast<double>(value.accepted) : 0.0);
            } else if (metric == "availability") {
                values.push_back(value.requests > 0
                    ? static_cast<double>(value.accepted) / static_cast<double>(value.requests) : 0.0);
            } else {
                values.push_back(value.requests > 0
                    ? static_cast<double>(value.false_accepts) / static_cast<double>(value.requests) : 0.0);
            }
        }
        return values;
    };
    const auto emit_summary = [](std::ostringstream& out, const Summary& summary) {
        out << "{\"mean\": " << summary.mean << ", \"median\": " << summary.median
            << ", \"stddev\": " << summary.stddev << ", \"min\": " << summary.minimum
            << ", \"max\": " << summary.maximum << ", \"Q1\": " << summary.q1
            << ", \"Q3\": " << summary.q3 << "}";
    };

    std::ostringstream oss;
    oss << std::setprecision(std::numeric_limits<double>::max_digits10);
    oss << "{\n  \"replicate_count\": " << total_runs << ",\n  \"methods\": {\n";
    for (std::size_t method = 0; method < 3; ++method) {
        const char* key = method == 0 ? "B0" : method == 1 ? "B1" : "P";
        oss << "    \"" << key << "\": {\"FV\": ";
        emit_summary(oss, summarize(values_for(method, "FV")));
        oss << ", \"availability\": ";
        emit_summary(oss, summarize(values_for(method, "availability")));
        oss << ", \"UAR\": ";
        emit_summary(oss, summarize(values_for(method, "UAR")));
        oss << "}" << (method < 2 ? "," : "") << "\n";
    }
    std::vector<double> paired_delta;
    const auto p_values = values_for(2, "FV");
    const auto b1_values = values_for(1, "FV");
    for (std::size_t i = 0; i < std::min(p_values.size(), b1_values.size()); ++i) {
        paired_delta.push_back(p_values[i] - b1_values[i]);
    }
    oss << "  },\n  \"paired_P_minus_B1_delta_FV\": ";
    emit_summary(oss, summarize(std::move(paired_delta)));
    oss << "\n}\n";
    return oss.str();
}

std::string ExperimentResults::ToReplayJson() const {
    std::ostringstream oss;
    oss << std::setprecision(std::numeric_limits<double>::max_digits10)
        << "{\n  \"persistence_format\": \"JSON-lines\",\n"
        << "  \"contract_scope\": \"fixed canonical StateQualityContract supplied separately\",\n"
        << "  \"reconstructed_state\": [\"evidence\", \"session\", \"clock\", \"membership\", \"request_t_star\", \"request_r_star\", \"certificate\"],\n"
        << "  \"persisted_replay_decisions\": " << soundness_metrics.replayed_decisions << ",\n"
        << "  \"replay_agreements\": " << soundness_metrics.verifier_agreements << ",\n"
        << "  \"replay_disagreements\": " << soundness_metrics.replay_disagreements << ",\n"
        << "  \"replay_agreement_rate\": " << soundness_metrics.VerifierAgreementRate() << "\n}\n";
    return oss.str();
}

std::string ExperimentResults::ToMutationJson() const {
    std::ostringstream oss;
    oss << std::setprecision(std::numeric_limits<double>::max_digits10)
        << "{\n  \"mutation_classes_total\": " << soundness_metrics.mutation_classes_tested << ",\n"
        << "  \"mutation_classes_rejected\": " << soundness_metrics.mutation_classes_rejected << ",\n"
        << "  \"mutation_cases_total\": " << soundness_metrics.mutation_cases_tested << ",\n"
        << "  \"mutation_cases_rejected\": " << soundness_metrics.mutation_cases_rejected << ",\n"
        << "  \"mutation_rejection_rate\": " << soundness_metrics.MutationRejectionRate() << ",\n"
        << "  \"classes\": [\n";
    for (std::size_t i = 0; i < mutation_results.size(); ++i) {
        const auto& item = mutation_results[i];
        oss << "    {\"class_name\": \"" << item.class_name << "\", \"instances\": "
            << item.instances << ", \"rejected\": " << item.rejected
            << ", \"accepted\": " << item.Accepted() << "}"
            << (i + 1 < mutation_results.size() ? "," : "") << "\n";
    }
    oss << "  ]\n}\n";
    return oss.str();
}

std::string ExperimentResults::ToScalabilityJson() const {
    std::ostringstream oss;
    oss << std::setprecision(std::numeric_limits<double>::max_digits10)
        << "{\n  \"operation\": \"RequestSnapshot + BuildCertificate + SerializeCertificate\",\n"
        << "  \"clock\": \"std::chrono::steady_clock\",\n  \"build_type\": \"Release\",\n"
        << "  \"results\": [\n";
    for (std::size_t i = 0; i < scalability_results.size(); ++i) {
        const auto& result = scalability_results[i];
        oss << "    {\"N\": " << result.uav_count
            << ", \"warmup_iterations\": " << result.warmup_iterations
            << ", \"count\": " << result.timed_iterations
            << ", \"mean_us\": " << result.latency_mean_us
            << ", \"stddev_us\": " << result.latency_stddev_us
            << ", \"min_us\": " << result.latency_min_us
            << ", \"p50_us\": " << result.latency_p50_us
            << ", \"p90_us\": " << result.latency_p90_us
            << ", \"p95_us\": " << result.latency_p95_us
            << ", \"p99_us\": " << result.latency_p99_us
            << ", \"max_us\": " << result.latency_max_us
            << ", \"certificate_size_min_bytes\": " << result.certificate_size_min_bytes
            << ", \"certificate_size_median_bytes\": " << result.certificate_size_median_bytes
            << ", \"certificate_size_p95_bytes\": " << result.certificate_size_p95_bytes
            << ", \"certificate_size_max_bytes\": " << result.certificate_size_max_bytes << "}"
            << (i + 1 < scalability_results.size() ? "," : "") << "\n";
    }
    oss << "  ]\n}\n";
    return oss.str();
}

std::string ExperimentResults::ToScalabilitySamplesCsv() const {
    std::ostringstream oss;
    oss << std::setprecision(std::numeric_limits<double>::max_digits10)
        << "N,iteration,latency_us,serialized_certificate_bytes\n";
    for (const auto& result : scalability_results) {
        const auto count = std::min(result.latency_samples_us.size(),
                                    result.certificate_size_samples_bytes.size());
        for (std::size_t i = 0; i < count; ++i) {
            oss << result.uav_count << "," << i << "," << result.latency_samples_us[i]
                << "," << result.certificate_size_samples_bytes[i] << "\n";
        }
    }
    return oss.str();
}

}  // namespace swarmkit::experiment
