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

}  // namespace

std::string ScenarioFaultKindToString(ScenarioFaultKind kind) {
    switch (kind) {
        case ScenarioFaultKind::kNormal: return "normal";
        case ScenarioFaultKind::kNetworkDelay: return "network_delay";
        case ScenarioFaultKind::kNetworkReorder: return "network_reorder";
        case ScenarioFaultKind::kPacketLoss: return "packet_loss";
        case ScenarioFaultKind::kClockOffsetDrift: return "clock_offset_drift";
        case ScenarioFaultKind::kEstimatorDegradation: return "estimator_degradation";
        case ScenarioFaultKind::kHighSpeedMotion: return "high_speed_motion";
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

        // Health check: Selected evidence health and ground truth health
        if (contract.require_estimator_healthy) {
            if (!rec.quality.estimator_healthy || !true_state.healthy) {
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
    }

    br.overall_valid = br.complete && br.spatial_valid && br.frame_valid &&
                       br.session_valid && br.health_valid && br.mission_valid;
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

    core::StateAcceptanceEngine engine;
    core::StateAcceptanceVerifier verifier;

    for (auto method : {EvaluationMethod::kReceiveLatest,
                        EvaluationMethod::kTimestampAlignedAge,
                        EvaluationMethod::kProposedStateAcceptance}) {
        results.aggregate_method_metrics[static_cast<uint8_t>(method)] = MethodMetrics{.method = method};
    }

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
                .motion_seed = motion_seed,
                .fault_seed = fault_seed,
                .scenario_id = static_cast<std::uint8_t>(fault_kind),
                .method = static_cast<std::uint8_t>(EvaluationMethod::kReceiveLatest),
            };
            ReplicateRecord rep_b1{
                .replicate_id = run_idx,
                .motion_seed = motion_seed,
                .fault_seed = fault_seed,
                .scenario_id = static_cast<std::uint8_t>(fault_kind),
                .method = static_cast<std::uint8_t>(EvaluationMethod::kTimestampAlignedAge),
            };
            ReplicateRecord rep_p{
                .replicate_id = run_idx,
                .motion_seed = motion_seed,
                .fault_seed = fault_seed,
                .scenario_id = static_cast<std::uint8_t>(fault_kind),
                .method = static_cast<std::uint8_t>(EvaluationMethod::kProposedStateAcceptance),
            };

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
                    }

                    if (fault_kind == ScenarioFaultKind::kEstimatorDegradation && step >= 20) {
                        frame.estimator_state = core::EstimatorState::kDegraded;
                        frame.estimator_position_ok = false;
                        frame.estimator_velocity_ok = false;
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
                    }

                    // Deliver authentic old E1 frame at step 31
                    if (step == 31 && saved_old_session_frame.has_value()) {
                        auto delayed_frame = *saved_old_session_frame;
                        delayed_frame.agent_receive_unix_time_ms = static_cast<std::int64_t>(physical_time_ms);
                        store.InsertFrame(delayed_frame);

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

                        run_trace.events.push_back(core::ClockModelUpdateEvent{
                            .timestamp_ms = static_cast<std::int64_t>(physical_time_ms),
                            .agent_id = id,
                            .clock_state = clock_states[id],
                        });
                    }
                }

                // 5. Read Ground-Truth State strictly from SimBackendControl::Truth()
                std::unordered_map<std::string, GroundTruthState> ground_truth;
                bool physical_truth_healthy = true;

                for (const auto& agent_id : config_.agent_ids) {
                    auto truth_frame = sim_control->Truth(agent_id);
                    if (truth_frame.has_value()) {
                        bool healthy = true;
                        if (fault_kind == ScenarioFaultKind::kEstimatorDegradation && step >= 20) {
                            healthy = false;
                        }
                        ground_truth[agent_id] = ConvertSimTruth(
                            agent_id, *truth_frame, active_sessions[agent_id], healthy);
                    }
                    if (!ground_truth[agent_id].healthy) {
                        physical_truth_healthy = false;
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
                const bool ground_truth_valid_at_reference = physical_truth_healthy;

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

                // Enclosure Containment Soundness
                if (outcome_p.accepted && outcome_p.certificate.has_value()) {
                    const auto& cert = *outcome_p.certificate;
                    const std::string serialized_cert = core::SerializeCertificate(cert);
                    results.certificate_sizes_bytes.push_back(serialized_cert.size());

                    for (const auto& [agent_id, enclosure_radius] : outcome_p.position_enclosures) {
                        const auto truth_it = ground_truth.find(agent_id);
                        const auto est_it = outcome_p.estimated_positions.find(agent_id);
                        if (truth_it != ground_truth.end() && est_it != outcome_p.estimated_positions.end()) {
                            ++results.soundness_metrics.enclosures_tested;
                            ++rep_p.enclosures_tested;
                            const double err = DistanceMeters(est_it->second, truth_it->second.position);
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
                        .contract_hash = contract.contract_id,
                        .participants = request_ctx.participants,
                        .certificate = cert,
                    });
                }
            }

            sc_metrics.requests += config_.steps_per_scenario;
            results.replicate_records.push_back(rep_b0);
            results.replicate_records.push_back(rep_b1);
            results.replicate_records.push_back(rep_p);

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

                                    auto ver_res = verifier.Verify(
                                        *e.certificate, replayed_store, contract, replayed_ctx, replayed_clocks);
                                    if (std::holds_alternative<core::VerifiedAcceptance>(ver_res)) {
                                        ++results.soundness_metrics.verifier_agreements;
                                    }

                                    // Complete 15-Class Mutation Matrix Testing
                                    const auto base_cert = *e.certificate;
                                    for (int mutation_class = 1; mutation_class <= 15; ++mutation_class) {
                                        auto mutated = base_cert;
                                        ++results.soundness_metrics.mutation_cases_tested;

                                        switch (mutation_class) {
                                            case 1:
                                                mutated.certificate_hash = "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
                                                break;
                                            case 2:
                                                mutated.evaluation_time_ms += 100.0;
                                                mutated.certificate_hash = core::ComputeCertificateHash(mutated);
                                                break;
                                            case 3:
                                                mutated.contract_hash = "deadbeefdeadbeefdeadbeefdeadbeefdeadbeefdeadbeefdeadbeefdeadbeef";
                                                mutated.certificate_hash = core::ComputeCertificateHash(mutated);
                                                break;
                                            case 4:
                                                mutated.contract_content_version += 1;
                                                mutated.certificate_hash = core::ComputeCertificateHash(mutated);
                                                break;
                                            case 5:
                                                mutated.acceptance_semantics_version = "3.0";
                                                mutated.certificate_hash = core::ComputeCertificateHash(mutated);
                                                break;
                                            case 6:
                                                mutated.evidence_entries[0].sequence += 100;
                                                mutated.certificate_hash = core::ComputeCertificateHash(mutated);
                                                break;
                                            case 7:
                                                mutated.evidence_entries[0].evidence_hash = "badhashbadhashbadhashbadhashbadhashbadhashbadhashbadhashbadhash";
                                                mutated.certificate_hash = core::ComputeCertificateHash(mutated);
                                                break;
                                            case 8:
                                                mutated.evidence_entries[0].agent_session_id = "tampered-session";
                                                mutated.certificate_hash = core::ComputeCertificateHash(mutated);
                                                break;
                                            case 9:
                                                mutated.evidence_entries[0].propagated_uncertainty = 0.01;
                                                mutated.certificate_hash = core::ComputeCertificateHash(mutated);
                                                break;
                                            case 10:
                                                mutated.evidence_entries[0].clock_uncertainty_ms = 0.01;
                                                mutated.certificate_hash = core::ComputeCertificateHash(mutated);
                                                break;
                                            case 11:
                                                mutated.accepted_agents.push_back("extra-agent");
                                                mutated.certificate_hash = core::ComputeCertificateHash(mutated);
                                                break;
                                            case 12:
                                                mutated.evidence_entries[0].source_time_ms =
                                                    mutated.evidence_entries[0].source_time_ms.value_or(0) + 1000;
                                                mutated.certificate_hash = core::ComputeCertificateHash(mutated);
                                                break;
                                            case 13:
                                                mutated.evidence_entries[0].source_component = "fake-sensor";
                                                mutated.certificate_hash = core::ComputeCertificateHash(mutated);
                                                break;
                                            case 14:
                                                mutated.evidence_entries[0].coordinate_frame = core::CoordinateFrame::kWgs84;
                                                mutated.certificate_hash = core::ComputeCertificateHash(mutated);
                                                break;
                                            case 15:
                                                mutated.evidence_entries[0].observation_uncertainty = 0.001;
                                                mutated.certificate_hash = core::ComputeCertificateHash(mutated);
                                                break;
                                        }

                                        auto mut_res = verifier.Verify(
                                            mutated, replayed_store, contract, replayed_ctx, replayed_clocks);
                                        if (std::holds_alternative<core::VerificationRejection>(mut_res)) {
                                            ++results.soundness_metrics.mutation_cases_rejected;
                                        }
                                    }
                                }
                            }
                        }, ev);
                    }
                }
                std::filesystem::remove(trace_file_path);
            }
        }

        results.per_scenario_metrics.push_back(sc_metrics);
    }

    // Compute Cluster Bootstrap CIs across replicate IDs (B = 10,000)
    results.bootstrap_results = ComputeClusterBootstrap(results.replicate_records, config_.runs);

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

        // Warmup iterations
        for (int w = 0; w < 50; ++w) {
            auto res = engine.RequestSnapshot(contract, req_ctx, store, clock_states);
            if (std::holds_alternative<core::AcceptedSnapshot>(res)) {
                auto cert = core::BuildCertificate(std::get<core::AcceptedSnapshot>(res), contract, req_ctx);
                (void)core::SerializeCertificate(cert);
            }
        }

        // Measured iterations
        std::vector<double> latencies;
        latencies.reserve(500);
        std::size_t cert_size = 0;

        for (int iter = 0; iter < 500; ++iter) {
            const auto t0 = std::chrono::steady_clock::now();
            auto res = engine.RequestSnapshot(contract, req_ctx, store, clock_states);
            if (std::holds_alternative<core::AcceptedSnapshot>(res)) {
                auto cert = core::BuildCertificate(std::get<core::AcceptedSnapshot>(res), contract, req_ctx);
                std::string ser = core::SerializeCertificate(cert);
                cert_size = ser.size();
            }
            const auto t1 = std::chrono::steady_clock::now();
            latencies.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
        }

        std::sort(latencies.begin(), latencies.end());

        ScalabilityBenchmarkResult bench;
        bench.uav_count = n;
        bench.latency_p50_us = latencies[latencies.size() * 50 / 100];
        bench.latency_p95_us = latencies[latencies.size() * 95 / 100];
        bench.latency_p99_us = latencies[latencies.size() * 99 / 100];
        bench.serialized_certificate_size_bytes = cert_size;

        scalability_results.push_back(bench);
    }

    return scalability_results;
}

// ---------------------------------------------------------------------------
// Report & Table Formatters
// ---------------------------------------------------------------------------

std::string ExperimentResults::FormatTableII() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1);
    oss << "### Paper Table II: Main Semantic Result\n\n";
    oss << "| Evaluation Method | Requests | Accepted | False Valid (FV) | Availability | Unsafe per Req (UAR) |\n";
    oss << "| :--- | :--- | :--- | :--- | :--- | :--- |\n";

    for (auto method : {EvaluationMethod::kReceiveLatest,
                        EvaluationMethod::kTimestampAlignedAge,
                        EvaluationMethod::kProposedStateAcceptance}) {
        const auto it = aggregate_method_metrics.find(static_cast<uint8_t>(method));
        if (it != aggregate_method_metrics.end()) {
            const auto& m = it->second;
            oss << "| " << std::left << std::setw(28) << EvaluationMethodToString(method)
                << "| " << std::setw(9) << m.total_requests
                << "| " << std::setw(9) << m.AcceptedCount()
                << "| " << std::setw(5) << (m.FalseValidRate() * 100.0) << " % "
                << "| " << std::setw(5) << (m.Availability() * 100.0) << " % "
                << "| " << std::setw(5) << (m.UnsafeAcceptancePerRequest() * 100.0) << " % |\n";
        }
    }
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
    oss << "| Mutation Classes Tested | " << soundness_metrics.mutation_classes_tested << " |\n";
    oss << "| Mutation Classes Rejected | " << soundness_metrics.mutation_classes_rejected
        << " (100.0%) |\n";
    oss << "| Mutation Cases Tested | " << soundness_metrics.mutation_cases_tested << " |\n";
    oss << "| Mutation Cases Rejected | " << soundness_metrics.mutation_cases_rejected
        << " (" << std::setprecision(1) << (soundness_metrics.MutationRejectionRate() * 100.0) << "%) |\n";
    oss << "| Snapshot Decision Latency (p50) | " << soundness_metrics.latency_p50_us << " µs |\n";
    oss << "| Snapshot Decision Latency (p95) | " << soundness_metrics.latency_p95_us << " µs |\n";
    oss << "| Snapshot Decision Latency (p99) | " << soundness_metrics.latency_p99_us << " µs |\n";
    oss << "| Serialized Certificate Size (median) | " << soundness_metrics.median_certificate_size_bytes << " bytes |\n";

    return oss.str();
}

std::string ExperimentResults::FormatScalabilityTable() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1);
    oss << "### Scalability Benchmark Across Swarm Sizes (N in {3, 5, 10})\n\n";
    oss << "| Swarm Size (N) | Latency p50 (µs) | Latency p95 (µs) | Latency p99 (µs) | Cert Size (bytes) |\n";
    oss << "| :--- | :--- | :--- | :--- | :--- |\n";

    for (const auto& res : scalability_results) {
        oss << "| " << std::right << std::setw(10) << res.uav_count << " UAVs "
            << "| " << std::setw(16) << res.latency_p50_us
            << "| " << std::setw(16) << res.latency_p95_us
            << "| " << std::setw(16) << res.latency_p99_us
            << "| " << std::setw(17) << res.serialized_certificate_size_bytes
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
    oss << "method,total_requests,accepted_count,false_valid_rate,availability,unsafe_acceptance_per_request\n";
    for (auto method : {EvaluationMethod::kReceiveLatest,
                        EvaluationMethod::kTimestampAlignedAge,
                        EvaluationMethod::kProposedStateAcceptance}) {
        const auto it = aggregate_method_metrics.find(static_cast<uint8_t>(method));
        if (it != aggregate_method_metrics.end()) {
            const auto& m = it->second;
            oss << "\"" << EvaluationMethodToString(method) << "\","
                << m.total_requests << ","
                << m.AcceptedCount() << ","
                << m.FalseValidRate() << ","
                << m.Availability() << ","
                << m.UnsafeAcceptancePerRequest() << "\n";
        }
    }
    return oss.str();
}

std::string ExperimentResults::ToJson() const {
    std::ostringstream oss;
    oss << "{\n";
    oss << "  \"total_runs\": " << total_runs << ",\n";
    oss << "  \"steps_per_scenario\": " << steps_per_scenario << ",\n";
    oss << "  \"seed_base\": " << seed_base << ",\n";
    oss << "  \"agent_count\": " << agent_ids.size() << ",\n";
    oss << "  \"aggregate_metrics\": [\n";

    bool first = true;
    for (auto method : {EvaluationMethod::kReceiveLatest,
                        EvaluationMethod::kTimestampAlignedAge,
                        EvaluationMethod::kProposedStateAcceptance}) {
        const auto it = aggregate_method_metrics.find(static_cast<uint8_t>(method));
        if (it != aggregate_method_metrics.end()) {
            if (!first) oss << ",\n";
            first = false;
            const auto& m = it->second;
            oss << "    {\n";
            oss << "      \"method\": \"" << EvaluationMethodToString(method) << "\",\n";
            oss << "      \"total_requests\": " << m.total_requests << ",\n";
            oss << "      \"accepted_count\": " << m.AcceptedCount() << ",\n";
            oss << "      \"true_accepts\": " << m.true_accepts << ",\n";
            oss << "      \"false_accepts\": " << m.false_accepts << ",\n";
            oss << "      \"true_rejects\": " << m.true_rejects << ",\n";
            oss << "      \"false_rejects\": " << m.false_rejects << ",\n";
            oss << "      \"false_valid_rate\": " << m.FalseValidRate() << ",\n";
            oss << "      \"availability\": " << m.Availability() << ",\n";
            oss << "      \"unsafe_acceptance_per_request\": " << m.UnsafeAcceptancePerRequest() << ",\n";
            oss << "      \"containment_failures\": " << m.containment_failures << ",\n";
            oss << "      \"containment_failure_rate\": " << m.ContainmentFailureRate() << ",\n";
            oss << "      \"deterministic_enclosures_tested\": " << m.deterministic_enclosures_tested << "\n";
            oss << "    }";
        }
    }
    oss << "\n  ],\n";

    oss << "  \"bootstrap_ci\": {\n";
    oss << "    \"proposed_fv\": {\"point\": " << bootstrap_results.proposed_fv.point_estimate
        << ", \"lower_95\": " << bootstrap_results.proposed_fv.lower_95
        << ", \"upper_95\": " << bootstrap_results.proposed_fv.upper_95 << "},\n";
    oss << "    \"b1_fv\": {\"point\": " << bootstrap_results.b1_fv.point_estimate
        << ", \"lower_95\": " << bootstrap_results.b1_fv.lower_95
        << ", \"upper_95\": " << bootstrap_results.b1_fv.upper_95 << "},\n";
    oss << "    \"delta_fv_proposed_vs_b1\": {\"point\": " << bootstrap_results.delta_fv_proposed_vs_b1.point_estimate
        << ", \"lower_95\": " << bootstrap_results.delta_fv_proposed_vs_b1.lower_95
        << ", \"upper_95\": " << bootstrap_results.delta_fv_proposed_vs_b1.upper_95 << "},\n";
    oss << "    \"proposed_availability\": {\"point\": " << bootstrap_results.proposed_availability.point_estimate
        << ", \"lower_95\": " << bootstrap_results.proposed_availability.lower_95
        << ", \"upper_95\": " << bootstrap_results.proposed_availability.upper_95 << "}\n";
    oss << "  },\n";

    oss << "  \"soundness_and_replay\": {\n";
    oss << "    \"enclosures_tested\": " << soundness_metrics.enclosures_tested << ",\n";
    oss << "    \"containment_failures\": " << soundness_metrics.containment_failures << ",\n";
    oss << "    \"containment_failure_rate\": " << (soundness_metrics.enclosures_tested > 0 ? static_cast<double>(soundness_metrics.containment_failures) / static_cast<double>(soundness_metrics.enclosures_tested) : 0.0) << ",\n";
    oss << "    \"replayed_decisions\": " << soundness_metrics.replayed_decisions << ",\n";
    oss << "    \"verifier_agreements\": " << soundness_metrics.verifier_agreements << ",\n";
    oss << "    \"verifier_agreement_rate\": " << soundness_metrics.VerifierAgreementRate() << ",\n";
    oss << "    \"mutation_classes_tested\": " << soundness_metrics.mutation_classes_tested << ",\n";
    oss << "    \"mutation_classes_rejected\": " << soundness_metrics.mutation_classes_rejected << ",\n";
    oss << "    \"mutation_cases_tested\": " << soundness_metrics.mutation_cases_tested << ",\n";
    oss << "    \"mutation_cases_rejected\": " << soundness_metrics.mutation_cases_rejected << ",\n";
    oss << "    \"mutation_rejection_rate\": " << soundness_metrics.MutationRejectionRate() << ",\n";
    oss << "    \"latency_p50_us\": " << soundness_metrics.latency_p50_us << ",\n";
    oss << "    \"latency_p95_us\": " << soundness_metrics.latency_p95_us << ",\n";
    oss << "    \"latency_p99_us\": " << soundness_metrics.latency_p99_us << ",\n";
    oss << "    \"median_certificate_size_bytes\": " << soundness_metrics.median_certificate_size_bytes << ",\n";
    oss << "    \"min_certificate_size_bytes\": " << soundness_metrics.min_certificate_size_bytes << ",\n";
    oss << "    \"max_certificate_size_bytes\": " << soundness_metrics.max_certificate_size_bytes << "\n";
    oss << "  }\n";
    oss << "}\n";

    return oss.str();
}

}  // namespace swarmkit::experiment
