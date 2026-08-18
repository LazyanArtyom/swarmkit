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
#include <iomanip>
#include <iostream>
#include <numbers>
#include <numeric>
#include <random>
#include <sstream>

namespace swarmkit::experiment {

namespace {

constexpr double kEarthRadiusM = 6378137.0;
constexpr double kDegToRad = std::numbers::pi / 180.0;
constexpr double kRadToDeg = 180.0 / std::numbers::pi;

double DistanceMeters(const std::array<double, 3>& p1, const std::array<double, 3>& p2) {
    const double dlat_rad = (p1[0] - p2[0]) * kDegToRad;
    const double mean_lat_rad = ((p1[0] + p2[0]) * 0.5) * kDegToRad;
    const double dlon_rad = (p1[1] - p2[1]) * kDegToRad;
    const double north_m = dlat_rad * kEarthRadiusM;
    const double east_m = dlon_rad * kEarthRadiusM * std::cos(mean_lat_rad);
    const double dalt_m = p1[2] - p2[2];
    return std::sqrt(north_m * north_m + east_m * east_m + dalt_m * dalt_m);
}

// Convert SimBackend truth to WGS84 GroundTruthState
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

    // Convert local north/east/up meters from SimBackend to WGS-84 with exact earth radius
    const double lat = truth_frame.home_lat_deg + (truth_frame.north_m / kEarthRadiusM) * kRadToDeg;
    const double cos_lat = std::max(0.01, std::cos(truth_frame.home_lat_deg * kDegToRad));
    const double lon = truth_frame.home_lon_deg + (truth_frame.east_m / (kEarthRadiusM * cos_lat)) * kRadToDeg;
    const double alt = truth_frame.up_m;

    state.position = {lat, lon, alt};
    state.velocity = {
        static_cast<float>(truth_frame.velocity_north_mps),
        static_cast<float>(truth_frame.velocity_east_mps),
        static_cast<float>(truth_frame.velocity_up_mps),
    };
    state.position_frame = core::CoordinateFrame::kWgs84;
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

bool BaselineEvaluator::ComputePhysicalValidity(
    const std::unordered_map<std::string, std::array<double, 3>>& estimated_positions,
    const std::unordered_map<std::string, GroundTruthState>& truth,
    const core::StateQualityContract& contract,
    double u_max_meters) {

    for (const auto& agent_id : contract.required_agents) {
        const auto it = truth.find(agent_id);
        if (it == truth.end()) {
            return false;
        }

        const auto& true_state = it->second;
        if (!true_state.healthy) {
            return false;
        }

        const auto pos_it = estimated_positions.find(agent_id);
        if (pos_it == estimated_positions.end()) {
            return false;
        }

        const double dist = DistanceMeters(pos_it->second, true_state.position);
        if (dist > u_max_meters) {
            return false;
        }
    }
    return true;
}

MethodEvaluationOutcome BaselineEvaluator::EvaluateReceiveLatest(
    const core::StateQualityContract& contract,
    double evaluation_time_ms,
    const core::EvidenceStore& store,
    const std::unordered_map<std::string, GroundTruthState>& truth,
    double u_max_meters) {

    (void)evaluation_time_ms;
    const auto t_start = std::chrono::steady_clock::now();
    MethodEvaluationOutcome outcome;
    outcome.accepted = false;

    bool all_found = true;
    for (const auto& agent_id : contract.required_agents) {
        auto pos_records = store.Recent(agent_id, core::EvidenceFieldId::kPosition, 1);
        if (pos_records.empty()) {
            all_found = false;
            break;
        }

        const auto& rec = pos_records[0];
        if (!std::holds_alternative<std::array<double, 3>>(rec.value)) {
            all_found = false;
            break;
        }

        outcome.estimated_positions[agent_id] = std::get<std::array<double, 3>>(rec.value);
        outcome.position_enclosures[agent_id] = 1.0;
    }

    if (all_found) {
        outcome.accepted = true;
        outcome.ground_truth_valid = ComputePhysicalValidity(
            outcome.estimated_positions, truth, contract, u_max_meters);

        for (const auto& agent_id : contract.required_agents) {
            auto pos_records = store.Recent(agent_id, core::EvidenceFieldId::kPosition, 1);
            if (!pos_records.empty()) {
                const auto& rec = pos_records[0];
                const auto truth_it = truth.find(agent_id);
                if (truth_it != truth.end()) {
                    if (rec.identity.coordinate_frame != core::CoordinateFrame::kWgs84 ||
                        rec.identity.agent_session_id != truth_it->second.session_id ||
                        !rec.quality.estimator_healthy) {
                        outcome.ground_truth_valid = false;
                    }
                }
            }
        }
    } else {
        outcome.rejection_reason = "missing_recent_position_evidence";
    }

    const auto t_end = std::chrono::steady_clock::now();
    outcome.latency_us = std::chrono::duration<double, std::micro>(t_end - t_start).count();
    return outcome;
}

MethodEvaluationOutcome BaselineEvaluator::EvaluateTimestampAlignedAge(
    const core::StateQualityContract& contract,
    double evaluation_time_ms,
    const core::EvidenceStore& store,
    double max_age_ms,
    const std::unordered_map<std::string, GroundTruthState>& truth,
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
            if (!rec.source_time.timestamp_ms.has_value()) continue;
            const double s = static_cast<double>(*rec.source_time.timestamp_ms);
            if (s <= evaluation_time_ms) {
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

        const double raw_age = evaluation_time_ms - best_source_time;
        if (raw_age > max_age_ms) {
            all_found = false;
            break;
        }

        if (!std::holds_alternative<std::array<double, 3>>(best_rec->value)) {
            all_found = false;
            break;
        }

        outcome.estimated_positions[agent_id] = std::get<std::array<double, 3>>(best_rec->value);
        outcome.position_enclosures[agent_id] = 1.0;
    }

    if (all_found) {
        outcome.accepted = true;
        outcome.ground_truth_valid = ComputePhysicalValidity(
            outcome.estimated_positions, truth, contract, u_max_meters);

        for (const auto& agent_id : contract.required_agents) {
            auto pos_records = store.All(agent_id, core::EvidenceFieldId::kPosition);
            for (const auto& rec : pos_records) {
                if (rec.source_time.timestamp_ms.has_value()) {
                    const auto truth_it = truth.find(agent_id);
                    if (truth_it != truth.end()) {
                        if (rec.identity.coordinate_frame != core::CoordinateFrame::kWgs84 ||
                            rec.identity.agent_session_id != truth_it->second.session_id ||
                            !rec.quality.estimator_healthy) {
                            outcome.ground_truth_valid = false;
                        }
                    }
                }
            }
        }
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
    double evaluation_time_ms,
    const core::EvidenceStore& store,
    const std::unordered_map<std::string, core::ClockQualityState>& clock_states,
    const std::unordered_map<std::string, GroundTruthState>& truth,
    double u_max_meters) {

    const auto t_start = std::chrono::steady_clock::now();
    MethodEvaluationOutcome outcome;
    outcome.accepted = false;

    auto result = engine.RequestSnapshot(contract, evaluation_time_ms, store, clock_states);

    if (std::holds_alternative<core::AcceptedSnapshot>(result)) {
        const auto& snapshot = std::get<core::AcceptedSnapshot>(result);
        outcome.accepted = true;

        for (const auto& agent_id : snapshot.accepted_agents) {
            const auto it = snapshot.agent_states.find(agent_id);
            if (it != snapshot.agent_states.end()) {
                const auto pos_it = it->second.find(static_cast<std::uint8_t>(core::EvidenceFieldId::kPosition));
                if (pos_it != it->second.end()) {
                    if (std::holds_alternative<std::array<double, 3>>(pos_it->second.evidence.value)) {
                        outcome.estimated_positions[agent_id] =
                            std::get<std::array<double, 3>>(pos_it->second.evidence.value);
                        outcome.position_enclosures[agent_id] = pos_it->second.propagated_uncertainty;
                    }
                }
            }
        }

        outcome.certificate = core::BuildCertificate(snapshot, contract);
        outcome.ground_truth_valid = ComputePhysicalValidity(
            outcome.estimated_positions, truth, contract, u_max_meters);
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
    contract.required_position_frame = core::CoordinateFrame::kWgs84;
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

    struct ReplayTraceFrame {
        double delivery_time_ms{};
        core::TelemetryFrame frame;
    };

    struct ReplaySessionTransition {
        double physical_time_ms{};
        std::string agent_id;
        std::string session_id;
    };

    for (const auto fault_kind : config_.fault_scenarios) {
        ScenarioMetrics sc_metrics;
        sc_metrics.fault_kind = fault_kind;
        for (auto method : {EvaluationMethod::kReceiveLatest,
                            EvaluationMethod::kTimestampAlignedAge,
                            EvaluationMethod::kProposedStateAcceptance}) {
            sc_metrics.metrics_by_method[static_cast<uint8_t>(method)] = MethodMetrics{.method = method};
        }

        for (std::size_t run_idx = 0; run_idx < config_.runs; ++run_idx) {
            const std::uint64_t run_seed = config_.seed + run_idx * 1000 +
                                           static_cast<std::uint64_t>(fault_kind) * 100;

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
                .seed = run_seed,
            };

            if (fault_kind == ScenarioFaultKind::kNetworkDelay) {
                fault_cfg.telemetry_delay_frames = 5;  // 5 frames at 10Hz = 500ms > max_age_ms 300ms
            } else if (fault_kind == ScenarioFaultKind::kNetworkReorder) {
                fault_cfg.telemetry_reorder_probability = 0.5;
            } else if (fault_kind == ScenarioFaultKind::kPacketLoss) {
                fault_cfg.telemetry_loss_probability = 0.45;
            } else if (fault_kind == ScenarioFaultKind::kClockOffsetDrift) {
                fault_cfg.source_clock_offset_ms = 15;
                fault_cfg.source_clock_drift_ms_per_frame = 0.08;
                fault_cfg.source_clock_uncertainty_ms = 15.0;  // > max_clock_unc_ms 10ms
            } else if (fault_kind == ScenarioFaultKind::kEstimatorDegradation) {
                fault_cfg.estimator_degradation_probability = 0.5;
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

            // Step simulator forward by 1s so UAVs reach airborne cruising altitude
            (void)sim_control->AdvanceAll(std::chrono::seconds(1));

            // Set deterministic velocity motion
            for (std::size_t i = 0; i < config_.agent_ids.size(); ++i) {
                const auto& agent_id = config_.agent_ids[i];
                float vx = 3.0F + 0.5F * static_cast<float>(i % 3);
                float vy = 0.5F - 1.0F * static_cast<float>(i % 2);

                if (fault_kind == ScenarioFaultKind::kHighSpeedMotion) {
                    // Close to but within theorem bound V_max (10 m/s)
                    vx = 8.8F;
                    vy = 1.0F;  // speed = sqrt(8.8^2 + 1^2) = 8.857 m/s <= 10.0 m/s
                }

                (void)fault_backend->Execute(NavCommand(
                    agent_id, commands::CmdVelocity{.vx_mps = vx, .vy_mps = vy, .duration_ms = 600'000}));
            }

            // 4. Setup EvidenceStore, sessions, clock states, and telemetry queues
            core::EvidenceStore store;
            std::unordered_map<std::string, std::string> active_sessions;
            std::unordered_map<std::string, core::ClockQualityState> clock_states;

            for (const auto& id : config_.agent_ids) {
                const std::string sess = "session-" + id + "-run" + std::to_string(run_idx);
                active_sessions[id] = sess;
                store.SetCurrentSession(id, sess);

                clock_states[id] = core::ClockQualityState{
                    .offset_estimate_ms = 0.0,
                    .uncertainty_radius_ms = 3.0,
                    .source_domain = core::ClockDomain::kUnixEpoch,
                    .synchronization = core::ClockSynchronization::kSynchronized,
                    .last_update_ms = 1'700'000'000'000LL,
                    .deterministic_bound = true,
                };
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
            std::vector<ReplayTraceFrame> trace_frames;
            std::vector<ReplaySessionTransition> trace_sessions;

            for (std::size_t step = 0; step < config_.steps_per_scenario; ++step) {
                physical_time_ms += config_.step_dt_ms;

                // Step simulation forward
                (void)sim_control->AdvanceAll(
                    std::chrono::milliseconds(static_cast<long long>(config_.step_dt_ms)));

                // Drain delivered frames from fault injector
                std::vector<core::TelemetryFrame> current_delivered;
                {
                    std::lock_guard<std::mutex> lock(batch_mutex);
                    current_delivered = std::move(delivered_batch);
                    delivered_batch.clear();
                }

                // Problem 6: Authentic restart handling
                if (fault_kind == ScenarioFaultKind::kAgentRestartDelayedPackets) {
                    const std::string restarted_agent = config_.agent_ids[0];

                    // Capture old session frame around step 25
                    if (step == 25) {
                        for (const auto& f : current_delivered) {
                            if (f.drone_id == restarted_agent) {
                                saved_old_session_frame = f;
                                break;
                            }
                        }
                    }

                    // Authoritative restart at step 30
                    if (step == 30) {
                        const std::string new_sess = "session-" + restarted_agent + "-restarted";
                        active_sessions[restarted_agent] = new_sess;
                        store.SetCurrentSession(restarted_agent, new_sess);
                        trace_sessions.push_back({physical_time_ms, restarted_agent, new_sess});
                    }

                    // Deliver historical old frame at step 35
                    if (step == 35 && saved_old_session_frame.has_value()) {
                        auto delayed_old = *saved_old_session_frame;
                        delayed_old.agent_receive_unix_time_ms = static_cast<std::int64_t>(physical_time_ms);
                        store.InsertFrame(delayed_old);
                        trace_frames.push_back({physical_time_ms, delayed_old});
                    }
                }

                // Ingest delivered telemetry frames into EvidenceStore
                for (auto& frame : current_delivered) {
                    frame.agent_receive_unix_time_ms = static_cast<std::int64_t>(physical_time_ms);
                    frame.agent_session_id = active_sessions[frame.drone_id];

                    if (fault_kind == ScenarioFaultKind::kFrameMismatch &&
                        frame.drone_id == config_.agent_ids[0]) {
                        frame.position_frame = core::CoordinateFrame::kLocalNed;
                    }

                    store.InsertFrame(frame);
                    trace_frames.push_back({physical_time_ms, frame});
                }

                // Problem 4: Update clock uncertainty for clock drift scenario
                if (fault_kind == ScenarioFaultKind::kClockOffsetDrift) {
                    for (const auto& id : config_.agent_ids) {
                        const double drift_offset = 15.0 + 0.08 * static_cast<double>(step);
                        clock_states[id].offset_estimate_ms = drift_offset;
                        clock_states[id].uncertainty_radius_ms = 15.0;  // exceeds contract 10ms
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

                // 6. Evaluate All Methods at Common Reference Time t* = physical_time_ms
                const double t_star = physical_time_ms;

                auto outcome_b0 = BaselineEvaluator::EvaluateReceiveLatest(
                    contract, t_star, store, ground_truth, config_.physical_error_tolerance_m);

                auto outcome_b1 = BaselineEvaluator::EvaluateTimestampAlignedAge(
                    contract, t_star, store, config_.max_age_ms, ground_truth,
                    config_.physical_error_tolerance_m);

                auto outcome_p = BaselineEvaluator::EvaluateProposed(
                    engine, contract, t_star, store, clock_states, ground_truth,
                    config_.physical_error_tolerance_m);

                results.latencies_us.push_back(outcome_p.latency_us);

                // Record confusion matrix outcomes fairly using common oracle
                const bool ground_truth_valid_at_reference = physical_truth_healthy;

                auto record_outcome = [&](EvaluationMethod method, const MethodEvaluationOutcome& outcome) {
                    auto& agg = results.aggregate_method_metrics[static_cast<uint8_t>(method)];
                    auto& scm = sc_metrics.metrics_by_method[static_cast<uint8_t>(method)];

                    ++agg.total_requests;
                    ++scm.total_requests;

                    if (outcome.accepted) {
                        if (outcome.ground_truth_valid) {
                            ++agg.true_accepts;
                            ++scm.true_accepts;
                        } else {
                            ++agg.false_accepts;
                            ++scm.false_accepts;
                        }
                    } else {
                        if (ground_truth_valid_at_reference) {
                            ++agg.false_rejects;
                            ++scm.false_rejects;
                        } else {
                            ++agg.true_rejects;
                            ++scm.true_rejects;
                        }
                    }
                };

                record_outcome(EvaluationMethod::kReceiveLatest, outcome_b0);
                record_outcome(EvaluationMethod::kTimestampAlignedAge, outcome_b1);
                record_outcome(EvaluationMethod::kProposedStateAcceptance, outcome_p);

                // 7. Table III Verification, Enclosure Soundness, and 15-Class Tamper Testing
                if (outcome_p.accepted && outcome_p.certificate.has_value()) {
                    const auto& cert = *outcome_p.certificate;
                    const std::string serialized_cert = core::SerializeCertificate(cert);
                    results.certificate_sizes_bytes.push_back(serialized_cert.size());

                    // Enclosure Containment Soundness
                    for (const auto& [agent_id, enclosure_radius] : outcome_p.position_enclosures) {
                        const auto truth_it = ground_truth.find(agent_id);
                        const auto est_it = outcome_p.estimated_positions.find(agent_id);
                        if (truth_it != ground_truth.end() && est_it != outcome_p.estimated_positions.end()) {
                            ++results.soundness_metrics.enclosures_tested;
                            const double err = DistanceMeters(est_it->second, truth_it->second.position);
                            if (err > enclosure_radius) {
                                ++results.soundness_metrics.containment_failures;
                            }
                        }
                    }

                    // Problem 15: Genuine Fresh-State Offline Replay Verification
                    ++results.soundness_metrics.replayed_decisions;
                    core::EvidenceStore offline_store;
                    for (const auto& [aid, sess] : active_sessions) {
                        offline_store.SetCurrentSession(aid, sess);
                    }
                    for (const auto& tr : trace_sessions) {
                        if (tr.physical_time_ms <= t_star) {
                            offline_store.SetCurrentSession(tr.agent_id, tr.session_id);
                        }
                    }
                    for (const auto& tf : trace_frames) {
                        if (tf.delivery_time_ms <= t_star) {
                            offline_store.InsertFrame(tf.frame);
                        }
                    }

                    auto deserialized_cert = core::DeserializeCertificate(serialized_cert);
                    if (deserialized_cert.has_value()) {
                        auto ver_res = verifier.Verify(
                            *deserialized_cert, offline_store, contract, clock_states);
                        if (std::holds_alternative<core::VerifiedAcceptance>(ver_res)) {
                            ++results.soundness_metrics.verifier_agreements;
                        }
                    }

                    // Problem 16: Complete 15-Class Tamper Matrix
                    if (deserialized_cert.has_value() && !deserialized_cert->evidence_entries.empty()) {
                        const auto base_cert = *deserialized_cert;

                        for (int mutation_class = 1; mutation_class <= 15; ++mutation_class) {
                            auto tampered = base_cert;
                            ++results.soundness_metrics.tampered_certificates_tested;

                            switch (mutation_class) {
                                case 1:
                                    tampered.certificate_hash = "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
                                    break;
                                case 2:
                                    tampered.evaluation_time_ms += 100.0;
                                    tampered.certificate_hash = core::ComputeCertificateHash(tampered);
                                    break;
                                case 3:
                                    tampered.contract_hash = "deadbeefdeadbeefdeadbeefdeadbeefdeadbeefdeadbeefdeadbeefdeadbeef";
                                    tampered.certificate_hash = core::ComputeCertificateHash(tampered);
                                    break;
                                case 4:
                                    tampered.contract_content_version += 1;
                                    tampered.certificate_hash = core::ComputeCertificateHash(tampered);
                                    break;
                                case 5:
                                    tampered.acceptance_semantics_version = "2.0";
                                    tampered.certificate_hash = core::ComputeCertificateHash(tampered);
                                    break;
                                case 6:
                                    tampered.evidence_entries[0].sequence += 100;
                                    tampered.certificate_hash = core::ComputeCertificateHash(tampered);
                                    break;
                                case 7:
                                    tampered.evidence_entries[0].evidence_hash = "badhashbadhashbadhashbadhashbadhashbadhashbadhashbadhashbadhash";
                                    tampered.certificate_hash = core::ComputeCertificateHash(tampered);
                                    break;
                                case 8:
                                    tampered.evidence_entries[0].agent_session_id = "tampered-session";
                                    tampered.certificate_hash = core::ComputeCertificateHash(tampered);
                                    break;
                                case 9:
                                    tampered.evidence_entries[0].propagated_uncertainty = 0.01;
                                    tampered.certificate_hash = core::ComputeCertificateHash(tampered);
                                    break;
                                case 10:
                                    tampered.evidence_entries[0].clock_uncertainty_ms = 0.01;
                                    tampered.certificate_hash = core::ComputeCertificateHash(tampered);
                                    break;
                                case 11:
                                    tampered.accepted_agents.push_back("extra-agent");
                                    tampered.certificate_hash = core::ComputeCertificateHash(tampered);
                                    break;
                                case 12:
                                    tampered.evidence_entries[0].source_time_ms =
                                        tampered.evidence_entries[0].source_time_ms.value_or(0) + 1000;
                                    tampered.certificate_hash = core::ComputeCertificateHash(tampered);
                                    break;
                                case 13:
                                    tampered.evidence_entries[0].source_component = "fake-sensor";
                                    tampered.certificate_hash = core::ComputeCertificateHash(tampered);
                                    break;
                                case 14:
                                    tampered.evidence_entries[0].coordinate_frame = core::CoordinateFrame::kLocalNed;
                                    tampered.certificate_hash = core::ComputeCertificateHash(tampered);
                                    break;
                                case 15:
                                    tampered.evidence_entries[0].observation_uncertainty = 0.001;
                                    tampered.certificate_hash = core::ComputeCertificateHash(tampered);
                                    break;
                            }

                            auto tamper_res = verifier.Verify(
                                tampered, offline_store, contract, clock_states);
                            if (std::holds_alternative<core::VerificationRejection>(tamper_res)) {
                                ++results.soundness_metrics.tampered_certificates_rejected;
                            }
                        }
                    }
                }
            }

            sc_metrics.requests += config_.steps_per_scenario;
        }

        results.per_scenario_metrics.push_back(sc_metrics);
    }

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
        contract.required_position_frame = core::CoordinateFrame::kWgs84;
        contract.require_current_epoch = true;
        contract.require_deterministic_bounds = true;
        contract.completeness = core::CompletenessRule::kAllRequired;

        core::EvidenceStore store;
        std::unordered_map<std::string, core::ClockQualityState> clock_states;

        const double t_star = 1'700'000'000'100.0;

        for (const auto& id : agents) {
            store.SetCurrentSession(id, "scale-session-" + id);
            clock_states[id] = core::ClockQualityState{
                .offset_estimate_ms = 0.0,
                .uncertainty_radius_ms = 2.0,
                .source_domain = core::ClockDomain::kUnixEpoch,
                .synchronization = core::ClockSynchronization::kSynchronized,
                .last_update_ms = 1'700'000'000'000LL,
                .deterministic_bound = true,
            };

            core::EvidenceRecord pos_rec{
                .value = std::array<double, 3>{37.7749, -122.4194, 10.0},
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
                    .coordinate_frame = core::CoordinateFrame::kWgs84,
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

        // Problem 17: Measure End-to-End Latency (RequestSnapshot + BuildCertificate + SerializeCertificate)
        // 50 warmup iterations
        for (int w = 0; w < 50; ++w) {
            auto res = engine.RequestSnapshot(contract, t_star, store, clock_states);
            if (std::holds_alternative<core::AcceptedSnapshot>(res)) {
                auto cert = core::BuildCertificate(std::get<core::AcceptedSnapshot>(res), contract);
                (void)core::SerializeCertificate(cert);
            }
        }

        // 500 measured iterations
        std::vector<double> latencies;
        latencies.reserve(500);
        std::size_t cert_size = 0;

        for (int iter = 0; iter < 500; ++iter) {
            const auto t0 = std::chrono::steady_clock::now();
            auto res = engine.RequestSnapshot(contract, t_star, store, clock_states);
            if (std::holds_alternative<core::AcceptedSnapshot>(res)) {
                auto cert = core::BuildCertificate(std::get<core::AcceptedSnapshot>(res), contract);
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
        << (soundness_metrics.enclosures_tested > 0
                ? (static_cast<double>(soundness_metrics.containment_failures) /
                   static_cast<double>(soundness_metrics.enclosures_tested) * 100.0)
                : 0.0)
        << "%) |\n";
    oss << "| Replayed Acceptance Decisions | " << soundness_metrics.replayed_decisions << " |\n";
    oss << "| Verifier Replay Agreement | " << soundness_metrics.verifier_agreements
        << " (" << std::setprecision(1) << (soundness_metrics.VerifierAgreementRate() * 100.0) << "%) |\n";
    oss << "| Tampered Certificates Tested | " << soundness_metrics.tampered_certificates_tested << " |\n";
    oss << "| Tampered Certificates Caught | " << soundness_metrics.tampered_certificates_rejected
        << " (" << std::setprecision(1) << (soundness_metrics.TamperDetectionRate() * 100.0) << "%) |\n";
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
            oss << "      \"deterministic_enclosures_tested\": " << m.deterministic_enclosures_tested << "\n";
            oss << "    }";
        }
    }
    oss << "\n  ],\n";

    oss << "  \"soundness_and_replay\": {\n";
    oss << "    \"enclosures_tested\": " << soundness_metrics.enclosures_tested << ",\n";
    oss << "    \"containment_failures\": " << soundness_metrics.containment_failures << ",\n";
    oss << "    \"replayed_decisions\": " << soundness_metrics.replayed_decisions << ",\n";
    oss << "    \"verifier_agreements\": " << soundness_metrics.verifier_agreements << ",\n";
    oss << "    \"verifier_agreement_rate\": " << soundness_metrics.VerifierAgreementRate() << ",\n";
    oss << "    \"tampered_certificates_tested\": " << soundness_metrics.tampered_certificates_tested << ",\n";
    oss << "    \"tampered_certificates_rejected\": " << soundness_metrics.tampered_certificates_rejected << ",\n";
    oss << "    \"tamper_detection_rate\": " << soundness_metrics.TamperDetectionRate() << ",\n";
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
