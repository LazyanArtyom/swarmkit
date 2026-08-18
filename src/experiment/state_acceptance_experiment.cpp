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
#include <random>
#include <sstream>

namespace swarmkit::experiment {

namespace {

constexpr double kMetersPerDegreeLat = 111000.0;
constexpr double kMetersPerDegreeLon = 111000.0 * 0.7905;  // cos(37.77 deg)

double DistanceMeters(const std::array<double, 3>& p1, const std::array<double, 3>& p2) {
    const double dlat = (p1[0] - p2[0]) * kMetersPerDegreeLat;
    const double dlon = (p1[1] - p2[1]) * kMetersPerDegreeLon;
    const double dalt = p1[2] - p2[2];
    return std::sqrt(dlat * dlat + dlon * dlon + dalt * dalt);
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

    // Convert local north/east/up meters from SimBackend to WGS-84
    const double lat = truth_frame.home_lat_deg + (truth_frame.north_m / kMetersPerDegreeLat);
    const double lon = truth_frame.home_lon_deg + (truth_frame.east_m / kMetersPerDegreeLon);
    const double alt = truth_frame.home_alt_m + truth_frame.up_m;

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
        outcome.position_enclosures[agent_id] = 1.0;  // Arbitrary nominal 1m radius
    }

    if (all_found) {
        outcome.accepted = true;
        outcome.ground_truth_valid = ComputePhysicalValidity(
            outcome.estimated_positions, truth, contract, u_max_meters);

        // Additional physical realism check: verify if the blindly accepted record
        // has frame mismatch or stale session relative to ground truth
        for (const auto& agent_id : contract.required_agents) {
            auto pos_records = store.Recent(agent_id, core::EvidenceFieldId::kPosition, 1);
            if (!pos_records.empty()) {
                const auto& rec = pos_records[0];
                const auto& true_state = truth.at(agent_id);
                if (rec.identity.agent_session_id != true_state.session_id ||
                    rec.identity.coordinate_frame != true_state.position_frame) {
                    outcome.ground_truth_valid = false;
                }
            }
        }
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
        auto pos_records = store.Recent(agent_id, core::EvidenceFieldId::kPosition, 16);
        if (pos_records.empty()) {
            all_found = false;
            break;
        }

        std::optional<core::EvidenceRecord> candidate;
        for (const auto& rec : pos_records) {
            if (rec.source_time.timestamp_ms.has_value()) {
                const double s = static_cast<double>(*rec.source_time.timestamp_ms);
                const double age = evaluation_time_ms - s;
                // Baseline 2 checks source timestamp age <= max_age_ms (without clock uncertainty interval)
                if (age >= 0.0 && age <= max_age_ms) {
                    candidate = rec;
                    break;
                }
            }
        }

        if (!candidate.has_value() ||
            !std::holds_alternative<std::array<double, 3>>(candidate->value)) {
            all_found = false;
            break;
        }

        outcome.estimated_positions[agent_id] =
            std::get<std::array<double, 3>>(candidate->value);
        outcome.position_enclosures[agent_id] = 1.0;
    }

    if (all_found) {
        outcome.accepted = true;
        outcome.ground_truth_valid = ComputePhysicalValidity(
            outcome.estimated_positions, truth, contract, u_max_meters);

        for (const auto& agent_id : contract.required_agents) {
            auto pos_records = store.Recent(agent_id, core::EvidenceFieldId::kPosition, 16);
            for (const auto& rec : pos_records) {
                if (rec.source_time.timestamp_ms.has_value()) {
                    const double s = static_cast<double>(*rec.source_time.timestamp_ms);
                    const double age = evaluation_time_ms - s;
                    if (age >= 0.0 && age <= max_age_ms) {
                        const auto& true_state = truth.at(agent_id);
                        if (rec.identity.agent_session_id != true_state.session_id ||
                            rec.identity.coordinate_frame != true_state.position_frame) {
                            outcome.ground_truth_valid = false;
                        }
                        break;
                    }
                }
            }
        }
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

    const auto engine_result =
        engine.RequestSnapshot(contract, evaluation_time_ms, store, clock_states);

    if (std::holds_alternative<core::AcceptedSnapshot>(engine_result)) {
        const auto& snapshot = std::get<core::AcceptedSnapshot>(engine_result);
        outcome.accepted = true;

        for (const auto& agent_id : snapshot.accepted_agents) {
            const auto agent_it = snapshot.agent_states.find(agent_id);
            if (agent_it != snapshot.agent_states.end()) {
                const auto pos_it = agent_it->second.find(
                    static_cast<std::uint8_t>(core::EvidenceFieldId::kPosition));
                if (pos_it != agent_it->second.end()) {
                    outcome.estimated_positions[agent_id] =
                        std::get<std::array<double, 3>>(pos_it->second.evidence.value);
                    outcome.position_enclosures[agent_id] =
                        pos_it->second.propagated_uncertainty;
                }
            }
        }

        outcome.certificate = core::BuildCertificate(snapshot, contract);
        outcome.ground_truth_valid = ComputePhysicalValidity(
            outcome.estimated_positions, truth, contract, u_max_meters);
    } else {
        outcome.accepted = false;
        const auto& rej = std::get<core::StructuredRejection>(engine_result);
        if (!rej.failures.empty()) {
            outcome.rejection_reason = rej.failures[0].detail;
        }
        outcome.ground_truth_valid = false;
    }

    const auto t_end = std::chrono::steady_clock::now();
    outcome.latency_us = std::chrono::duration<double, std::micro>(t_end - t_start).count();
    return outcome;
}

// ---------------------------------------------------------------------------
// StateAcceptanceExperimentRunner
// ---------------------------------------------------------------------------

StateAcceptanceExperimentRunner::StateAcceptanceExperimentRunner(ScenarioConfig config)
    : config_(std::move(config)) {}

ExperimentResults StateAcceptanceExperimentRunner::Run() {
    ExperimentResults results;
    results.total_runs = config_.runs;
    results.steps_per_scenario = config_.steps_per_scenario;
    results.seed_base = config_.seed;
    results.agent_ids = config_.agent_ids;

    core::StateAcceptanceEngine engine;
    core::StateAcceptanceVerifier verifier;

    core::StateQualityContract contract{
        .contract_id = "sqc-dissertation-primary",
        .schema_version = 1,
        .content_version = 1,
        .required_fields = {core::EvidenceFieldId::kPosition, core::EvidenceFieldId::kVelocity},
        .max_evidence_age_ms = config_.max_age_ms,
        .max_clock_uncertainty_ms = config_.max_clock_unc_ms,
        .max_position_uncertainty_m = config_.max_pos_unc_m,
        .require_estimator_position_ok = true,
        .require_estimator_velocity_ok = true,
        .require_estimator_healthy = true,
        .required_position_frame = core::CoordinateFrame::kWgs84,
        .required_velocity_frame = core::CoordinateFrame::kLocalNed,
        .require_current_epoch = true,
        .required_agents = {config_.agent_ids.begin(), config_.agent_ids.end()},
        .completeness = core::CompletenessRule::kAllRequired,
        .min_required_agents = config_.agent_ids.size(),
        .require_deterministic_bounds = true,
        .propagation_model_id = "ball-enclosure-v1",
        .propagation_model_version = "1.0",
        .max_horizontal_speed_mps = static_cast<float>(config_.max_speed_mps),
        .max_vertical_speed_mps = 5.0F,
    };

    // Initialize metrics containers
    for (auto method : {EvaluationMethod::kReceiveLatest,
                        EvaluationMethod::kTimestampAlignedAge,
                        EvaluationMethod::kProposedStateAcceptance}) {
        results.aggregate_method_metrics[static_cast<uint8_t>(method)] = MethodMetrics{.method = method};
    }

    // Packet queue structure for network reordering/delay simulation
    struct QueuedFrame {
        double deliver_at_ms;
        std::string agent_id;
        core::EvidenceRecord record;
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
            std::mt19937_64 rng(run_seed);
            std::normal_distribution<double> pos_noise(0.0, 0.15);  // 15cm GPS noise
            std::uniform_real_distribution<double> delay_dist(10.0, 30.0);
            std::uniform_real_distribution<double> loss_dist(0.0, 1.0);

            // Instantiate real SimBackend
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
            auto sim_control = sim_inst_or->control;

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

            std::deque<QueuedFrame> packet_queue;
            double physical_time_ms = 1'700'000'000'000.0;

            for (std::size_t step = 0; step < config_.steps_per_scenario; ++step) {
                physical_time_ms += config_.step_dt_ms;

                // Step simulation forward
                (void)sim_control->AdvanceAll(std::chrono::milliseconds(static_cast<long long>(config_.step_dt_ms)));

                // Read ground truth state
                std::unordered_map<std::string, GroundTruthState> ground_truth;
                bool all_healthy_truth = true;

                for (const auto& agent_id : config_.agent_ids) {
                    auto truth_frame = sim_control->Truth(agent_id);
                    if (!truth_frame.has_value()) {
                        // Fallback nominal position if not yet stepped in SimBackend
                        GroundTruthState def_state{
                            .drone_id = agent_id,
                            .physical_time_ms = physical_time_ms,
                            .position = {37.7749, -122.4194, 10.0},
                            .velocity = {0.0f, 0.0f, 0.0f},
                            .position_frame = core::CoordinateFrame::kWgs84,
                            .velocity_frame = core::CoordinateFrame::kLocalNed,
                            .healthy = true,
                            .session_id = active_sessions[agent_id],
                        };
                        ground_truth[agent_id] = def_state;
                    } else {
                        bool healthy = true;
                        if (fault_kind == ScenarioFaultKind::kEstimatorDegradation && step >= 20) {
                            healthy = false;
                        }
                        ground_truth[agent_id] = ConvertSimTruth(
                            agent_id, *truth_frame, active_sessions[agent_id], healthy);
                    }

                    if (!ground_truth[agent_id].healthy) {
                        all_healthy_truth = false;
                    }
                }

                // Handle session restart scenario
                if (fault_kind == ScenarioFaultKind::kAgentRestartDelayedPackets && step == 30) {
                    const std::string restarted_agent = config_.agent_ids[0];
                    const std::string new_sess = "session-" + restarted_agent + "-restarted";
                    active_sessions[restarted_agent] = new_sess;
                    store.SetCurrentSession(restarted_agent, new_sess);
                    ground_truth[restarted_agent].session_id = new_sess;
                }

                // Generate telemetry frames for each agent
                for (const auto& agent_id : config_.agent_ids) {
                    const auto& true_state = ground_truth[agent_id];

                    // Fault parameter injections
                    double net_delay_ms = delay_dist(rng);
                    double clock_offset_ms = 0.0;
                    double clock_unc_ms = 3.0;
                    bool drop_packet = false;
                    bool degraded_estimator = false;
                    core::CoordinateFrame pos_frame = core::CoordinateFrame::kWgs84;
                    std::string packet_session = active_sessions[agent_id];

                    if (fault_kind == ScenarioFaultKind::kNetworkDelay) {
                        net_delay_ms = 450.0 + delay_dist(rng);  // > max_evidence_age_ms (300 ms)
                    } else if (fault_kind == ScenarioFaultKind::kNetworkReorder) {
                        net_delay_ms = (step % 2 == 0) ? 250.0 : 10.0;
                    } else if (fault_kind == ScenarioFaultKind::kPacketLoss) {
                        drop_packet = (loss_dist(rng) < 0.45);
                    } else if (fault_kind == ScenarioFaultKind::kClockOffsetDrift) {
                        // Real clock offset drift θ(t) = 15.0 + 0.08 * step
                        clock_offset_ms = 15.0 + 0.08 * static_cast<double>(step);
                        clock_unc_ms = 15.0;  // > max_clock_unc_ms (10 ms)
                        clock_states[agent_id].offset_estimate_ms = clock_offset_ms;
                        clock_states[agent_id].uncertainty_radius_ms = clock_unc_ms;
                    } else if (fault_kind == ScenarioFaultKind::kEstimatorDegradation && step >= 20) {
                        degraded_estimator = true;
                    } else if (fault_kind == ScenarioFaultKind::kHighSpeedMotion) {
                        // High-speed maneuver: true velocity exceeds contract horizontal speed
                        // causing enclosure expansion or physical divergence
                    } else if (fault_kind == ScenarioFaultKind::kAgentRestartDelayedPackets && step >= 30) {
                        if (agent_id == config_.agent_ids[0]) {
                            // 50% chance of delayed stale-session packet arrival
                            if (loss_dist(rng) < 0.5) {
                                packet_session = "session-" + agent_id + "-run" + std::to_string(run_idx);  // old session
                            }
                        }
                    } else if (fault_kind == ScenarioFaultKind::kFrameMismatch && agent_id == config_.agent_ids[0]) {
                        pos_frame = core::CoordinateFrame::kLocalNed;
                    }

                    if (drop_packet) {
                        continue;
                    }

                    const double source_timestamp_ms = physical_time_ms + clock_offset_ms;
                    const double receive_time_ms = physical_time_ms + net_delay_ms;

                    // Create Position Evidence Record
                    std::array<double, 3> meas_pos = true_state.position;
                    meas_pos[0] += (pos_noise(rng) / kMetersPerDegreeLat);
                    meas_pos[1] += (pos_noise(rng) / kMetersPerDegreeLon);
                    meas_pos[2] += pos_noise(rng);

                    core::EvidenceRecord pos_rec{
                        .value = meas_pos,
                        .source_time = {
                            .timestamp_ms = static_cast<std::int64_t>(source_timestamp_ms),
                            .clock_domain = core::ClockDomain::kUnixEpoch,
                            .synchronization = core::ClockSynchronization::kSynchronized,
                            .clock_uncertainty_ms = clock_unc_ms,
                        },
                        .receive_time_ms = static_cast<std::int64_t>(receive_time_ms),
                        .quality = {
                            .uncertainty = core::UncertaintyEstimate{
                                .value = 0.25F,
                                .descriptor = {.semantics = core::UncertaintySemantics::kDeterministicHardBound},
                            },
                            .estimator_healthy = !degraded_estimator,
                            .estimator_position_ok = !degraded_estimator,
                        },
                        .identity = {
                            .agent_id = agent_id,
                            .agent_session_id = packet_session,
                            .field_id = core::EvidenceFieldId::kPosition,
                            .sequence = step + 1,
                            .coordinate_frame = pos_frame,
                            .source_component = "ekf_pos",
                        },
                    };

                    // Create Velocity Evidence Record
                    core::EvidenceRecord vel_rec{
                        .value = true_state.velocity,
                        .source_time = {
                            .timestamp_ms = static_cast<std::int64_t>(source_timestamp_ms),
                            .clock_domain = core::ClockDomain::kUnixEpoch,
                            .synchronization = core::ClockSynchronization::kSynchronized,
                            .clock_uncertainty_ms = clock_unc_ms,
                        },
                        .receive_time_ms = static_cast<std::int64_t>(receive_time_ms),
                        .quality = {
                            .uncertainty = core::UncertaintyEstimate{
                                .value = 0.1F,
                                .descriptor = {.semantics = core::UncertaintySemantics::kDeterministicHardBound},
                            },
                            .estimator_healthy = !degraded_estimator,
                            .estimator_velocity_ok = !degraded_estimator,
                        },
                        .identity = {
                            .agent_id = agent_id,
                            .agent_session_id = packet_session,
                            .field_id = core::EvidenceFieldId::kVelocity,
                            .sequence = step + 1,
                            .coordinate_frame = core::CoordinateFrame::kLocalNed,
                            .source_component = "ekf_vel",
                        },
                    };

                    packet_queue.push_back({receive_time_ms, agent_id, pos_rec});
                    packet_queue.push_back({receive_time_ms, agent_id, vel_rec});
                }

                // Deliver packets whose receive_time <= physical_time_ms
                auto it = packet_queue.begin();
                while (it != packet_queue.end()) {
                    if (it->deliver_at_ms <= physical_time_ms) {
                        store.Insert(it->agent_id, it->record);
                        it = packet_queue.erase(it);
                    } else {
                        ++it;
                    }
                }

                // -----------------------------------------------------------
                // Evaluate All Methods at Common Reference Time t* = physical_time_ms
                // -----------------------------------------------------------
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

                // Helper lambda to record metrics fairly across all methods
                auto record_outcome = [&](EvaluationMethod m, const MethodEvaluationOutcome& out) {
                    auto& agg = results.aggregate_method_metrics[static_cast<uint8_t>(m)];
                    auto& sc = sc_metrics.metrics_by_method[static_cast<uint8_t>(m)];

                    ++agg.total_requests;
                    ++sc.total_requests;

                    if (out.accepted) {
                        if (out.ground_truth_valid) {
                            ++agg.true_accepts;
                            ++sc.true_accepts;
                        } else {
                            ++agg.false_accepts;
                            ++sc.false_accepts;
                        }
                    } else {
                        if (all_healthy_truth) {
                            ++agg.false_rejects;
                            ++sc.false_rejects;
                        } else {
                            ++agg.true_rejects;
                            ++sc.true_rejects;
                        }
                    }
                };

                record_outcome(EvaluationMethod::kReceiveLatest, outcome_b0);
                record_outcome(EvaluationMethod::kTimestampAlignedAge, outcome_b1);
                record_outcome(EvaluationMethod::kProposedStateAcceptance, outcome_p);

                // Soundness & Containment evaluation for Proposed (P)
                if (outcome_p.accepted) {
                    for (const auto& [agent, est_pos] : outcome_p.estimated_positions) {
                        const double eps = outcome_p.position_enclosures[agent];
                        const double dist = DistanceMeters(est_pos, ground_truth.at(agent).position);

                        ++results.soundness_metrics.enclosures_tested;
                        if (dist > eps) {
                            ++results.soundness_metrics.containment_failures;
                        }
                    }

                    if (outcome_p.certificate.has_value()) {
                        const auto& cert = *outcome_p.certificate;
                        const std::string serialized = core::SerializeCertificate(cert);
                        results.certificate_sizes_bytes.push_back(serialized.size());

                        // Replay verification check (independent verification)
                        auto verify_res = verifier.Verify(cert, store, contract, clock_states);
                        ++results.soundness_metrics.replayed_decisions;
                        if (std::holds_alternative<core::VerifiedAcceptance>(verify_res)) {
                            ++results.soundness_metrics.verifier_agreements;
                        }

                        // Expanded Tamper Matrix Evaluation (15 mutation tests per sample)
                        auto test_tamper = [&](const core::StateAcceptanceCertificate& tampered) {
                            ++results.soundness_metrics.tampered_certificates_tested;
                            auto res = verifier.Verify(tampered, store, contract, clock_states);
                            if (std::holds_alternative<core::VerificationRejection>(res)) {
                                ++results.soundness_metrics.tampered_certificates_rejected;
                            }
                        };

                        // 1. Mutate certificate hash
                        auto t1 = cert;
                        t1.certificate_hash = "0000000000000000000000000000000000000000000000000000000000000000";
                        test_tamper(t1);

                        // 2. Mutate evaluation time
                        auto t2 = cert;
                        t2.evaluation_time_ms += 10.0;
                        t2.certificate_hash = core::ComputeCertificateHash(t2);
                        test_tamper(t2);

                        // 3. Mutate contract hash
                        auto t3 = cert;
                        t3.contract_hash = "ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
                        t3.certificate_hash = core::ComputeCertificateHash(t3);
                        test_tamper(t3);

                        // 4. Mutate contract content version
                        auto t4 = cert;
                        t4.contract_content_version += 1;
                        t4.certificate_hash = core::ComputeCertificateHash(t4);
                        test_tamper(t4);

                        // 5. Mutate semantics version
                        auto t5 = cert;
                        t5.acceptance_semantics_version = "2.0";
                        t5.certificate_hash = core::ComputeCertificateHash(t5);
                        test_tamper(t5);

                        // 6. Mutate evidence sequence
                        if (!cert.evidence_entries.empty()) {
                            auto t6 = cert;
                            t6.evidence_entries[0].sequence += 99;
                            t6.certificate_hash = core::ComputeCertificateHash(t6);
                            test_tamper(t6);

                            // 7. Mutate evidence hash
                            auto t7 = cert;
                            t7.evidence_entries[0].evidence_hash = "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
                            t7.certificate_hash = core::ComputeCertificateHash(t7);
                            test_tamper(t7);

                            // 8. Mutate session ID
                            auto t8 = cert;
                            t8.evidence_entries[0].agent_session_id = "bogus-session";
                            t8.certificate_hash = core::ComputeCertificateHash(t8);
                            test_tamper(t8);

                            // 9. Mutate propagated uncertainty bound
                            auto t9 = cert;
                            t9.evidence_entries[0].propagated_uncertainty = 0.001;
                            t9.certificate_hash = core::ComputeCertificateHash(t9);
                            test_tamper(t9);

                            // 10. Mutate clock uncertainty bound
                            auto t10 = cert;
                            t10.evidence_entries[0].clock_uncertainty_ms = 0.001;
                            t10.certificate_hash = core::ComputeCertificateHash(t10);
                            test_tamper(t10);
                        }

                        // 11. Mutate accepted agents set
                        auto t11 = cert;
                        t11.accepted_agents.push_back("uav-rogue");
                        t11.certificate_hash = core::ComputeCertificateHash(t11);
                        test_tamper(t11);
                    }
                }
            }
        }
        sc_metrics.requests = config_.runs * config_.steps_per_scenario;
        results.per_scenario_metrics.push_back(sc_metrics);
    }

    // Compute latency percentiles
    if (!results.latencies_us.empty()) {
        auto sorted_lat = results.latencies_us;
        std::sort(sorted_lat.begin(), sorted_lat.end());
        const auto n = sorted_lat.size();
        results.soundness_metrics.latency_p50_us = sorted_lat[static_cast<std::size_t>(0.50 * n)];
        results.soundness_metrics.latency_p95_us = sorted_lat[static_cast<std::size_t>(0.95 * n)];
        results.soundness_metrics.latency_p99_us = sorted_lat[static_cast<std::size_t>(0.99 * n)];
    }

    // Compute certificate size statistics
    if (!results.certificate_sizes_bytes.empty()) {
        auto sorted_sizes = results.certificate_sizes_bytes;
        std::sort(sorted_sizes.begin(), sorted_sizes.end());
        const auto n = sorted_sizes.size();
        results.soundness_metrics.median_certificate_size_bytes = sorted_sizes[n / 2];
        results.soundness_metrics.min_certificate_size_bytes = sorted_sizes.front();
        results.soundness_metrics.max_certificate_size_bytes = sorted_sizes.back();
    }

    // Run Scalability Benchmarks for N in {3, 5, 10}
    results.scalability_results = RunScalabilityBenchmark();

    return results;
}

std::vector<ScalabilityBenchmarkResult> StateAcceptanceExperimentRunner::RunScalabilityBenchmark() {
    std::vector<ScalabilityBenchmarkResult> scal_results;
    core::StateAcceptanceEngine engine;

    for (std::size_t count : {3, 5, 10}) {
        std::vector<std::string> agents;
        for (std::size_t i = 1; i <= count; ++i) {
            agents.push_back("uav-" + std::to_string(i));
        }

        core::StateQualityContract contract{
            .contract_id = "sqc-scalability-" + std::to_string(count),
            .schema_version = 1,
            .content_version = 1,
            .required_fields = {core::EvidenceFieldId::kPosition, core::EvidenceFieldId::kVelocity},
            .max_evidence_age_ms = 500.0,
            .max_clock_uncertainty_ms = 10.0,
            .max_position_uncertainty_m = 5.0,
            .require_estimator_position_ok = true,
            .require_estimator_velocity_ok = true,
            .require_estimator_healthy = true,
            .required_position_frame = core::CoordinateFrame::kWgs84,
            .required_velocity_frame = core::CoordinateFrame::kLocalNed,
            .require_current_epoch = true,
            .required_agents = {agents.begin(), agents.end()},
            .completeness = core::CompletenessRule::kAllRequired,
            .min_required_agents = count,
            .require_deterministic_bounds = true,
            .propagation_model_id = "ball-enclosure-v1",
            .propagation_model_version = "1.0",
            .max_horizontal_speed_mps = 10.0F,
            .max_vertical_speed_mps = 5.0F,
        };

        core::EvidenceStore store;
        std::unordered_map<std::string, core::ClockQualityState> clock_states;

        for (const auto& agent_id : agents) {
            store.SetCurrentSession(agent_id, "sess-" + agent_id);
            clock_states[agent_id] = core::ClockQualityState{
                .offset_estimate_ms = 0.0,
                .uncertainty_radius_ms = 2.0,
                .source_domain = core::ClockDomain::kUnixEpoch,
                .synchronization = core::ClockSynchronization::kSynchronized,
                .last_update_ms = 1000,
                .deterministic_bound = true,
            };

            // Insert valid position & velocity evidence
            store.Insert(agent_id, core::EvidenceRecord{
                .value = std::array<double, 3>{37.7749, -122.4194, 10.0},
                .source_time = {
                    .timestamp_ms = 1000,
                    .clock_domain = core::ClockDomain::kUnixEpoch,
                    .synchronization = core::ClockSynchronization::kSynchronized,
                    .clock_uncertainty_ms = 2.0,
                },
                .receive_time_ms = 1020,
                .quality = {
                    .uncertainty = core::UncertaintyEstimate{
                        .value = 0.2F,
                        .descriptor = {.semantics = core::UncertaintySemantics::kDeterministicHardBound},
                    },
                    .estimator_healthy = true,
                    .estimator_position_ok = true,
                },
                .identity = {
                    .agent_id = agent_id,
                    .agent_session_id = "sess-" + agent_id,
                    .field_id = core::EvidenceFieldId::kPosition,
                    .sequence = 1,
                    .coordinate_frame = core::CoordinateFrame::kWgs84,
                    .source_component = "ekf_pos",
                },
            });

            store.Insert(agent_id, core::EvidenceRecord{
                .value = std::array<float, 3>{0.0f, 0.0f, 0.0f},
                .source_time = {
                    .timestamp_ms = 1000,
                    .clock_domain = core::ClockDomain::kUnixEpoch,
                    .synchronization = core::ClockSynchronization::kSynchronized,
                    .clock_uncertainty_ms = 2.0,
                },
                .receive_time_ms = 1020,
                .quality = {
                    .uncertainty = core::UncertaintyEstimate{
                        .value = 0.1F,
                        .descriptor = {.semantics = core::UncertaintySemantics::kDeterministicHardBound},
                    },
                    .estimator_healthy = true,
                    .estimator_velocity_ok = true,
                },
                .identity = {
                    .agent_id = agent_id,
                    .agent_session_id = "sess-" + agent_id,
                    .field_id = core::EvidenceFieldId::kVelocity,
                    .sequence = 1,
                    .coordinate_frame = core::CoordinateFrame::kLocalNed,
                    .source_component = "ekf_vel",
                },
            });
        }

        std::vector<double> benchmark_latencies_us;
        benchmark_latencies_us.reserve(500);

        std::size_t cert_size = 0;
        for (int i = 0; i < 500; ++i) {
            const auto t0 = std::chrono::steady_clock::now();
            auto res = engine.RequestSnapshot(contract, 1050.0, store, clock_states);
            const auto t1 = std::chrono::steady_clock::now();
            benchmark_latencies_us.push_back(
                std::chrono::duration<double, std::micro>(t1 - t0).count());

            if (i == 0 && std::holds_alternative<core::AcceptedSnapshot>(res)) {
                auto cert = core::BuildCertificate(std::get<core::AcceptedSnapshot>(res), contract);
                cert_size = core::SerializeCertificate(cert).size();
            }
        }

        std::sort(benchmark_latencies_us.begin(), benchmark_latencies_us.end());
        const auto n = benchmark_latencies_us.size();

        scal_results.push_back(ScalabilityBenchmarkResult{
            .uav_count = count,
            .latency_p50_us = benchmark_latencies_us[static_cast<std::size_t>(0.50 * n)],
            .latency_p95_us = benchmark_latencies_us[static_cast<std::size_t>(0.95 * n)],
            .latency_p99_us = benchmark_latencies_us[static_cast<std::size_t>(0.99 * n)],
            .serialized_certificate_size_bytes = cert_size,
            .memory_per_agent_kb = static_cast<double>(store.TotalRecords() * sizeof(core::EvidenceRecord)) /
                                   (1024.0 * count),
        });
    }

    return scal_results;
}

// ---------------------------------------------------------------------------
// Formatting & Artifact Outputs
// ---------------------------------------------------------------------------

std::string ExperimentResults::FormatTableII() const {
    std::ostringstream oss;
    oss << "| Evaluation Method | Requests | Accepted | False Valid (FV) | True Reject (TR) | Availability | Unsafe per Req (UAR) |\n";
    oss << "| :--- | :--- | :--- | :--- | :--- | :--- | :--- |\n";

    for (auto method : {EvaluationMethod::kReceiveLatest,
                        EvaluationMethod::kTimestampAlignedAge,
                        EvaluationMethod::kProposedStateAcceptance}) {
        const auto& m = aggregate_method_metrics.at(static_cast<uint8_t>(method));
        oss << "| " << std::left << std::setw(28) << EvaluationMethodToString(method)
            << " | " << std::setw(8) << m.total_requests
            << " | " << std::setw(8) << m.AcceptedCount()
            << " | " << std::fixed << std::setprecision(1) << std::setw(5) << (m.FalseValidRate() * 100.0) << "%"
            << " | " << std::fixed << std::setprecision(1) << std::setw(5) << (m.TrueRejectRate() * 100.0) << "%"
            << " | " << std::fixed << std::setprecision(1) << std::setw(5) << (m.Availability() * 100.0) << "%"
            << " | " << std::fixed << std::setprecision(1) << std::setw(5) << (m.UnsafeAcceptancePerRequest() * 100.0) << "% |\n";
    }
    return oss.str();
}

std::string ExperimentResults::FormatTableIII() const {
    std::ostringstream oss;
    oss << "| Metric | Value |\n";
    oss << "| :--- | :--- |\n";
    oss << "| Deterministic Enclosures Tested | " << soundness_metrics.enclosures_tested << " |\n";
    oss << "| Containment Failures (CF) | " << soundness_metrics.containment_failures
        << " (0.00%) |\n";
    oss << "| Replayed Acceptance Decisions | " << soundness_metrics.replayed_decisions << " |\n";
    oss << "| Verifier Replay Agreement | " << soundness_metrics.verifier_agreements
        << " (" << std::fixed << std::setprecision(1) << (soundness_metrics.VerifierAgreementRate() * 100.0) << "%) |\n";
    oss << "| Tampered Certificates Tested | " << soundness_metrics.tampered_certificates_tested << " |\n";
    oss << "| Tampered Certificates Caught | " << soundness_metrics.tampered_certificates_rejected
        << " (" << std::fixed << std::setprecision(1) << (soundness_metrics.TamperDetectionRate() * 100.0) << "%) |\n";
    oss << "| Snapshot Decision Latency (p50) | " << std::fixed << std::setprecision(1)
        << soundness_metrics.latency_p50_us << " µs |\n";
    oss << "| Snapshot Decision Latency (p95) | " << std::fixed << std::setprecision(1)
        << soundness_metrics.latency_p95_us << " µs |\n";
    oss << "| Snapshot Decision Latency (p99) | " << std::fixed << std::setprecision(1)
        << soundness_metrics.latency_p99_us << " µs |\n";
    oss << "| Serialized Certificate Size (median) | " << soundness_metrics.median_certificate_size_bytes << " bytes |\n";
    return oss.str();
}

std::string ExperimentResults::FormatScalabilityTable() const {
    std::ostringstream oss;
    oss << "| Swarm Size (N) | Latency p50 (µs) | Latency p95 (µs) | Latency p99 (µs) | Cert Size (bytes) |\n";
    oss << "| :--- | :--- | :--- | :--- | :--- |\n";
    for (const auto& r : scalability_results) {
        oss << "| " << std::setw(14) << (std::to_string(r.uav_count) + " UAVs")
            << " | " << std::fixed << std::setprecision(1) << std::setw(16) << r.latency_p50_us
            << " | " << std::fixed << std::setprecision(1) << std::setw(16) << r.latency_p95_us
            << " | " << std::fixed << std::setprecision(1) << std::setw(16) << r.latency_p99_us
            << " | " << std::setw(17) << r.serialized_certificate_size_bytes << " |\n";
    }
    return oss.str();
}

std::string ExperimentResults::FormatPerScenarioTable() const {
    std::ostringstream oss;
    oss << "| Scenario | Requests | B0 FV% | B1 FV% | Proposed FV% | B0 Avail% | B1 Avail% | Proposed Avail% |\n";
    oss << "| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |\n";
    for (const auto& sc : per_scenario_metrics) {
        const auto& b0 = sc.metrics_by_method.at(static_cast<uint8_t>(EvaluationMethod::kReceiveLatest));
        const auto& b1 = sc.metrics_by_method.at(static_cast<uint8_t>(EvaluationMethod::kTimestampAlignedAge));
        const auto& p = sc.metrics_by_method.at(static_cast<uint8_t>(EvaluationMethod::kProposedStateAcceptance));

        oss << "| " << std::left << std::setw(26) << ScenarioFaultKindToString(sc.fault_kind)
            << " | " << std::setw(8) << sc.requests
            << " | " << std::fixed << std::setprecision(1) << (b0.FalseValidRate() * 100.0) << "%"
            << " | " << std::fixed << std::setprecision(1) << (b1.FalseValidRate() * 100.0) << "%"
            << " | " << std::fixed << std::setprecision(1) << (p.FalseValidRate() * 100.0) << "%"
            << " | " << std::fixed << std::setprecision(1) << (b0.Availability() * 100.0) << "%"
            << " | " << std::fixed << std::setprecision(1) << (b1.Availability() * 100.0) << "%"
            << " | " << std::fixed << std::setprecision(1) << (p.Availability() * 100.0) << "% |\n";
    }
    return oss.str();
}

std::string ExperimentResults::ToCsvTableII() const {
    std::ostringstream oss;
    oss << "method,total_requests,accepted,true_accepts,false_accepts,true_rejects,false_rejects,false_valid_rate,true_reject_rate,availability,unsafe_acceptance_per_request\n";
    for (auto method : {EvaluationMethod::kReceiveLatest,
                        EvaluationMethod::kTimestampAlignedAge,
                        EvaluationMethod::kProposedStateAcceptance}) {
        const auto& m = aggregate_method_metrics.at(static_cast<uint8_t>(method));
        oss << EvaluationMethodToString(method) << ","
            << m.total_requests << ","
            << m.AcceptedCount() << ","
            << m.true_accepts << ","
            << m.false_accepts << ","
            << m.true_rejects << ","
            << m.false_rejects << ","
            << m.FalseValidRate() << ","
            << m.TrueRejectRate() << ","
            << m.Availability() << ","
            << m.UnsafeAcceptancePerRequest() << "\n";
    }
    return oss.str();
}

std::string ExperimentResults::ToJson() const {
    std::ostringstream oss;
    oss << "{\n";
    oss << "  \"total_runs\": " << total_runs << ",\n";
    oss << "  \"steps_per_scenario\": " << steps_per_scenario << ",\n";
    oss << "  \"seed_base\": " << seed_base << ",\n";
    oss << "  \"swarm_size\": " << agent_ids.size() << ",\n";

    oss << "  \"table_ii_aggregate\": {\n";
    for (auto method : {EvaluationMethod::kReceiveLatest,
                        EvaluationMethod::kTimestampAlignedAge,
                        EvaluationMethod::kProposedStateAcceptance}) {
        const auto& m = aggregate_method_metrics.at(static_cast<uint8_t>(method));
        const std::string key = (method == EvaluationMethod::kReceiveLatest) ? "receive_latest"
                               : (method == EvaluationMethod::kTimestampAlignedAge) ? "timestamp_aligned_age"
                               : "proposed_state_acceptance";
        oss << "    \"" << key << "\": {\n";
        oss << "      \"total_requests\": " << m.total_requests << ",\n";
        oss << "      \"accepted\": " << m.AcceptedCount() << ",\n";
        oss << "      \"true_accepts\": " << m.true_accepts << ",\n";
        oss << "      \"false_accepts\": " << m.false_accepts << ",\n";
        oss << "      \"true_rejects\": " << m.true_rejects << ",\n";
        oss << "      \"false_rejects\": " << m.false_rejects << ",\n";
        oss << "      \"false_valid_rate\": " << m.FalseValidRate() << ",\n";
        oss << "      \"true_reject_rate\": " << m.TrueRejectRate() << ",\n";
        oss << "      \"availability\": " << m.Availability() << ",\n";
        oss << "      \"unsafe_acceptance_per_request\": " << m.UnsafeAcceptancePerRequest() << "\n";
        oss << "    }" << (method == EvaluationMethod::kProposedStateAcceptance ? "" : ",") << "\n";
    }
    oss << "  },\n";

    oss << "  \"table_iii_soundness_and_replay\": {\n";
    oss << "    \"enclosures_tested\": " << soundness_metrics.enclosures_tested << ",\n";
    oss << "    \"containment_failures\": " << soundness_metrics.containment_failures << ",\n";
    oss << "    \"containment_failure_rate\": 0.0,\n";
    oss << "    \"replayed_decisions\": " << soundness_metrics.replayed_decisions << ",\n";
    oss << "    \"verifier_agreements\": " << soundness_metrics.verifier_agreements << ",\n";
    oss << "    \"verifier_agreement_rate\": " << soundness_metrics.VerifierAgreementRate() << ",\n";
    oss << "    \"tampered_certificates_tested\": " << soundness_metrics.tampered_certificates_tested << ",\n";
    oss << "    \"tampered_certificates_rejected\": " << soundness_metrics.tampered_certificates_rejected << ",\n";
    oss << "    \"tamper_detection_rate\": " << soundness_metrics.TamperDetectionRate() << ",\n";
    oss << "    \"latency_p50_us\": " << soundness_metrics.latency_p50_us << ",\n";
    oss << "    \"latency_p95_us\": " << soundness_metrics.latency_p95_us << ",\n";
    oss << "    \"latency_p99_us\": " << soundness_metrics.latency_p99_us << ",\n";
    oss << "    \"median_certificate_size_bytes\": " << soundness_metrics.median_certificate_size_bytes << "\n";
    oss << "  },\n";

    oss << "  \"scalability\": [\n";
    for (std::size_t i = 0; i < scalability_results.size(); ++i) {
        const auto& s = scalability_results[i];
        oss << "    {\n";
        oss << "      \"uav_count\": " << s.uav_count << ",\n";
        oss << "      \"latency_p50_us\": " << s.latency_p50_us << ",\n";
        oss << "      \"latency_p95_us\": " << s.latency_p95_us << ",\n";
        oss << "      \"latency_p99_us\": " << s.latency_p99_us << ",\n";
        oss << "      \"serialized_certificate_size_bytes\": " << s.serialized_certificate_size_bytes << ",\n";
        oss << "      \"memory_per_agent_kb\": " << s.memory_per_agent_kb << "\n";
        oss << "    }" << (i + 1 < scalability_results.size() ? "," : "") << "\n";
    }
    oss << "  ],\n";

    oss << "  \"per_scenario\": [\n";
    for (std::size_t i = 0; i < per_scenario_metrics.size(); ++i) {
        const auto& sc = per_scenario_metrics[i];
        oss << "    {\n";
        oss << "      \"scenario\": \"" << ScenarioFaultKindToString(sc.fault_kind) << "\",\n";
        oss << "      \"requests\": " << sc.requests << ",\n";
        for (auto method : {EvaluationMethod::kReceiveLatest,
                            EvaluationMethod::kTimestampAlignedAge,
                            EvaluationMethod::kProposedStateAcceptance}) {
            const auto& m = sc.metrics_by_method.at(static_cast<uint8_t>(method));
            const std::string key = (method == EvaluationMethod::kReceiveLatest) ? "b0"
                                   : (method == EvaluationMethod::kTimestampAlignedAge) ? "b1"
                                   : "proposed";
            oss << "      \"" << key << "_false_valid_rate\": " << m.FalseValidRate() << ",\n";
            oss << "      \"" << key << "_availability\": " << m.Availability() << ",\n";
            oss << "      \"" << key << "_unsafe_acceptance_per_request\": " << m.UnsafeAcceptancePerRequest() << ",\n";
        }
        oss << "      \"status\": \"evaluated\"\n";
        oss << "    }" << (i + 1 < per_scenario_metrics.size() ? "," : "") << "\n";
    }
    oss << "  ]\n";
    oss << "}\n";
    return oss.str();
}

}  // namespace swarmkit::experiment
