// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "swarmkit/experiment/state_acceptance_experiment.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
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
        case EvaluationMethod::kReceiveLatest: return "Receive-latest";
        case EvaluationMethod::kTimestampAlignedAge: return "Timestamp-aligned + age";
        case EvaluationMethod::kProposedStateAcceptance: return "Proposed state acceptance";
    }
    return "unknown";
}

// ---------------------------------------------------------------------------
// BaselineEvaluator
// ---------------------------------------------------------------------------

MethodEvaluationOutcome BaselineEvaluator::EvaluateReceiveLatest(
    const core::StateQualityContract& contract,
    double evaluation_time_ms,
    const core::EvidenceStore& store,
    const std::unordered_map<std::string, GroundTruthState>& truth) {

    (void)evaluation_time_ms;
    MethodEvaluationOutcome outcome;
    outcome.accepted = false;

    for (const auto& agent_id : contract.required_agents) {
        auto pos_records = store.Recent(agent_id, core::EvidenceFieldId::kPosition, 1);
        if (pos_records.empty()) {
            outcome.rejection_reason = "missing position for " + agent_id;
            return outcome;
        }

        const auto& rec = pos_records[0];
        if (!std::holds_alternative<std::array<double, 3>>(rec.value)) {
            outcome.rejection_reason = "invalid position type for " + agent_id;
            return outcome;
        }

        outcome.estimated_positions[agent_id] = std::get<std::array<double, 3>>(rec.value);
        outcome.position_enclosures[agent_id] = 1.0;  // Fixed arbitrary nominal radius
    }

    outcome.accepted = true;

    // Ground truth validation: check if true positions at evaluation_time_ms are within nominal tolerance (0.5m)
    // and whether agents are in valid sessions, healthy estimators, and correct frame.
    bool valid = true;
    for (const auto& agent_id : contract.required_agents) {
        auto it = truth.find(agent_id);
        if (it == truth.end() || !it->second.healthy) {
            valid = false;
            break;
        }

        auto pos_recs = store.Recent(agent_id, core::EvidenceFieldId::kPosition, 1);
        if (pos_recs.empty()) {
            valid = false;
            break;
        }
        const auto& rec = pos_recs[0];
        if (rec.identity.agent_session_id != it->second.session_id ||
            rec.identity.coordinate_frame != it->second.position_frame) {
            valid = false;
            break;
        }

        const auto est_pos = outcome.estimated_positions[agent_id];
        const double dist = DistanceMeters(est_pos, it->second.position);
        if (dist > 0.5) {  // Nominal physical error tolerance
            valid = false;
            break;
        }
    }

    outcome.ground_truth_valid = valid;
    return outcome;
}

MethodEvaluationOutcome BaselineEvaluator::EvaluateTimestampAlignedAge(
    const core::StateQualityContract& contract,
    double evaluation_time_ms,
    const core::EvidenceStore& store,
    double max_age_ms,
    const std::unordered_map<std::string, GroundTruthState>& truth) {

    MethodEvaluationOutcome outcome;
    outcome.accepted = false;

    std::unordered_map<std::string, core::EvidenceRecord> selected_records;

    for (const auto& agent_id : contract.required_agents) {
        auto pos_records = store.Recent(agent_id, core::EvidenceFieldId::kPosition, 8);
        if (pos_records.empty()) {
            outcome.rejection_reason = "missing position for " + agent_id;
            return outcome;
        }

        // Find newest record within max_age_ms relative to evaluation_time_ms
        std::optional<core::EvidenceRecord> candidate;
        for (const auto& rec : pos_records) {
            if (rec.source_time.timestamp_ms.has_value()) {
                const double s = static_cast<double>(*rec.source_time.timestamp_ms);
                const double age = evaluation_time_ms - s;
                if (age >= 0.0 && age <= max_age_ms) {
                    candidate = rec;
                    break;
                }
            }
        }

        if (!candidate.has_value()) {
            outcome.rejection_reason = "age exceeded for " + agent_id;
            return outcome;
        }

        selected_records[agent_id] = *candidate;
        outcome.estimated_positions[agent_id] = std::get<std::array<double, 3>>(candidate->value);
        outcome.position_enclosures[agent_id] = 1.0;
    }

    outcome.accepted = true;

    // Ground truth validation
    bool valid = true;
    for (const auto& agent_id : contract.required_agents) {
        auto it = truth.find(agent_id);
        if (it == truth.end() || !it->second.healthy) {
            valid = false;
            break;
        }

        const auto& rec = selected_records[agent_id];
        if (rec.identity.agent_session_id != it->second.session_id ||
            rec.identity.coordinate_frame != it->second.position_frame) {
            valid = false;
            break;
        }

        const auto est_pos = outcome.estimated_positions[agent_id];
        const double dist = DistanceMeters(est_pos, it->second.position);
        if (dist > 0.5) {
            valid = false;
            break;
        }
    }

    outcome.ground_truth_valid = valid;
    return outcome;
}

MethodEvaluationOutcome BaselineEvaluator::EvaluateProposed(
    const core::StateAcceptanceEngine& engine,
    const core::StateQualityContract& contract,
    double evaluation_time_ms,
    const core::EvidenceStore& store,
    const std::unordered_map<std::string, core::ClockQualityState>& clock_states,
    const std::unordered_map<std::string, GroundTruthState>& truth) {

    MethodEvaluationOutcome outcome;
    auto result = engine.RequestSnapshot(contract, evaluation_time_ms, store, clock_states);

    if (std::holds_alternative<core::StructuredRejection>(result)) {
        outcome.accepted = false;
        outcome.rejection_reason = "rejected by contract";
        return outcome;
    }

    const auto& snapshot = std::get<core::AcceptedSnapshot>(result);
    outcome.accepted = true;
    outcome.certificate = core::BuildCertificate(snapshot, contract);

    bool all_contained = true;
    for (const auto& agent_id : snapshot.accepted_agents) {
        auto agent_it = snapshot.agent_states.find(agent_id);
        if (agent_it == snapshot.agent_states.end()) continue;

        auto pos_it = agent_it->second.find(static_cast<std::uint8_t>(core::EvidenceFieldId::kPosition));
        if (pos_it == agent_it->second.end()) continue;

        const auto& field_state = pos_it->second;
        const auto est_pos = std::get<std::array<double, 3>>(field_state.evidence.value);
        const double enc_radius = field_state.propagated_uncertainty;

        outcome.estimated_positions[agent_id] = est_pos;
        outcome.position_enclosures[agent_id] = enc_radius;

        auto truth_it = truth.find(agent_id);
        if (truth_it == truth.end()) {
            all_contained = false;
            continue;
        }

        const double physical_error = DistanceMeters(est_pos, truth_it->second.position);
        // Containment check: true physical error must be <= propagated enclosure radius
        // and true vehicle condition must be valid (session, frame, health)
        if (physical_error > enc_radius || !truth_it->second.healthy ||
            field_state.evidence.identity.agent_session_id != truth_it->second.session_id ||
            field_state.evidence.identity.coordinate_frame != truth_it->second.position_frame) {
            all_contained = false;
        }
    }

    outcome.ground_truth_valid = all_contained;
    return outcome;
}

// ---------------------------------------------------------------------------
// StateAcceptanceExperimentRunner
// ---------------------------------------------------------------------------

StateAcceptanceExperimentRunner::StateAcceptanceExperimentRunner(ScenarioConfig config)
    : config_(std::move(config)) {}

ExperimentResults StateAcceptanceExperimentRunner::Run() {
    ExperimentResults results;

    // Initialize metrics
    results.method_metrics[static_cast<uint8_t>(EvaluationMethod::kReceiveLatest)] = {
        .method = EvaluationMethod::kReceiveLatest,
    };
    results.method_metrics[static_cast<uint8_t>(EvaluationMethod::kTimestampAlignedAge)] = {
        .method = EvaluationMethod::kTimestampAlignedAge,
    };
    results.method_metrics[static_cast<uint8_t>(EvaluationMethod::kProposedStateAcceptance)] = {
        .method = EvaluationMethod::kProposedStateAcceptance,
    };

    core::StateAcceptanceEngine engine;
    core::StateAcceptanceVerifier verifier;

    std::mt19937_64 rng(config_.seed);
    std::uniform_real_distribution<double> noise_dist(-0.1, 0.1);

    for (const auto scenario : config_.fault_scenarios) {
        core::EvidenceStore store;
        std::unordered_map<std::string, core::ClockQualityState> clock_states;

        // Base coordinates around 37.7749 N, -122.4194 W
        std::unordered_map<std::string, std::array<double, 3>> base_pos;
        std::unordered_map<std::string, std::string> session_ids;

        for (std::size_t i = 0; i < config_.agent_ids.size(); ++i) {
            const auto& id = config_.agent_ids[i];
            base_pos[id] = {37.7749 + i * 0.001, -122.4194 + i * 0.001, 10.0 + i * 2.0};
            session_ids[id] = "sess-" + id + "-run1";

            // Clock state
            double offset = 0.0;
            double unc = 2.0;
            if (scenario == ScenarioFaultKind::kClockOffsetDrift) {
                offset = 35.0;  // 35 ms offset
                unc = 6.0;
            }

            clock_states[id] = core::ClockQualityState{
                .offset_estimate_ms = offset,
                .uncertainty_radius_ms = unc,
                .source_domain = core::ClockDomain::kUnixEpoch,
                .synchronization = core::ClockSynchronization::kEstimated,
                .last_update_ms = 1000,
                .deterministic_bound = true,
            };
        }

        // Build contract for this scenario
        core::StateQualityContract contract{
            .contract_id = "sqc-exp-" + ScenarioFaultKindToString(scenario),
            .schema_version = 1,
            .content_version = 1,
            .required_fields = {core::EvidenceFieldId::kPosition},
            .max_evidence_age_ms = config_.max_age_ms,
            .max_clock_uncertainty_ms = config_.max_clock_unc_ms,
            .max_position_uncertainty_m = config_.max_pos_unc_m,
            .require_estimator_position_ok = true,
            .require_estimator_healthy = true,
            .required_position_frame = core::CoordinateFrame::kWgs84,
            .require_current_epoch = true,
            .required_agents = {config_.agent_ids.begin(), config_.agent_ids.end()},
            .completeness = core::CompletenessRule::kAllRequired,
            .require_deterministic_bounds = true,
            .max_horizontal_speed_mps = static_cast<float>(config_.max_speed_mps),
        };

        double sim_time_ms = 1000.0;
        std::uint64_t seq = 1;

        for (std::size_t step = 0; step < config_.steps_per_scenario; ++step) {
            sim_time_ms += config_.step_dt_ms;
            const double t_star = sim_time_ms;

            // 1. Generate physical ground truth at t*
            std::unordered_map<std::string, GroundTruthState> truth;
            for (const auto& id : config_.agent_ids) {
                const double t_sec = (sim_time_ms - 1000.0) / 1000.0;
                const double speed_mps = (scenario == ScenarioFaultKind::kHighSpeedMotion) ? 8.0 : 2.0;

                // Physical motion model
                const double lat = base_pos[id][0] + (speed_mps * t_sec) / kMetersPerDegreeLat;
                const double lon = base_pos[id][1] + (speed_mps * std::sin(t_sec)) / kMetersPerDegreeLon;
                const double alt = base_pos[id][2] + (std::cos(t_sec) * 0.5);

                bool is_healthy = true;
                if (scenario == ScenarioFaultKind::kEstimatorDegradation && step > 20) {
                    is_healthy = false;
                }

                truth[id] = GroundTruthState{
                    .drone_id = id,
                    .physical_time_ms = t_star,
                    .position = {lat, lon, alt},
                    .velocity = {static_cast<float>(speed_mps), 0.0F, 0.0F},
                    .position_frame = (scenario == ScenarioFaultKind::kFrameMismatch && step > 20)
                                          ? core::CoordinateFrame::kLocalNed
                                          : core::CoordinateFrame::kWgs84,
                    .healthy = is_healthy,
                    .session_id = (scenario == ScenarioFaultKind::kAgentRestartDelayedPackets && step > 25)
                                      ? "sess-" + id + "-run2"
                                      : session_ids[id],
                };
            }

            // 2. Generate and insert telemetry observations for this step
            for (const auto& id : config_.agent_ids) {
                const double speed_mps = (scenario == ScenarioFaultKind::kHighSpeedMotion) ? 8.0 : 2.0;

                // Source timestamp in source clock domain
                double source_ts = t_star;
                if (scenario == ScenarioFaultKind::kClockOffsetDrift) {
                    source_ts = t_star + 35.0;  // source clock is ahead by 35 ms
                } else if (scenario == ScenarioFaultKind::kNetworkDelay) {
                    source_ts = t_star - 350.0;  // delayed packet was generated 350 ms ago
                }

                // Packet loss
                if (scenario == ScenarioFaultKind::kPacketLoss && (step % 2 == 1)) {
                    continue;  // packet dropped
                }

                core::TelemetryFrame frame;
                frame.drone_id = id;
                frame.agent_session_id = session_ids[id];  // if restart happened, old packets still carry old session
                frame.telemetry_sequence = seq++;
                frame.agent_receive_unix_time_ms = static_cast<std::int64_t>(t_star);

                // Compute observation position based on the time it was generated
                const double gen_t_sec = (source_ts - 1000.0) / 1000.0;
                double obs_lat = base_pos[id][0] + (speed_mps * gen_t_sec) / kMetersPerDegreeLat;
                double obs_lon = base_pos[id][1] + (speed_mps * std::sin(gen_t_sec)) / kMetersPerDegreeLon;
                double obs_alt = base_pos[id][2] + (std::cos(gen_t_sec) * 0.5);

                double obs_unc = 0.2;
                if (scenario == ScenarioFaultKind::kEstimatorDegradation && step > 20) {
                    frame.estimator_state = core::EstimatorState::kDegraded;
                    frame.estimator_position_ok = false;
                    obs_lat += 5.0 / kMetersPerDegreeLat;  // 5m bias
                    obs_unc = 4.5;
                } else {
                    frame.estimator_state = core::EstimatorState::kHealthy;
                    frame.estimator_position_ok = true;
                }

                if (scenario == ScenarioFaultKind::kFrameMismatch && step > 20) {
                    frame.position_frame = core::CoordinateFrame::kLocalNed;
                    obs_lat = 15.0;
                    obs_lon = 20.0;
                } else {
                    frame.position_frame = core::CoordinateFrame::kWgs84;
                }

                frame.lat_deg = obs_lat + noise_dist(rng) * 0.000005;
                frame.lon_deg = obs_lon + noise_dist(rng) * 0.000005;
                frame.rel_alt_m = static_cast<float>(obs_alt);
                frame.validity.position = true;

                frame.provenance.position.source_time.timestamp_ms = static_cast<std::int64_t>(source_ts);
                frame.provenance.position.source_time.clock_domain = core::ClockDomain::kUnixEpoch;
                frame.provenance.position.source_time.synchronization = core::ClockSynchronization::kEstimated;
                frame.provenance.position.source_time.clock_uncertainty_ms = clock_states[id].uncertainty_radius_ms;
                frame.provenance.position.source = "ekf";

                frame.accuracy.horizontal_position = core::UncertaintyEstimate{
                    .value = static_cast<float>(obs_unc),
                    .descriptor = {.semantics = core::UncertaintySemantics::kDeterministicHardBound},
                };
                frame.validity.estimator = true;

                // If agent restarted, new frames with new session also appear in store
                if (scenario == ScenarioFaultKind::kAgentRestartDelayedPackets && step > 25) {
                    // Current session in store is updated
                    core::TelemetryFrame new_sess_frame = frame;
                    new_sess_frame.agent_session_id = "sess-" + id + "-run2";
                    new_sess_frame.validity.position = false;  // only heartbeat or battery
                    new_sess_frame.validity.battery = true;
                    new_sess_frame.battery_percent = 90.0F;
                    store.InsertFrame(new_sess_frame);
                }

                store.InsertFrame(frame);
            }

            // 3. Evaluate all three methods on the IDENTICAL trace at t_star!
            auto& m1 = results.method_metrics[static_cast<uint8_t>(EvaluationMethod::kReceiveLatest)];
            auto& m2 = results.method_metrics[static_cast<uint8_t>(EvaluationMethod::kTimestampAlignedAge)];
            auto& m3 = results.method_metrics[static_cast<uint8_t>(EvaluationMethod::kProposedStateAcceptance)];

            m1.total_requests++;
            m2.total_requests++;
            m3.total_requests++;

            // Baseline 1
            auto out1 = BaselineEvaluator::EvaluateReceiveLatest(contract, t_star, store, truth);
            if (out1.accepted) {
                m1.accepted_count++;
                if (!out1.ground_truth_valid) {
                    m1.invalid_accepted_count++;
                }
            }

            // Baseline 2
            auto out2 = BaselineEvaluator::EvaluateTimestampAlignedAge(contract, t_star, store, config_.max_age_ms, truth);
            if (out2.accepted) {
                m2.accepted_count++;
                if (!out2.ground_truth_valid) {
                    m2.invalid_accepted_count++;
                }
            }

            // Proposed Method (timing measurement)
            const auto start_time = std::chrono::high_resolution_clock::now();
            auto out3 = BaselineEvaluator::EvaluateProposed(engine, contract, t_star, store, clock_states, truth);
            const auto end_time = std::chrono::high_resolution_clock::now();
            const double latency_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            results.latencies_ms.push_back(latency_ms);

            if (out3.accepted) {
                m3.accepted_count++;
                if (!out3.ground_truth_valid) {
                    m3.invalid_accepted_count++;
                }

                // Containment test
                for (const auto& [agent_id, radius] : out3.position_enclosures) {
                    m3.deterministic_enclosures_tested++;
                    results.soundness_metrics.enclosures_tested++;

                    const auto est_pos = out3.estimated_positions[agent_id];
                    const auto true_pos = truth[agent_id].position;
                    const double err = DistanceMeters(est_pos, true_pos);

                    if (err > radius) {
                        m3.containment_failures++;
                        results.soundness_metrics.containment_failures++;
                    }
                }

                // Replay verification test
                if (out3.certificate.has_value()) {
                    results.certificate_sizes_bytes.push_back(
                        out3.certificate->certificate_hash.size() + sizeof(core::StateAcceptanceCertificate));

                    results.soundness_metrics.replayed_decisions++;
                    auto v_result = verifier.Verify(*out3.certificate, store, contract, clock_states);
                    if (std::holds_alternative<core::VerifiedAcceptance>(v_result)) {
                        results.soundness_metrics.verifier_agreements++;
                    }

                    // Tamper test
                    results.soundness_metrics.tampered_certificates_tested++;
                    auto tampered = *out3.certificate;
                    tampered.evaluation_time_ms += 1.0;
                    auto v_tampered = verifier.Verify(tampered, store, contract, clock_states);
                    if (std::holds_alternative<core::VerificationRejection>(v_tampered)) {
                        results.soundness_metrics.tampered_certificates_rejected++;
                    }
                }
            }
        }
    }

    // Compute percentile latency & median certificate size
    if (!results.latencies_ms.empty()) {
        auto sorted_lat = results.latencies_ms;
        std::sort(sorted_lat.begin(), sorted_lat.end());
        const std::size_t p95_idx = static_cast<std::size_t>(0.95 * sorted_lat.size());
        results.soundness_metrics.p95_latency_ms = sorted_lat[std::min(p95_idx, sorted_lat.size() - 1)];
    }

    if (!results.certificate_sizes_bytes.empty()) {
        auto sorted_sizes = results.certificate_sizes_bytes;
        std::sort(sorted_sizes.begin(), sorted_sizes.end());
        results.soundness_metrics.median_certificate_size_bytes = sorted_sizes[sorted_sizes.size() / 2];
    }

    return results;
}

std::string ExperimentResults::FormatTableII() const {
    std::ostringstream oss;
    oss << "| Method | False-valid rate | Snapshot availability | Unsafe acceptance/request |\n";
    oss << "|---|---:|---:|---:|\n";

    const auto& m1 = method_metrics.at(static_cast<uint8_t>(EvaluationMethod::kReceiveLatest));
    const auto& m2 = method_metrics.at(static_cast<uint8_t>(EvaluationMethod::kTimestampAlignedAge));
    const auto& m3 = method_metrics.at(static_cast<uint8_t>(EvaluationMethod::kProposedStateAcceptance));

    oss << std::fixed << std::setprecision(1);
    oss << "| " << EvaluationMethodToString(EvaluationMethod::kReceiveLatest) << " | "
        << (m1.FalseValidRate() * 100.0) << "% | "
        << (m1.Availability() * 100.0) << "% | "
        << (m1.UnsafeAcceptancePerRequest() * 100.0) << "% |\n";

    oss << "| " << EvaluationMethodToString(EvaluationMethod::kTimestampAlignedAge) << " | "
        << (m2.FalseValidRate() * 100.0) << "% | "
        << (m2.Availability() * 100.0) << "% | "
        << (m2.UnsafeAcceptancePerRequest() * 100.0) << "% |\n";

    oss << "| **" << EvaluationMethodToString(EvaluationMethod::kProposedStateAcceptance) << "** | **"
        << (m3.FalseValidRate() * 100.0) << "%** | **"
        << (m3.Availability() * 100.0) << "%** | **"
        << (m3.UnsafeAcceptancePerRequest() * 100.0) << "%** |\n";

    return oss.str();
}

std::string ExperimentResults::FormatTableIII() const {
    std::ostringstream oss;
    oss << "| Property | Result |\n";
    oss << "|---|---:|\n";
    oss << "| Accepted deterministic enclosures tested | " << soundness_metrics.enclosures_tested << " |\n";
    oss << "| Containment failures | " << soundness_metrics.containment_failures << " |\n";
    oss << std::fixed << std::setprecision(1);
    oss << "| Runtime/verifier agreement | " << (soundness_metrics.VerifierAgreementRate() * 100.0) << "% |\n";
    oss << "| Tampered certificates rejected | " << soundness_metrics.tampered_certificates_rejected << "/"
        << soundness_metrics.tampered_certificates_tested << " |\n";
    oss << std::setprecision(2);
    oss << "| p95 snapshot + certificate latency | " << soundness_metrics.p95_latency_ms << " ms |\n";
    oss << "| Median certificate size | " << soundness_metrics.median_certificate_size_bytes << " bytes |\n";

    return oss.str();
}

std::string ExperimentResults::ToJson() const {
    std::ostringstream oss;
    oss << "{\n";
    oss << "  \"table_ii\": {\n";
    for (const auto& [method_id, m] : method_metrics) {
        oss << "    \"" << EvaluationMethodToString(m.method) << "\": {\n";
        oss << "      \"total_requests\": " << m.total_requests << ",\n";
        oss << "      \"accepted_count\": " << m.accepted_count << ",\n";
        oss << "      \"invalid_accepted_count\": " << m.invalid_accepted_count << ",\n";
        oss << "      \"false_valid_rate\": " << m.FalseValidRate() << ",\n";
        oss << "      \"availability\": " << m.Availability() << ",\n";
        oss << "      \"unsafe_acceptance_per_request\": " << m.UnsafeAcceptancePerRequest() << "\n";
        oss << "    }" << (method_id == static_cast<uint8_t>(EvaluationMethod::kProposedStateAcceptance) ? "" : ",") << "\n";
    }
    oss << "  },\n";
    oss << "  \"table_iii\": {\n";
    oss << "    \"enclosures_tested\": " << soundness_metrics.enclosures_tested << ",\n";
    oss << "    \"containment_failures\": " << soundness_metrics.containment_failures << ",\n";
    oss << "    \"verifier_agreement_rate\": " << soundness_metrics.VerifierAgreementRate() << ",\n";
    oss << "    \"tampered_rejected\": " << soundness_metrics.tampered_certificates_rejected << ",\n";
    oss << "    \"tampered_tested\": " << soundness_metrics.tampered_certificates_tested << ",\n";
    oss << "    \"p95_latency_ms\": " << soundness_metrics.p95_latency_ms << ",\n";
    oss << "    \"median_certificate_size_bytes\": " << soundness_metrics.median_certificate_size_bytes << "\n";
    oss << "  }\n";
    oss << "}\n";
    return oss.str();
}

}  // namespace swarmkit::experiment
