// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "swarmkit/agent/sim_backend.h"
#include "swarmkit/core/clock_quality.h"
#include "swarmkit/core/evidence_record.h"
#include "swarmkit/core/evidence_store.h"
#include "swarmkit/core/state_acceptance_certificate.h"
#include "swarmkit/core/state_acceptance_engine.h"
#include "swarmkit/core/state_acceptance_verifier.h"
#include "swarmkit/core/state_quality_contract.h"
#include "swarmkit/core/telemetry.h"
#include "swarmkit/experiment/fault_injection.h"

#include "swarmkit/experiment/bootstrap_statistics.h"

namespace swarmkit::experiment {

/// Scenario fault condition (§21.1).
enum class ScenarioFaultKind : std::uint8_t {
    kNormal,
    kNetworkDelay,
    kNetworkReorder,
    kPacketLoss,
    kClockOffsetDrift,
    kEstimatorDegradation,
    kHighSpeedMotion,
    kAgentRestartDelayedPackets,
    kFrameMismatch,
};

[[nodiscard]] std::string ScenarioFaultKindToString(ScenarioFaultKind kind);

/// Evaluation baseline / method (§21.1).
enum class EvaluationMethod : std::uint8_t {
    /// Baseline 0: Newest received value for each field (B_0).
    kReceiveLatest,
    /// Baseline 1: Timestamp-aligned proximity + age rejection (B_1).
    kTimestampAlignedAge,
    /// Proposed: Full common-time State-Quality Contract acceptance (P).
    kProposedStateAcceptance,
};

[[nodiscard]] std::string EvaluationMethodToString(EvaluationMethod method);

/// Ground-truth physical vehicle state at a given physical reference time.
struct GroundTruthState {
    std::string drone_id;
    double physical_time_ms{};
    std::array<double, 3> position{};   // WGS84 lat, lon, alt
    std::array<float, 3> velocity{};    // NED vx, vy, vz
    core::CoordinateFrame position_frame{core::CoordinateFrame::kWgs84};
    core::CoordinateFrame velocity_frame{core::CoordinateFrame::kLocalNed};
    bool healthy{true};
    std::string session_id;
};

/// Method snapshot evaluation outcome for one evaluation query.
struct MethodEvaluationOutcome {
    bool accepted{false};
    /// Per-agent estimated positions when accepted.
    std::unordered_map<std::string, std::array<double, 3>> estimated_positions;
    /// Per-agent estimated position enclosures (radius in meters) when accepted.
    std::unordered_map<std::string, double> position_enclosures;
    /// Ground-truth validity: whether accepted state satisfied physical truth via common oracle.
    bool ground_truth_valid{false};
    /// Rejection reason string if rejected.
    std::string rejection_reason;
    /// Certificate if produced by proposed method.
    std::optional<core::StateAcceptanceCertificate> certificate;
    /// Execution latency in microseconds.
    double latency_us{0.0};
};

/// Aggregated metrics for one evaluation method (§21.2, §23 Table II, §25 Table III).
struct MethodMetrics {
    EvaluationMethod method{EvaluationMethod::kReceiveLatest};
    std::size_t total_requests{};
    std::size_t true_accepts{};          // Accepted AND Ground-Truth Valid (Safe True Accept)
    std::size_t false_accepts{};         // Accepted AND Ground-Truth Invalid (False Valid / Unsafe)
    std::size_t true_rejects{};          // Rejected AND Ground-Truth Invalid (True Reject / Safety Catch)
    std::size_t false_rejects{};         // Rejected AND Ground-Truth Valid (False Reject / Availability Loss)
    std::size_t deterministic_enclosures_tested{};
    std::size_t containment_failures{};  // Truth position outside spatial enclosure epsilon

    /// Total accepted = true_accepts + false_accepts
    [[nodiscard]] std::size_t AcceptedCount() const noexcept {
        return true_accepts + false_accepts;
    }

    /// Total rejected = true_rejects + false_rejects
    [[nodiscard]] std::size_t RejectedCount() const noexcept {
        return true_rejects + false_rejects;
    }

    /// False-valid rate: FV = N_{accepted and invalid} / N_{accepted}
    [[nodiscard]] double FalseValidRate() const {
        const auto acc = AcceptedCount();
        if (acc == 0) return 0.0;
        return static_cast<double>(false_accepts) / static_cast<double>(acc);
    }

    /// True-reject rate: TR = N_{rejected and invalid} / N_{invalid requests}
    /// Returns std::nullopt when no invalid requests occurred (denominator = 0).
    [[nodiscard]] std::optional<double> TrueRejectRate() const {
        const auto total_invalid = false_accepts + true_rejects;
        if (total_invalid == 0) return std::nullopt;
        return static_cast<double>(true_rejects) / static_cast<double>(total_invalid);
    }

    /// Availability: Availability = N_{accepted} / N_{requests}
    [[nodiscard]] double Availability() const {
        if (total_requests == 0) return 0.0;
        return static_cast<double>(AcceptedCount()) / static_cast<double>(total_requests);
    }

    /// Unsafe acceptance per request: UAR = N_{accepted and invalid} / N_{requests}
    [[nodiscard]] double UnsafeAcceptancePerRequest() const {
        if (total_requests == 0) return 0.0;
        return static_cast<double>(false_accepts) / static_cast<double>(total_requests);
    }

    /// Containment failure rate: CF = N_{truth outside enclosure} / N_{enclosures}
    [[nodiscard]] double ContainmentFailureRate() const {
        if (deterministic_enclosures_tested == 0) return 0.0;
        return static_cast<double>(containment_failures) / static_cast<double>(deterministic_enclosures_tested);
    }
};

/// Soundness and replay metrics for the proposed method (§25 Table III).
struct SoundnessAndReplayMetrics {
    std::size_t enclosures_tested{};
    std::size_t containment_failures{};
    std::size_t replayed_decisions{};
    std::size_t verifier_agreements{};
    std::size_t tampered_certificates_tested{};
    std::size_t tampered_certificates_rejected{};
    double latency_p50_us{0.0};
    double latency_p95_us{0.0};
    double latency_p99_us{0.0};
    std::size_t median_certificate_size_bytes{};
    std::size_t min_certificate_size_bytes{};
    std::size_t max_certificate_size_bytes{};

    [[nodiscard]] double VerifierAgreementRate() const {
        if (replayed_decisions == 0) return 1.0;
        return static_cast<double>(verifier_agreements) / static_cast<double>(replayed_decisions);
    }

    [[nodiscard]] double TamperDetectionRate() const {
        if (tampered_certificates_tested == 0) return 1.0;
        return static_cast<double>(tampered_certificates_rejected) /
               static_cast<double>(tampered_certificates_tested);
    }
};

/// Per-scenario breakdown metrics.
struct ScenarioMetrics {
    ScenarioFaultKind fault_kind{ScenarioFaultKind::kNormal};
    std::size_t requests{};
    std::unordered_map<std::uint8_t, MethodMetrics> metrics_by_method;
};

/// Scalability benchmark metrics for N UAVs.
struct ScalabilityBenchmarkResult {
    std::size_t uav_count{};
    double latency_p50_us{0.0};
    double latency_p95_us{0.0};
    double latency_p99_us{0.0};
    std::size_t serialized_certificate_size_bytes{};
};

/// Complete paired-trace experiment results.
struct ExperimentResults {
    std::size_t total_runs{50};
    std::size_t steps_per_scenario{100};
    std::uint64_t seed_base{42};
    std::vector<std::string> agent_ids;
    std::unordered_map<uint8_t, MethodMetrics> aggregate_method_metrics;
    std::vector<ScenarioMetrics> per_scenario_metrics;
    std::vector<ReplicateRecord> replicate_records;
    BootstrapAnalysisResult bootstrap_results;
    SoundnessAndReplayMetrics soundness_metrics;
    std::vector<ScalabilityBenchmarkResult> scalability_results;
    std::vector<double> latencies_us;
    std::vector<std::size_t> certificate_sizes_bytes;

    [[nodiscard]] std::string FormatTableII() const;
    [[nodiscard]] std::string FormatTableIII() const;
    [[nodiscard]] std::string FormatScalabilityTable() const;
    [[nodiscard]] std::string FormatPerScenarioTable() const;
    [[nodiscard]] std::string ToJson() const;
    [[nodiscard]] std::string ToCsvTableII() const;
};

/// Configuration for paired-trace scenario generation.
struct ScenarioConfig {
    std::uint64_t seed{42};
    std::size_t runs{50};
    std::size_t steps_per_scenario{100};
    std::vector<std::string> agent_ids{"uav-1", "uav-2", "uav-3"};
    std::vector<ScenarioFaultKind> fault_scenarios{
        ScenarioFaultKind::kNormal,
        ScenarioFaultKind::kNetworkDelay,
        ScenarioFaultKind::kNetworkReorder,
        ScenarioFaultKind::kPacketLoss,
        ScenarioFaultKind::kClockOffsetDrift,
        ScenarioFaultKind::kEstimatorDegradation,
        ScenarioFaultKind::kHighSpeedMotion,
        ScenarioFaultKind::kAgentRestartDelayedPackets,
        ScenarioFaultKind::kFrameMismatch,
    };
    double step_dt_ms{100.0};  // 10 Hz
    double max_speed_mps{10.0};
    double max_age_ms{300.0};
    double max_clock_unc_ms{10.0};
    double max_pos_unc_m{3.0};
    double physical_error_tolerance_m{3.0};  // U_max: identical common ground-truth oracle bound
};

/// Evaluator baseline implementations.
class BaselineEvaluator {
   public:
    /// Evaluate Baseline 0: Receive-latest (B_0).
    [[nodiscard]] static MethodEvaluationOutcome EvaluateReceiveLatest(
        const core::StateQualityContract& contract,
        const core::SnapshotRequestContext& request_ctx,
        const core::EvidenceStore& store,
        const std::unordered_map<std::string, GroundTruthState>& truth,
        double u_max_meters);

    /// Evaluate Baseline 1: Timestamp-aligned + age (B_1).
    [[nodiscard]] static MethodEvaluationOutcome EvaluateTimestampAlignedAge(
        const core::StateQualityContract& contract,
        const core::SnapshotRequestContext& request_ctx,
        const core::EvidenceStore& store,
        double max_age_ms,
        const std::unordered_map<std::string, GroundTruthState>& truth,
        double u_max_meters);

    /// Evaluate Proposed (P): StateAcceptanceEngine + StateQualityContract.
    [[nodiscard]] static MethodEvaluationOutcome EvaluateProposed(
        const core::StateAcceptanceEngine& engine,
        const core::StateQualityContract& contract,
        const core::SnapshotRequestContext& request_ctx,
        const core::EvidenceStore& store,
        const std::unordered_map<std::string, core::ClockQualityState>& clock_states,
        const std::unordered_map<std::string, GroundTruthState>& truth,
        double u_max_meters);

    /// Common ground-truth physical validity oracle: PhysicalValid(p_hat, truth, contract, U_max).
    [[nodiscard]] static bool ComputePhysicalValidity(
        const std::unordered_map<std::string, std::array<double, 3>>& estimated_positions,
        const std::unordered_map<std::string, GroundTruthState>& truth,
        const core::StateQualityContract& contract,
        double u_max_meters);
};

/// Paired-Trace Experiment Runner (§21, §24).
class StateAcceptanceExperimentRunner {
   public:
    explicit StateAcceptanceExperimentRunner(ScenarioConfig config = {});

    /// Run full paired-trace evaluation matrix on SimBackend and compute Tables II and III.
    [[nodiscard]] ExperimentResults Run();

    /// Run scalability benchmark across N = 3, 5, 10 UAV swarms.
    [[nodiscard]] std::vector<ScalabilityBenchmarkResult> RunScalabilityBenchmark();

   private:
    ScenarioConfig config_;
};

}  // namespace swarmkit::experiment
