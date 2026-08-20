// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include <sys/utsname.h>

#ifdef __APPLE__
#include <sys/sysctl.h>
#endif

#include "swarmkit/client/swarm_client.h"
#include "swarmkit/commands.h"
#include "swarmkit/core/evidence_store.h"
#include "swarmkit/core/replay_trace.h"
#include "swarmkit/core/state_acceptance_certificate.h"
#include "swarmkit/core/state_acceptance_engine.h"
#include "swarmkit/core/state_acceptance_verifier.h"
#include "swarmkit/core/state_quality_contract.h"

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
using Clock = std::chrono::steady_clock;
using swarmkit::client::CommandResult;
using swarmkit::client::SwarmClient;
using swarmkit::core::AcceptedSnapshot;
using swarmkit::core::AcceptanceResult;
using swarmkit::core::ClockDomain;
using swarmkit::core::ClockQualityState;
using swarmkit::core::ClockSynchronization;
using swarmkit::core::CoordinateFrame;
using swarmkit::core::EvidenceFieldId;
using swarmkit::core::EvidenceRecord;
using swarmkit::core::ReplayTrace;
using swarmkit::core::SnapshotRequestContext;
using swarmkit::core::StateAcceptanceCertificate;
using swarmkit::core::StateAcceptanceEngine;
using swarmkit::core::StateAcceptanceVerifier;
using swarmkit::core::StateQualityContract;
using swarmkit::core::StructuredRejection;

namespace {

constexpr int kTelemetryRateHz = 5;
constexpr int kTrialsPerScenario = 10;
constexpr int kRequestsPerTrial = 20;
constexpr int kRequestPeriodMs = 20;
constexpr int kInjectedDelayMs = 300;
constexpr std::size_t kClockSamples = 5;
constexpr std::size_t kOldEpochFramesToInject = 5;
constexpr std::size_t kSpotcheckCertificates = 10;
constexpr std::array<std::string_view, 3> kDroneIds{"drone-1", "drone-2", "drone-3"};

std::int64_t NowUnixMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

std::string JsonEscape(std::string_view value) {
    std::ostringstream out;
    for (const unsigned char ch : value) {
        switch (ch) {
            case '\\': out << "\\\\"; break;
            case '"': out << "\\\""; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default:
                if (ch < 0x20U) {
                    out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                        << static_cast<int>(ch) << std::dec << std::setfill(' ');
                } else {
                    out << static_cast<char>(ch);
                }
        }
    }
    return out.str();
}

bool WriteFile(const fs::path& path, std::string_view content) {
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
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
#endif
    return "unknown";
}

double Percentile(std::vector<double> values, double quantile) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const double position = quantile * static_cast<double>(values.size() - 1);
    const std::size_t lower = static_cast<std::size_t>(std::floor(position));
    const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
    const double fraction = position - static_cast<double>(lower);
    return values[lower] * (1.0 - fraction) + values[upper] * fraction;
}

struct SummaryStats {
    std::size_t count{};
    double mean{};
    double stddev{};
    double p50{};
    double p95{};
    double p99{};
    double maximum{};
};

SummaryStats Summarize(const std::vector<double>& values) {
    SummaryStats result;
    result.count = values.size();
    if (values.empty()) return result;
    result.mean = std::accumulate(values.begin(), values.end(), 0.0) /
                  static_cast<double>(values.size());
    double squared = 0.0;
    for (const double value : values) squared += (value - result.mean) * (value - result.mean);
    result.stddev = std::sqrt(squared / static_cast<double>(values.size()));
    result.p50 = Percentile(values, 0.50);
    result.p95 = Percentile(values, 0.95);
    result.p99 = Percentile(values, 0.99);
    result.maximum = *std::max_element(values.begin(), values.end());
    return result;
}

std::string RejectionName(swarmkit::core::RejectionReason reason) {
    using R = swarmkit::core::RejectionReason;
    switch (reason) {
        case R::kAgeExceeded: return "age";
        case R::kClockUncertaintyExceeded:
        case R::kMissingClockQuality:
        case R::kMissingSourceTimestamp:
        case R::kInvalidClockBound: return "clock";
        case R::kStateUncertaintyExceeded:
        case R::kMissingUncertainty:
        case R::kUncertaintySemanticsMismatch:
        case R::kUnsupportedUncertaintySemantics:
        case R::kInvalidUncertaintyBound: return "uncertainty";
        case R::kFrameMismatch:
        case R::kUnsupportedPropagationFrame: return "frame";
        case R::kStaleAgentEpoch: return "session";
        case R::kEstimatorUnhealthy:
        case R::kGpsQualityInsufficient: return "health";
        default: return "other";
    }
}

struct RejectionCounts {
    std::size_t age{};
    std::size_t clock{};
    std::size_t uncertainty{};
    std::size_t frame{};
    std::size_t session{};
    std::size_t health{};
    std::size_t other{};

    void Add(std::string_view name) {
        if (name == "age") ++age;
        else if (name == "clock") ++clock;
        else if (name == "uncertainty") ++uncertainty;
        else if (name == "frame") ++frame;
        else if (name == "session") ++session;
        else if (name == "health") ++health;
        else ++other;
    }
};

struct TrialResult {
    std::string trial_id;
    std::string scenario;
    std::size_t requests{};
    std::size_t accepted{};
    RejectionCounts rejections;
    std::size_t certificates{};
    std::size_t frames_received{};
    std::size_t frames_delayed{};
    std::size_t restart_events{};
    std::size_t old_epoch_frames_injected{};
    std::size_t old_epoch_frames_accepted{};
    std::size_t e2_requests_before_clock{};
    std::size_t e2_accepts_before_clock{};
    std::size_t e2_clock_reestablishments{};
    double mean_speed_mps{};
    double max_speed_mps{};
    double mean_age_ms{};
    double max_age_ms{};
    double mean_propagated_uncertainty_m{};
    double max_propagated_uncertainty_m{};
};

struct PropagationSample {
    std::string evidence_key;
    double age_ms{};
    double uncertainty_m{};
};

struct ScenarioResult {
    std::string id;
    std::string name;
    std::vector<TrialResult> trials;
    std::size_t requests{};
    std::size_t accepted{};
    RejectionCounts rejections;
    std::size_t certificates{};
    std::size_t frames_received{};
    std::size_t frames_delayed{};
    std::vector<double> realized_delays_ms;
    std::vector<double> ages_ms;
    std::vector<double> propagated_uncertainties_m;
    std::vector<PropagationSample> propagation_samples;
    std::vector<double> speeds_mps;
    std::vector<double> latencies_us;
    std::size_t restart_events{};
    std::size_t old_epoch_frames_injected{};
    std::size_t old_epoch_frames_accepted{};
    std::size_t e2_requests_before_clock{};
    std::size_t e2_accepts_before_clock{};
    std::size_t e2_clock_reestablishments{};
    std::size_t e2_accepts_after_clock{};
    bool uncertainty_monotonic_with_age{true};
    std::size_t comparable_uncertainty_pairs{};
    std::string trace_path;
};

struct ReplayMetrics {
    std::size_t decisions{};
    std::size_t agreements{};
    std::size_t disagreements{};
    std::vector<bool> decision_agreements;
    std::size_t certificates_selected{};
    std::map<std::string, std::pair<std::size_t, std::size_t>> mutations;
};

struct FlightObservation {
    bool initialized{false};
    double initial_lat{};
    double initial_lon{};
    float initial_alt{};
    double final_lat{};
    double final_lon{};
    float final_alt{};
    double max_speed_mps{};
    bool armed_seen{false};
    bool airborne_seen{false};
    bool final_armed{false};
    bool final_landed{true};
    std::string final_mode;
};

double GreatCircleDistanceM(double lat1, double lon1, double lat2, double lon2) {
    constexpr double radius_m = 6'371'000.0;
    constexpr double radians_per_degree = 3.14159265358979323846 / 180.0;
    const double phi1 = lat1 * radians_per_degree;
    const double phi2 = lat2 * radians_per_degree;
    const double dphi = (lat2 - lat1) * radians_per_degree;
    const double dlambda = (lon2 - lon1) * radians_per_degree;
    const double a = std::sin(dphi / 2.0) * std::sin(dphi / 2.0) +
                     std::cos(phi1) * std::cos(phi2) *
                         std::sin(dlambda / 2.0) * std::sin(dlambda / 2.0);
    return radius_m * 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

std::pair<bool, std::size_t> VerifyUncertaintyMonotonicity(
    const std::vector<PropagationSample>& samples) {
    std::map<std::string, std::vector<std::pair<double, double>>> comparable;
    for (const auto& sample : samples) {
        comparable[sample.evidence_key].emplace_back(sample.age_ms, sample.uncertainty_m);
    }
    bool monotonic = true;
    std::size_t pairs = 0;
    for (auto& [unused_key, observations] : comparable) {
        static_cast<void>(unused_key);
        std::sort(observations.begin(), observations.end());
        for (std::size_t index = 1; index < observations.size(); ++index) {
            const auto& previous = observations[index - 1];
            const auto& current = observations[index];
            if (current.first <= previous.first + 1e-9) continue;
            ++pairs;
            if (current.second + 1e-9 < previous.second) monotonic = false;
        }
    }
    return {monotonic, pairs};
}

struct ClockCalibration {
    std::vector<double> offset_candidates_ms;
};

struct DelayedFrame {
    swarmkit::core::TelemetryFrame frame;
    Clock::time_point arrival;
    Clock::time_point due;
};

class ScenarioRuntime {
   public:
    ScenarioRuntime(std::string scenario_id, std::string scenario_name, int delay_ms,
                    StateQualityContract contract)
        : result{.id = std::move(scenario_id), .name = std::move(scenario_name)},
          contract_(std::move(contract)), delay_ms_(delay_ms) {
        trace.trace_id = "sitl-" + result.id;
        trace.events.push_back(swarmkit::core::MembershipChangeEvent{
            .timestamp_ms = NowUnixMs(),
            .participants = {.agent_ids = {"drone-1", "drone-2", "drone-3"},
                             .membership_revision = 1},
        });
    }

    void OnFrame(const swarmkit::core::TelemetryFrame& frame) {
        std::lock_guard lock(mutex_);
        ++result.frames_received;
        if (result.frames_received <= 18) {
            std::cout << "SITL_FRAME_DIAGNOSTIC scenario=" << result.id
                      << " drone=" << frame.drone_id
                      << " session=" << frame.agent_session_id
                      << " position_updated=" << (frame.provenance.position.updated ? "true" : "false")
                      << " source_time=";
            if (frame.provenance.position.source_time.timestamp_ms.has_value()) {
                std::cout << *frame.provenance.position.source_time.timestamp_ms;
            } else {
                std::cout << "absent";
            }
            std::cout << " source_domain="
                      << static_cast<int>(frame.provenance.position.source_time.clock_domain)
                      << " agent_receive_ms=" << frame.agent_receive_unix_time_ms << '\n';
        }
        if (delay_ms_ > 0) {
            ++result.frames_delayed;
            delayed_.push_back({.frame = frame,
                                .arrival = Clock::now(),
                                .due = Clock::now() + std::chrono::milliseconds(delay_ms_)});
            CalibrateClockLocked(frame);
            return;
        }
        DeliverFrameLocked(frame, Clock::now(), Clock::now());
    }

    void PumpDelayed() {
        std::lock_guard lock(mutex_);
        const auto now = Clock::now();
        while (!delayed_.empty() && delayed_.front().due <= now) {
            auto delayed = std::move(delayed_.front());
            delayed_.pop_front();
            const double realized = std::chrono::duration<double, std::milli>(
                                        now - delayed.arrival)
                                        .count();
            result.realized_delays_ms.push_back(realized);
            DeliverFrameLocked(delayed.frame, delayed.arrival, now);
        }
    }

    bool Ready() const {
        std::lock_guard lock(mutex_);
        for (const auto drone : kDroneIds) {
            const auto session = sessions_.find(std::string(drone));
            const auto clock = clock_states_.find(std::string(drone));
            if (session == sessions_.end() || session->second.empty() ||
                clock == clock_states_.end() || !clock->second.IsValid()) {
                return false;
            }
        }
        return true;
    }

    bool WaitReady(std::chrono::milliseconds timeout) {
        const auto deadline = Clock::now() + timeout;
        while (Clock::now() < deadline) {
            PumpDelayed();
            if (Ready()) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        return false;
    }

    TrialResult RunTrial(int index, bool count_as_preclock = false) {
        TrialResult trial;
        trial.trial_id = result.id + "-trial-" + std::to_string(index);
        trial.scenario = result.name;
        const std::size_t frames_before = result.frames_received;
        const std::size_t delayed_before = result.frames_delayed;
        const std::size_t accepted_before = result.accepted;
        const std::size_t speed_before = result.speeds_mps.size();
        const std::size_t age_before = result.ages_ms.size();

        for (int request_index = 0; request_index < kRequestsPerTrial; ++request_index) {
            PumpDelayed();
            EvaluateRequest(index, request_index, &trial);
            std::this_thread::sleep_for(std::chrono::milliseconds(kRequestPeriodMs));
        }

        trial.requests = kRequestsPerTrial;
        trial.accepted = result.accepted - accepted_before;
        trial.certificates = trial.accepted;
        trial.frames_received = result.frames_received - frames_before;
        trial.frames_delayed = result.frames_delayed - delayed_before;
        if (count_as_preclock) {
            trial.e2_requests_before_clock = trial.requests;
            trial.e2_accepts_before_clock = trial.accepted;
        }
        if (result.speeds_mps.size() > speed_before) {
            const auto begin = result.speeds_mps.begin() + static_cast<std::ptrdiff_t>(speed_before);
            trial.mean_speed_mps = std::accumulate(begin, result.speeds_mps.end(), 0.0) /
                                   static_cast<double>(result.speeds_mps.end() - begin);
            trial.max_speed_mps = *std::max_element(begin, result.speeds_mps.end());
        }
        if (result.ages_ms.size() > age_before) {
            const auto begin = result.ages_ms.begin() + static_cast<std::ptrdiff_t>(age_before);
            trial.mean_age_ms = std::accumulate(begin, result.ages_ms.end(), 0.0) /
                                static_cast<double>(result.ages_ms.end() - begin);
            trial.max_age_ms = *std::max_element(begin, result.ages_ms.end());
            const auto uncertainty_begin =
                result.propagated_uncertainties_m.begin() + static_cast<std::ptrdiff_t>(age_before);
            trial.mean_propagated_uncertainty_m =
                std::accumulate(uncertainty_begin, result.propagated_uncertainties_m.end(), 0.0) /
                static_cast<double>(result.propagated_uncertainties_m.end() - uncertainty_begin);
            trial.max_propagated_uncertainty_m =
                *std::max_element(uncertainty_begin, result.propagated_uncertainties_m.end());
        }
        result.trials.push_back(trial);
        return trial;
    }

    void BlockClockForNextSession(std::string agent_id, std::string old_session) {
        std::lock_guard lock(mutex_);
        blocked_clock_agent_ = std::move(agent_id);
        blocked_old_session_ = std::move(old_session);
        clock_blocked_ = true;
    }

    void EnableBlockedClock() {
        std::lock_guard lock(mutex_);
        clock_blocked_ = false;
        calibrations_.erase(blocked_clock_agent_);
    }

    std::string Session(const std::string& agent_id) const {
        std::lock_guard lock(mutex_);
        const auto iter = sessions_.find(agent_id);
        return iter == sessions_.end() ? std::string{} : iter->second;
    }

    bool HasNewSession(const std::string& agent_id, const std::string& old_session) const {
        const std::string current = Session(agent_id);
        return !current.empty() && current != old_session;
    }

    std::vector<swarmkit::core::TelemetryFrame> OldFrames(const std::string& agent_id) const {
        std::lock_guard lock(mutex_);
        const auto iter = retained_frames_.find(agent_id);
        if (iter == retained_frames_.end()) return {};
        return {iter->second.begin(), iter->second.end()};
    }

    void InjectOldFrames(const std::vector<swarmkit::core::TelemetryFrame>& frames) {
        std::lock_guard lock(mutex_);
        std::size_t injected = 0;
        for (auto iter = frames.rbegin(); iter != frames.rend() && injected < kOldEpochFramesToInject;
             ++iter, ++injected) {
            const std::int64_t delivery_ms = NowUnixMs();
            for (auto record : swarmkit::core::DecomposeToEvidence(*iter)) {
                record.receive_time_ms = delivery_ms;
                store_.Insert(iter->drone_id, record);
                trace.events.push_back(swarmkit::core::EvidenceReceivedEvent{
                    .receive_time_ms = delivery_ms,
                    .agent_id = iter->drone_id,
                    .record = std::move(record),
                });
            }
        }
        result.old_epoch_frames_injected += injected;
    }

    bool ClockValidForSession(const std::string& agent_id, const std::string& session) const {
        std::lock_guard lock(mutex_);
        const auto iter = clock_states_.find(agent_id);
        return iter != clock_states_.end() && iter->second.IsValid() &&
               iter->second.agent_incarnation_id == session;
    }

    const StateQualityContract& Contract() const { return contract_; }
    ReplayTrace trace;
    ScenarioResult result;

   private:
    void DeliverFrameLocked(const swarmkit::core::TelemetryFrame& frame, Clock::time_point arrival,
                            Clock::time_point delivery) {
        static_cast<void>(arrival);
        const auto session_iter = sessions_.find(frame.drone_id);
        if (session_iter == sessions_.end() || session_iter->second != frame.agent_session_id) {
            sessions_[frame.drone_id] = frame.agent_session_id;
            store_.SetCurrentSession(frame.drone_id, frame.agent_session_id);
            clock_states_.erase(frame.drone_id);
            calibrations_.erase(frame.drone_id);
            trace.events.push_back(swarmkit::core::SessionTransitionEvent{
                .timestamp_ms = NowUnixMs(),
                .agent_id = frame.drone_id,
                .new_session_id = frame.agent_session_id,
            });
        }

        CalibrateClockLocked(frame);
        const std::int64_t delivery_ms = NowUnixMs();
        for (auto record : swarmkit::core::DecomposeToEvidence(frame)) {
            record.receive_time_ms = delivery_ms;
            store_.Insert(frame.drone_id, record);
            trace.events.push_back(swarmkit::core::EvidenceReceivedEvent{
                .receive_time_ms = delivery_ms,
                .agent_id = frame.drone_id,
                .record = std::move(record),
            });
        }

        if (frame.provenance.velocity.updated && frame.validity.velocity) {
            result.speeds_mps.push_back(std::hypot(static_cast<double>(frame.vx_mps),
                                                   static_cast<double>(frame.vy_mps)));
        }
        auto& retained = retained_frames_[frame.drone_id];
        retained.push_back(frame);
        while (retained.size() > kOldEpochFramesToInject) retained.pop_front();
        static_cast<void>(delivery);
    }

    void CalibrateClockLocked(const swarmkit::core::TelemetryFrame& frame) {
        if (!frame.provenance.position.updated ||
            !frame.provenance.position.source_time.timestamp_ms.has_value() ||
            frame.agent_receive_unix_time_ms <= 0 || frame.agent_session_id.empty()) {
            return;
        }
        if (clock_states_.contains(frame.drone_id)) return;
        if (clock_blocked_ && frame.drone_id == blocked_clock_agent_ &&
            frame.agent_session_id != blocked_old_session_) {
            return;
        }
        auto& candidates = calibrations_[frame.drone_id].offset_candidates_ms;
        const double source_ms =
            static_cast<double>(*frame.provenance.position.source_time.timestamp_ms);
        candidates.push_back(source_ms - static_cast<double>(frame.agent_receive_unix_time_ms));
        if (candidates.size() < kClockSamples) return;
        const auto [minimum, maximum] = std::minmax_element(candidates.begin(), candidates.end());
        const double theta_hat = (*minimum + *maximum) / 2.0;
        const double rho = std::max(2.0, (*maximum - *minimum) / 2.0 + 1.0);
        ClockQualityState state{
            .offset_estimate_ms = theta_hat,
            .uncertainty_radius_ms = rho,
            .max_drift_rate_ppm = 0.0,
            .source_domain = frame.provenance.position.source_time.clock_domain,
            .synchronization = ClockSynchronization::kEstimated,
            .last_update_ms = frame.agent_receive_unix_time_ms,
            .deterministic_bound = false,
            .agent_incarnation_id = frame.agent_session_id,
            .clock_model_version = "sitl-observed-receive-envelope-v1",
        };
        clock_states_[frame.drone_id] = state;
        trace.events.push_back(swarmkit::core::ClockModelUpdateEvent{
            .timestamp_ms = NowUnixMs(), .agent_id = frame.drone_id, .clock_state = state});
    }

    void EvaluateRequest(int trial_index, int request_index, TrialResult* trial) {
        std::lock_guard lock(mutex_);
        const std::int64_t now_ms = NowUnixMs();
        const SnapshotRequestContext context{
            .evaluation_time_ms = static_cast<double>(now_ms),
            .evidence_freeze_ms = now_ms,
            .participants = {.agent_ids = {"drone-1", "drone-2", "drone-3"},
                             .membership_revision = 1},
        };

        const auto started = Clock::now();
        AcceptanceResult outcome = engine_.RequestSnapshot(contract_, context, store_, clock_states_);
        std::optional<StateAcceptanceCertificate> certificate;
        if (const auto* accepted = std::get_if<AcceptedSnapshot>(&outcome)) {
            certificate = swarmkit::core::BuildCertificate(*accepted, contract_, context);
            const std::string serialized = swarmkit::core::SerializeCertificate(*certificate);
            serialization_sink_ ^= serialized.size();
            ++result.accepted;
            ++result.certificates;
            for (const auto& [agent_id, fields] : accepted->agent_states) {
                static_cast<void>(agent_id);
                const auto position = fields.find(static_cast<std::uint8_t>(EvidenceFieldId::kPosition));
                if (position != fields.end()) {
                    result.ages_ms.push_back(position->second.conservative_elapsed_ms);
                    result.propagated_uncertainties_m.push_back(
                        position->second.propagated_uncertainty);
                    const auto& identity = position->second.evidence.identity;
                    result.propagation_samples.push_back(PropagationSample{
                        .evidence_key = identity.agent_id + "|" +
                                        identity.agent_session_id + "|" +
                                        std::to_string(identity.sequence),
                        .age_ms = position->second.conservative_elapsed_ms,
                        .uncertainty_m = position->second.propagated_uncertainty,
                    });
                    if (position->second.evidence.identity.agent_session_id !=
                        sessions_[position->second.evidence.identity.agent_id]) {
                        ++result.old_epoch_frames_accepted;
                    }
                }
            }
        } else {
            const auto& rejection = std::get<StructuredRejection>(outcome);
            std::map<std::string, bool> seen;
            for (const auto& failure : rejection.failures) {
                seen[RejectionName(failure.reason)] = true;
            }
            if (seen.empty()) seen["other"] = true;
            for (const auto& [name, unused] : seen) {
                static_cast<void>(unused);
                result.rejections.Add(name);
                if (trial != nullptr) trial->rejections.Add(name);
            }
        }
        const double latency_us =
            std::chrono::duration<double, std::micro>(Clock::now() - started).count();
        result.latencies_us.push_back(latency_us);
        ++result.requests;
        trace.events.push_back(swarmkit::core::SnapshotRequestEvent{
            .request_id = result.id + "-trial-" + std::to_string(trial_index) + "-request-" +
                          std::to_string(request_index),
            .evaluation_time_ms = context.evaluation_time_ms,
            .evidence_freeze_ms = context.evidence_freeze_ms,
            .contract_hash = swarmkit::core::ComputeContractHash(contract_),
            .participants = context.participants,
            .certificate = std::move(certificate),
        });
    }

    mutable std::mutex mutex_;
    StateQualityContract contract_;
    int delay_ms_{};
    swarmkit::core::EvidenceStore store_;
    StateAcceptanceEngine engine_;
    std::unordered_map<std::string, std::string> sessions_;
    std::unordered_map<std::string, ClockQualityState> clock_states_;
    std::unordered_map<std::string, ClockCalibration> calibrations_;
    std::unordered_map<std::string, std::deque<swarmkit::core::TelemetryFrame>> retained_frames_;
    std::deque<DelayedFrame> delayed_;
    std::string blocked_clock_agent_;
    std::string blocked_old_session_;
    bool clock_blocked_{false};
    std::size_t serialization_sink_{};
};

StateQualityContract BuildSitlContract() {
    StateQualityContract contract;
    contract.contract_id = "contract-sitl-mavlink-integration-v1";
    contract.schema_version = 1;
    contract.content_version = 1;
    contract.required_agents = {"drone-1", "drone-2", "drone-3"};
    contract.required_fields = {EvidenceFieldId::kPosition, EvidenceFieldId::kVelocity};
    contract.max_evidence_age_ms = 1000.0;
    contract.max_clock_uncertainty_ms = 250.0;
    contract.max_position_uncertainty_m = 12.0;
    contract.require_estimator_healthy = true;
    contract.require_estimator_position_ok = true;
    contract.require_estimator_velocity_ok = true;
    contract.required_position_frame = CoordinateFrame::kWgs84;
    contract.required_velocity_frame = CoordinateFrame::kLocalNed;
    contract.require_current_epoch = true;
    contract.require_current_mission = false;
    contract.completeness = swarmkit::core::CompletenessRule::kAllRequired;
    contract.min_required_agents = 3;
    contract.require_deterministic_bounds = false;
    contract.propagation_model_id = "linear_bounded_vmax";
    contract.propagation_model_version = "1.0";
    contract.max_horizontal_speed_mps = 8.0F;
    contract.max_vertical_speed_mps = 3.0F;
    return contract;
}

swarmkit::commands::CommandEnvelope VelocityCommand(std::string drone_id, float vx, float vy,
                                                     int duration_ms) {
    return {
        .context = {.drone_id = std::move(drone_id),
                    .client_id = "swarmkit-sitl-validation",
                    .priority = swarmkit::commands::CommandPriority::kSupervisor,
                    .correlation_id = "sitl-velocity-" + std::to_string(NowUnixMs())},
        .command = swarmkit::commands::NavCmd{swarmkit::commands::CmdVelocity{
            .vx_mps = vx, .vy_mps = vy, .vz_mps = 0.0F, .duration_ms = duration_ms}},
    };
}

std::vector<std::future<CommandResult>> LaunchMotion(
    SwarmClient* client, const std::array<std::array<float, 2>, 3>& velocities, int duration_ms) {
    std::vector<std::future<CommandResult>> futures;
    futures.reserve(kDroneIds.size());
    for (std::size_t index = 0; index < kDroneIds.size(); ++index) {
        auto command = VelocityCommand(std::string(kDroneIds[index]), velocities[index][0],
                                       velocities[index][1], duration_ms);
        futures.push_back(std::async(std::launch::async,
                                     [client, command = std::move(command)]() mutable {
                                         return client->SendCommand(command);
                                     }));
    }
    return futures;
}

bool WaitMotion(std::vector<std::future<CommandResult>>* futures, std::string_view scenario) {
    bool ok = true;
    if (futures == nullptr) return false;
    for (std::size_t index = 0; index < futures->size(); ++index) {
        const auto result = futures->at(index).get();
        if (!result.ok) {
            std::cerr << scenario << " motion command failed for " << kDroneIds[index] << ": "
                      << result.message << "\n";
            ok = false;
        }
    }
    return ok;
}

void ApplyMutation(std::string_view name, StateAcceptanceCertificate* certificate) {
    if (certificate == nullptr || certificate->evidence_entries.empty()) return;
    auto& entry = certificate->evidence_entries.front();
    if (name == "r_star") ++certificate->evidence_freeze_ms;
    else if (name == "session_incarnation") entry.agent_session_id += "-mutated";
    else if (name == "theta_hat") entry.theta_hat_ms += 0.5;
    else if (name == "clock_model_version") entry.clock_model_version += "-mutated";
    else if (name == "frame") {
        entry.coordinate_frame = entry.coordinate_frame == CoordinateFrame::kWgs84
                                     ? CoordinateFrame::kLocalNed
                                     : CoordinateFrame::kWgs84;
    } else if (name == "participant_set") {
        certificate->participants.agent_ids.push_back("mutated-participant");
    } else if (name == "propagated_uncertainty") {
        entry.propagated_uncertainty += 0.25;
    }
    certificate->certificate_hash = swarmkit::core::ComputeCertificateHash(*certificate);
}

ReplayMetrics ReplayAndSpotcheck(const fs::path& trace_path, const StateQualityContract& contract,
                                 const fs::path& certificate_dir,
                                 std::size_t* remaining_certificates) {
    ReplayMetrics metrics;
    for (const std::string name : {"r_star", "session_incarnation", "theta_hat",
                                   "clock_model_version", "frame", "participant_set",
                                   "propagated_uncertainty"}) {
        metrics.mutations[name] = {0, 0};
    }
    const auto trace = ReplayTrace::LoadFromFile(trace_path.string());
    if (!trace.has_value()) return metrics;

    swarmkit::core::EvidenceStore store;
    std::unordered_map<std::string, ClockQualityState> clocks;
    StateAcceptanceEngine engine;
    StateAcceptanceVerifier verifier;
    for (const auto& event : trace->events) {
        std::visit(
            [&](const auto& value) {
                using T = std::decay_t<decltype(value)>;
                if constexpr (std::is_same_v<T, swarmkit::core::SessionTransitionEvent>) {
                    store.SetCurrentSession(value.agent_id, value.new_session_id);
                    clocks.erase(value.agent_id);
                } else if constexpr (std::is_same_v<T, swarmkit::core::ClockModelUpdateEvent>) {
                    clocks[value.agent_id] = value.clock_state;
                } else if constexpr (std::is_same_v<T, swarmkit::core::EvidenceReceivedEvent>) {
                    store.Insert(value.agent_id, value.record);
                } else if constexpr (std::is_same_v<T, swarmkit::core::SnapshotRequestEvent>) {
                    ++metrics.decisions;
                    const SnapshotRequestContext context{
                        .evaluation_time_ms = value.evaluation_time_ms,
                        .evidence_freeze_ms = value.evidence_freeze_ms,
                        .participants = value.participants,
                    };
                    const AcceptanceResult replayed = engine.RequestSnapshot(contract, context, store, clocks);
                    bool agreement = false;
                    if (value.certificate.has_value()) {
                        const auto verified = verifier.Verify(*value.certificate, store, contract,
                                                              context, clocks);
                        agreement = std::holds_alternative<swarmkit::core::VerifiedAcceptance>(verified) &&
                                    std::holds_alternative<AcceptedSnapshot>(replayed);
                        if (remaining_certificates != nullptr && *remaining_certificates > 0) {
                            const std::size_t certificate_index =
                                kSpotcheckCertificates - *remaining_certificates;
                            --*remaining_certificates;
                            ++metrics.certificates_selected;
                            const fs::path certificate_path =
                                certificate_dir / ("sitl-certificate-" +
                                                   std::to_string(certificate_index) + ".cert");
                            static_cast<void>(WriteFile(
                                certificate_path,
                                swarmkit::core::SerializeCertificate(*value.certificate)));
                            for (auto& [name, counts] : metrics.mutations) {
                                auto mutated = *value.certificate;
                                ApplyMutation(name, &mutated);
                                ++counts.first;
                                const auto mutation_result =
                                    verifier.Verify(mutated, store, contract, context, clocks);
                                if (std::holds_alternative<swarmkit::core::VerificationRejection>(
                                        mutation_result)) {
                                    ++counts.second;
                                }
                            }
                        }
                    } else {
                        agreement = std::holds_alternative<StructuredRejection>(replayed);
                    }
                    metrics.decision_agreements.push_back(agreement);
                    if (agreement) ++metrics.agreements;
                    else ++metrics.disagreements;
                }
            },
            event);
    }
    return metrics;
}

std::string TrialsCsv(const std::vector<ScenarioResult>& scenarios,
                      const std::vector<ReplayMetrics>& replay) {
    std::ostringstream out;
    out << "trial_id,scenario,git_commit,git_dirty,uav_count,agent_ids,sysids,requests,accepted,"
           "rejected,availability,rejection_age,rejection_clock,rejection_uncertainty,"
           "rejection_frame,rejection_session,rejection_health,other_rejections,certificates,"
           "persisted_replays,replay_agreements,replay_disagreements,frames_received,"
           "frames_delayed,frames_dropped,reorder_events,restart_events,old_epoch_frames_injected,"
           "old_epoch_frames_accepted,e2_requests_before_new_clock,e2_accepts_before_new_clock,"
           "e2_clock_reestablishments,mean_speed_mps,max_speed_mps,mean_age_ms,max_age_ms,"
           "mean_propagated_uncertainty_m,max_propagated_uncertainty_m,sitl_truth_available\n";
    for (std::size_t scenario_index = 0; scenario_index < scenarios.size(); ++scenario_index) {
        const auto& scenario = scenarios[scenario_index];
        const ReplayMetrics* scenario_replay =
            scenario_index < replay.size() ? &replay[scenario_index] : nullptr;
        std::size_t replay_offset = 0;
        for (const auto& trial : scenario.trials) {
            const std::size_t rejected = trial.requests - trial.accepted;
            const double availability = trial.requests == 0
                                            ? 0.0
                                            : static_cast<double>(trial.accepted) /
                                                  static_cast<double>(trial.requests);
            const std::size_t persisted_replays =
                scenario_replay == nullptr || replay_offset >= scenario_replay->decision_agreements.size()
                    ? 0
                    : std::min(trial.requests,
                               scenario_replay->decision_agreements.size() - replay_offset);
            const std::size_t replay_agreements =
                scenario_replay == nullptr
                    ? 0
                    : static_cast<std::size_t>(std::count(
                          scenario_replay->decision_agreements.begin() +
                              static_cast<std::ptrdiff_t>(replay_offset),
                          scenario_replay->decision_agreements.begin() +
                              static_cast<std::ptrdiff_t>(replay_offset + persisted_replays),
                          true));
            const std::size_t replay_disagreements = persisted_replays - replay_agreements;
            replay_offset += trial.requests;
            out << trial.trial_id << ',' << trial.scenario << ',' << SWARMKIT_EXPERIMENT_GIT_COMMIT
                << ',' << (SWARMKIT_EXPERIMENT_GIT_DIRTY ? "true" : "false")
                << ",3,\"sitl-agent-1;sitl-agent-2;sitl-agent-3\",\"1;2;3\"," << trial.requests
                << ',' << trial.accepted << ',' << rejected << ',' << std::setprecision(17)
                << availability << ',' << trial.rejections.age << ',' << trial.rejections.clock
                << ',' << trial.rejections.uncertainty << ',' << trial.rejections.frame << ','
                << trial.rejections.session << ',' << trial.rejections.health << ','
                << trial.rejections.other << ',' << trial.certificates
                << ',' << persisted_replays << ',' << replay_agreements << ','
                << replay_disagreements << ',' << trial.frames_received << ',' << trial.frames_delayed
                << ",0,0," << trial.restart_events << ',' << trial.old_epoch_frames_injected
                << ',' << trial.old_epoch_frames_accepted << ',' << trial.e2_requests_before_clock
                << ',' << trial.e2_accepts_before_clock << ',' << trial.e2_clock_reestablishments
                << ',' << trial.mean_speed_mps << ',' << trial.max_speed_mps << ','
                << trial.mean_age_ms << ',' << trial.max_age_ms << ','
                << trial.mean_propagated_uncertainty_m << ','
                << trial.max_propagated_uncertainty_m << ",false\n";
        }
    }
    return out.str();
}

std::string ScenarioSummaryJson(const std::vector<ScenarioResult>& scenarios,
                                const std::vector<ReplayMetrics>& replay) {
    std::ostringstream out;
    out << std::setprecision(17) << "{\n  \"sitl_truth_available\": false,\n  \"scenarios\": [\n";
    for (std::size_t index = 0; index < scenarios.size(); ++index) {
        const auto& scenario = scenarios[index];
        const auto delay = Summarize(scenario.realized_delays_ms);
        const auto age = Summarize(scenario.ages_ms);
        const auto epsilon = Summarize(scenario.propagated_uncertainties_m);
        const auto speed = Summarize(scenario.speeds_mps);
        const double availability = scenario.requests == 0
                                        ? 0.0
                                        : static_cast<double>(scenario.accepted) /
                                              static_cast<double>(scenario.requests);
        out << "    {\"id\": \"" << JsonEscape(scenario.id) << "\", \"name\": \""
            << JsonEscape(scenario.name) << "\", \"trials\": " << scenario.trials.size()
            << ", \"requests\": " << scenario.requests << ", \"accepted\": "
            << scenario.accepted << ", \"rejected\": "
            << scenario.requests - scenario.accepted << ", \"availability\": " << availability
            << ", \"certificates\": " << scenario.certificates
            << ", \"rejections\": {\"age\": " << scenario.rejections.age
            << ", \"clock\": " << scenario.rejections.clock
            << ", \"uncertainty\": " << scenario.rejections.uncertainty
            << ", \"frame\": " << scenario.rejections.frame
            << ", \"session\": " << scenario.rejections.session
            << ", \"health\": " << scenario.rejections.health
            << ", \"other\": " << scenario.rejections.other << "}, \"frames_received\": "
            << scenario.frames_received << ", \"frames_delayed\": " << scenario.frames_delayed
            << ", \"delay_mean_ms\": " << delay.mean << ", \"delay_p95_ms\": " << delay.p95
            << ", \"delay_max_ms\": " << delay.maximum << ", \"speed_mean_mps\": "
            << speed.mean << ", \"speed_max_mps\": " << speed.maximum
            << ", \"age_mean_ms\": " << age.mean << ", \"age_p50_ms\": " << age.p50
            << ", \"age_p95_ms\": " << age.p95 << ", \"age_max_ms\": " << age.maximum
            << ", \"propagated_uncertainty_mean_m\": " << epsilon.mean
            << ", \"propagated_uncertainty_p50_m\": " << epsilon.p50
            << ", \"propagated_uncertainty_p95_m\": " << epsilon.p95
            << ", \"propagated_uncertainty_max_m\": " << epsilon.maximum
            << ", \"uncertainty_monotonic_with_age\": "
            << (scenario.uncertainty_monotonic_with_age ? "true" : "false")
            << ", \"comparable_uncertainty_pairs\": "
            << scenario.comparable_uncertainty_pairs
            << ", \"restart_events\": " << scenario.restart_events
            << ", \"old_epoch_frames_injected\": " << scenario.old_epoch_frames_injected
            << ", \"old_epoch_frames_accepted\": " << scenario.old_epoch_frames_accepted
            << ", \"e2_requests_before_new_clock\": " << scenario.e2_requests_before_clock
            << ", \"e2_accepts_before_new_clock\": " << scenario.e2_accepts_before_clock
            << ", \"e2_clock_reestablishments\": " << scenario.e2_clock_reestablishments
            << ", \"e2_accepts_after_clock\": " << scenario.e2_accepts_after_clock
            << ", \"persisted_replays\": " << replay[index].decisions
            << ", \"replay_agreements\": " << replay[index].agreements
            << ", \"replay_disagreements\": " << replay[index].disagreements << "}";
        out << (index + 1 == scenarios.size() ? "\n" : ",\n");
    }
    out << "  ]\n}\n";
    return out.str();
}

std::string ReplayJson(const std::vector<ReplayMetrics>& replay) {
    std::size_t decisions = 0;
    std::size_t agreements = 0;
    std::size_t disagreements = 0;
    for (const auto& value : replay) {
        decisions += value.decisions;
        agreements += value.agreements;
        disagreements += value.disagreements;
    }
    const double rate = decisions == 0 ? 0.0 : static_cast<double>(agreements) / decisions;
    std::ostringstream out;
    out << std::setprecision(17) << "{\n  \"persisted_sitl_decisions\": " << decisions
        << ",\n  \"agreements\": " << agreements << ",\n  \"disagreements\": "
        << disagreements << ",\n  \"agreement_rate\": " << rate
        << ",\n  \"state_reconstructed_from_disk\": true\n}\n";
    return out.str();
}

std::string SpotcheckJson(const std::vector<ReplayMetrics>& replay) {
    std::size_t selected = 0;
    std::map<std::string, std::pair<std::size_t, std::size_t>> combined;
    for (const auto& value : replay) {
        selected += value.certificates_selected;
        for (const auto& [name, counts] : value.mutations) {
            combined[name].first += counts.first;
            combined[name].second += counts.second;
        }
    }
    std::size_t cases = 0;
    std::size_t rejected = 0;
    std::ostringstream out;
    out << "{\n  \"certificates_selected\": " << selected << ",\n  \"outer_hash_recomputed\": true,\n"
        << "  \"mutation_classes\": [\n";
    std::size_t index = 0;
    for (const auto& [name, counts] : combined) {
        cases += counts.first;
        rejected += counts.second;
        out << "    {\"name\": \"" << JsonEscape(name) << "\", \"cases\": " << counts.first
            << ", \"rejected\": " << counts.second << "}"
            << (++index == combined.size() ? "\n" : ",\n");
    }
    out << "  ],\n  \"cases_tested\": " << cases << ",\n  \"cases_rejected\": " << rejected
        << ",\n  \"all_rejected\": " << (cases > 0 && cases == rejected ? "true" : "false")
        << "\n}\n";
    return out.str();
}

std::string LatencyCsv(const std::vector<ScenarioResult>& scenarios) {
    std::ostringstream out;
    out << "scenario,sample_index,request_certificate_serialize_latency_us\n";
    for (const auto& scenario : scenarios) {
        for (std::size_t index = 0; index < scenario.latencies_us.size(); ++index) {
            out << scenario.name << ',' << index << ',' << std::setprecision(17)
                << scenario.latencies_us[index] << '\n';
        }
    }
    return out.str();
}

std::string LatencySummaryJson(const std::vector<ScenarioResult>& scenarios) {
    std::vector<double> all;
    for (const auto& scenario : scenarios) {
        all.insert(all.end(), scenario.latencies_us.begin(), scenario.latencies_us.end());
    }
    const auto stats = Summarize(all);
    std::ostringstream out;
    out << std::setprecision(17)
        << "{\n  \"label\": \"SITL integration latency on this host\",\n"
        << "  \"scope\": \"RequestSnapshot + BuildCertificate + SerializeCertificate\",\n"
        << "  \"count\": " << stats.count << ",\n  \"mean_us\": " << stats.mean
        << ",\n  \"stddev_us\": " << stats.stddev << ",\n  \"p50_us\": " << stats.p50
        << ",\n  \"p95_us\": " << stats.p95 << ",\n  \"p99_us\": " << stats.p99
        << ",\n  \"max_us\": " << stats.maximum << "\n}\n";
    return out.str();
}

std::string FaultJson(const std::vector<ScenarioResult>& scenarios) {
    std::ostringstream out;
    out << std::setprecision(17) << "{\n  \"configured_delay_ms\": " << kInjectedDelayMs
        << ",\n  \"scenarios\": [\n";
    for (std::size_t index = 0; index < scenarios.size(); ++index) {
        const auto& scenario = scenarios[index];
        const auto delay = Summarize(scenario.realized_delays_ms);
        out << "    {\"scenario\": \"" << JsonEscape(scenario.name)
            << "\", \"frames_received\": " << scenario.frames_received
            << ", \"frames_delayed\": " << scenario.frames_delayed
            << ", \"actual_delay_mean_ms\": " << delay.mean
            << ", \"actual_delay_p95_ms\": " << delay.p95
            << ", \"actual_delay_max_ms\": " << delay.maximum << "}"
            << (index + 1 == scenarios.size() ? "\n" : ",\n");
    }
    out << "  ],\n  \"delay_location\": \"after SDK delivery of production-normalized TelemetryFrame and before DecomposeToEvidence/EvidenceStore delivery\",\n"
        << "  \"source_timestamps_modified\": false\n}\n";
    return out.str();
}

std::string ContractJson(const StateQualityContract& contract) {
    std::ostringstream out;
    out << std::setprecision(17)
        << "{\n  \"contract_id\": \"" << contract.contract_id << "\",\n"
        << "  \"canonical_hash\": \"" << swarmkit::core::ComputeContractHash(contract)
        << "\",\n  \"required_agents\": [\"drone-1\", \"drone-2\", \"drone-3\"],\n"
        << "  \"required_fields\": [\"position\", \"velocity\"],\n"
        << "  \"max_age_ms\": " << *contract.max_evidence_age_ms
        << ",\n  \"max_clock_uncertainty_ms\": " << *contract.max_clock_uncertainty_ms
        << ",\n  \"max_propagated_position_uncertainty_m\": "
        << *contract.max_position_uncertainty_m
        << ",\n  \"position_frame\": \"WGS84\",\n  \"velocity_frame\": \"LocalNED\",\n"
        << "  \"require_estimator_healthy\": true,\n  \"require_estimator_position_ok\": true,\n"
        << "  \"require_estimator_velocity_ok\": true,\n  \"require_current_incarnation\": true,\n"
        << "  \"require_current_mission\": false,\n  \"require_deterministic_bounds\": false,\n"
        << "  \"clock_profile\": \"estimated/non-deterministic observed receive envelope\",\n"
        << "  \"uncertainty_profile\": \"MAVLink backend-specific h_acc; no deterministic hard-bound claim\",\n"
        << "  \"gps_predicate_enabled\": false,\n"
        << "  \"gps_predicate_disabled_reason\": \"GPS-quality evidence has no independent causal source timestamp in the current MAVLink decomposition\",\n"
        << "  \"mission_predicate_disabled_reason\": \"velocity control telemetry is not bound to an active mission/goal\",\n"
        << "  \"propagation_model_id\": \"" << contract.propagation_model_id
        << "\",\n  \"propagation_model_version\": \"" << contract.propagation_model_version
        << "\",\n  \"vmax_horizontal_mps\": " << contract.max_horizontal_speed_mps
        << ",\n  \"vmax_vertical_mps\": " << contract.max_vertical_speed_mps
        << ",\n  \"membership_revision\": 1\n}\n";
    return out.str();
}

std::string FlightJson(const std::map<std::string, FlightObservation>& observations) {
    std::ostringstream out;
    out << std::setprecision(17) << "{\n  \"vehicles\": [\n";
    std::size_t index = 0;
    for (const auto& [drone, value] : observations) {
        const double distance = GreatCircleDistanceM(value.initial_lat, value.initial_lon,
                                                     value.final_lat, value.final_lon);
        out << "    {\"drone_id\": \"" << drone << "\", \"initial_lat_deg\": "
            << value.initial_lat << ", \"initial_lon_deg\": " << value.initial_lon
            << ", \"initial_alt_m\": " << value.initial_alt << ", \"final_lat_deg\": "
            << value.final_lat << ", \"final_lon_deg\": " << value.final_lon
            << ", \"final_alt_m\": " << value.final_alt << ", \"distance_moved_m\": "
            << distance << ", \"max_observed_horizontal_speed_mps\": " << value.max_speed_mps
            << ", \"armed_seen\": " << (value.armed_seen ? "true" : "false")
            << ", \"takeoff_success\": " << (value.airborne_seen ? "true" : "false")
            << ", \"final_armed\": " << (value.final_armed ? "true" : "false")
            << ", \"final_landed\": " << (value.final_landed ? "true" : "false")
            << ", \"final_mode\": \"" << JsonEscape(value.final_mode) << "\"}"
            << (++index == observations.size() ? "\n" : ",\n");
    }
    out << "  ]\n}\n";
    return out.str();
}

std::string ManifestJson(const StateQualityContract& contract) {
    struct utsname system {};
    const bool uname_ok = uname(&system) == 0;
    std::ostringstream out;
    out << "{\n  \"git_commit\": \"" << SWARMKIT_EXPERIMENT_GIT_COMMIT << "\",\n"
        << "  \"git_dirty_state\": " << (SWARMKIT_EXPERIMENT_GIT_DIRTY ? "true" : "false")
        << ",\n  \"campaign_timestamp_utc\": \"" << Iso8601UtcNow() << "\",\n"
        << "  \"os\": \"" << (uname_ok ? JsonEscape(system.sysname) : "unknown") << ' '
        << (uname_ok ? JsonEscape(system.release) : "unknown") << "\",\n"
        << "  \"architecture\": \"" << (uname_ok ? JsonEscape(system.machine) : "unknown")
        << "\",\n  \"cpu\": \"" << JsonEscape(CpuName()) << "\",\n"
        << "  \"compiler\": \"" << SWARMKIT_EXPERIMENT_COMPILER_ID << "\",\n"
        << "  \"compiler_version\": \"" << SWARMKIT_EXPERIMENT_COMPILER_VERSION << "\",\n"
        << "  \"build_type\": \"" << SWARMKIT_EXPERIMENT_BUILD_TYPE << "\",\n"
        << "  \"ardupilot_version\": \"AION-Orca-Alpha2-26682-g958493b474\",\n"
        << "  \"ardupilot_git_commit\": \"958493b47428a8f5e4ee5ed3f23d5ac193c24299\",\n"
        << "  \"vehicle_type\": \"ArduCopter SITL\",\n"
        << "  \"sitl_launch_command\": \"~/swarm_sitl_direct.sh start 3 192.168.123.36\",\n"
        << "  \"mavlink_router_version\": \"v4-16-g2362c62\",\n"
        << "  \"mavlink_router_config\": {\"input_ports\": [24561, 24562, 24563], \"qgc_output\": \"192.168.123.36:14550\", \"agent_outputs\": [\"192.168.123.36:14601\", \"192.168.123.36:14602\", \"192.168.123.36:14603\"]},\n"
        << "  \"qgc_connected\": true,\n  \"qgc_endpoint\": \"udp://192.168.123.36:14550\",\n"
        << "  \"sysids\": [1, 2, 3],\n  \"sitl_instances\": [0, 1, 2],\n"
        << "  \"router_input_ports\": [24561, 24562, 24563],\n"
        << "  \"agent_udp_ports\": [14601, 14602, 14603],\n"
        << "  \"agent_grpc_ports\": [50061, 50062, 50063],\n"
        << "  \"agent_ids\": [\"sitl-agent-1\", \"sitl-agent-2\", \"sitl-agent-3\"],\n"
        << "  \"swarmkit_agent_launch_commands\": [\n"
        << "    \"./build/mac-release/apps/swarmkit-agent --config testdata/agent_mavlink_sitl_validation.yaml --id sitl-agent-1 --bind 127.0.0.1:50061 --mavlink-drone drone-1 --mavlink-bind 0.0.0.0:14601 --mavlink-target-system 1 --evidence-file /tmp/swarmkit-sitl-final/agent1-final.evidence --evidence-run-id sitl-final-agent-1-20260820 --evidence-scenario-id sitl-production-path --evidence-seed 2026082001 --mavlink-set-guided-before-arm false --mavlink-set-guided-before-takeoff false --log-level debug\",\n"
        << "    \"./build/mac-release/apps/swarmkit-agent --config testdata/agent_mavlink_sitl_validation.yaml --id sitl-agent-2 --bind 127.0.0.1:50062 --mavlink-drone drone-2 --mavlink-bind 0.0.0.0:14602 --mavlink-target-system 2 --evidence-file /tmp/swarmkit-sitl-final/agent2-final.evidence --evidence-run-id sitl-final-agent-2-20260820 --evidence-scenario-id sitl-production-path --evidence-seed 2026082002 --mavlink-set-guided-before-arm false --mavlink-set-guided-before-takeoff false --log-level debug\",\n"
        << "    \"./build/mac-release/apps/swarmkit-agent --config testdata/agent_mavlink_sitl_validation.yaml --id sitl-agent-2 --bind 127.0.0.1:50062 --mavlink-drone drone-2 --mavlink-bind 0.0.0.0:14602 --mavlink-target-system 2 --evidence-file /tmp/swarmkit-sitl-final/agent2-e2b-final.evidence --evidence-run-id sitl-final-agent-2-e2b-20260821 --evidence-scenario-id sitl-production-path --evidence-seed 2026082102 --mavlink-set-guided-before-arm false --mavlink-set-guided-before-takeoff false --log-level debug\",\n"
        << "    \"./build/mac-release/apps/swarmkit-agent --config testdata/agent_mavlink_sitl_validation.yaml --id sitl-agent-3 --bind 127.0.0.1:50063 --mavlink-drone drone-3 --mavlink-bind 0.0.0.0:14603 --mavlink-target-system 3 --evidence-file /tmp/swarmkit-sitl-final/agent3-final.evidence --evidence-run-id sitl-final-agent-3-20260820 --evidence-scenario-id sitl-production-path --evidence-seed 2026082003 --mavlink-set-guided-before-arm false --mavlink-set-guided-before-takeoff false --log-level debug\"\n"
        << "  ],\n"
        << "  \"contract_canonical_hash\": \"" << swarmkit::core::ComputeContractHash(contract)
        << "\",\n  \"acceptance_semantics_version\": \""
        << swarmkit::core::kAcceptanceSemanticsVersion << "\",\n"
        << "  \"certificate_schema_version\": \"" << swarmkit::core::kCertificateSchemaVersion
        << "\",\n  \"propagation_model_id\": \"" << contract.propagation_model_id
        << "\",\n  \"propagation_model_version\": \"" << contract.propagation_model_version
        << "\",\n  \"vmax_horizontal_mps\": " << contract.max_horizontal_speed_mps
        << ",\n  \"vmax_vertical_mps\": " << contract.max_vertical_speed_mps
        << ",\n  \"max_age_ms\": " << *contract.max_evidence_age_ms
        << ",\n  \"max_clock_uncertainty_ms\": " << *contract.max_clock_uncertainty_ms
        << ",\n  \"max_position_uncertainty_m\": " << *contract.max_position_uncertainty_m
        << ",\n  \"scenario_definitions\": [\n"
        << "    {\"id\": \"s0\", \"name\": \"normal_moving_flight\", \"delay_ms\": 0, \"velocity_ned_mps\": [[3.0, 0.0], [2.5, 2.0], [2.0, -2.5]], \"duration_ms\": 3000},\n"
        << "    {\"id\": \"s1\", \"name\": \"delayed_telemetry\", \"delay_ms\": 300, \"velocity_ned_mps\": [[3.0, 0.5], [2.5, 2.0], [2.5, -2.0]], \"duration_ms\": 3000},\n"
        << "    {\"id\": \"s2\", \"name\": \"agent_restart_delayed_old_epoch\", \"restart_agent\": \"drone-2\", \"old_epoch_frames_to_inject\": 5},\n"
        << "    {\"id\": \"s3\", \"name\": \"high_speed_delayed_motion\", \"delay_ms\": 300, \"velocity_ned_mps\": [[6.0, 0.0], [4.0, 5.5], [5.0, -5.5]], \"duration_ms\": 3000}\n"
        << "  ]"
        << ",\n  \"trial_count_per_scenario\": " << kTrialsPerScenario
        << ",\n  \"requests_per_trial\": " << kRequestsPerTrial
        << ",\n  \"truth_channel_used\": false,\n"
        << "  \"truth_details\": \"No independent simulator-truth channel was established; evaluated MAVLink estimator telemetry is not reused as truth.\"\n}\n";
    return out.str();
}

class Campaign {
   public:
    explicit Campaign(fs::path output_dir) : output_dir_(std::move(output_dir)) {}

    int Run() {
        fs::create_directories(output_dir_ / "traces");
        fs::create_directories(output_dir_ / "certificates");
        contract_ = BuildSitlContract();
        const auto valid = swarmkit::core::ValidateStateQualityContract(contract_);
        if (!valid.IsOk()) {
            std::cerr << "SITL contract invalid: " << valid.message << '\n';
            return 1;
        }

        swarmkit::client::ClientConfig config;
        config.client_id = "swarmkit-sitl-validation";
        config.deadline_ms = 5000;
        config.priority = swarmkit::commands::CommandPriority::kSupervisor;
        config.security.transport_security = swarmkit::core::TransportSecurityMode::kInsecure;
        client_ = std::make_unique<SwarmClient>(config);
        client_->AddDrone("drone-1", "127.0.0.1:50061");
        client_->AddDrone("drone-2", "127.0.0.1:50062");
        client_->AddDrone("drone-3", "127.0.0.1:50063");

        campaign_start_ms_ = NowUnixMs();
        auto subscriptions = client_->StartAllTelemetry(
            kTelemetryRateHz,
            [this](const swarmkit::client::TelemetryObservation& observation) {
                OnObservation(observation);
            },
            [this](const std::string& error) {
                std::lock_guard lock(error_mutex_);
                stream_errors_.push_back(error);
            });
        for (const auto& [drone, result] : subscriptions) {
            if (!result.has_value()) {
                std::cerr << "failed to start telemetry for " << drone << ": "
                          << result.error().user_message << '\n';
                return 1;
            }
        }

        bool campaign_ok = true;
        campaign_ok &= RunMovingScenario("s0", "normal_moving_flight", 0,
                                         {{{3.0F, 0.0F}, {2.5F, 2.0F}, {2.0F, -2.5F}}}, 3000);
        campaign_ok &= RunMovingScenario("s1", "delayed_telemetry", kInjectedDelayMs,
                                         {{{3.0F, 0.5F}, {2.5F, 2.0F}, {2.5F, -2.0F}}}, 3000);
        campaign_ok &= RunRestartScenario();
        campaign_ok &= RunMovingScenario("s3", "high_speed_delayed_motion", kInjectedDelayMs,
                                         {{{6.0F, 0.0F}, {4.0F, 5.5F}, {5.0F, -5.5F}}}, 3000);

        for (auto& [drone, subscription] : subscriptions) {
            static_cast<void>(drone);
            if (subscription.has_value()) subscription->Stop();
        }
        client_->StopAllTelemetry();

        if (!WriteOutputs()) campaign_ok = false;
        if (!stream_errors_.empty()) {
            std::cerr << "telemetry stream errors observed: " << stream_errors_.size() << '\n';
            campaign_ok = false;
        }
        const std::size_t replay_disagreements =
            std::accumulate(replay_.begin(), replay_.end(), std::size_t{0},
                            [](std::size_t value, const ReplayMetrics& item) {
                                return value + item.disagreements;
                            });
        if (replay_disagreements != 0) campaign_ok = false;
        if (scenarios_.size() != 4) campaign_ok = false;
        std::cout << "SITL_CAMPAIGN_COMPLETE status=" << (campaign_ok ? "PASS" : "FAIL")
                  << " output_dir=" << output_dir_.string() << '\n';
        return campaign_ok ? 0 : 1;
    }

   private:
    void OnObservation(const swarmkit::client::TelemetryObservation& observation) {
        const auto* frame_observation =
            std::get_if<swarmkit::client::TelemetryFrameObservation>(&observation);
        if (frame_observation == nullptr) return;
        const auto& frame = frame_observation->delivery.frame;
        if (frame.agent_receive_unix_time_ms < campaign_start_ms_ - 1000) return;
        {
            std::lock_guard lock(flight_mutex_);
            auto& flight = flight_[frame.drone_id];
            if (frame.validity.position) {
                if (!flight.initialized) {
                    flight.initialized = true;
                    flight.initial_lat = frame.lat_deg;
                    flight.initial_lon = frame.lon_deg;
                    flight.initial_alt = frame.rel_alt_m;
                }
                flight.final_lat = frame.lat_deg;
                flight.final_lon = frame.lon_deg;
                flight.final_alt = frame.rel_alt_m;
            }
            if (frame.validity.velocity && frame.provenance.velocity.updated) {
                flight.max_speed_mps = std::max(
                    flight.max_speed_mps,
                    std::hypot(static_cast<double>(frame.vx_mps),
                               static_cast<double>(frame.vy_mps)));
            }
            if (frame.validity.armed) {
                flight.armed_seen = flight.armed_seen || frame.armed;
                flight.final_armed = frame.armed;
            }
            if (frame.validity.landed) {
                flight.airborne_seen = flight.airborne_seen || !frame.landed;
                flight.final_landed = frame.landed;
            }
            if (frame.validity.mode) flight.final_mode = frame.mode;
        }
        std::lock_guard lock(active_mutex_);
        if (active_ != nullptr) active_->OnFrame(frame);
    }

    void SetActive(ScenarioRuntime* runtime) {
        std::lock_guard lock(active_mutex_);
        active_ = runtime;
    }

    bool FinalizeScenario(std::unique_ptr<ScenarioRuntime> runtime) {
        SetActive(nullptr);
        runtime->PumpDelayed();
        const auto [monotonic, comparable_pairs] =
            VerifyUncertaintyMonotonicity(runtime->result.propagation_samples);
        runtime->result.uncertainty_monotonic_with_age = monotonic;
        runtime->result.comparable_uncertainty_pairs = comparable_pairs;
        const fs::path trace_path = output_dir_ / "traces" / (runtime->result.id + ".jsonl");
        runtime->result.trace_path = trace_path.string();
        if (!runtime->trace.SaveToFile(trace_path.string())) {
            std::cerr << "failed to save trace " << trace_path << '\n';
            return false;
        }
        ScenarioResult result = std::move(runtime->result);
        runtime.reset();
        const ReplayMetrics replay = ReplayAndSpotcheck(
            trace_path, contract_, output_dir_ / "certificates", &remaining_spotcheck_);
        scenarios_.push_back(std::move(result));
        replay_.push_back(replay);
        return replay.disagreements == 0;
    }

    bool RunMovingScenario(std::string id, std::string name, int delay_ms,
                           const std::array<std::array<float, 2>, 3>& velocities,
                           int duration_ms) {
        std::cout << "SITL_SCENARIO_START id=" << id << " name=" << name << '\n';
        auto runtime =
            std::make_unique<ScenarioRuntime>(std::move(id), std::move(name), delay_ms, contract_);
        SetActive(runtime.get());
        if (!runtime->WaitReady(std::chrono::seconds(20))) {
            std::cerr << "scenario clock/evidence warmup failed\n";
            SetActive(nullptr);
            return false;
        }
        auto motion = LaunchMotion(client_.get(), velocities, duration_ms);
        for (int trial = 0; trial < kTrialsPerScenario; ++trial) {
            runtime->RunTrial(trial);
        }
        const bool motion_ok = WaitMotion(&motion, runtime->result.name);
        const bool finalize_ok = FinalizeScenario(std::move(runtime));
        return motion_ok && finalize_ok;
    }

    bool RunRestartScenario() {
        std::cout << "SITL_SCENARIO_START id=s2 name=agent_restart_delayed_old_epoch\n";
        auto runtime = std::make_unique<ScenarioRuntime>(
            "s2", "agent_restart_delayed_old_epoch", 0, contract_);
        SetActive(runtime.get());
        if (!runtime->WaitReady(std::chrono::seconds(20))) {
            std::cerr << "restart scenario warmup failed\n";
            SetActive(nullptr);
            return false;
        }
        const std::string old_session = runtime->Session("drone-2");
        const auto old_frames = runtime->OldFrames("drone-2");
        if (old_session.empty() || old_frames.empty()) {
            std::cerr << "restart scenario lacks genuine E1 frames\n";
            SetActive(nullptr);
            return false;
        }
        runtime->BlockClockForNextSession("drone-2", old_session);
        std::cout << "SITL_RESTART_REQUIRED agent=drone-2 old_session=" << old_session << '\n'
                  << std::flush;
        const auto deadline = Clock::now() + std::chrono::seconds(120);
        while (Clock::now() < deadline && !runtime->HasNewSession("drone-2", old_session)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (!runtime->HasNewSession("drone-2", old_session)) {
            std::cerr << "timed out waiting for drone-2 agent restart\n";
            SetActive(nullptr);
            return false;
        }
        ++runtime->result.restart_events;
        runtime->InjectOldFrames(old_frames);
        auto preclock = runtime->RunTrial(0, true);
        runtime->result.e2_requests_before_clock = preclock.requests;
        runtime->result.e2_accepts_before_clock = preclock.accepted;
        runtime->result.trials.back().restart_events = 1;
        runtime->result.trials.back().old_epoch_frames_injected =
            runtime->result.old_epoch_frames_injected;
        runtime->result.trials.back().old_epoch_frames_accepted =
            runtime->result.old_epoch_frames_accepted;
        runtime->EnableBlockedClock();
        const std::string new_session = runtime->Session("drone-2");
        const auto clock_deadline = Clock::now() + std::chrono::seconds(20);
        while (Clock::now() < clock_deadline &&
               !runtime->ClockValidForSession("drone-2", new_session)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        if (!runtime->ClockValidForSession("drone-2", new_session)) {
            std::cerr << "E2 clock re-establishment failed\n";
            SetActive(nullptr);
            return false;
        }
        runtime->result.e2_clock_reestablishments = 1;
        for (int trial = 1; trial < kTrialsPerScenario; ++trial) runtime->RunTrial(trial);
        runtime->result.e2_accepts_after_clock =
            runtime->result.accepted - runtime->result.e2_accepts_before_clock;
        if (runtime->result.old_epoch_frames_accepted != 0 ||
            runtime->result.e2_accepts_before_clock != 0 ||
            runtime->result.e2_accepts_after_clock == 0) {
            std::cerr << "restart semantic invariant failed\n";
            const bool finalized = FinalizeScenario(std::move(runtime));
            static_cast<void>(finalized);
            return false;
        }
        return FinalizeScenario(std::move(runtime));
    }

    bool WriteOutputs() {
        bool ok = true;
        ok &= WriteFile(output_dir_ / "sitl_manifest.json", ManifestJson(contract_));
        ok &= WriteFile(output_dir_ / "sitl_contract.json", ContractJson(contract_));
        ok &= WriteFile(output_dir_ / "sitl_trials.csv", TrialsCsv(scenarios_, replay_));
        ok &= WriteFile(output_dir_ / "sitl_scenario_summary.json",
                        ScenarioSummaryJson(scenarios_, replay_));
        ok &= WriteFile(output_dir_ / "sitl_replay_results.json", ReplayJson(replay_));
        ok &= WriteFile(output_dir_ / "sitl_certificate_spotcheck.json", SpotcheckJson(replay_));
        ok &= WriteFile(output_dir_ / "sitl_latency_samples.csv", LatencyCsv(scenarios_));
        ok &= WriteFile(output_dir_ / "sitl_latency_summary.json", LatencySummaryJson(scenarios_));
        ok &= WriteFile(output_dir_ / "sitl_fault_realization.json", FaultJson(scenarios_));
        {
            std::lock_guard lock(flight_mutex_);
            ok &= WriteFile(output_dir_ / "sitl_flight_diagnostics.json", FlightJson(flight_));
        }
        return ok;
    }

    fs::path output_dir_;
    StateQualityContract contract_;
    std::unique_ptr<SwarmClient> client_;
    std::mutex active_mutex_;
    ScenarioRuntime* active_{nullptr};
    std::int64_t campaign_start_ms_{};
    std::mutex error_mutex_;
    std::vector<std::string> stream_errors_;
    std::mutex flight_mutex_;
    std::map<std::string, FlightObservation> flight_;
    std::vector<ScenarioResult> scenarios_;
    std::vector<ReplayMetrics> replay_;
    std::size_t remaining_spotcheck_{kSpotcheckCertificates};
};

}  // namespace

int main(int argc, char** argv) {
    fs::path output_dir = "results/dissertation/sitl";
    for (int index = 1; index < argc; ++index) {
        const std::string_view arg = argv[index];
        if (arg == "--help") {
            std::cout << "Usage: swarmkit-sitl-validation [--output-dir PATH]\n";
            return 0;
        }
        if (arg == "--output-dir" && index + 1 < argc) {
            output_dir = argv[++index];
        }
    }
    Campaign campaign(std::move(output_dir));
    return campaign.Run();
}
