// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
// Deterministic, intentionally simple strip-survey experiment. Truth is private
// to Trial and is never passed to EdgeExecutionSupervisor.
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "swarmkit/edge/vecr.h"

namespace {
using swarmkit::edge::CaptureEvidence;
using swarmkit::edge::EdgeExecutionSupervisor;
using swarmkit::edge::EdgeRequest;
using swarmkit::edge::Interval;
using swarmkit::edge::IntervalSet;
using swarmkit::edge::Obligation;
using swarmkit::edge::Phase;
using swarmkit::edge::Point;
using swarmkit::edge::PositionObservation;

constexpr int kCells = 400;
constexpr double kLengthM = 100.0;
constexpr double kCellM = kLengthM / kCells;

struct Scenario {
    const char* id;
    bool jitter{};
    bool detour{};
    bool camera_outage{};
    bool metadata_loss{};
    bool off_center{};
    bool sparse{};
    bool false_capture{};
};

constexpr std::array<Scenario, 8> kScenarios{{
    {"S00"},
    {"S01", true},
    {"S03", false, true},
    {"S04", false, false, true},
    {"S05", false, false, false, true},
    {"S06", false, false, false, false, true},
    {"S07", false, false, false, false, false, true},
    {"S11", false, false, false, false, false, false, true},
}};

struct Truth {
    std::array<bool, kCells> covered{};
    double distance_m{};

    void Capture(double x_m, double y_m, bool camera_working, double half_width_m) {
        if (!camera_working || std::abs(y_m) + 4.0 > half_width_m) return;
        for (int cell = 0; cell < kCells; ++cell) {
            const double center = (static_cast<double>(cell) + 0.5) * kCellM;
            if (center >= x_m - 2.0 && center <= x_m + 2.0) covered[cell] = true;
        }
    }
    [[nodiscard]] double MissingFraction() const {
        int missing = 0;
        for (const bool cell : covered) missing += !cell;
        return static_cast<double>(missing) / kCells;
    }
};

struct Trial {
    explicit Trial(Scenario selected, int selected_seed)
        : scenario(selected), seed(selected_seed), rng(static_cast<std::uint32_t>(selected_seed)),
          request(MakeRequest(selected_seed)), verifier(request), coverage(kLengthM),
          fault_start_m(35.0 + static_cast<double>(selected_seed % 11)),
          fault_end_m(fault_start_m + 25.0) {}

    static EdgeRequest MakeRequest(int seed) {
        EdgeRequest req;
        req.mission_id = "paired-strip-v1";
        req.mission_revision = 1;
        req.edge_id = "A-B";
        req.attempt_id = "attempt-" + std::to_string(seed);
        req.drone_id = "uav-1";
        req.route.points = {{0.0, 0.0}, {kLengthM, 0.0}};
        req.obligation = Obligation::kSurvey;
        req.corridor_half_width_m = 4.0;
        req.arrival_radius_m = 1.0;
        req.arrival_max_speed_mps = 0.5;
        req.maximum_missing_m = 0.0;
        req.maximum_speed_mps = 20.0;
        req.recovery_overlap_m = 2.0;
        req.max_recovery_attempts = 2;
        req.sustained_deviation_samples = 1000;  // Isolate coverage-gap recovery in this trial.
        req.recovery_cooldown_ms = 1000;
        req.timeout_ms = 1'000'000;
        return req;
    }

    void Sample(double x, double y, bool stopped, bool repair) {
        const double observation_y = y + (scenario.jitter && !repair ? jitter(rng) : 0.0);
        const bool camera_working = repair ||
            (!(scenario.camera_outage && x >= fault_start_m && x <= fault_end_m) &&
             !(scenario.false_capture && x >= fault_start_m && x <= fault_end_m));
        const bool metadata_delivered = repair ||
            !(scenario.metadata_loss && x >= fault_start_m && x <= fault_end_m);
        const double camera_half_width = scenario.off_center ? 14.0 : 8.0;
        truth.Capture(x, y, camera_working, camera_half_width);
        const bool sample_allowed = !scenario.sparse || repair ||
                                    static_cast<int>(x) % 10 == 0;
        if (sample_allowed) {
            const PositionObservation observation{
                .position = Point{x, observation_y},
                .monotonic_ms = time_ms,
                .valid = true,
                .safe_to_navigate = true,
                .failsafe = false,
                .speed_mps = stopped ? 0.0 : 1.0,
                .horizontal_error_hard_bound_m = 0.0,
            };
            const auto state = verifier.ObservePosition(observation);
            arrived = arrived || state.at_b_verified;
            sampled_corridor_ok = sampled_corridor_ok && std::abs(observation_y) <= 4.0;
        }
        if (metadata_delivered) {
            CaptureEvidence capture;
            capture.attempt_id = request.attempt_id;
            capture.capture_id = "frame-" + std::to_string(time_ms);
            capture.calibration_id = "modeled-strip-camera-v1";
            capture.monotonic_ms = time_ms;
            capture.guaranteed_full_strip_interval = {x - 2.0, x + 2.0};
            capture.capture_succeeded = camera_working || (!repair && scenario.false_capture);
            capture.full_strip_quality_established = camera_working &&
                std::abs(y) + 4.0 <= camera_half_width;
            static_cast<void>(verifier.ObserveCapture(capture));
            if (capture.capture_succeeded && capture.full_strip_quality_established)
                static_cast<void>(coverage.Add(capture.guaranteed_full_strip_interval));
        }
        time_ms += 1000;
    }

    void Leg(double from_x, double to_x, bool repair) {
        const int direction = from_x <= to_x ? 1 : -1;
        for (int step = static_cast<int>(from_x);; step += direction) {
            const double x = static_cast<double>(step);
            const double y = !repair && scenario.detour && x >= fault_start_m &&
                                     x <= fault_end_m + 5.0 ? 15.0 :
                             !repair && scenario.off_center && x >= fault_start_m &&
                                     x <= fault_end_m + 5.0 ? 6.0 : 0.0;
            Sample(x, y, step == static_cast<int>(to_x), repair);
            if (step == static_cast<int>(to_x)) break;
        }
        truth.distance_m += std::abs(to_x - from_x);
    }

    Scenario scenario;
    int seed;
    std::mt19937 rng;
    std::uniform_real_distribution<double> jitter{-0.5, 0.5};
    EdgeRequest request;
    EdgeExecutionSupervisor verifier;
    IntervalSet coverage;
    Truth truth;
    std::int64_t time_ms{1000};
    bool arrived{};
    bool sampled_corridor_ok{true};
    double fault_start_m{};
    double fault_end_m{};
};

struct Result {
    const char* scenario;
    int seed{};
    const char* method;
    bool reported_complete{};
    bool truth_complete{};
    double truth_missing_fraction{};
    double total_distance_m{};
    int interventions{};
    bool initial_arrived{};
    double initial_truth_missing_fraction{};
    double initial_predicted_missing_fraction{};
    const char* outcome{};
    double injected_fault_start_m{};
    double injected_fault_end_m{};
};

Result Run(Scenario scenario, int seed, const char* method) {
    Trial trial(scenario, seed);
    trial.Leg(0.0, 100.0, false);
    const bool initial_arrived = trial.arrived;
    const double initial_truth_missing = trial.truth.MissingFraction();
    const double initial_predicted_missing =
        std::string(method) == "P"
            ? (kLengthM - trial.verifier.Snapshot().survey_covered_m) / kLengthM
            : std::string(method) == "B2"
                  ? (kLengthM - trial.coverage.CoveredMeters()) / kLengthM
                  : -1.0;
    if (std::string(method) == "B2" && !trial.coverage.Missing().empty()) {
        // Strong coverage-aware baseline: same sensor observations, full-edge retry.
        trial.Leg(100.0, 0.0, true);
        trial.Leg(0.0, 100.0, true);
    } else if (std::string(method) == "P" && trial.verifier.Snapshot().phase != Phase::kFulfilled) {
        const auto status = trial.verifier.Snapshot();
        if (status.pending_reentry) {
            const double reentry = std::floor(status.pending_reentry->east_m);
            static_cast<void>(trial.verifier.RecoveryDispatched(true, trial.time_ms));
            trial.Leg(100.0, reentry, true);
            trial.Leg(reentry, 100.0, true);
        }
    }
    const auto state = trial.verifier.Snapshot();
    const std::string name(method);
    const bool complete = name == "B0" ? trial.arrived :
                          name == "B1" ? trial.arrived && trial.sampled_corridor_ok :
                          name == "B2" ? trial.arrived && trial.coverage.Missing().empty() :
                          state.phase == Phase::kFulfilled;
    const double missing_fraction = trial.truth.MissingFraction();
    return {scenario.id, seed, method, complete, missing_fraction == 0.0,
            missing_fraction, trial.truth.distance_m,
            static_cast<int>(trial.truth.distance_m > 100.0), initial_arrived,
            initial_truth_missing, initial_predicted_missing,
            complete ? "fulfilled" : state.phase == Phase::kUnknown ? "unknown" : "incomplete",
            trial.fault_start_m, trial.fault_end_m};
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "usage: swarmkit-vecr-experiment OUTPUT_CSV SEED_COUNT\n";
        return 2;
    }
    const int seed_count = std::stoi(argv[2]);
    if (seed_count < 1 || seed_count > 10000) return 2;
    const std::filesystem::path output(argv[1]);
    std::filesystem::create_directories(output.parent_path());
    std::ofstream csv(output);
    if (!csv) return 1;
    csv << "scenario,seed,method,reported_complete,truth_complete,truth_missing_fraction,"
           "total_distance_m,interventions,initial_arrived,initial_truth_missing_fraction,"
           "initial_predicted_missing_fraction,outcome,injected_fault_start_m,"
           "injected_fault_end_m\n";
    for (const auto& scenario : kScenarios) {
        for (int seed = 1; seed <= seed_count; ++seed) {
            for (const char* method : {"B0", "B1", "B2", "P"}) {
                const Result result = Run(scenario, seed, method);
                csv << result.scenario << ',' << result.seed << ',' << result.method << ','
                    << result.reported_complete << ',' << result.truth_complete << ','
                    << result.truth_missing_fraction << ',' << result.total_distance_m << ','
                    << result.interventions << ',' << result.initial_arrived << ','
                    << result.initial_truth_missing_fraction << ','
                    << result.initial_predicted_missing_fraction << ',' << result.outcome << ','
                    << result.injected_fault_start_m << ',' << result.injected_fault_end_m << '\n';
            }
        }
    }
    if (!csv) return 1;
    std::cout << "wrote " << kScenarios.size() * seed_count * 4 << " paired method trials to "
              << output << '\n';
}
