// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

#include "vecr_agent_trial.h"

namespace {

[[nodiscard]] std::string Arg(int argc, char** argv, std::string_view name) {
    for (int i = 1; i + 1 < argc; ++i) {
        if (argv[i] == name)
            return argv[i + 1];
    }
    throw std::invalid_argument("missing argument " + std::string(name));
}

[[nodiscard]] swarmkit::testsupport::TrialFault ParseFault(const std::string& value) {
    using Fault = swarmkit::testsupport::TrialFault;
    if (value == "none")
        return Fault::kNone;
    if (value == "transient_camera_gap")
        return Fault::kTransientCameraGap;
    if (value == "persistent_camera_gap")
        return Fault::kPersistentCameraGap;
    if (value == "motor_failure_on_repair")
        return Fault::kMotorFailureOnRepair;
    if (value == "metadata_drop")
        return Fault::kMetadataDrop;
    if (value == "dishonest_camera")
        return Fault::kDishonestCamera;
    if (value == "detectable_bad_metadata")
        return Fault::kDetectableBadMetadata;
    if (value == "persistent_wind")
        return Fault::kPersistentWind;
    if (value == "detour_wind")
        return Fault::kDetourWind;
    if (value == "offcenter_wide_camera")
        return Fault::kOffcenterWideCamera;
    if (value == "gps_jitter")
        return Fault::kGpsJitter;
    if (value == "sparse_telemetry")
        return Fault::kSparseTelemetry;
    if (value == "gnss_outage")
        return Fault::kGnssOutage;
    if (value == "estimator_jump")
        return Fault::kEstimatorJump;
    if (value == "unbounded_accuracy")
        return Fault::kUnboundedAccuracy;
    if (value == "recovery_command_rejected")
        return Fault::kRecoveryCommandRejected;
    if (value == "safety_battery")
        return Fault::kSafetyBattery;
    if (value == "fast_fly_through")
        return Fault::kFastFlyThrough;
    throw std::invalid_argument("unknown fault " + value);
}

[[nodiscard]] std::string Phase(swarmkit::edge::Phase phase) {
    using Phase = swarmkit::edge::Phase;
    switch (phase) {
        case Phase::kMoving:
            return "MOVING";
        case Phase::kRecovering:
            return "RECOVERING";
        case Phase::kFulfilled:
            return "FULFILLED";
        case Phase::kInterrupted:
            return "INTERRUPTED";
        case Phase::kUnknown:
            return "UNKNOWN";
    }
    return "INVALID";
}

[[nodiscard]] std::string Escape(std::string_view value) {
    std::string escaped = "\"";
    for (const char character : value) {
        if (character == '"')
            escaped += '"';
        escaped += character;
    }
    escaped += '"';
    return escaped;
}

void Write(const std::filesystem::path& output,
           const swarmkit::testsupport::VecrTrialResult& result) {
    std::ofstream attempt(output / "attempt.csv");
    std::ofstream trace(output / "trace.csv");
    std::ofstream events(output / "events.csv");
    if (!attempt || !trace || !events)
        throw std::runtime_error("cannot create trial output");
    attempt << std::setprecision(17);
    trace << std::setprecision(17);
    events << std::setprecision(17);
    const auto& config = result.config;
    const auto policy =
        config.policy == swarmkit::edge::RepairPolicy::kFullEdgeRestart ? "B2" : "P";
    attempt << "scenario_id,seed,policy,attempt_id,reported_phase,truth_survey_complete,"
               "truth_at_b,false_complete,physical_distance_m,max_truth_cross_track_m,"
               "elapsed_simulation_ms,"
               "truth_missing_fraction,truth_longest_gap_m,truth_covered_cells,truth_cells,"
               "reported_covered_m,recovery_attempts,backend_goto_accepted,"
               "backend_goto_rejected,capture_acquired,capture_submitted,capture_accepted,"
               "requested_fault_realized,gap_begin_m,gap_end_m,wind_north_mps,"
               "commanded_speed_mps,acceleration_mps2,capture_period_ticks,"
               "camera_along_half_m,camera_cross_half_m\n";
    attempt << config.scenario_id << ',' << config.seed << ',' << policy << ',' << result.attempt_id
            << ',' << Phase(result.reported_phase) << ',' << result.truth_survey_complete << ','
            << result.truth_at_b << ','
            << (result.reported_phase == swarmkit::edge::Phase::kFulfilled &&
                !result.truth_survey_complete)
            << ',' << result.physical_distance_m << ',' << result.max_truth_cross_track_m << ','
            << result.elapsed_simulation_ms << ',' << result.truth_missing_fraction << ','
            << result.truth_longest_gap_m << ',' << result.truth_covered_cells << ','
            << result.truth_cells << ',' << result.reported_covered_m << ','
            << result.recovery_attempts << ',' << result.backend_goto_accepted << ','
            << result.backend_goto_rejected << ',' << result.capture_acquired << ','
            << result.capture_submitted << ',' << result.capture_accepted << ','
            << result.requested_fault_realized << ',' << config.gap_begin_m << ','
            << config.gap_end_m << ',' << config.wind_north_mps << ',' << config.commanded_speed_mps
            << ',' << config.max_horizontal_acceleration_mps2 << ',' << config.capture_period_ticks
            << ',' << config.camera_along_half_m << ',' << config.camera_cross_half_m << '\n';
    trace << "scenario_id,seed,policy,attempt_id,tick,simulation_time_ms,truth_east_m,"
             "truth_north_m,truth_up_m,physical_distance_m,truth_covered_cells,"
             "reported_covered_m,reported_phase,recovery_attempts,capture_acquired,"
             "capture_submitted,capture_accepted\n";
    for (const auto& row : result.trace) {
        trace << config.scenario_id << ',' << config.seed << ',' << policy << ','
              << result.attempt_id << ',' << row.tick << ',' << row.simulation_time_ms << ','
              << row.truth_east_m << ',' << row.truth_north_m << ',' << row.truth_up_m << ','
              << row.physical_distance_m << ',' << row.truth_covered_cells << ','
              << row.reported_covered_m << ',' << Phase(row.reported_phase) << ','
              << row.recovery_attempts << ',' << row.capture_acquired << ','
              << row.capture_submitted << ',' << row.capture_accepted << '\n';
    }
    events << "scenario_id,seed,policy,attempt_id,tick,simulation_time_ms,kind,"
              "truth_east_m,truth_north_m,detail\n";
    for (const auto& event : result.events) {
        events << config.scenario_id << ',' << config.seed << ',' << policy << ','
               << result.attempt_id << ',' << event.tick << ',' << event.simulation_time_ms << ','
               << event.kind << ',';
        if (event.has_truth_pose)
            events << event.truth_east_m;
        events << ',';
        if (event.has_truth_pose)
            events << event.truth_north_m;
        events << ',' << Escape(event.detail) << '\n';
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        swarmkit::testsupport::VecrTrialConfig config;
        config.scenario_id = Arg(argc, argv, "--scenario");
        config.seed = std::stoull(Arg(argc, argv, "--seed"));
        const auto policy = Arg(argc, argv, "--policy");
        if (policy == "B2")
            config.policy = swarmkit::edge::RepairPolicy::kFullEdgeRestart;
        else if (policy == "P")
            config.policy = swarmkit::edge::RepairPolicy::kTargetedGap;
        else
            throw std::invalid_argument("policy must be B2 or P");
        config.fault = ParseFault(Arg(argc, argv, "--fault"));
        config.gap_begin_m = std::stod(Arg(argc, argv, "--gap-begin-m"));
        config.gap_end_m = std::stod(Arg(argc, argv, "--gap-end-m"));
        for (int i = 1; i + 1 < argc; ++i) {
            if (std::string_view(argv[i]) == "--wind-north-mps")
                config.wind_north_mps = std::stod(argv[i + 1]);
            if (std::string_view(argv[i]) == "--speed-mps")
                config.commanded_speed_mps = std::stod(argv[i + 1]);
            if (std::string_view(argv[i]) == "--acceleration-mps2")
                config.max_horizontal_acceleration_mps2 = std::stod(argv[i + 1]);
            if (std::string_view(argv[i]) == "--capture-period-ticks")
                config.capture_period_ticks = std::stoi(argv[i + 1]);
            if (std::string_view(argv[i]) == "--camera-along-half-m")
                config.camera_along_half_m = std::stod(argv[i + 1]);
            if (std::string_view(argv[i]) == "--camera-cross-half-m")
                config.camera_cross_half_m = std::stod(argv[i + 1]);
        }
        const auto output = std::filesystem::path(Arg(argc, argv, "--output"));
        if (!std::filesystem::create_directories(output))
            throw std::runtime_error("refusing to overwrite existing trial directory");
        const auto result = swarmkit::testsupport::RunVecrAgentTrial(config);
        Write(output, result);
        std::cout << config.scenario_id << " seed=" << config.seed << " policy=" << policy
                  << " phase=" << Phase(result.reported_phase)
                  << " distance_m=" << result.physical_distance_m << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "VECR agent trial failed: " << error.what() << '\n';
        return 1;
    }
}
