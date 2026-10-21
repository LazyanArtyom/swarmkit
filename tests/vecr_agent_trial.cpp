// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#include "vecr_agent_trial.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <numbers>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "swarmkit/agent/sim_backend.h"
#include "swarmkit/client/client.h"
#include "swarmkit/commands.h"
#include "test_support.h"
#include "vecr_truth_oracle.h"

namespace swarmkit::testsupport {
namespace {

using namespace std::chrono_literals;

[[nodiscard]] bool Terminal(edge::Phase phase) {
    return phase == edge::Phase::kFulfilled || phase == edge::Phase::kInterrupted;
}

[[nodiscard]] edge::NadirCaptureInput CameraMessage(const VecrTrialConfig& config,
                                                    const agent::SimulationTruthFrame& truth,
                                                    std::int64_t monotonic_ms, int tick) {
    edge::NadirCaptureInput capture;
    capture.mission_id = "v4-graph-edge";
    capture.mission_revision = 4;
    capture.edge_id = "A-B";
    capture.attempt_id = config.scenario_id + "-" + std::to_string(config.seed) +
                         (config.policy == edge::RepairPolicy::kFullEdgeRestart ? "-B2" : "-P");
    capture.capture_id = "synthetic-frame-" + std::to_string(tick);
    capture.calibration_id = "simulated-nadir-v4";
    capture.monotonic_ms = monotonic_ms;
    capture.camera_center = {truth.east_m, truth.north_m};
    capture.altitude_above_ground_m = truth.up_m;
    capture.altitude_error_hard_bound_m = 0;
    capture.horizontal_error_hard_bound_m = 0;
    capture.along_fov_deg =
        2 * std::atan(config.camera_along_half_m / truth.up_m) * 180.0 / std::numbers::pi;
    capture.cross_fov_deg =
        2 * std::atan(config.camera_cross_half_m / truth.up_m) * 180.0 / std::numbers::pi;
    capture.capture_succeeded = true;
    capture.image_quality_established = true;
    capture.nadir_route_alignment_established = true;
    capture.flat_ground_established = true;
    return capture;
}

}  // namespace

VecrTrialResult RunVecrAgentTrial(const VecrTrialConfig& config) {
    if (config.scenario_id.empty() || config.edge_length_m <= 0 || config.gap_begin_m < 0 ||
        config.gap_end_m > config.edge_length_m || config.gap_begin_m >= config.gap_end_m ||
        config.capture_period_ticks <= 0 || config.max_ticks <= 0)
        throw std::invalid_argument("invalid agent trial configuration");
    VecrTrialResult result;
    result.config = config;
    result.attempt_id = config.scenario_id + "-" + std::to_string(config.seed) +
                        (config.policy == edge::RepairPolicy::kFullEdgeRestart ? "-B2" : "-P");
    auto oracle = std::make_shared<VecrTruthOracle>(config.edge_length_m, 0.25, 4);
    auto record_truth = std::make_shared<std::atomic<bool>>(false);
    agent::SimBackendConfig sim_config;
    sim_config.clock_mode = agent::SimulationClockMode::kManual;
    sim_config.max_horizontal_acceleration_mps2 =
        static_cast<float>(config.max_horizontal_acceleration_mps2);
    sim_config.stop_at_target =
        config.fault != TrialFault::kFastFlyThrough && config.stop_at_target;
    if (config.fault == TrialFault::kSafetyBattery) {
        sim_config.initial_battery_percent = 27.0F;
        sim_config.battery_drain_percent_per_second = 1.0F;
    }
    sim_config.truth_observer = [oracle, record_truth](const auto& truth) {
        if (record_truth->load(std::memory_order_relaxed))
            oracle->ObservePose(truth);
    };
    auto simulator = agent::MakeSimBackend(sim_config);
    if (!simulator)
        throw std::runtime_error("cannot construct SimBackend");
    auto control = simulator->control;
    commands::CommandEnvelope command;
    command.context.drone_id = "drone-1";
    command.context.client_id = "test-client";
    command.command = commands::FlightCmd{commands::CmdArm{}};
    if (!simulator->backend->Execute(command).IsOk())
        throw std::runtime_error("simulator arm failed");
    command.command = commands::FlightCmd{commands::CmdTakeoff{.alt_m = 10}};
    if (!simulator->backend->Execute(command).IsOk())
        throw std::runtime_error("simulator takeoff failed");
    if (!simulator->backend->StartTelemetry("drone-1", 10, [](const core::TelemetryFrame&) {})
             .IsOk() ||
        !control->Advance("drone-1", 3000ms).IsOk() ||
        !simulator->backend->StopTelemetry("drone-1").IsOk())
        throw std::runtime_error("simulator initialization failed");
    const auto initial_truth = control->Truth("drone-1");
    if (!initial_truth)
        throw std::runtime_error("initial simulator truth absent");
    const auto trial_start_ms = initial_truth->simulation_time_ms;

    const auto certificates = MakeDevMtlsPaths();
    agent::AgentConfig agent_config;
    agent_config.agent_id = "v4-agent";
    agent_config.bind_addr = "127.0.0.1:0";
    agent_config.security.root_ca_cert_path = certificates.root_ca_cert_path;
    agent_config.security.cert_chain_path = certificates.server_cert_chain_path;
    agent_config.security.private_key_path = certificates.server_private_key_path;
    agent_config.vecr_sensor_client_id = "test-client";
    agent_config.vecr_sensor_calibration_id = "simulated-nadir-v4";
    agent_config.vecr_sim_recovery_enabled = true;
    auto clock_ns = std::make_shared<std::atomic<std::int64_t>>(1'000'000'000LL);
    auto providers = agent::internal::RuntimeProviders::System();
    providers.monotonic_time_ns = [clock_ns] { return clock_ns->load(); };

    std::mutex command_mutex;
    std::vector<VecrTrialEvent> command_events;
    auto telemetry_fault_realized = std::make_shared<std::atomic<bool>>(false);
    auto telemetry_filter_count = std::make_shared<std::atomic<int>>(0);
    auto recovery_rejected = std::make_shared<std::atomic<bool>>(false);
    auto goto_count = std::make_shared<std::atomic<int>>(0);
    const auto execute_filter = [config, recovery_rejected,
                                 goto_count](const commands::CommandEnvelope& envelope)
        -> std::optional<core::BackendCommandOutcome> {
        if (const auto* nav = std::get_if<commands::NavCmd>(&envelope.command)) {
            if (std::get_if<commands::CmdGoto>(nav) && goto_count->fetch_add(1) >= 1 &&
                config.fault == TrialFault::kRecoveryCommandRejected) {
                recovery_rejected->store(true);
                return core::BackendCommandOutcome{
                    .result = core::Result::Rejected("scripted recovery command refusal"),
                    .dispatch_state = core::BackendDispatchState::kRejected};
            }
        }
        return std::nullopt;
    };
    const auto telemetry_filter =
        [config, sim_config, telemetry_fault_realized, telemetry_filter_count](
            const core::TelemetryFrame& source) -> std::optional<core::TelemetryFrame> {
        auto frame = source;
        const int observation_index = telemetry_filter_count->fetch_add(1);
        const auto local = edge::Wgs84ToLocal(sim_config.home_lat_deg, sim_config.home_lon_deg,
                                              frame.lat_deg, frame.lon_deg);
        const bool in_region =
            local.east_m >= config.gap_begin_m && local.east_m <= config.gap_end_m;
        if (config.fault == TrialFault::kSparseTelemetry && observation_index % 5 != 0) {
            telemetry_fault_realized->store(true);
            return std::nullopt;
        }
        if (!in_region)
            return frame;
        if (config.fault == TrialFault::kGnssOutage) {
            frame.validity.position = false;
            frame.gps_fix_type = 0;
            telemetry_fault_realized->store(true);
        } else if (config.fault == TrialFault::kEstimatorJump) {
            frame.position_frame = core::CoordinateFrame::kUnknown;
            frame.estimator_position_ok = false;
            telemetry_fault_realized->store(true);
        } else if (config.fault == TrialFault::kUnboundedAccuracy) {
            if (frame.accuracy.horizontal_position)
                frame.accuracy.horizontal_position->descriptor.semantics =
                    core::UncertaintySemantics::kStandardDeviation;
            telemetry_fault_realized->store(true);
        } else if (config.fault == TrialFault::kGpsJitter) {
            const std::uint64_t mixed =
                static_cast<std::uint64_t>(observation_index) * 6364136223846793005ULL +
                config.seed * 1442695040888963407ULL;
            const double offset = static_cast<double>((mixed >> 32) % 1001) / 1000.0 - 0.5;
            const auto perturbed =
                edge::LocalToWgs84(sim_config.home_lat_deg, sim_config.home_lon_deg,
                                   {local.east_m, local.north_m + offset});
            frame.lat_deg = perturbed.lat_deg;
            frame.lon_deg = perturbed.lon_deg;
            if (frame.accuracy.horizontal_position)
                frame.accuracy.horizontal_position->value = 0.6F;
            telemetry_fault_realized->store(true);
        }
        return frame;
    };
    auto observer = [&](const commands::CommandEnvelope& envelope,
                        const core::BackendCommandOutcome& outcome) {
        const auto nav = std::get_if<commands::NavCmd>(&envelope.command);
        if (!nav)
            return;
        const auto goto_command = std::get_if<commands::CmdGoto>(nav);
        if (!goto_command)
            return;
        const auto target = edge::Wgs84ToLocal(sim_config.home_lat_deg, sim_config.home_lon_deg,
                                               goto_command->lat_deg, goto_command->lon_deg);
        std::lock_guard<std::mutex> lock(command_mutex);
        command_events.push_back(
            {.tick = -1,
             .simulation_time_ms = clock_ns->load() / 1'000'000 - 1000,
             .kind = outcome.IsOk() ? "backend_goto_accepted" : "backend_goto_rejected",
             .has_truth_pose = false,
             .detail = "target_east_m=" + std::to_string(target.east_m) +
                       ";target_north_m=" + std::to_string(target.north_m)});
    };
    std::shared_ptr<agent::IDroneBackend> backend = std::move(simulator->backend);
    AgentServerHarness harness(
        agent_config,
        std::make_unique<SharedBackendHandle>(backend, execute_filter, observer, telemetry_filter),
        providers);
    auto client_config = MakeMtlsClientConfig(harness.Address());
    client_config.retry_policy.max_attempts = 1;
    client::Client client(std::move(client_config));
    client::EdgeOperationRequest request;
    request.edge.mission_id = "v4-graph-edge";
    request.edge.mission_revision = 4;
    request.edge.edge_id = "A-B";
    request.edge.attempt_id = result.attempt_id;
    request.edge.drone_id = "drone-1";
    request.edge.route.points = {{0, 0}, {config.edge_length_m, 0}};
    request.edge.obligation = edge::Obligation::kSurvey;
    request.edge.repair_policy = config.policy;
    request.edge.corridor_half_width_m = 4;
    request.edge.arrival_radius_m = 2;
    request.edge.arrival_max_speed_mps = 0.5;
    request.edge.maximum_speed_mps = 10;
    request.edge.recovery_overlap_m = 2;
    request.edge.max_recovery_attempts = 1;
    request.edge.sustained_deviation_samples = 1000;
    request.edge.recovery_cooldown_ms = 1000;
    request.edge.timeout_ms = 60000;
    request.wgs84_origin = {sim_config.home_lat_deg, sim_config.home_lon_deg};
    request.altitude_m = 10;
    request.commanded_speed_mps = static_cast<float>(config.commanded_speed_mps);
    const auto started = client.ExecuteEdge(request);
    if (!started.ok)
        throw std::runtime_error("agent edge dispatch failed: " + started.message);
    oracle->ObservePose(*initial_truth);
    record_truth->store(true, std::memory_order_relaxed);
    bool gap_fault_recorded = false;
    bool motor_failure_injected = false;
    bool wind_fault_recorded = false;

    for (int tick = 0; tick < config.max_ticks; ++tick) {
        const auto before_advance = control->Truth("drone-1");
        if (!before_advance)
            throw std::runtime_error("simulator truth disappeared");
        const std::int64_t before_elapsed_ms = before_advance->simulation_time_ms - trial_start_ms;
        const bool localized_wind = before_elapsed_ms <= 12'000 &&
                                    before_advance->east_m >= config.gap_begin_m &&
                                    before_advance->east_m <= config.gap_end_m;
        const bool wind_active = config.fault == TrialFault::kPersistentWind ||
                                 ((config.fault == TrialFault::kDetourWind ||
                                   config.fault == TrialFault::kOffcenterWideCamera) &&
                                  localized_wind);
        control->SetWind("drone-1", wind_active ? config.wind_north_mps : 0.0, 0.0);
        clock_ns->fetch_add(100'000'000LL);
        if (!control->Advance("drone-1", 100ms).IsOk())
            throw std::runtime_error("simulator advance failed");
        std::this_thread::sleep_for(2ms);
        const auto truth = control->Truth("drone-1");
        if (!truth)
            throw std::runtime_error("simulator truth disappeared");
        result.max_truth_cross_track_m =
            std::max(result.max_truth_cross_track_m, std::abs(truth->north_m));
        if (wind_active && !wind_fault_recorded &&
            std::abs(truth->north_m - before_advance->north_m) > 0.02) {
            wind_fault_recorded = true;
            result.requested_fault_realized = true;
            result.events.push_back(
                {.tick = tick,
                 .simulation_time_ms = truth->simulation_time_ms - trial_start_ms,
                 .kind = "wind_realized",
                 .has_truth_pose = true,
                 .truth_east_m = truth->east_m,
                 .truth_north_m = truth->north_m,
                 .detail = "north_mps=" + std::to_string(config.wind_north_mps)});
        }
        const auto observed = client.GetEdgeStatus("drone-1", result.attempt_id);
        if (!observed.ok || !observed.status)
            throw std::runtime_error("agent edge status unavailable: " + observed.message);
        const std::int64_t elapsed_ms = truth->simulation_time_ms - trial_start_ms;
        const bool in_gap =
            truth->east_m >= config.gap_begin_m && truth->east_m <= config.gap_end_m;
        const bool transient_gap = in_gap && elapsed_ms <= 12'000;
        bool actual_image = !truth->failed;
        bool metadata_sent = true;
        if ((config.fault == TrialFault::kTransientCameraGap ||
             config.fault == TrialFault::kMotorFailureOnRepair ||
             config.fault == TrialFault::kRecoveryCommandRejected) &&
            transient_gap)
            actual_image = false;
        if (config.fault == TrialFault::kPersistentCameraGap && in_gap)
            actual_image = false;
        if (config.fault == TrialFault::kMetadataDrop && transient_gap)
            metadata_sent = false;
        if (config.fault == TrialFault::kDishonestCamera && in_gap)
            actual_image = false;
        if (config.fault == TrialFault::kDetectableBadMetadata && transient_gap)
            actual_image = false;
        const bool realized =
            in_gap && (((config.fault == TrialFault::kTransientCameraGap ||
                         config.fault == TrialFault::kMotorFailureOnRepair ||
                         config.fault == TrialFault::kRecoveryCommandRejected) &&
                        transient_gap) ||
                       config.fault == TrialFault::kPersistentCameraGap ||
                       (config.fault == TrialFault::kMetadataDrop && transient_gap) ||
                       config.fault == TrialFault::kDishonestCamera ||
                       (config.fault == TrialFault::kDetectableBadMetadata && transient_gap));
        if (realized && !gap_fault_recorded) {
            gap_fault_recorded = true;
            if (config.fault != TrialFault::kMotorFailureOnRepair)
                result.requested_fault_realized = true;
            result.events.push_back({.tick = tick,
                                     .simulation_time_ms = elapsed_ms,
                                     .kind = "fault_realized",
                                     .has_truth_pose = true,
                                     .truth_east_m = truth->east_m,
                                     .truth_north_m = truth->north_m,
                                     .detail = config.scenario_id});
        }
        if (config.fault == TrialFault::kMotorFailureOnRepair &&
            observed.status->status.recovery_attempts > 0 && !motor_failure_injected) {
            control->SetFailure("drone-1", true, "v4 scripted motor failure");
            motor_failure_injected = true;
            result.requested_fault_realized = true;
            result.events.push_back({.tick = tick,
                                     .simulation_time_ms = elapsed_ms,
                                     .kind = "motor_failure_realized",
                                     .has_truth_pose = true,
                                     .truth_east_m = truth->east_m,
                                     .truth_north_m = truth->north_m});
        }
        bool captured = false;
        bool submitted = false;
        bool accepted = false;
        if (tick % config.capture_period_ticks == 0) {
            captured = actual_image;
            const double cross_half = config.fault == TrialFault::kOffcenterWideCamera ? 14.0
                                      : config.fault == TrialFault::kDetourWind
                                          ? 5.0
                                          : config.camera_cross_half_m;
            oracle->ObserveActualCapture(*truth, actual_image, config.camera_along_half_m,
                                         cross_half);
            if (actual_image)
                ++result.capture_acquired;
            const bool full_strip_in_view = std::abs(truth->north_m) + 4.0 <= cross_half;
            if (metadata_sent &&
                (config.fault == TrialFault::kDishonestCamera ||
                 (config.fault == TrialFault::kDetectableBadMetadata && transient_gap) ||
                 (actual_image && full_strip_in_view))) {
                auto capture = CameraMessage(config, *truth, clock_ns->load() / 1'000'000, tick);
                if (config.fault == TrialFault::kDetectableBadMetadata && transient_gap)
                    capture.camera_center.east_m += 20.0;
                capture.cross_fov_deg =
                    2 * std::atan(cross_half / truth->up_m) * 180.0 / std::numbers::pi;
                client::EdgeResult reply;
                accepted = WaitUntil(
                    [&] {
                        reply = client.SubmitEdgeCapture("drone-1", capture);
                        return reply.ok;
                    },
                    100ms, 5ms);
                submitted = true;
                ++result.capture_submitted;
                if (accepted)
                    ++result.capture_accepted;
            }
        }
        const auto current = client.GetEdgeStatus("drone-1", result.attempt_id);
        if (!current.ok || !current.status)
            throw std::runtime_error("agent edge status unavailable after capture");
        result.reported_phase = current.status->status.phase;
        result.reported_covered_m = current.status->status.survey_covered_m;
        result.recovery_attempts = current.status->status.recovery_attempts;
        result.ticks = tick + 1;
        result.elapsed_simulation_ms = elapsed_ms;
        result.trace.push_back({.tick = tick,
                                .simulation_time_ms = elapsed_ms,
                                .truth_east_m = truth->east_m,
                                .truth_north_m = truth->north_m,
                                .truth_up_m = truth->up_m,
                                .physical_distance_m = oracle->DistanceM(),
                                .truth_covered_cells = oracle->CoveredCells(),
                                .reported_covered_m = result.reported_covered_m,
                                .reported_phase = result.reported_phase,
                                .recovery_attempts = result.recovery_attempts,
                                .capture_acquired = captured,
                                .capture_submitted = submitted,
                                .capture_accepted = accepted});
        if (Terminal(result.reported_phase))
            break;
    }
    record_truth->store(false, std::memory_order_relaxed);
    if (telemetry_fault_realized->load()) {
        result.requested_fault_realized = true;
        result.events.push_back({.tick = -1,
                                 .simulation_time_ms = 0,
                                 .kind = "telemetry_fault_realized",
                                 .detail = "filtered simulator telemetry before agent ingress"});
    }
    if (recovery_rejected->load()) {
        result.requested_fault_realized = true;
        result.events.push_back(
            {.tick = -1, .simulation_time_ms = 0, .kind = "recovery_command_rejected"});
    }
    if (config.fault == TrialFault::kSafetyBattery &&
        result.reported_phase == edge::Phase::kInterrupted) {
        result.requested_fault_realized = true;
        result.events.push_back({.tick = -1,
                                 .simulation_time_ms = result.elapsed_simulation_ms,
                                 .kind = "simulated_battery_safety_interruption"});
    }
    result.truth_survey_complete = oracle->Complete();
    result.truth_at_b = oracle->AtB(2, 0.5, 10);
    result.physical_distance_m = oracle->DistanceM();
    result.truth_missing_fraction = oracle->MissingFraction();
    result.truth_longest_gap_m = oracle->LongestMissingM();
    result.truth_covered_cells = oracle->CoveredCells();
    result.truth_cells = oracle->CellCount();
    {
        std::lock_guard<std::mutex> lock(command_mutex);
        for (const auto& event : command_events) {
            if (event.kind == "backend_goto_accepted")
                ++result.backend_goto_accepted;
            else
                ++result.backend_goto_rejected;
            result.events.push_back(event);
        }
    }
    std::stable_sort(result.events.begin(), result.events.end(), [](const auto& a, const auto& b) {
        return a.simulation_time_ms < b.simulation_time_ms;
    });
    return result;
}

}  // namespace swarmkit::testsupport
