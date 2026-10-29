// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#include <sys/resource.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "swarmkit/agent/sim_backend.h"
#include "swarmkit/client/client.h"
#include "swarmkit/client/swarm_client.h"
#include "test_support.h"
#include "vecr_sensor_models.h"

namespace {
using namespace swarmkit;
using namespace swarmkit::testsupport;
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
std::string Quote(std::string text) {
    std::string out = "\"";
    for (char c : text) {
        if (c == '"')
            out += '"';
        out += c;
    }
    return out + '"';
}
std::string Phase(edge::Phase p) {
    switch (p) {
        case edge::Phase::kMoving:
            return "MOVING";
        case edge::Phase::kDeviating:
            return "DEVIATING";
        case edge::Phase::kRecovering:
            return "RECOVERING";
        case edge::Phase::kArrived:
            return "ARRIVED";
        case edge::Phase::kFailed:
            return "FAILED";
        case edge::Phase::kCancelled:
            return "CANCELLED";
        case edge::Phase::kUnknown:
            return "UNKNOWN";
    }
    return "INVALID";
}
bool Terminal(edge::Phase p) {
    return p == edge::Phase::kArrived || p == edge::Phase::kFailed ||
           p == edge::Phase::kCancelled;
}
double Unit(std::uint64_t seed, std::uint64_t tick, std::uint64_t lane) {
    auto x = seed * 0x9e3779b97f4a7c15ULL + tick * 0xbf58476d1ce4e5b9ULL + lane;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    return static_cast<double>((x ^ (x >> 31)) >> 11) / 9007199254740992.0;
}
std::string Gaps(const edge::Status& s) {
    std::ostringstream out;
    out << std::setprecision(12);
    for (const auto& g : s.survey_missing)
        out << g.begin_m << ':' << g.end_m << ';';
    return out.str();
}
struct Pending {
    int due_tick{};
    edge::NadirCaptureInput input;
    bool acquired{};
    std::optional<std::pair<double, double>> actual;
};

void Run(const std::map<std::string, std::string>& args) {
    auto str = [&](const std::string& key, const std::string& value) {
        auto it = args.find(key);
        return it == args.end() ? value : it->second;
    };
    auto num = [&](const std::string& key, double value) {
        return std::stod(str(key, std::to_string(value)));
    };
    const auto scenario = str("scenario", "nominal"), fault = str("fault", "none"),
               policy = str("policy", "VECR");
#if defined(SWARMKIT_RESEARCH_FULL_RESTART)
    if (policy != "B2")
#else
    if (policy != "VECR")
#endif
        throw std::invalid_argument("policy does not match this executable");
    const auto client_kind = str("client-kind", "direct");
    if (client_kind != "direct" && client_kind != "routed")
        throw std::invalid_argument("invalid client kind");
    const auto seed = static_cast<std::uint64_t>(num("seed", 1));
    const double length = num("length", 40), along = num("along", 2), cross = num("cross", 7);
    const double bound = num("bound", 0.3), wind = num("wind", 3),
                 wind_begin = num("wind-begin", 6);
    const double wind_end = num("wind-end", 10), gap_begin = num("gap-begin", length * 0.4);
    const double gap_end = num("gap-end", length * 0.6), period = num("period", 0.5);
    const int delay = static_cast<int>(num("delay", 0.2) * 10),
              attempts = static_cast<int>(num("attempts", 2));
    const int max_ticks = static_cast<int>(num("timeout", 90) * 10) + 5;
    if (length <= 0 || along <= bound || cross <= 0 || period < 0.1 || delay < 0 || attempts < 0)
        throw std::invalid_argument("invalid trial configuration");
    const auto output = std::filesystem::path(str("output", ""));
    if (output.empty() || !std::filesystem::create_directories(output))
        throw std::runtime_error("output must be new directory");
    std::ofstream trace(output / "trace.csv"), captures(output / "captures.csv"),
        events(output / "events.csv"), summary(output / "attempt.csv"),
        trajectory(output / "trajectory.csv"), acquisitions(output / "acquisitions.csv"),
        measurements(output / "measurements.csv");
    for (auto* stream :
         {&trace, &captures, &events, &summary, &trajectory, &acquisitions, &measurements}) {
        if (!*stream)
            throw std::runtime_error("output unavailable");
        *stream << std::setprecision(17);
    }
    trace << "tick,time_ms,east_m,north_m,up_m,speed_mps,distance_m,truth_covered_m,truth_at_b,"
             "phase,reason,reported_covered_m,at_b_verified,recoveries,missing\n";
    captures << "tick,acquisition_ms,delivery_ms,acquired,submitted,accepted,reason,claimed_east,"
                "claimed_north,horizontal_bound,claimed_begin,claimed_end,actual_begin,actual_end,"
                "contained,latency_us\n";
    events << "time_ms,kind,target_east,target_north,detail\n";
    trajectory << "time_ms,east_m,north_m,up_m,speed_mps\n";
    acquisitions << "tick,time_ms,east_m,north_m,up_m,along_half_m,cross_half_m,yaw_deg,acquired,"
                    "begin_m,end_m\n";
    measurements << "time_ms,east_m,north_m,altitude_m,north_velocity_mps,east_velocity_mps,"
                    "position_valid,wgs84_frame,hard_bound_semantics,horizontal_bound_m\n";
    auto oracle = std::make_shared<sensing::ExactStripOracle>(length);
    auto clock_ns = std::make_shared<std::atomic<std::int64_t>>(1'000'000'000LL);
    bool record = false;
    double distance = 0, max_cross = 0;
    std::int64_t arrival_since = -1;
    bool truth_at_b = false;
    std::optional<agent::SimulationTruthFrame> previous;
    agent::SimBackendConfig sim;
    sim.clock_mode = agent::SimulationClockMode::kManual;
    sim.max_horizontal_acceleration_mps2 = 2;
    if (fault == "low_battery") {
        sim.initial_battery_percent = 30;
        sim.battery_drain_percent_per_second = 1;
    }
    sim.truth_observer = [&](const agent::SimulationTruthFrame& t) {
        if (!record)
            return;
        if (previous)
            distance += std::hypot(t.east_m - previous->east_m, t.north_m - previous->north_m);
        previous = t;
        max_cross = std::max(max_cross, std::abs(t.north_m));
        const double speed = std::hypot(t.velocity_east_mps, t.velocity_north_mps);
        const bool stopped =
            std::hypot(std::hypot(t.east_m - length, t.north_m), t.up_m - 10) <= 2 && speed <= 0.5;
        if (stopped) {
            if (arrival_since < 0)
                arrival_since = t.simulation_time_ms;
        } else
            arrival_since = -1;
        truth_at_b = stopped && t.simulation_time_ms - arrival_since >= 200;
        trajectory << t.simulation_time_ms - 3000 << ',' << t.east_m << ',' << t.north_m << ','
                   << t.up_m << ',' << speed << '\n';
    };
    auto instance = agent::MakeSimBackend(sim);
    if (!instance)
        throw std::runtime_error("sim construction failed");
    const auto control = instance->control;
    commands::CommandEnvelope command;
    command.context.drone_id = "drone-1";
    command.context.client_id = "test-client";
    command.command = commands::FlightCmd{commands::CmdArm{}};
    if (!instance->backend->Execute(command).IsOk())
        throw std::runtime_error("arm failed");
    command.command = commands::FlightCmd{commands::CmdTakeoff{.alt_m = 10}};
    if (!instance->backend->Execute(command).IsOk())
        throw std::runtime_error("takeoff failed");
    if (!instance->backend->StartTelemetry("drone-1", 10, [](const auto&) {}).IsOk() ||
        !control->Advance("drone-1", 3000ms).IsOk() ||
        !instance->backend->StopTelemetry("drone-1").IsOk())
        throw std::runtime_error("initialization failed");
    previous = control->Truth("drone-1");
    std::mutex sensor_mutex, event_mutex;
    std::optional<core::TelemetryFrame> measurement;
    std::atomic<int> gotos{0}, rejects{0}, report_count{0};
    std::atomic<bool> fault_realized{false};
    bool motor_failed = false;
    int midflight_repairs = 0;
    double first_repair_east = -1;
    std::int64_t first_repair_ms = -1;
    auto filter = [&](const core::TelemetryFrame& raw) -> std::optional<core::TelemetryFrame> {
        auto m = raw;
        const auto tick = static_cast<std::uint64_t>((clock_ns->load() / 1'000'000 - 1000) / 100);
        const auto local =
            edge::Wgs84ToLocal(sim.home_lat_deg, sim.home_lon_deg, raw.lat_deg, raw.lon_deg);
        const double elapsed_s = static_cast<double>(tick) / 10;
        const double measured_bound =
            (fault == "changing_uncertainty" || fault == "uncertainty_gap") && elapsed_s >= 8 &&
                    elapsed_s < 18
                ? std::max(bound, 0.9)
                : bound;
        const double phi = 2 * std::numbers::pi * Unit(seed, tick, 0);
        const auto geo = edge::LocalToWgs84(
            sim.home_lat_deg, sim.home_lon_deg,
            {local.east_m + 0.3 * measured_bound + 0.4 * measured_bound * std::sin(phi),
             local.north_m + 0.4 * measured_bound * std::cos(phi)});
        m.lat_deg = geo.lat_deg;
        m.lon_deg = geo.lon_deg;
        m.rel_alt_m += static_cast<float>(0.03 * (2 * Unit(seed, tick, 1) - 1));
        m.vx_mps += static_cast<float>(0.01 * (2 * Unit(seed, tick, 2) - 1));
        m.vy_mps += static_cast<float>(0.01 * (2 * Unit(seed, tick, 3) - 1));
        m.accuracy.horizontal_position->value = static_cast<float>(measured_bound + 0.00001);
        m.accuracy.vertical_position->value = 0.05001F;
        m.accuracy.horizontal_velocity->value = 0.03001F;
        m.accuracy.horizontal_position->descriptor.source = "v8.bounded_measurement_adapter";
        const bool in_region = local.east_m >= gap_begin && local.east_m <= gap_end;
        if (in_region && fault == "gnss_outage") {
            m.validity.position = false;
            fault_realized = true;
        }
        if (in_region && fault == "estimator_invalid") {
            m.position_frame = core::CoordinateFrame::kUnknown;
            fault_realized = true;
        }
        if (fault == "unbounded") {
            m.accuracy.horizontal_position->descriptor.semantics =
                core::UncertaintySemantics::kStandardDeviation;
            fault_realized = true;
        }
        {
            std::lock_guard lock(sensor_mutex);
            measurement = m;
            const auto measured_local =
                edge::Wgs84ToLocal(sim.home_lat_deg, sim.home_lon_deg, m.lat_deg, m.lon_deg);
            measurements << clock_ns->load() / 1'000'000 - 1000 << ',' << measured_local.east_m
                         << ',' << measured_local.north_m << ',' << m.rel_alt_m << ',' << m.vx_mps
                         << ',' << m.vy_mps << ',' << m.validity.position << ','
                         << (m.position_frame == core::CoordinateFrame::kWgs84) << ','
                         << (m.accuracy.horizontal_position->descriptor.semantics ==
                             core::UncertaintySemantics::kDeterministicHardBound)
                         << ',' << m.accuracy.horizontal_position->value << '\n';
        }
        return m;
    };
    auto execute_filter =
        [&](const commands::CommandEnvelope& e) -> std::optional<core::BackendCommandOutcome> {
        const auto* nav = std::get_if<commands::NavCmd>(&e.command);
        if (nav && std::get_if<commands::CmdGoto>(nav) && gotos.load() > 0 &&
            fault == "command_rejected") {
            fault_realized = true;
            return core::BackendCommandOutcome{
                .result = core::Result::Rejected("v8 scheduled recovery refusal"),
                .dispatch_state = core::BackendDispatchState::kRejected};
        }
        return std::nullopt;
    };
    auto command_observer = [&](const commands::CommandEnvelope& e,
                                const core::BackendCommandOutcome& outcome) {
        const auto* nav = std::get_if<commands::NavCmd>(&e.command);
        if (!nav)
            return;
        const auto* go = std::get_if<commands::CmdGoto>(nav);
        if (!go)
            return;
        const auto target =
            edge::Wgs84ToLocal(sim.home_lat_deg, sim.home_lon_deg, go->lat_deg, go->lon_deg);
        std::lock_guard lock(event_mutex);
        if (outcome.IsOk())
            ++gotos;
        else
            ++rejects;
        const auto time_ms = clock_ns->load() / 1'000'000 - 1000;
        events << time_ms << ',' << (outcome.IsOk() ? "goto_accepted" : "goto_rejected") << ','
               << target.east_m << ',' << target.north_m << ',' << Quote(outcome.result.message)
               << '\n';
        if (target.east_m < length - 1e-3) {
            const auto truth = control->Truth("drone-1");
            if (outcome.IsOk() && truth && truth->east_m < length - 2) {
                ++midflight_repairs;
                if (first_repair_ms < 0) {
                    first_repair_ms = time_ms;
                    first_repair_east = truth->east_m;
                }
            }
        }
    };
    const auto cert = MakeDevMtlsPaths();
    agent::AgentConfig cfg;
    cfg.agent_id = "v8-agent";
    cfg.data.artifact_dir = (output / "agent_artifacts").string();
    cfg.vecr_attempt_journal_path = (output / "attempt.journal").string();
    cfg.bind_addr = "127.0.0.1:0";
    cfg.security.root_ca_cert_path = cert.root_ca_cert_path;
    cfg.security.cert_chain_path = cert.server_cert_chain_path;
    cfg.security.private_key_path = cert.server_private_key_path;
    cfg.vecr_sensor_client_id = "test-client";
    cfg.vecr_sensor_calibration_id = "v8-calibration";
    cfg.vecr_sim_recovery_enabled = true;
    cfg.default_telemetry_rate_hz = 10;
    auto providers = agent::internal::RuntimeProviders::System();
    providers.monotonic_time_ns = [clock_ns] { return clock_ns->load(); };
    std::shared_ptr<agent::IDroneBackend> backend = std::move(instance->backend);
    AgentServerHarness harness(
        cfg,
        std::make_unique<SharedBackendHandle>(backend, execute_filter, command_observer, filter),
        providers);
    auto client_config = MakeMtlsClientConfig(harness.Address());
    client_config.retry_policy.max_attempts = 1;
    client::SwarmClient routed(client_config);
    if (client_kind == "routed")
        routed.AddDrone("drone-1", harness.Address());
    client::Client client(std::move(client_config));
    auto subscription =
        client.StartReports({.drone_id = "drone-1"}, [&](const client::AgentReport& r) {
            if (!r.edge)
                return;
            ++report_count;
            std::lock_guard lock(event_mutex);
            events << clock_ns->load() / 1'000'000 - 1000 << ",report,,,"
                   << Quote(Phase(r.edge->status.phase) + ":" + r.edge->status.reason) << '\n';
        });
    if (!subscription)
        throw std::runtime_error("report subscription failed");
    client::EdgeOperationRequest req;
    req.edge.mission_id = "v8-graph-edge";
    req.edge.mission_revision = 7;
    req.edge.edge_id = "A-B";
    req.edge.attempt_id = scenario + "-" + std::to_string(seed) + "-" + policy;
    req.edge.drone_id = "drone-1";
    req.edge.route.points = {{0, 0}, {length, 0}};
    req.edge.obligation = edge::Obligation::kSurvey;
    req.edge.corridor_half_width_m = 4;
    req.edge.arrival_radius_m = 2;
    req.edge.arrival_max_speed_mps = 0.5;
    req.edge.maximum_speed_mps = 5;
    req.edge.maximum_capture_age_ms = static_cast<std::int64_t>(num("max-age", 3) * 1000);
    req.edge.endpoint_evidence_wait_ms = static_cast<std::int64_t>(num("wait", 3) * 1000);
    req.edge.max_recovery_attempts = static_cast<std::uint32_t>(attempts);
    req.edge.sustained_deviation_samples = 3;
    req.edge.deviation_hysteresis_m = 0.5;
    req.edge.recovery_cooldown_ms = 1000;
    req.edge.timeout_ms = static_cast<std::int64_t>(num("timeout", 90) * 1000);
    req.wgs84_origin = {sim.home_lat_deg, sim.home_lon_deg};
    req.altitude_m = 10;
    req.commanded_speed_mps = 2;
    const auto start_wall = Clock::now();
    auto get_status = [&] {
        return client_kind == "routed" ? routed.GetEdgeStatus("drone-1", req.edge.attempt_id)
                                       : client.GetEdgeStatus("drone-1", req.edge.attempt_id);
    };
    const auto dispatch =
        client_kind == "routed" ? routed.ExecuteEdge(req) : client.ExecuteEdge(req);
    if (!dispatch.ok)
        throw std::runtime_error("edge dispatch failed: " + dispatch.message);
    record = true;
    sensing::CameraEstimator estimator(sim.home_lat_deg, sim.home_lon_deg, along, cross);
    std::deque<Pending> pending;
    edge::Status last;
    std::string last_phase;
    int capture_count = 0, accepted = 0, submitted = 0, uncontained = 0, unknown_ticks = 0,
        deviating_ticks = 0, reentry_completions = 0;
    double max_latency = 0;
    std::vector<double> latencies;
    int elapsed_ms = 0;
    const double optic_scale = 1 + 0.02 * Unit(seed, 0, 8);
    for (int tick = 0; tick < max_ticks; ++tick) {
        const double elapsed = static_cast<double>(tick) / 10;
        if (fault == "motor_failure" && last.recovery_attempts > 0 && !motor_failed) {
            control->SetFailure("drone-1", true, "v8 recovery motor failure");
            motor_failed = true;
            fault_realized = true;
            std::lock_guard lock(event_mutex);
            events << elapsed_ms << ",motor_failure,,,\"actual simulator failure\"\n";
        }

        const bool wind_case = fault == "gust" || fault == "crosswind" || fault == "departure" ||
                               fault == "visible" || fault == "hidden" ||
                               fault == "command_rejected" || fault == "motor_failure" ||
                               fault == "no_repair";
        const bool wind_active =
            wind_case && elapsed >= wind_begin && (fault == "crosswind" || elapsed < wind_end);
        control->SetWind("drone-1", wind_active ? wind : 0, 0);
        if (wind_active)
            fault_realized = true;
        clock_ns->fetch_add(100'000'000);
        if (!control->Advance("drone-1", 100ms).IsOk())
            throw std::runtime_error("advance failed");
        const auto now = clock_ns->load() / 1'000'000;
        client::EdgeResult reply;
        if (!WaitUntil(
                [&] {
                    reply = get_status();
                    return reply.status && (Terminal(reply.status->status.phase) ||
                                            reply.status->status.last_position_monotonic_ms == now);
                },
                2000ms, 1ms))
            throw std::runtime_error("agent did not consume scheduled telemetry");
        last = reply.status->status;
        const auto truth = control->Truth("drone-1");
        if (!truth)
            throw std::runtime_error("missing private truth");
        elapsed_ms = static_cast<int>(truth->simulation_time_ms - 3000);
        const int period_ticks = static_cast<int>(std::round(period * 10));
        if (tick % period_ticks == 0) {
            const bool in_gap =
                truth->east_m + along >= gap_begin && truth->east_m - along <= gap_end;
            const bool in_two_gaps =
                (truth->east_m + along >= length * 0.2 && truth->east_m - along <= length * 0.3) ||
                (truth->east_m + along >= length * 0.65 && truth->east_m - along <= length * 0.75);
            const bool camera_outage = ((fault == "multi_gap" || fault == "uncertainty_gap") &&
                                        in_two_gaps && elapsed < length / 2 + 3) ||
                                       (fault == "persistent_sensor" && in_gap);
            bool acquired = !truth->failed && !camera_outage;
            if (camera_outage)
                fault_realized = true;
            if ((fault == "metadata_gap" || fault == "false_frame") && in_gap &&
                elapsed < length / 2 + 3) {
                if (fault == "false_frame")
                    acquired = false;
                fault_realized = true;
            }
            const bool angled = fault == "declared_angle" || fault == "undeclared_angle";
            const double yaw = angled ? 45.0 : 0;
            if (angled)
                fault_realized = true;
            const auto actual = acquired
                                    ? oracle->Footprint(truth->east_m, truth->north_m,
                                                        along * optic_scale * truth->up_m / 10,
                                                        cross * optic_scale * truth->up_m / 10, yaw)
                                    : std::nullopt;
            oracle->Add(actual);
            acquisitions << tick << ',' << elapsed_ms << ',' << truth->east_m << ','
                         << truth->north_m << ',' << truth->up_m << ','
                         << along * optic_scale * truth->up_m / 10 << ','
                         << cross * optic_scale * truth->up_m / 10 << ',' << yaw << ',' << acquired
                         << ',';
            if (actual)
                acquisitions << actual->first << ',' << actual->second;
            else
                acquisitions << ',';
            acquisitions << '\n';
            if (acquired)
                ++capture_count;
            std::optional<core::TelemetryFrame> measured;
            {
                std::lock_guard lock(sensor_mutex);
                measured = measurement;
            }
            const bool missing = fault == "metadata_gap" && in_gap && elapsed < length / 2 + 3;
            const bool bad_cal = fault == "bad_calibration";
            if (bad_cal || fault == "stale")
                fault_realized = true;
            auto input =
                measured ? estimator.Estimate(*measured, now, acquired || fault == "false_frame",
                                              fault != "declared_angle",
                                              bad_cal ? "wrong-calibration" : "v8-calibration")
                         : std::nullopt;
            if (input && !missing) {
                input->mission_id = req.edge.mission_id;
                input->mission_revision = 7;
                input->edge_id = "A-B";
                input->attempt_id = req.edge.attempt_id;
                input->capture_id = "frame-" + std::to_string(tick);
                const int delivery_delay =
                    fault == "reordered"
                        ? static_cast<int>(Unit(seed, static_cast<std::uint64_t>(tick), 31) * 26)
                        : delay;
                pending.push_back({tick + delivery_delay, *input, acquired, actual});
                std::stable_sort(pending.begin(), pending.end(), [](const auto& a, const auto& b) {
                    return a.due_tick < b.due_tick;
                });
            } else {
                captures << tick << ',' << now << ',' << now << ',' << acquired << ",0,0,"
                         << Quote(missing ? "metadata intentionally absent"
                                          : "estimator lacks bounded valid pose")
                         << ",,,,,,,,,0\n";
            }
        }
        while (!pending.empty() && pending.front().due_tick <= tick) {
            const auto item = pending.front();
            pending.pop_front();
            const auto begin = Clock::now();
            const auto response = client.SubmitEdgeCapture("drone-1", item.input);
            const double us =
                std::chrono::duration<double, std::micro>(Clock::now() - begin).count();
            latencies.push_back(us);
            max_latency = std::max(max_latency, us);
            ++submitted;
            if (response.ok)
                ++accepted;
            const auto claim = edge::BuildNadirStripEvidence(req.edge.route, 4, item.input);
            const bool contained =
                claim && item.actual &&
                claim->guaranteed_full_strip_interval.begin_m >= item.actual->first - 1e-6 &&
                claim->guaranteed_full_strip_interval.end_m <= item.actual->second + 1e-6;
            if (response.ok && !contained)
                ++uncontained;
            captures << tick << ',' << item.input.monotonic_ms << ',' << now << ',' << item.acquired
                     << ",1," << response.ok << ',' << Quote(response.message) << ','
                     << item.input.camera_center.east_m << ',' << item.input.camera_center.north_m
                     << ',' << item.input.horizontal_error_hard_bound_m << ',';
            if (claim)
                captures << claim->guaranteed_full_strip_interval.begin_m << ','
                         << claim->guaranteed_full_strip_interval.end_m;
            else
                captures << ',';
            captures << ',';
            if (item.actual)
                captures << item.actual->first << ',' << item.actual->second;
            else
                captures << ',';
            captures << ',' << contained << ',' << us << '\n';
        }
        reply = get_status();
        if (!reply.status)
            throw std::runtime_error("status absent");
        last = reply.status->status;
        if (fault == "low_battery" && last.phase == edge::Phase::kFailed &&
            last.reason.find("battery") != std::string::npos)
            fault_realized = true;
        const auto phase = Phase(last.phase);
        if (phase != last_phase) {
            std::lock_guard lock(event_mutex);
            events << elapsed_ms << ",phase,,,"
                   << Quote(last_phase + "->" + phase + ":" + last.reason) << '\n';
            if (last_phase == "RECOVERING" && phase == "MOVING")
                ++reentry_completions;
            last_phase = phase;
        }
        if (last.phase == edge::Phase::kUnknown)
            ++unknown_ticks;
        if (last.phase == edge::Phase::kDeviating)
            ++deviating_ticks;
        trace << tick << ',' << elapsed_ms << ',' << truth->east_m << ',' << truth->north_m << ','
              << truth->up_m << ','
              << std::hypot(truth->velocity_east_mps, truth->velocity_north_mps) << ',' << distance
              << ',' << oracle->Covered() << ',' << truth_at_b << ',' << phase << ','
              << Quote(last.reason) << ',' << last.survey_covered_m << ',' << last.at_b_verified
              << ',' << last.recovery_attempts << ',' << Quote(Gaps(last)) << '\n';
        if (Terminal(last.phase))
            break;
    }
    for (const auto& item : pending) {
        captures << -1 << ',' << item.input.monotonic_ms << ",," << item.acquired
                 << ",0,0,\"delivery pending at attempt termination\",,,,,,,,,0\n";
    }
    record = false;
    client.StopReports();
    const double wall = std::chrono::duration<double>(Clock::now() - start_wall).count();
    struct rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    const double cpu = static_cast<double>(usage.ru_utime.tv_sec + usage.ru_stime.tv_sec) +
                       static_cast<double>(usage.ru_utime.tv_usec + usage.ru_stime.tv_usec) / 1e6;
    double rss = static_cast<double>(usage.ru_maxrss);
#ifndef __APPLE__
    rss *= 1024;
#endif
    std::sort(latencies.begin(), latencies.end());
    auto quantile = [&](double q) {
        return latencies.empty() ? 0.0
                                 : latencies[static_cast<std::size_t>(
                                       q * static_cast<double>(latencies.size() - 1))];
    };
    summary << "client_kind,scenario,seed,policy,fault,phase,reason,truth_survey,truth_at_b,truth_"
               "complete,"
               "reported_arrived,reported_complete,survey_verified,false_complete,false_negative,"
               "unresolved_m,distance_m,time_ms,max_cross_m,"
               "truth_covered_m,reported_covered_m,missing,recoveries,midflight_repairs,first_"
               "repair_ms,first_repair_east,reentry_completions,gotos,rejected_commands,captures,"
               "submitted,accepted,accepted_uncontained,pending_captures,unknown_ticks,deviating_"
               "ticks,fault_realized,wall_s,cpu_s,peak_rss_bytes,reports,capture_latency_p50_us,"
               "capture_latency_p95_us,capture_latency_max_us";
    for (const double cell : {0.1, 0.25, 0.5, 1.0})
        summary << ",raster_full_" << cell << ",raster_center_" << cell;
    summary << '\n';
    const bool true_complete = oracle->Complete() && truth_at_b;
    const bool reported_arrived = last.phase == edge::Phase::kArrived;
    const bool reported = reported_arrived && last.survey_verified;
    summary << client_kind << ',' << scenario << ',' << seed << ',' << policy << ',' << fault << ','
            << Phase(last.phase) << ',' << Quote(last.reason) << ',' << oracle->Complete() << ','
            << truth_at_b << ',' << true_complete << ',' << reported_arrived << ',' << reported
            << ',' << last.survey_verified << ',' << (reported && !true_complete) << ','
            << (true_complete && !reported) << ',' << length - last.survey_covered_m << ','
            << distance << ',' << elapsed_ms << ',' << max_cross << ',' << oracle->Covered() << ','
            << last.survey_covered_m << ',' << Quote(Gaps(last)) << ',' << last.recovery_attempts
            << ',' << midflight_repairs << ',' << first_repair_ms << ',' << first_repair_east << ','
            << reentry_completions << ',' << gotos << ',' << rejects << ',' << capture_count << ','
            << submitted << ',' << accepted << ',' << uncontained << ',' << pending.size() << ','
            << unknown_ticks << ',' << deviating_ticks << ',' << fault_realized << ',' << wall
            << ',' << cpu << ',' << rss << ',' << report_count << ',';
    if (!latencies.empty())
        summary << quantile(0.5) << ',' << quantile(0.95) << ',' << max_latency;
    else
        summary << ",,";
    for (const double cell : {0.1, 0.25, 0.5, 1.0})
        summary << ',' << oracle->RasterFraction(cell, false) << ','
                << oracle->RasterFraction(cell, true);
    summary << '\n';
    std::cout << scenario << ' ' << seed << ' ' << policy << ' ' << Phase(last.phase)
              << " true=" << true_complete << " distance=" << distance
              << " midflight=" << midflight_repairs << " uncontained=" << uncontained << '\n';
}
}  // namespace
int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--describe") {
#if defined(SWARMKIT_RESEARCH_FULL_RESTART)
            std::cout << R"({"protocol":"v8","policy":"B2"})" << '\n';
#else
            std::cout << R"({"protocol":"v8","policy":"VECR"})" << '\n';
#endif
            return 0;
        }
        std::map<std::string, std::string> args;
        for (int i = 1; i + 1 < argc; i += 2) {
            std::string name = argv[i];
            if (!name.starts_with("--"))
                throw std::invalid_argument("expected --option value");
            args.emplace(name.substr(2), argv[i + 1]);
        }
        Run(args);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "v8 infrastructure failure: " << e.what() << '\n';
        return 1;
    }
}
