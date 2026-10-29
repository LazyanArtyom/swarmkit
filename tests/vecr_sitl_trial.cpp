// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
// ArduPilot qualification harness. Synthetic navigation/camera evidence is explicit.
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
#include <sstream>
#include <thread>

#include "swarmkit/agent/mavlink_backend.h"
#include "swarmkit/client/client.h"
#include "test_support.h"
#include "vecr_sensor_models.h"
namespace {
using namespace swarmkit;
using namespace swarmkit::testsupport;
using namespace std::chrono_literals;
std::int64_t Now() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
struct Truth {
    double ms{}, lat{}, lon{}, alt{}, vn{}, ve{}, vd{}, roll{}, pitch{}, yaw{};
    std::uint64_t seq{};
};
std::optional<Truth> ReadTruth(const std::filesystem::path& path) {
    std::ifstream in(path);
    Truth t;
    if (in >> t.ms >> t.lat >> t.lon >> t.alt >> t.vn >> t.ve >> t.vd >> t.roll >> t.pitch >>
        t.yaw >> t.seq)
        return t;
    return std::nullopt;
}
struct Sample {
    core::TelemetryFrame measured;
    Truth truth;
    std::int64_t time_ms{};
};
class SitlAdapter final : public agent::IDroneBackend {
   public:
    SitlAdapter(std::unique_ptr<agent::IDroneBackend> raw, std::filesystem::path root,
                bool synthetic, std::string fault, std::uint64_t seed)
        : raw_(std::move(raw)),
          root_(std::move(root)),
          synthetic_(synthetic),
          fault_(std::move(fault)),
          seed_(seed),
          commands_(root_ / "commands.csv"),
          observations_(root_ / "observations.csv") {
        commands_ << "time_ms,target_east,target_north,accepted\n";
        observations_ << "time_ms,truth_time_ms,truth_east,truth_north,measured_east,measured_"
                         "north,hard_bound,synthetic\n";
        commands_ << std::setprecision(17);
        observations_ << std::setprecision(17);
    }
    core::Result Start() override {
        return raw_->Start();
    }
    core::BackendCommandOutcome Execute(const commands::CommandEnvelope& c) override {
        const auto* nav = std::get_if<commands::NavCmd>(&c.command);
        const auto* go = nav ? std::get_if<commands::CmdGoto>(nav) : nullptr;
        std::optional<edge::Point> q;
        if (go)
            q = edge::Wgs84ToLocal(40, 44, go->lat_deg, go->lon_deg);
        core::BackendCommandOutcome result;
        if (q && (q->east_m < -10 || q->east_m > 40 || std::abs(q->north_m) > 15 || go->alt_m < 5 ||
                  go->alt_m > 15))
            result = {.result = core::Result::Rejected("SITL laboratory navigation box exceeded"),
                      .dispatch_state = core::BackendDispatchState::kRejected};
        else if (go && gotos_.load() > 0 && fault_ == "command_rejected")
            result = {.result = core::Result::Rejected("injected SITL backend recovery refusal"),
                      .dispatch_state = core::BackendDispatchState::kRejected};
        else
            result = raw_->Execute(c);
        if (q) {
            std::lock_guard lock(mutex_);
            commands_ << Now() << ',' << q->east_m << ',' << q->north_m << ',' << result.IsOk()
                      << '\n';
            commands_.flush();
            if (result.IsOk())
                ++gotos_;
        }
        return result;
    }
    core::Result StartTelemetry(const std::string& id, int hz, TelemetryCallback cb) override {
        return raw_->StartTelemetry(
            id, hz, [this, cb = std::move(cb)](const core::TelemetryFrame& raw) {
                auto m = raw;
                const auto now = Now();
                const auto t = ReadTruth(root_ / "truth_snapshot.txt");
                if (!t || now - t->ms < 0 || now - t->ms > 100) {
                    m.validity.position = false;
                    cb(m);
                    return;
                }
                const auto actual = edge::Wgs84ToLocal(40, 44, t->lat, t->lon);
                if (synthetic_) {
                    const double noise = .08 * std::sin(static_cast<double>(seed_) +
                                                        static_cast<double>(t->seq) * .37);
                    const auto geo =
                        edge::LocalToWgs84(40, 44, {actual.east_m + noise, actual.north_m + .06});
                    m.lat_deg = geo.lat_deg;
                    m.lon_deg = geo.lon_deg;
                    m.rel_alt_m = static_cast<float>(t->alt - 100 + .02);
                    m.vx_mps = static_cast<float>(t->vn);
                    m.vy_mps = static_cast<float>(t->ve);
                    m.position_frame = core::CoordinateFrame::kWgs84;
                    m.velocity_frame = core::CoordinateFrame::kLocalNed;
                    m.validity.position = m.validity.relative_altitude = m.validity.velocity = true;
                    // Laboratory assumption: SIM_STATE transport age <=50 ms and speed <=10 m/s.
                    // Measured post-run against independent truth; not an aircraft accuracy claim.
                    const core::UncertaintyDescriptor d{
                        .semantics = core::UncertaintySemantics::kDeterministicHardBound,
                        .source = "sitl.synthetic_navigation_with_declared_timing_envelope"};
                    m.accuracy.horizontal_position = core::UncertaintyEstimate{
                        .value = static_cast<float>(.7 + 10 * (now - t->ms) / 1000),
                        .descriptor = d};
                    m.accuracy.vertical_position =
                        core::UncertaintyEstimate{.value = .3F, .descriptor = d};
                    m.accuracy.horizontal_velocity =
                        core::UncertaintyEstimate{.value = .1F, .descriptor = d};
                }
                {
                    std::lock_guard lock(mutex_);
                    sample_ = Sample{m, *t, now};
                    const auto measured = edge::Wgs84ToLocal(40, 44, m.lat_deg, m.lon_deg);
                    observations_ << now << ',' << t->ms << ',' << actual.east_m << ','
                                  << actual.north_m << ',' << measured.east_m << ','
                                  << measured.north_m << ',';
                    if (synthetic_)
                        observations_ << m.accuracy.horizontal_position->value;
                    observations_ << ',' << synthetic_ << '\n';
                    observations_.flush();
                }
                cb(m);
            });
    }
    core::Result StopTelemetry(const std::string& id) override {
        return raw_->StopTelemetry(id);
    }
    agent::BackendHealth GetHealth() const override {
        return raw_->GetHealth();
    }
    core::BackendCapabilities GetCapabilities() const override {
        auto c = raw_->GetCapabilities();
        if (synthetic_)
            c.backend_name = "ardupilot-sitl";
        return c;
    }
    void SetEvidenceCallback(const EvidenceCallback& cb) override {
        raw_->SetEvidenceCallback(cb);
    }
    std::optional<Sample> Latest() {
        std::lock_guard lock(mutex_);
        return sample_;
    }

   private:
    std::unique_ptr<agent::IDroneBackend> raw_;
    std::filesystem::path root_;
    bool synthetic_;
    std::string fault_;
    std::uint64_t seed_;
    std::atomic<int> gotos_{0};
    std::mutex mutex_;
    std::optional<Sample> sample_;
    std::ofstream commands_, observations_;
};
std::string Phase(edge::Phase p) {
    switch (p) {
        case edge::Phase::kArrived:
            return "ARRIVED";
        case edge::Phase::kFailed:
            return "FAILED";
        case edge::Phase::kCancelled:
            return "CANCELLED";
        case edge::Phase::kRecovering:
            return "RECOVERING";
        case edge::Phase::kMoving:
            return "MOVING";
        case edge::Phase::kDeviating:
            return "DEVIATING";
        case edge::Phase::kUnknown:
            return "UNKNOWN";
    }
    return "INVALID";
}
int Run(const std::map<std::string, std::string>& a) {
    auto get = [&](const std::string& k, const std::string& d) {
        const auto i = a.find(k);
        return i == a.end() ? d : i->second;
    };
    const auto root = std::filesystem::path(get("output", ""));
    const auto policy = get("policy", "VECR"), fault = get("fault", "none");
    const bool synthetic = get("navigation", "synthetic") == "synthetic";
    if (root.empty() || (policy != "VECR"))
        throw std::runtime_error("invalid SITL options");
    agent::MavlinkBackendConfig mc;
    mc.bind_addr = get("bind", "127.0.0.1:14601");
    mc.telemetry_rate_hz = 20;
    mc.peer_discovery_timeout_ms = 10000;
    auto raw = agent::MakeMavlinkBackend(mc);
    if (!raw)
        throw std::runtime_error(raw.error().message);
    auto decorated = std::make_unique<SitlAdapter>(std::move(*raw), root, synthetic, fault,
                                                   std::stoull(get("seed", "1")));
    auto* producer = decorated.get();
    const auto cert = MakeDevMtlsPaths();
    agent::AgentConfig cfg;
    cfg.agent_id = "v7-sitl";
    cfg.bind_addr = "127.0.0.1:0";
    cfg.data.artifact_dir = (root / "agent_artifacts").string();
    cfg.vecr_attempt_journal_path = (root / "attempt.journal").string();
    cfg.default_telemetry_rate_hz = 20;
    cfg.security.root_ca_cert_path = cert.root_ca_cert_path;
    cfg.security.cert_chain_path = cert.server_cert_chain_path;
    cfg.security.private_key_path = cert.server_private_key_path;
    cfg.vecr_sensor_client_id = "test-client";
    cfg.vecr_sensor_calibration_id = "sitl-ideal-nadir";
    cfg.vecr_sim_recovery_enabled = true;
    AgentServerHarness harness(cfg, std::move(decorated));
    client::Client client(MakeMtlsClientConfig(harness.Address()));
    if (!WaitUntil(
            [&] {
                auto h = producer->GetHealth();
                return h.ready && h.armed == true && h.gps_ok == true && h.ekf_ok == true &&
                       h.mode == "GUIDED";
            },
            15s, 100ms)) {
        const auto h = producer->GetHealth();
        throw std::runtime_error("SITL backend not ready: " + h.message + " mode=" + h.mode);
    }
    client::EdgeOperationRequest req;
    req.edge.mission_id = "sitl-v6";
    req.edge.mission_revision = 7;
    req.edge.edge_id = "A-B";
    req.edge.attempt_id = "sitl-attempt";
    req.edge.drone_id = "drone-1";
    req.edge.route.points = {{0, 0}, {20, 0}};
    req.edge.obligation = edge::Obligation::kSurvey;
    req.edge.corridor_half_width_m = 4;
    req.edge.arrival_radius_m = 2;
    req.edge.arrival_max_speed_mps = .5;
    req.edge.maximum_speed_mps = 10;
    req.edge.timeout_ms = synthetic ? 90000 : 25000;
    req.edge.maximum_capture_age_ms = 3000;
    req.edge.endpoint_evidence_wait_ms = 3000;
    req.edge.max_recovery_attempts = 2;
    req.wgs84_origin = {40, 44};
    req.altitude_m = 10;
    req.commanded_speed_mps = 2;
    const auto start = Now();
    const auto dispatch = client.ExecuteEdge(req);
    if (!dispatch.ok)
        throw std::runtime_error("SITL edge rejected: " + dispatch.message);
    std::ofstream trace(root / "agent_trace.csv"), caps(root / "captures.csv"),
        summary(root / "attempt.csv"), acquisitions(root / "acquisitions.csv");
    trace << "time_ms,phase,covered_m,at_b,recoveries,reason\n";
    caps << "acquisition_ms,delivery_ms,truth_time_ms,actual_east,actual_north,actual_up,acquired,"
            "accepted,claimed_begin,claimed_end,reason\n";
    caps << std::setprecision(17);
    trace << std::setprecision(17);
    acquisitions
        << "time_ms,truth_time_ms,east_m,north_m,up_m,along_half_m,cross_half_m,acquired\n";
    acquisitions << std::setprecision(17);
    struct Pending {
        edge::NadirCaptureInput input;
        Truth truth;
        bool acquired;
        std::int64_t due;
    };
    std::deque<Pending> pending;
    sensing::CameraEstimator estimator(40, 44, 3, 8);
    edge::Status last;
    std::int64_t last_capture = -1;
    int counter = 0;
    while (Now() - start < req.edge.timeout_ms + 5000) {
        const auto now = Now();
        const auto measured = producer->Latest();
        if (measured && measured->time_ms - last_capture >= 200) {
            last_capture = measured->time_ms;
            const auto local = edge::Wgs84ToLocal(40, 44, measured->truth.lat, measured->truth.lon);
            const bool gap = (local.east_m + 3 >= 4 && local.east_m - 3 <= 6) ||
                             (local.east_m + 3 >= 14 && local.east_m - 3 <= 16);
            const bool acquired = !((fault == "multi_gap" || fault == "command_rejected") && gap &&
                                    now - start < 16000);
            acquisitions << measured->time_ms << ',' << measured->truth.ms << ',' << local.east_m
                         << ',' << local.north_m << ',' << measured->truth.alt - 100 << ','
                         << 3 * (measured->truth.alt - 100) / 10 << ','
                         << 8 * (measured->truth.alt - 100) / 10 << ',' << acquired << '\n';
            acquisitions.flush();
            auto input = estimator.Estimate(measured->measured, measured->time_ms, acquired, true,
                                            "sitl-ideal-nadir");
            if (input) {
                input->mission_id = req.edge.mission_id;
                input->mission_revision = 7;
                input->edge_id = "A-B";
                input->attempt_id = "sitl-attempt";
                input->capture_id = "capture-" + std::to_string(counter++);
                pending.push_back(
                    {*input, measured->truth, acquired, now + (fault == "delay" ? 2500 : 0)});
            }
        }
        while (!pending.empty() && pending.front().due <= now) {
            auto p = pending.front();
            pending.pop_front();
            const auto r = client.SubmitEdgeCapture("drone-1", p.input);
            const auto actual = edge::Wgs84ToLocal(40, 44, p.truth.lat, p.truth.lon);
            const auto evidence = edge::BuildNadirStripEvidence(req.edge.route, 4, p.input);
            caps << p.input.monotonic_ms << ',' << now << ',' << p.truth.ms << ',' << actual.east_m
                 << ',' << actual.north_m << ',' << p.truth.alt - 100 << ',' << p.acquired << ','
                 << r.ok << ',';
            if (evidence)
                caps << evidence->guaranteed_full_strip_interval.begin_m << ','
                     << evidence->guaranteed_full_strip_interval.end_m;
            else
                caps << ',';
            caps << ",\"" << r.message << "\"\n";
            caps.flush();
        }
        const auto r = client.GetEdgeStatus("drone-1", "sitl-attempt");
        if (!r.status)
            throw std::runtime_error("missing SITL agent status");
        last = r.status->status;
        trace << now << ',' << Phase(last.phase) << ',' << last.survey_covered_m << ','
              << last.at_b_verified << ',' << last.recovery_attempts << ",\"" << last.reason
              << "\"\n";
        trace.flush();
        if (last.phase == edge::Phase::kArrived || last.phase == edge::Phase::kFailed)
            break;
        std::this_thread::sleep_for(50ms);
    }
    summary << std::setprecision(17);
    summary << "policy,fault,navigation,start_ms,end_ms,phase,reported_complete,covered_m,"
               "recoveries,pending_captures\n";
    summary << policy << ',' << fault << ',' << (synthetic ? "synthetic_bounded" : "raw_mavlink")
            << ',' << start << ',' << Now() << ',' << Phase(last.phase) << ','
            << (last.phase == edge::Phase::kArrived) << ',' << last.survey_covered_m << ','
            << last.recovery_attempts << ',' << pending.size() << '\n';
    std::cout << Phase(last.phase) << " " << last.reason << '\n';
    return 0;
}
}  // namespace
int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--clock-ms") {
            std::cout << Now() << '\n';
            return 0;
        }
        std::map<std::string, std::string> a;
        for (int i = 1; i + 1 < argc; i += 2)
            a.emplace(std::string(argv[i]).substr(2), argv[i + 1]);
        return Run(a);
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
