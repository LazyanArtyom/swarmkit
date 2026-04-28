// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "swarmkit/agent/sim_backend.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <exception>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

#include "swarmkit/core/logger.h"
#include "swarmkit/core/overloaded.h"

namespace swarmkit::agent {

using namespace swarmkit::commands;  // NOLINT(google-build-using-namespace)

namespace {

constexpr int kDefaultRateHz = 5;
constexpr int kMillisecondsPerSecond = 1000;
constexpr double kInitialLatDeg = 40.1811;
constexpr double kInitialLonDeg = 44.5136;
constexpr float kInitialAltMeters = 10.0F;
constexpr float kInitialBatteryPct = 95.0F;
constexpr double kGpsDriftDegPerTick = 0.00001;
constexpr float kAltClimbMPerTick = 0.02F;
constexpr float kBatteryDrainPct = 0.01F;

/**
 * @brief Built-in drone simulator for development and testing.
 *
 * Accepts every command (logs it and returns Ok).
 * Generates synthetic telemetry at the requested rate: GPS drifts slightly,
 * altitude climbs, battery drains -- just enough to verify the pipeline.
 */
class SimBackend final : public IDroneBackend {
   public:
    ~SimBackend() override {
        try {
            std::vector<std::string> drone_ids;
            {
                std::lock_guard<std::mutex> lock(telemetry_mutex_);
                drone_ids.reserve(telemetry_streams_.size());
                for (const auto& [drone_id, stream] : telemetry_streams_) {
                    static_cast<void>(stream);
                    drone_ids.push_back(drone_id);
                }
            }

            for (const auto& drone_id : drone_ids) {
                static_cast<void>(StopTelemetry(drone_id));
            }
        } catch (const std::exception& exc) {
            core::Logger::WarnFmt("SimBackend::~SimBackend failed to stop telemetry: {}",
                                  exc.what());
        } catch (...) {
            core::Logger::Warn("SimBackend::~SimBackend failed to stop telemetry");
        }
    }

    core::Result Execute(const CommandEnvelope& envelope) override {
        const CommandContext& context = envelope.context;

        std::visit(
            core::Overloaded{

                /// @name Flight commands
                /// @{
                [&](const FlightCmd& flight) {
                    std::visit(
                        core::Overloaded{
                            [&](const CmdArm&) {
                                core::Logger::InfoFmt("SimBackend: ARM  drone={}",
                                                      context.drone_id);
                            },
                            [&](const CmdDisarm&) {
                                core::Logger::InfoFmt("SimBackend: DISARM  drone={}",
                                                      context.drone_id);
                            },
                            [&](const CmdTakeoff& takeoff_cmd) {
                                core::Logger::InfoFmt("SimBackend: TAKEOFF alt_m={}  drone={}",
                                                      takeoff_cmd.alt_m, context.drone_id);
                            },
                            [&](const CmdLand&) {
                                core::Logger::InfoFmt("SimBackend: LAND  drone={}",
                                                      context.drone_id);
                            },
                            [&](const CmdSetMode& mode) {
                                core::Logger::InfoFmt(
                                    "SimBackend: SET_MODE mode={} custom={}  drone={}", mode.mode,
                                    mode.custom_mode, context.drone_id);
                            },
                            [&](const CmdForceDisarm&) {
                                core::Logger::WarnFmt("SimBackend: FORCE_DISARM  drone={}",
                                                      context.drone_id);
                            },
                            [&](const CmdFlightTerminate&) {
                                core::Logger::WarnFmt("SimBackend: FLIGHT_TERMINATE  drone={}",
                                                      context.drone_id);
                            },
                        },
                        flight);
                },
                /// @}

                /// @name Navigation commands
                /// @{
                [&](const NavCmd& nav) {
                    std::visit(
                        core::Overloaded{
                            [&](const CmdSetWaypoint& waypoint_cmd) {
                                core::Logger::InfoFmt(
                                    "SimBackend: WAYPOINT lat={} lon={} alt={}m  drone={}",
                                    waypoint_cmd.lat_deg, waypoint_cmd.lon_deg, waypoint_cmd.alt_m,
                                    context.drone_id);
                            },
                            [&](const CmdReturnHome&) {
                                core::Logger::InfoFmt("SimBackend: RETURN_HOME  drone={}",
                                                      context.drone_id);
                            },
                            [&](const CmdHoldPosition&) {
                                core::Logger::InfoFmt("SimBackend: HOLD  drone={}",
                                                      context.drone_id);
                            },
                            [&](const CmdSetSpeed& speed) {
                                core::Logger::InfoFmt(
                                    "SimBackend: SET_SPEED ground={}m/s  drone={}",
                                    speed.ground_mps, context.drone_id);
                            },
                            [&](const CmdGoto& go_to) {
                                core::Logger::InfoFmt(
                                    "SimBackend: GOTO lat={} lon={} alt={}m speed={}  "
                                    "drone={}",
                                    go_to.lat_deg, go_to.lon_deg, go_to.alt_m, go_to.speed_mps,
                                    context.drone_id);
                            },
                            [&](const CmdPause&) {
                                core::Logger::InfoFmt("SimBackend: PAUSE  drone={}",
                                                      context.drone_id);
                            },
                            [&](const CmdResume&) {
                                core::Logger::InfoFmt("SimBackend: RESUME  drone={}",
                                                      context.drone_id);
                            },
                            [&](const CmdSetYaw& yaw) {
                                core::Logger::InfoFmt(
                                    "SimBackend: SET_YAW yaw={} rate={} relative={}  "
                                    "drone={}",
                                    yaw.yaw_deg, yaw.rate_deg_s, yaw.relative, context.drone_id);
                            },
                            [&](const CmdVelocity& velocity) {
                                core::Logger::InfoFmt(
                                    "SimBackend: VELOCITY vx={} vy={} vz={} duration={} "
                                    "body={}  drone={}",
                                    velocity.vx_mps, velocity.vy_mps, velocity.vz_mps,
                                    velocity.duration_ms, velocity.body_frame, context.drone_id);
                            },
                            [&](const CmdSetHome& home) {
                                core::Logger::InfoFmt(
                                    "SimBackend: SET_HOME current={} lat={} lon={} alt={}  "
                                    "drone={}",
                                    home.use_current, home.lat_deg, home.lon_deg, home.alt_m,
                                    context.drone_id);
                            },
                        },
                        nav);
                },
                /// @}

                /// @name Mission commands
                /// @{
                [&](const MissionCmd& mission) {
                    std::visit(core::Overloaded{
                                   [&](const CmdUploadMission& upload) {
                                       core::Logger::InfoFmt(
                                           "SimBackend: UPLOAD_MISSION items={}  drone={}",
                                           upload.items.size(), context.drone_id);
                                   },
                                   [&](const CmdClearMission&) {
                                       core::Logger::InfoFmt("SimBackend: CLEAR_MISSION  drone={}",
                                                             context.drone_id);
                                   },
                                   [&](const CmdStartMission& start) {
                                       core::Logger::InfoFmt(
                                           "SimBackend: START_MISSION first={} last={}  drone={}",
                                           start.first_item, start.last_item, context.drone_id);
                                   },
                                   [&](const CmdPauseMission&) {
                                       core::Logger::InfoFmt("SimBackend: PAUSE_MISSION  drone={}",
                                                             context.drone_id);
                                   },
                                   [&](const CmdResumeMission&) {
                                       core::Logger::InfoFmt("SimBackend: RESUME_MISSION  drone={}",
                                                             context.drone_id);
                                   },
                                   [&](const CmdSetCurrentMissionItem& current) {
                                       core::Logger::InfoFmt(
                                           "SimBackend: SET_CURRENT_MISSION_ITEM seq={}  drone={}",
                                           current.seq, context.drone_id);
                                   },
                               },
                               mission);
                },
                /// @}

                /// @name Swarm commands
                /// @{
                [&](const SwarmCmd& swarm) {
                    std::visit(core::Overloaded{
                                   [&](const CmdSetRole& role_cmd) {
                                       core::Logger::InfoFmt(
                                           "SimBackend: SET_ROLE role={}  drone={}", role_cmd.role,
                                           context.drone_id);
                                   },
                                   [&](const CmdSetFormation& formation_cmd) {
                                       core::Logger::InfoFmt(
                                           "SimBackend: SET_FORMATION id={} slot={}  drone={}",
                                           formation_cmd.formation_id, formation_cmd.slot_index,
                                           context.drone_id);
                                   },
                                   [&](const CmdRunSequence& sequence_cmd) {
                                       core::Logger::InfoFmt(
                                           "SimBackend: RUN_SEQUENCE id={} sync_ms={}  drone={}",
                                           sequence_cmd.sequence_id, sequence_cmd.sync_unix_ms,
                                           context.drone_id);
                                   },
                               },
                               swarm);
                },
                /// @}

                /// @name Payload commands
                /// @{
                [&](const PayloadCmd& payload) {
                    std::visit(
                        [&](const auto&) {
                            core::Logger::InfoFmt("SimBackend: PAYLOAD command  drone={}",
                                                  context.drone_id);
                        },
                        payload);
                },
                /// @}

            },
            envelope.command);

        return core::Result::Success();
    }

    core::Result StartTelemetry(const std::string& drone_id, int rate_hertz,
                                TelemetryCallback callback) override {
        auto stream = std::make_shared<TelemetryStream>();

        {
            std::lock_guard<std::mutex> lock(telemetry_mutex_);
            if (telemetry_streams_.contains(drone_id)) {
                return core::Result::Rejected("telemetry already running for drone '" + drone_id +
                                              "'");
            }
            telemetry_streams_.emplace(drone_id, stream);
        }

        const int kEffectiveRate = (rate_hertz <= 0) ? kDefaultRateHz : rate_hertz;

        stream->worker = std::thread([stream, drone_id,  // NOLINT(bugprone-exception-escape)
                                      kEffectiveRate, callback = std::move(callback)]() noexcept {
            try {
                using std::chrono::milliseconds;
                using std::chrono::steady_clock;
                using std::chrono::system_clock;
                using std::chrono::duration_cast;

                auto next_tick = steady_clock::now();
                double lat_deg = kInitialLatDeg;
                double lon_deg = kInitialLonDeg;
                float alt_m = kInitialAltMeters;
                float battery_pct = kInitialBatteryPct;

                const auto kPeriod =
                    milliseconds(kMillisecondsPerSecond / std::max(1, kEffectiveRate));

                while (stream->running.load(std::memory_order_relaxed)) {
                    next_tick += kPeriod;

                    lat_deg += kGpsDriftDegPerTick;
                    lon_deg += kGpsDriftDegPerTick;
                    alt_m += kAltClimbMPerTick;
                    battery_pct = std::max(0.0F, battery_pct - kBatteryDrainPct);

                    core::TelemetryFrame frame;
                    frame.drone_id = drone_id;
                    frame.unix_time_ms =
                        duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
                    frame.lat_deg = lat_deg;
                    frame.lon_deg = lon_deg;
                    frame.rel_alt_m = alt_m;
                    frame.battery_percent = battery_pct;
                    frame.mode = "SIM";

                    callback(frame);

                    std::this_thread::sleep_until(next_tick);
                }
            } catch (...) {
                static_cast<void>(0);
            }
        });

        return core::Result::Success();
    }

    core::Result StopTelemetry(const std::string& drone_id) override {
        std::shared_ptr<TelemetryStream> stream;
        {
            std::lock_guard<std::mutex> lock(telemetry_mutex_);
            auto iter = telemetry_streams_.find(drone_id);
            if (iter == telemetry_streams_.end()) {
                return core::Result::Success();
            }
            stream = std::move(iter->second);
            telemetry_streams_.erase(iter);
        }

        if (!stream) {
            return core::Result::Success();
        }

        stream->running.store(false, std::memory_order_relaxed);
        if (stream->worker.joinable()) {
            stream->worker.join();
        }
        return core::Result::Success();
    }

   private:
    struct TelemetryStream {
        std::atomic<bool> running{true};
        std::thread worker;
    };

    std::mutex telemetry_mutex_;
    std::unordered_map<std::string, std::shared_ptr<TelemetryStream>> telemetry_streams_;
};

}  // namespace

DroneBackendPtr MakeSimBackend() {
    return std::make_unique<SimBackend>();
}

}  // namespace swarmkit::agent
