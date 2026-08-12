// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include "swarmkit/agent/sim_backend.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <limits>
#include <mutex>
#include <numbers>
#include <thread>
#include <unordered_map>
#include <utility>

#include "swarmkit/core/logger.h"
#include "swarmkit/core/overloaded.h"

namespace swarmkit::agent {
namespace {

constexpr double kEarthRadiusM = 6'371'000.0;
constexpr int kDefaultTelemetryRateHz = 5;
constexpr double kArrivalEpsilonM = 1e-5;

struct Target {
    double north_m{};
    double east_m{};
    double up_m{};
    float speed_mps{};
};

struct VelocityCommand {
    double north_mps{};
    double east_mps{};
    double up_mps{};
    std::int64_t remaining_ms{};
};

struct VehicleState {
    double north_m{};
    double east_m{};
    double up_m{};
    double velocity_north_mps{};
    double velocity_east_mps{};
    double velocity_up_mps{};
    float battery_percent{};
    float cruise_speed_mps{};
    float yaw_deg{};
    bool armed{false};
    bool landed{true};
    bool failed{false};
    bool paused{false};
    std::string failure_detail;
    std::string mode{"SIM_HOLD"};
    std::int64_t simulation_time_ms{};
    std::uint64_t truth_sequence{};
    std::optional<Target> target;
    std::optional<VelocityCommand> velocity_command;
};

struct StreamState {
    std::atomic<bool> running{true};
    int rate_hertz{kDefaultTelemetryRateHz};
    int accumulated_ms{};
    IDroneBackend::TelemetryCallback callback;
    std::thread worker;
};

[[nodiscard]] double DegreesToRadians(double value) {
    return value * std::numbers::pi / 180.0;
}

[[nodiscard]] double RadiansToDegrees(double value) {
    return value * 180.0 / std::numbers::pi;
}

[[nodiscard]] std::pair<double, double> GeoToLocal(const SimBackendConfig& config, double lat_deg,
                                                   double lon_deg) {
    const double north = DegreesToRadians(lat_deg - config.home_lat_deg) * kEarthRadiusM;
    const double east = DegreesToRadians(lon_deg - config.home_lon_deg) * kEarthRadiusM *
                        std::max(0.01, std::cos(DegreesToRadians(config.home_lat_deg)));
    return {north, east};
}

[[nodiscard]] std::pair<double, double> LocalToGeo(const SimBackendConfig& config, double north_m,
                                                   double east_m) {
    const double latitude = config.home_lat_deg + RadiansToDegrees(north_m / kEarthRadiusM);
    const double longitude =
        config.home_lon_deg +
        RadiansToDegrees(
            east_m /
            (kEarthRadiusM * std::max(0.01, std::cos(DegreesToRadians(config.home_lat_deg)))));
    return {latitude, longitude};
}

[[nodiscard]] double ClampMagnitude(double value, double positive_limit, double negative_limit) {
    return std::clamp(value, -negative_limit, positive_limit);
}

[[nodiscard]] core::BackendCommandOutcome Accepted(std::string message) {
    return {
        .result = core::Result::Success(std::move(message)),
        .dispatch_state = core::BackendDispatchState::kAccepted,
    };
}

[[nodiscard]] core::BackendCommandOutcome Rejected(std::string message) {
    return {
        .result = core::Result::Rejected(std::move(message)),
        .dispatch_state = core::BackendDispatchState::kRejected,
    };
}

}  // namespace

struct SimBackendControl::State {
    explicit State(SimBackendConfig input) : config(std::move(input)) {}

    mutable std::mutex mutex;
    SimBackendConfig config;
    std::unordered_map<std::string, VehicleState> vehicles;
    std::unordered_map<std::string, std::shared_ptr<StreamState>> streams;
    IDroneBackend::EvidenceCallback evidence_callback;
    std::uint64_t evidence_sequence{};
};

namespace {

[[nodiscard]] VehicleState& Vehicle(SimBackendControl::State* state, const std::string& drone_id) {
    auto [iter, inserted] = state->vehicles.try_emplace(drone_id);
    if (inserted) {
        iter->second.battery_percent = state->config.initial_battery_percent;
        iter->second.cruise_speed_mps = state->config.default_cruise_speed_mps;
    }
    return iter->second;
}

[[nodiscard]] SimulationTruthFrame TruthFrame(const SimBackendConfig& config,
                                              const std::string& drone_id,
                                              const VehicleState& vehicle) {
    return {
        .drone_id = drone_id,
        .truth_sequence = vehicle.truth_sequence,
        .simulation_time_ms = vehicle.simulation_time_ms,
        .home_lat_deg = config.home_lat_deg,
        .home_lon_deg = config.home_lon_deg,
        .home_alt_m = config.home_alt_m,
        .north_m = vehicle.north_m,
        .east_m = vehicle.east_m,
        .up_m = vehicle.up_m,
        .velocity_north_mps = vehicle.velocity_north_mps,
        .velocity_east_mps = vehicle.velocity_east_mps,
        .velocity_up_mps = vehicle.velocity_up_mps,
        .armed = vehicle.armed,
        .landed = vehicle.landed,
        .failed = vehicle.failed,
        .mode = vehicle.mode,
    };
}

[[nodiscard]] core::TelemetryFrame TelemetryFrame(const SimBackendConfig& config,
                                                  const std::string& drone_id,
                                                  const VehicleState& vehicle) {
    const auto [lat_deg, lon_deg] = LocalToGeo(config, vehicle.north_m, vehicle.east_m);
    core::TelemetryFrame frame;
    frame.drone_id = drone_id;
    frame.lat_deg = lat_deg;
    frame.lon_deg = lon_deg;
    frame.rel_alt_m = static_cast<float>(vehicle.up_m);
    frame.abs_alt_m = static_cast<float>(config.home_alt_m + vehicle.up_m);
    frame.vx_mps = static_cast<float>(vehicle.velocity_north_mps);
    frame.vy_mps = static_cast<float>(vehicle.velocity_east_mps);
    frame.vz_mps = static_cast<float>(-vehicle.velocity_up_mps);
    frame.yaw_deg = vehicle.yaw_deg;
    frame.battery_percent = vehicle.battery_percent;
    frame.mode = vehicle.mode;
    frame.armed = vehicle.armed;
    frame.landed = vehicle.landed;
    frame.failsafe = vehicle.failed;
    frame.gps_fix_type = 3;
    frame.satellites_visible = 12;
    frame.gps_hdop = 0.7F;
    frame.link_quality_percent = 100.0F;
    frame.position_frame = core::CoordinateFrame::kWgs84;
    frame.velocity_frame = core::CoordinateFrame::kLocalNed;
    frame.validity = {
        .position = true,
        .relative_altitude = true,
        .absolute_altitude = true,
        .velocity = true,
        .attitude = true,
        .battery = true,
        .mode = true,
        .armed = true,
        .landed = true,
        .failsafe = true,
        .gps = true,
        .gps_hdop = true,
        .link_quality = true,
        .estimator = true,
        .home_origin = true,
    };
    const core::UncertaintyDescriptor exact_descriptor{
        .semantics = core::UncertaintySemantics::kDeterministicHardBound,
        .source = "sim.exact_estimator_model",
    };
    frame.accuracy.horizontal_position =
        core::UncertaintyEstimate{.value = 0.0F, .descriptor = exact_descriptor};
    frame.accuracy.vertical_position =
        core::UncertaintyEstimate{.value = 0.0F, .descriptor = exact_descriptor};
    frame.accuracy.horizontal_velocity =
        core::UncertaintyEstimate{.value = 0.0F, .descriptor = exact_descriptor};
    frame.accuracy.vertical_velocity =
        core::UncertaintyEstimate{.value = 0.0F, .descriptor = exact_descriptor};
    frame.gps_quality = core::GpsQuality::kFix3D;
    frame.estimator_state =
        vehicle.failed ? core::EstimatorState::kFault : core::EstimatorState::kHealthy;
    frame.estimator_position_ok = !vehicle.failed;
    frame.estimator_velocity_ok = !vehicle.failed;
    frame.estimator_attitude_ok = !vehicle.failed;
    frame.home_origin = {
        .frame = core::CoordinateFrame::kWgs84,
        .lat_deg = config.home_lat_deg,
        .lon_deg = config.home_lon_deg,
        .alt_m = static_cast<float>(config.home_alt_m),
    };
    const core::TimestampEvidence source_time{
        .timestamp_ms = config.initial_source_unix_time_ms + vehicle.simulation_time_ms,
        .clock_domain = core::ClockDomain::kUnixEpoch,
        .synchronization = core::ClockSynchronization::kSynchronized,
        .clock_uncertainty_ms = 0.0,
    };
    const auto provenance = [&](std::string source) {
        return core::MeasurementProvenance{
            .updated = true,
            .source_time = source_time,
            .source = std::move(source),
        };
    };
    frame.provenance.position = provenance("sim.estimated_position");
    frame.provenance.velocity = provenance("sim.estimated_velocity");
    frame.provenance.accuracy = provenance("sim.estimator_contract");
    frame.provenance.estimator = provenance("sim.estimator_state");
    frame.provenance.vehicle_state = provenance("sim.vehicle_state");
    return frame;
}

void IntegrateVehicle(const SimBackendConfig& config, VehicleState* vehicle, int duration_ms) {
    const double duration_seconds = static_cast<double>(duration_ms) / 1000.0;
    vehicle->simulation_time_ms += duration_ms;
    ++vehicle->truth_sequence;
    vehicle->velocity_north_mps = 0.0;
    vehicle->velocity_east_mps = 0.0;
    vehicle->velocity_up_mps = 0.0;

    if (!vehicle->armed || vehicle->failed || vehicle->paused) {
        vehicle->landed = vehicle->up_m <= kArrivalEpsilonM;
    } else if (vehicle->velocity_command.has_value()) {
        auto& velocity = *vehicle->velocity_command;
        const int active_duration_ms =
            static_cast<int>(std::min<std::int64_t>(duration_ms, velocity.remaining_ms));
        const double active_dt = static_cast<double>(active_duration_ms) / 1000.0;
        const double horizontal = std::hypot(velocity.north_mps, velocity.east_mps);
        const double scale = horizontal > config.max_horizontal_speed_mps
                                 ? static_cast<double>(config.max_horizontal_speed_mps) / horizontal
                                 : 1.0;
        vehicle->velocity_north_mps = velocity.north_mps * scale;
        vehicle->velocity_east_mps = velocity.east_mps * scale;
        vehicle->velocity_up_mps = ClampMagnitude(velocity.up_mps, config.max_climb_speed_mps,
                                                  config.max_descent_speed_mps);
        vehicle->north_m += vehicle->velocity_north_mps * active_dt;
        vehicle->east_m += vehicle->velocity_east_mps * active_dt;
        vehicle->up_m = std::clamp(vehicle->up_m + (vehicle->velocity_up_mps * active_dt), 0.0,
                                   static_cast<double>(config.max_altitude_m));
        velocity.remaining_ms -= duration_ms;
        if (velocity.remaining_ms <= 0) {
            vehicle->velocity_command.reset();
            vehicle->mode = "SIM_HOLD";
        }
    } else if (vehicle->target.has_value()) {
        const Target target = *vehicle->target;
        const double delta_north = target.north_m - vehicle->north_m;
        const double delta_east = target.east_m - vehicle->east_m;
        const double delta_up = target.up_m - vehicle->up_m;
        const double horizontal_distance = std::hypot(delta_north, delta_east);
        const double requested_speed =
            target.speed_mps > 0.0F ? target.speed_mps : vehicle->cruise_speed_mps;
        const double horizontal_speed =
            std::min<double>(requested_speed, config.max_horizontal_speed_mps);
        if (horizontal_distance > kArrivalEpsilonM) {
            vehicle->velocity_north_mps = delta_north / horizontal_distance * horizontal_speed;
            vehicle->velocity_east_mps = delta_east / horizontal_distance * horizontal_speed;
        }
        vehicle->velocity_up_mps =
            ClampMagnitude(delta_up / std::max(duration_seconds, 1e-9), config.max_climb_speed_mps,
                           config.max_descent_speed_mps);

        const double horizontal_step = horizontal_speed * duration_seconds;
        if (config.stop_at_target && horizontal_step >= horizontal_distance) {
            vehicle->north_m = target.north_m;
            vehicle->east_m = target.east_m;
            vehicle->velocity_north_mps = 0.0;
            vehicle->velocity_east_mps = 0.0;
        } else {
            vehicle->north_m += vehicle->velocity_north_mps * duration_seconds;
            vehicle->east_m += vehicle->velocity_east_mps * duration_seconds;
        }
        const double next_up = vehicle->up_m + (vehicle->velocity_up_mps * duration_seconds);
        vehicle->up_m = config.stop_at_target && std::abs(vehicle->velocity_up_mps *
                                                          duration_seconds) >= std::abs(delta_up)
                            ? target.up_m
                            : next_up;
        vehicle->up_m = std::clamp(vehicle->up_m, 0.0, static_cast<double>(config.max_altitude_m));

        const bool horizontal_arrived =
            std::hypot(target.north_m - vehicle->north_m, target.east_m - vehicle->east_m) <=
            kArrivalEpsilonM;
        const bool vertical_arrived = std::abs(target.up_m - vehicle->up_m) <= kArrivalEpsilonM;
        if (config.stop_at_target && horizontal_arrived && vertical_arrived) {
            vehicle->target.reset();
            vehicle->velocity_north_mps = 0.0;
            vehicle->velocity_east_mps = 0.0;
            vehicle->velocity_up_mps = 0.0;
            if (vehicle->mode == "SIM_LAND" || vehicle->mode == "SIM_RTL") {
                vehicle->armed = false;
                vehicle->landed = true;
            }
            vehicle->mode = "SIM_HOLD";
        }
    }
    vehicle->landed = vehicle->up_m <= kArrivalEpsilonM;
    vehicle->battery_percent = std::max(
        0.0F, vehicle->battery_percent -
                  (config.battery_drain_percent_per_second * static_cast<float>(duration_seconds)));
}

void AdvanceVehicle(const std::shared_ptr<SimBackendControl::State>& state,
                    const std::string& drone_id, int duration_ms) {
    std::vector<SimulationTruthFrame> truth_frames;
    std::vector<core::TelemetryFrame> telemetry_frames;
    IDroneBackend::TelemetryCallback telemetry_callback;
    SimulationTruthObserver truth_observer;
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        VehicleState& vehicle = Vehicle(state.get(), drone_id);
        const auto stream_iter = state->streams.find(drone_id);
        std::shared_ptr<StreamState> stream =
            stream_iter == state->streams.end() ? nullptr : stream_iter->second;
        int remaining = duration_ms;
        while (remaining > 0) {
            int step = std::min(remaining, state->config.integration_step_ms);
            if (stream) {
                const int period_ms = 1000 / std::max(1, stream->rate_hertz);
                step = std::min(step, period_ms - stream->accumulated_ms);
            }
            IntegrateVehicle(state->config, &vehicle, step);
            remaining -= step;
            truth_frames.push_back(TruthFrame(state->config, drone_id, vehicle));
            if (stream) {
                stream->accumulated_ms += step;
                const int period_ms = 1000 / std::max(1, stream->rate_hertz);
                if (stream->accumulated_ms >= period_ms) {
                    stream->accumulated_ms %= period_ms;
                    telemetry_frames.push_back(TelemetryFrame(state->config, drone_id, vehicle));
                }
            }
        }
        if (stream) {
            telemetry_callback = stream->callback;
        }
        truth_observer = state->config.truth_observer;
    }
    if (truth_observer) {
        for (const auto& truth : truth_frames) {
            truth_observer(truth);
        }
    }
    if (telemetry_callback) {
        for (const auto& frame : telemetry_frames) {
            telemetry_callback(frame);
        }
    }
}

class SimBackend final : public IDroneBackend {
   public:
    explicit SimBackend(std::shared_ptr<SimBackendControl::State> state)
        : state_(std::move(state)) {}

    ~SimBackend() noexcept override {
        try {
            std::vector<std::string> drone_ids;
            {
                std::lock_guard<std::mutex> lock(state_->mutex);
                drone_ids.reserve(state_->streams.size());
                for (const auto& [drone_id, stream] : state_->streams) {
                    static_cast<void>(stream);
                    drone_ids.push_back(drone_id);
                }
            }
            for (const auto& drone_id : drone_ids) {
                static_cast<void>(StopTelemetry(drone_id));
            }
        } catch (...) {
            // Backend destruction must not terminate process shutdown.
            std::fputs("SwarmKit simulator shutdown failed\n", stderr);
        }
    }

    core::BackendCommandOutcome Execute(const commands::CommandEnvelope& envelope) override {
        std::lock_guard<std::mutex> lock(state_->mutex);
        VehicleState& vehicle = Vehicle(state_.get(), envelope.context.drone_id);
        if (vehicle.failed) {
            return Rejected("simulated vehicle is failed: " + vehicle.failure_detail);
        }

        core::BackendCommandOutcome outcome = Rejected("simulator command is unsupported");
        std::visit(
            core::Overloaded{
                [&](const commands::FlightCmd& flight) {
                    std::visit(
                        core::Overloaded{
                            [&](const commands::CmdArm&) {
                                vehicle.armed = true;
                                vehicle.mode = "SIM_HOLD";
                                outcome = Accepted("simulator armed vehicle");
                            },
                            [&](const commands::CmdForceArm&) {
                                vehicle.armed = true;
                                vehicle.mode = "SIM_HOLD";
                                outcome = Accepted("simulator force-armed vehicle");
                            },
                            [&](const commands::CmdDisarm&) {
                                if (!vehicle.landed) {
                                    outcome = Rejected("cannot disarm simulator while airborne");
                                    return;
                                }
                                vehicle.armed = false;
                                outcome = Accepted("simulator disarmed vehicle");
                            },
                            [&](const commands::CmdForceDisarm&) {
                                vehicle.armed = false;
                                vehicle.target.reset();
                                vehicle.velocity_command.reset();
                                vehicle.mode = "SIM_DISARMED";
                                outcome = Accepted("simulator force-disarmed vehicle");
                            },
                            [&](const commands::CmdTakeoff& command) {
                                if (!vehicle.armed) {
                                    outcome = Rejected("takeoff requires armed simulator vehicle");
                                    return;
                                }
                                if (!std::isfinite(command.alt_m) || command.alt_m <= 0.0F ||
                                    command.alt_m > state_->config.max_altitude_m) {
                                    outcome =
                                        Rejected("takeoff altitude is outside simulator limits");
                                    return;
                                }
                                vehicle.target = Target{.north_m = vehicle.north_m,
                                                        .east_m = vehicle.east_m,
                                                        .up_m = command.alt_m,
                                                        .speed_mps = vehicle.cruise_speed_mps};
                                vehicle.mode = "SIM_TAKEOFF";
                                outcome = Accepted("simulator started takeoff");
                            },
                            [&](const commands::CmdLand&) {
                                if (!vehicle.armed) {
                                    outcome = Rejected("land requires armed simulator vehicle");
                                    return;
                                }
                                vehicle.target = Target{.north_m = vehicle.north_m,
                                                        .east_m = vehicle.east_m,
                                                        .up_m = 0.0,
                                                        .speed_mps = vehicle.cruise_speed_mps};
                                vehicle.velocity_command.reset();
                                vehicle.mode = "SIM_LAND";
                                outcome = Accepted("simulator started landing");
                            },
                            [&](const commands::CmdSetMode& command) {
                                vehicle.mode = command.mode.empty() ? "SIM_CUSTOM" : command.mode;
                                outcome = Accepted("simulator mode updated");
                            },
                            [&](const commands::CmdFlightTerminate&) {
                                vehicle.failed = true;
                                vehicle.failure_detail = "flight termination";
                                vehicle.target.reset();
                                vehicle.velocity_command.reset();
                                vehicle.mode = "SIM_TERMINATED";
                                outcome = Accepted("simulator flight terminated");
                            },
                        },
                        flight);
                },
                [&](const commands::NavCmd& navigation) {
                    std::visit(
                        core::Overloaded{
                            [&](const commands::CmdSetWaypoint& command) {
                                if (!vehicle.armed) {
                                    outcome = Rejected("waypoint requires armed simulator vehicle");
                                    return;
                                }
                                if (!std::isfinite(command.lat_deg) ||
                                    !std::isfinite(command.lon_deg) ||
                                    !std::isfinite(command.alt_m) || command.alt_m < 0.0 ||
                                    command.alt_m > state_->config.max_altitude_m ||
                                    !std::isfinite(command.speed_mps) || command.speed_mps < 0.0F) {
                                    outcome = Rejected("waypoint is outside simulator limits");
                                    return;
                                }
                                const auto [north, east] =
                                    GeoToLocal(state_->config, command.lat_deg, command.lon_deg);
                                vehicle.target = Target{.north_m = north,
                                                        .east_m = east,
                                                        .up_m = command.alt_m,
                                                        .speed_mps = command.speed_mps};
                                vehicle.velocity_command.reset();
                                vehicle.mode = "SIM_GOTO";
                                outcome = Accepted("simulator waypoint accepted");
                            },
                            [&](const commands::CmdGoto& command) {
                                if (!vehicle.armed) {
                                    outcome = Rejected("goto requires armed simulator vehicle");
                                    return;
                                }
                                if (!std::isfinite(command.lat_deg) ||
                                    !std::isfinite(command.lon_deg) ||
                                    !std::isfinite(command.alt_m) || command.alt_m < 0.0 ||
                                    command.alt_m > state_->config.max_altitude_m ||
                                    !std::isfinite(command.speed_mps) || command.speed_mps < 0.0F) {
                                    outcome = Rejected("goto is outside simulator limits");
                                    return;
                                }
                                const auto [north, east] =
                                    GeoToLocal(state_->config, command.lat_deg, command.lon_deg);
                                vehicle.target = Target{.north_m = north,
                                                        .east_m = east,
                                                        .up_m = command.alt_m,
                                                        .speed_mps = command.speed_mps};
                                vehicle.velocity_command.reset();
                                vehicle.mode = "SIM_GOTO";
                                outcome = Accepted("simulator goto accepted");
                            },
                            [&](const commands::CmdReturnHome&) {
                                if (!vehicle.armed) {
                                    outcome =
                                        Rejected("return-home requires armed simulator vehicle");
                                    return;
                                }
                                vehicle.target = Target{.north_m = 0.0,
                                                        .east_m = 0.0,
                                                        .up_m = 0.0,
                                                        .speed_mps = vehicle.cruise_speed_mps};
                                vehicle.velocity_command.reset();
                                vehicle.mode = "SIM_RTL";
                                outcome = Accepted("simulator return-home accepted");
                            },
                            [&](const commands::CmdHoldPosition&) {
                                vehicle.target.reset();
                                vehicle.velocity_command.reset();
                                vehicle.paused = false;
                                vehicle.mode = "SIM_HOLD";
                                outcome = Accepted("simulator holding position");
                            },
                            [&](const commands::CmdSetSpeed& command) {
                                if (!std::isfinite(command.ground_mps) ||
                                    command.ground_mps <= 0.0F ||
                                    command.ground_mps > state_->config.max_horizontal_speed_mps) {
                                    outcome = Rejected("speed is outside simulator limits");
                                    return;
                                }
                                vehicle.cruise_speed_mps = command.ground_mps;
                                outcome = Accepted("simulator cruise speed updated");
                            },
                            [&](const commands::CmdPause&) {
                                vehicle.paused = true;
                                vehicle.mode = "SIM_PAUSED";
                                outcome = Accepted("simulator paused motion");
                            },
                            [&](const commands::CmdResume&) {
                                vehicle.paused = false;
                                vehicle.mode = vehicle.target.has_value() ? "SIM_GOTO" : "SIM_HOLD";
                                outcome = Accepted("simulator resumed motion");
                            },
                            [&](const commands::CmdSetYaw& command) {
                                vehicle.yaw_deg = command.relative
                                                      ? vehicle.yaw_deg + command.yaw_deg
                                                      : command.yaw_deg;
                                outcome = Accepted("simulator yaw updated");
                            },
                            [&](const commands::CmdVelocity& command) {
                                if (!vehicle.armed) {
                                    outcome = Rejected("velocity requires armed simulator vehicle");
                                    return;
                                }
                                if (command.duration_ms <= 0) {
                                    outcome = Rejected("velocity duration must be positive");
                                    return;
                                }
                                double north = command.vx_mps;
                                double east = command.vy_mps;
                                if (command.body_frame) {
                                    const double yaw = DegreesToRadians(vehicle.yaw_deg);
                                    north = command.vx_mps * std::cos(yaw) -
                                            command.vy_mps * std::sin(yaw);
                                    east = command.vx_mps * std::sin(yaw) +
                                           command.vy_mps * std::cos(yaw);
                                }
                                vehicle.velocity_command = VelocityCommand{
                                    .north_mps = north,
                                    .east_mps = east,
                                    .up_mps = -command.vz_mps,
                                    .remaining_ms = command.duration_ms,
                                };
                                vehicle.target.reset();
                                vehicle.mode = "SIM_VELOCITY";
                                outcome = Accepted("simulator velocity accepted");
                            },
                            [&](const commands::CmdSetHome& command) {
                                static_cast<void>(command);
                                outcome =
                                    Rejected("simulator home is immutable after scenario creation");
                            },
                        },
                        navigation);
                },
                [&](const commands::PayloadCmd&) {
                    outcome = Accepted("simulator accepted payload command");
                },
                [&](const commands::BackendCmd& backend) {
                    std::visit(
                        [&](const commands::CmdBackendCommand& command) {
                            if (command.backend_namespace == "sim" && command.name == "echo") {
                                outcome = Accepted("simulator echo accepted");
                            } else {
                                outcome = Rejected("unknown simulator backend command");
                            }
                        },
                        backend);
                },
            },
            envelope.command);
        return outcome;
    }

    core::Result StartTelemetry(const std::string& drone_id, int rate_hertz,
                                TelemetryCallback callback) override {
        if (!callback) {
            return core::Result::Rejected("simulator telemetry callback must not be empty");
        }
        const int effective_rate = rate_hertz <= 0 ? kDefaultTelemetryRateHz : rate_hertz;
        if (effective_rate > 1000) {
            return core::Result::Rejected("simulator telemetry rate must not exceed 1000 Hz");
        }
        auto stream = std::make_shared<StreamState>();
        stream->rate_hertz = effective_rate;
        stream->callback = std::move(callback);
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            if (state_->streams.contains(drone_id)) {
                return core::Result::Rejected("simulator telemetry already active");
            }
            static_cast<void>(Vehicle(state_.get(), drone_id));
            state_->streams.emplace(drone_id, stream);
        }

        if (state_->config.clock_mode == SimulationClockMode::kRealtime) {
            const int period_ms = std::max(1, 1000 / effective_rate);
            // NOLINTNEXTLINE(bugprone-exception-escape)
            stream->worker = std::thread([state = state_, stream, drone_id, period_ms]() {
                try {
                    auto next = std::chrono::steady_clock::now();
                    while (stream->running.load(std::memory_order_relaxed)) {
                        next += std::chrono::milliseconds(period_ms);
                        AdvanceVehicle(state, drone_id, period_ms);
                        std::this_thread::sleep_until(next);
                    }
                } catch (const std::exception& exception) {
                    core::Logger::ErrorFmt("simulator telemetry worker failed: {}",
                                           exception.what());
                } catch (...) {
                    core::Logger::Error("simulator telemetry worker failed");
                }
            });
        }
        return core::Result::Success();
    }

    core::Result StopTelemetry(const std::string& drone_id) override {
        std::shared_ptr<StreamState> stream;
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            const auto iter = state_->streams.find(drone_id);
            if (iter == state_->streams.end()) {
                return core::Result::Success();
            }
            stream = std::move(iter->second);
            state_->streams.erase(iter);
        }
        stream->running.store(false, std::memory_order_relaxed);
        if (stream->worker.joinable()) {
            stream->worker.join();
        }
        return core::Result::Success();
    }

    [[nodiscard]] BackendHealth GetHealth() const override {
        std::lock_guard<std::mutex> lock(state_->mutex);
        BackendHealth health{
            .ready = true,
            .message = "simulator ready",
            .backend_name = "sim",
            .protocol = "deterministic-model",
            .gps_ok = true,
            .gps_fix_type = 3,
            .satellites_visible = 12,
            .gps_hdop = 0.7F,
            .ekf_ok = true,
            .link_quality_percent = 100.0F,
        };
        if (!state_->vehicles.empty()) {
            const auto& vehicle = state_->vehicles.begin()->second;
            health.ready = !vehicle.failed;
            health.message = vehicle.failed ? vehicle.failure_detail : "simulator ready";
            health.armed = vehicle.armed;
            health.landed = vehicle.landed;
            health.mode = vehicle.mode;
            health.failsafe = vehicle.failed;
            health.ekf_ok = !vehicle.failed;
            health.has_relative_altitude = true;
            health.relative_alt_m = static_cast<float>(vehicle.up_m);
        }
        return health;
    }

    [[nodiscard]] core::BackendCapabilities GetCapabilities() const override {
        const auto enforced_limit = [](float value, std::string source) {
            return core::MotionLimit{
                .value = value,
                .semantics = core::MotionLimitSemantics::kValidatedBound,
                .source = std::move(source),
                .profile_id = "deterministic-multirotor-v1",
                .profile_version = "1",
            };
        };
        return {
            .backend_name = "sim",
            .protocol = "deterministic-model",
            .vehicle_class = "multirotor",
            .supports_payload_control = true,
            .supports_velocity_control = true,
            .supports_flight_termination = true,
            .supports_backend_commands = true,
            .autopilot_type = "swarmkit-sim",
            .supported_modes = {"sim_hold", "sim_takeoff", "sim_goto", "sim_velocity", "sim_land",
                                "sim_rtl"},
            .supported_commands = {"arm", "force-arm", "disarm", "force-disarm", "takeoff", "land",
                                   "goto", "return-home", "hold", "set-speed", "pause", "resume",
                                   "yaw", "velocity", "payload", "backend-command",
                                   "flight-terminate"},
            .supported_payloads = {"camera", "gimbal", "servo", "relay", "gripper"},
            .backend_command_names = {"sim.echo"},
            .evidence =
                {
                    .source_timestamp = core::CapabilitySupport::kSupported,
                    .source_clock_domains = {core::ClockDomain::kUnixEpoch},
                    .position_estimate = core::CapabilitySupport::kSupported,
                    .horizontal_position_uncertainty = core::CapabilitySupport::kSupported,
                    .vertical_position_uncertainty = core::CapabilitySupport::kSupported,
                    .horizontal_velocity = core::CapabilitySupport::kSupported,
                    .vertical_velocity = core::CapabilitySupport::kSupported,
                    .horizontal_velocity_uncertainty = core::CapabilitySupport::kSupported,
                    .vertical_velocity_uncertainty = core::CapabilitySupport::kSupported,
                    .speed_uncertainty = core::CapabilitySupport::kUnsupported,
                    .uncertainty_semantics = core::CapabilitySupport::kSupported,
                    .estimator_health = core::CapabilitySupport::kSupported,
                    .failsafe_state = core::CapabilitySupport::kSupported,
                },
            .max_horizontal_speed = enforced_limit(state_->config.max_horizontal_speed_mps,
                                                   "sim.vehicle_model.horizontal_clamp"),
            .max_climb_speed =
                enforced_limit(state_->config.max_climb_speed_mps, "sim.vehicle_model.climb_clamp"),
            .max_descent_speed = enforced_limit(state_->config.max_descent_speed_mps,
                                                "sim.vehicle_model.descent_clamp"),
            .max_altitude =
                enforced_limit(state_->config.max_altitude_m, "sim.vehicle_model.altitude_clamp"),
        };
    }

    void SetEvidenceCallback(const EvidenceCallback& callback) override {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->evidence_callback = callback;
    }

   private:
    std::shared_ptr<SimBackendControl::State> state_;
};

}  // namespace

core::Result SimBackendConfig::Validate() const {
    if (integration_step_ms <= 0 || integration_step_ms > 1000) {
        return core::Result::Rejected("sim integration_step_ms must be in [1, 1000]");
    }
    const std::array positive_values{max_horizontal_speed_mps, max_climb_speed_mps,
                                     max_descent_speed_mps, max_altitude_m,
                                     default_cruise_speed_mps};
    if (!std::ranges::all_of(positive_values,
                             [](float value) { return std::isfinite(value) && value > 0.0F; })) {
        return core::Result::Rejected("simulator limits must be finite and positive");
    }
    if (default_cruise_speed_mps > max_horizontal_speed_mps) {
        return core::Result::Rejected("default simulator cruise speed exceeds horizontal limit");
    }
    if (!std::isfinite(home_lat_deg) || home_lat_deg < -90.0 || home_lat_deg > 90.0 ||
        !std::isfinite(home_lon_deg) || home_lon_deg < -180.0 || home_lon_deg > 180.0 ||
        !std::isfinite(home_alt_m)) {
        return core::Result::Rejected("simulator home coordinates are invalid");
    }
    if (!std::isfinite(initial_battery_percent) || initial_battery_percent < 0.0F ||
        initial_battery_percent > 100.0F || !std::isfinite(battery_drain_percent_per_second) ||
        battery_drain_percent_per_second < 0.0F) {
        return core::Result::Rejected("simulator battery configuration is invalid");
    }
    return core::Result::Success();
}

SimBackendControl::SimBackendControl(std::shared_ptr<State> state) : state_(std::move(state)) {}

core::Result SimBackendControl::Advance(const std::string& drone_id,
                                        std::chrono::milliseconds duration) {
    if (state_->config.clock_mode != SimulationClockMode::kManual) {
        return core::Result::Rejected("manual advance requires SimulationClockMode::kManual");
    }
    if (drone_id.empty() || duration.count() <= 0 ||
        duration.count() > std::numeric_limits<int>::max()) {
        return core::Result::Rejected("manual simulator advance arguments are invalid");
    }
    AdvanceVehicle(state_, drone_id, static_cast<int>(duration.count()));
    return core::Result::Success();
}

core::Result SimBackendControl::AdvanceAll(std::chrono::milliseconds duration) {
    if (state_->config.clock_mode != SimulationClockMode::kManual) {
        return core::Result::Rejected("manual advance requires SimulationClockMode::kManual");
    }
    const auto drone_ids = DroneIds();
    for (const auto& drone_id : drone_ids) {
        const core::Result result = Advance(drone_id, duration);
        if (!result.IsOk()) {
            return result;
        }
    }
    return core::Result::Success();
}

void SimBackendControl::SetFailure(const std::string& drone_id, bool failed, std::string detail) {
    IDroneBackend::EvidenceCallback evidence_callback;
    BackendEvidenceEvent event;
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        VehicleState& vehicle = Vehicle(state_.get(), drone_id);
        vehicle.failed = failed;
        vehicle.failure_detail = failed ? std::move(detail) : std::string{};
        vehicle.mode = failed ? "SIM_FAILED" : "SIM_HOLD";
        if (failed) {
            vehicle.target.reset();
            vehicle.velocity_command.reset();
        }
        evidence_callback = state_->evidence_callback;
        event = {
            .source = "sim-vehicle-model",
            .kind = failed ? "failure-injected" : "failure-cleared",
            .source_sequence = ++state_->evidence_sequence,
            .attributes = {{"drone_id", drone_id}, {"detail", vehicle.failure_detail}},
        };
    }
    if (evidence_callback) {
        evidence_callback(event);
    }
}

std::optional<SimulationTruthFrame> SimBackendControl::Truth(const std::string& drone_id) const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    const auto iter = state_->vehicles.find(drone_id);
    if (iter == state_->vehicles.end()) {
        return std::nullopt;
    }
    return TruthFrame(state_->config, drone_id, iter->second);
}

std::vector<std::string> SimBackendControl::DroneIds() const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    std::vector<std::string> drone_ids;
    drone_ids.reserve(state_->vehicles.size());
    for (const auto& [drone_id, vehicle] : state_->vehicles) {
        static_cast<void>(vehicle);
        drone_ids.push_back(drone_id);
    }
    std::ranges::sort(drone_ids);
    return drone_ids;
}

std::expected<SimBackendInstance, core::Result> MakeSimBackend(SimBackendConfig config) {
    if (const core::Result validation = config.Validate(); !validation.IsOk()) {
        return std::unexpected(validation);
    }
    auto state = std::make_shared<SimBackendControl::State>(std::move(config));
    return SimBackendInstance{
        .backend = std::make_unique<SimBackend>(state),
        .control = std::shared_ptr<SimBackendControl>(new SimBackendControl(std::move(state))),
    };
}

}  // namespace swarmkit::agent
