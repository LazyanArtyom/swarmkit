// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "runtime_health.h"

#include "runtime_common.h"

namespace swarmkit::apps::cli::internal {

int RunPing(Client& client) {
    const auto kResult = client.Ping();

    if (!kResult.ok) {
        std::cerr << "Ping FAILED: " << kResult.error_message;
        if (!kResult.correlation_id.empty()) {
            std::cerr << " [corr=" << kResult.correlation_id << "]";
        }
        std::cerr << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "Ping OK\n"
              << "  agent_id  : " << kResult.agent_id << "\n"
              << "  version   : " << kResult.version << "\n"
              << "  time_ms   : " << kResult.unix_time_ms << "\n";
    return EXIT_SUCCESS;
}

struct TelemetrySampleResult {
    std::optional<swarmkit::core::TelemetryFrame> frame{};
    std::string error{};
};

[[nodiscard]] TelemetrySampleResult CollectTelemetrySample(Client& client,
                                                           std::string_view drone_id,
                                                           std::int64_t timeout_ms) {
    std::mutex mutex;
    std::condition_variable frame_cv;
    std::optional<swarmkit::core::TelemetryFrame> latest_frame;
    std::string stream_error;

    auto telemetry_stream = client.StartTelemetry(
        {.drone_id = std::string(drone_id), .rate_hertz = 2},
        [&](const swarmkit::client::TelemetryDelivery& delivery) {
            {
                std::lock_guard<std::mutex> lock(mutex);
                latest_frame = delivery.frame;
            }
            frame_cv.notify_all();
        },
        [&](const std::string& error_msg) {
            {
                std::lock_guard<std::mutex> lock(mutex);
                stream_error = error_msg;
            }
            std::cerr << "Preflight telemetry stream error: " << error_msg << "\n";
            frame_cv.notify_all();
        });
    if (!telemetry_stream.has_value()) {
        std::cerr << "Preflight telemetry FAILED: " << telemetry_stream.error().user_message
                  << "\n";
        return {.error = telemetry_stream.error().user_message};
    }

    std::unique_lock<std::mutex> lock(mutex);
    frame_cv.wait_for(lock, std::chrono::milliseconds{timeout_ms},
                      [&] { return latest_frame.has_value() || !stream_error.empty(); });
    TelemetrySampleResult result{.frame = latest_frame, .error = stream_error};
    lock.unlock();
    telemetry_stream->Stop();
    return result;
}

[[nodiscard]] std::expected<float, std::string> ParsePreflightMinBattery(int argc, char** argv) {
    const auto min_battery =
        ParseFloatArg(common::GetOptionValue(argc, argv, "--min-battery", "20"), "--min-battery");
    if (!min_battery.has_value()) {
        return std::unexpected(min_battery.error());
    }
    if (*min_battery < 0.0F || *min_battery > 100.0F) {
        return std::unexpected("--min-battery must be between 0 and 100");
    }
    return *min_battery;
}

void PrintPreflightCheck(bool check_ok, std::string_view name, std::string_view detail) {
    std::cout << "  [" << (check_ok ? "OK" : "FAIL") << "] " << name;
    if (!detail.empty()) {
        std::cout << " - " << detail;
    }
    std::cout << "\n";
}

int RunPreflight(Client& client, std::string_view drone_id, int argc, char** argv) {
    const auto timeout_ms = ParseDurationMs(argc, argv);
    if (!timeout_ms.has_value()) {
        std::cerr << timeout_ms.error() << "\n";
        return EXIT_FAILURE;
    }
    const std::int64_t sample_timeout_ms = *timeout_ms == 0 ? 3000 : *timeout_ms;
    const auto min_battery = ParsePreflightMinBattery(argc, argv);
    if (!min_battery.has_value()) {
        std::cerr << min_battery.error() << "\n";
        return EXIT_FAILURE;
    }

    const auto status = client.GetHealth();
    const std::int64_t now_ms = NowUnixMs();
    const bool heartbeat_ok = status.ok && !IsStaleTimestamp(status.last_heartbeat_unix_ms, now_ms);
    const bool telemetry_ok = status.ok && !IsStaleTimestamp(status.last_telemetry_unix_ms, now_ms);
    const auto telemetry = CollectTelemetrySample(client, drone_id, sample_timeout_ms);

    bool ready = true;
    std::cout << "Preflight " << drone_id << "\n";

    const bool agent_ok = status.ok && status.ready;
    ready = ready && agent_ok;
    PrintPreflightCheck(agent_ok, "agent", status.ok ? status.message : status.error.user_message);

    ready = ready && heartbeat_ok;
    PrintPreflightCheck(heartbeat_ok, "MAVLink heartbeat",
                        "age_ms=" + TimestampAgeMsText(status.last_heartbeat_unix_ms, now_ms));

    ready = ready && telemetry_ok && telemetry.frame.has_value();
    PrintPreflightCheck(telemetry_ok && telemetry.frame.has_value(), "telemetry",
                        telemetry.error.empty()
                            ? "age_ms=" + TimestampAgeMsText(status.last_telemetry_unix_ms, now_ms)
                            : telemetry.error);

    ready = ready && !status.armed;
    PrintPreflightCheck(!status.armed, "armed state",
                        status.armed ? "vehicle is already armed" : "disarmed");

    ready = ready && status.landed;
    PrintPreflightCheck(status.landed, "landed state",
                        std::string{"landed="} + BoolText(status.landed) +
                            " rel_alt=" + RelativeAltitudeText(status));

    ready = ready && !status.failsafe;
    PrintPreflightCheck(!status.failsafe, "failsafe", BoolText(status.failsafe));

    ready = ready && status.ekf_ok;
    PrintPreflightCheck(status.ekf_ok, "EKF", status.ekf_ok ? "healthy" : "unhealthy");

    const std::string gps_detail = "fix=" + std::to_string(status.gps_fix_type) + " (" +
                                   GpsFixText(status.gps_fix_type) +
                                   ") sats=" + std::to_string(status.satellites_visible) +
                                   " hdop=" + FloatText(status.gps_hdop);
    ready = ready && status.gps_ok;
    PrintPreflightCheck(status.gps_ok, "GPS", gps_detail);

    bool battery_ok = false;
    std::string battery_detail = "unknown";
    if (telemetry.frame.has_value() && telemetry.frame->HasBattery()) {
        battery_ok = telemetry.frame->battery_percent >= *min_battery;
        battery_detail = FloatText(telemetry.frame->battery_percent, 1) +
                         "% minimum=" + FloatText(*min_battery, 1) + "%";
    }
    ready = ready && battery_ok;
    PrintPreflightCheck(battery_ok, "battery", battery_detail);

    const std::string mode_text = status.mode.empty() ? "unknown" : status.mode;
    PrintPreflightCheck(true, "mode",
                        mode_text + " custom_mode=" + std::to_string(status.custom_mode));

    if (!status.gps_ok) {
        std::cout << "  guidance: GPS is not ready. Normal GPS modes like GUIDED/takeoff may "
                     "reject arming or takeoff. Use emergency force-arm only for bench/no-prop "
                     "tests.\n";
    }
    if (ready) {
        std::cout << "Preflight OK: drone appears ready for normal flight commands.\n";
        return EXIT_SUCCESS;
    }
    std::cout << "Preflight FAILED: fix the failed checks before flight.\n";
    return EXIT_FAILURE;
}

int RunHealth(Client& client) {
    const auto kStatus = client.GetHealth();
    if (!kStatus.ok) {
        std::cerr << "Health FAILED: " << kStatus.message;
        if (!kStatus.correlation_id.empty()) {
            std::cerr << " [corr=" << kStatus.correlation_id << "]";
        }
        std::cerr << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "Health OK\n"
              << "  ready                  : " << (kStatus.ready ? "true" : "false") << "\n"
              << "  agent_id               : " << kStatus.agent_id << "\n"
              << "  version                : " << kStatus.version << "\n"
              << "  time_ms                : " << kStatus.unix_time_ms << "\n"
              << "  backend_name           : " << kStatus.backend_name << "\n"
              << "  protocol               : " << kStatus.protocol << "\n"
              << "  last_heartbeat_unix_ms : " << kStatus.last_heartbeat_unix_ms << "\n"
              << "  last_telemetry_unix_ms : " << kStatus.last_telemetry_unix_ms << "\n"
              << "  armed                  : " << (kStatus.armed ? "true" : "false") << "\n"
              << "  landed                 : " << (kStatus.landed ? "true" : "false") << "\n"
              << "  mode                   : " << (kStatus.mode.empty() ? "unknown" : kStatus.mode)
              << "\n"
              << "  custom_mode            : " << kStatus.custom_mode << "\n"
              << "  failsafe               : " << (kStatus.failsafe ? "true" : "false") << "\n"
              << "  gps_ok                 : " << (kStatus.gps_ok ? "true" : "false") << "\n"
              << "  gps_fix                : " << kStatus.gps_fix_type << " ("
              << GpsFixText(kStatus.gps_fix_type) << ")\n"
              << "  satellites_visible     : " << kStatus.satellites_visible << "\n"
              << "  gps_hdop               : " << FloatText(kStatus.gps_hdop) << "\n"
              << "  ekf_ok                 : " << (kStatus.ekf_ok ? "true" : "false") << "\n"
              << "  relative_altitude      : " << RelativeAltitudeText(kStatus) << "\n"
              << "  autonomous_ready       : " << (kStatus.autonomous_ready ? "true" : "false")
              << "\n"
              << "  link_quality_percent   : " << OptionalFloatText(kStatus.link_quality_percent)
              << "\n"
              << "  message                : " << kStatus.message << "\n";
    if (!kStatus.arming_blockers.empty()) {
        std::cout << "  arming_blockers        : ";
        for (std::size_t index = 0; index < kStatus.arming_blockers.size(); ++index) {
            if (index > 0) {
                std::cout << ", ";
            }
            std::cout << kStatus.arming_blockers[index];
        }
        std::cout << "\n";
    }
    if (!kStatus.readiness_checks.empty()) {
        std::cout << "  readiness_checks       :\n";
        for (const auto& check : kStatus.readiness_checks) {
            std::cout << "    [" << (check.ok ? "OK" : "FAIL") << "] " << check.name
                      << " severity=" << ReadinessSeverityText(check.severity);
            if (!check.detail.empty()) {
                std::cout << " detail=" << check.detail;
            }
            std::cout << "\n";
        }
    }
    return EXIT_SUCCESS;
}

int RunStats(Client& client) {
    const auto kStats = client.GetRuntimeStats();
    if (!kStats.ok) {
        std::cerr << "Stats FAILED: " << kStats.error.user_message;
        if (!kStats.correlation_id.empty()) {
            std::cerr << " [corr=" << kStats.correlation_id << "]";
        }
        std::cerr << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "Runtime Stats\n"
              << "  agent_id                    : " << kStats.agent_id << "\n"
              << "  ready                       : " << (kStats.ready ? "true" : "false") << "\n"
              << "  ping_requests_total         : " << kStats.ping_requests_total << "\n"
              << "  health_requests_total       : " << kStats.health_requests_total << "\n"
              << "  stats_requests_total        : " << kStats.runtime_stats_requests_total << "\n"
              << "  command_requests_total      : " << kStats.command_requests_total << "\n"
              << "  command_rejected_total      : " << kStats.command_rejected_total << "\n"
              << "  command_failed_total        : " << kStats.command_failed_total << "\n"
              << "  lock_requests_total         : " << kStats.lock_requests_total << "\n"
              << "  watch_requests_total        : " << kStats.watch_requests_total << "\n"
              << "  current_authority_watchers  : " << kStats.current_authority_watchers << "\n"
              << "  total_telemetry_subs        : " << kStats.total_telemetry_subscriptions << "\n"
              << "  current_telemetry_streams   : " << kStats.current_telemetry_streams << "\n"
              << "  telemetry_frames_sent_total : " << kStats.telemetry_frames_sent_total << "\n"
              << "  backend_failures_total      : " << kStats.backend_failures_total << "\n"
              << "  data_messages_published     : " << kStats.data_messages_published_total << "\n"
              << "  data_messages_rejected      : " << kStats.data_messages_rejected_total << "\n"
              << "  current_message_subscribers : " << kStats.current_message_subscribers << "\n"
              << "  artifact_uploads_total      : " << kStats.artifact_uploads_total << "\n"
              << "  artifact_downloads_total    : " << kStats.artifact_downloads_total << "\n"
              << "  artifact_bytes_received     : " << kStats.artifact_bytes_received_total << "\n"
              << "  artifact_bytes_sent         : " << kStats.artifact_bytes_sent_total << "\n"
              << "  artifact_failures_total     : " << kStats.artifact_failures_total << "\n";
    return EXIT_SUCCESS;
}

[[nodiscard]] std::string MotionLimitText(const std::optional<swarmkit::core::MotionLimit>& limit) {
    if (!limit.has_value()) {
        return "unknown";
    }
    std::ostringstream out;
    out << limit->value;
    if (!limit->source.empty()) {
        out << " (" << limit->source << ")";
    }
    return out.str();
}

int RunCapabilities(Client& client) {
    const auto capabilities = client.GetCapabilities();
    if (!capabilities.ok) {
        std::cerr << "Capabilities FAILED: " << capabilities.error.user_message;
        if (!capabilities.correlation_id.empty()) {
            std::cerr << " [corr=" << capabilities.correlation_id << "]";
        }
        std::cerr << "\n";
        return EXIT_FAILURE;
    }

    const auto print_list = [](std::string_view label, const std::vector<std::string>& values) {
        std::cout << "  " << std::left << std::setw(29) << label << ": ";
        for (std::size_t idx = 0; idx < values.size(); ++idx) {
            if (idx != 0) {
                std::cout << ", ";
            }
            std::cout << values[idx];
        }
        std::cout << "\n";
    };

    std::cout << "Backend Capabilities\n"
              << "  agent_id                    : " << capabilities.agent_id << "\n"
              << "  backend_name                : " << capabilities.backend.backend_name << "\n"
              << "  protocol                    : " << capabilities.backend.protocol << "\n"
              << "  vehicle_class               : " << capabilities.backend.vehicle_class << "\n"
              << "  autopilot_type              : " << capabilities.backend.autopilot_type << "\n"
              << "  supports_payload_control    : "
              << (capabilities.backend.supports_payload_control ? "true" : "false") << "\n"
              << "  supports_velocity_control   : "
              << (capabilities.backend.supports_velocity_control ? "true" : "false") << "\n"
              << "  supports_flight_termination : "
              << (capabilities.backend.supports_flight_termination ? "true" : "false") << "\n"
              << "  supports_backend_commands   : "
              << (capabilities.backend.supports_backend_commands ? "true" : "false") << "\n"
              << "  max_horizontal_speed_mps    : "
              << MotionLimitText(capabilities.backend.max_horizontal_speed) << "\n"
              << "  max_climb_speed_mps         : "
              << MotionLimitText(capabilities.backend.max_climb_speed) << "\n"
              << "  max_descent_speed_mps       : "
              << MotionLimitText(capabilities.backend.max_descent_speed) << "\n"
              << "  max_altitude_m              : "
              << MotionLimitText(capabilities.backend.max_altitude) << "\n";
    print_list("supported_modes", capabilities.backend.supported_modes);
    print_list("supported_commands", capabilities.backend.supported_commands);
    print_list("supported_payloads", capabilities.backend.supported_payloads);
    print_list("backend_command_names", capabilities.backend.backend_command_names);
    return EXIT_SUCCESS;
}

}  // namespace swarmkit::apps::cli::internal
