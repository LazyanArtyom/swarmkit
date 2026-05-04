// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "swarmkit/agent/server.h"

#include <grpcpp/grpcpp.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdint>
#include <deque>
#include <expected>
#include <exception>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <queue>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>

#include "config_yaml.h"
#include "env_utils.h"
#include "runtime_counters.h"
#include "security_utils.h"
#include "server_test_support.h"
#include "swarmkit/agent/arbiter.h"
#include "swarmkit/core/logger.h"
#include "swarmkit/core/version.h"
#include "swarmkit/v1/swarmkit.grpc.pb.h"
#include "swarmkit/v1/swarmkit.pb.h"
#include "telemetry_manager.h"

namespace swarmkit::agent {

using namespace swarmkit::commands;  // NOLINT(google-build-using-namespace)
using core::internal::GetEnvValue;
using core::internal::IsValidPriority;
using core::internal::LooksLikeAddress;
using core::internal::MakeCorrelationId;
using core::internal::ParseIntValue;

namespace {
using grpc::Status;

constexpr int kMillisecondsPerSecond = 1000;
constexpr std::string_view kCorrelationMetadataKey = "x-correlation-id";
constexpr std::string_view kAgentEnvId = "AGENT_ID";
constexpr std::string_view kAgentEnvBindAddr = "BIND_ADDR";
constexpr std::string_view kAgentEnvDefaultAuthorityTtlMs = "DEFAULT_AUTHORITY_TTL_MS";
constexpr std::string_view kAgentEnvDefaultTelemetryRateHz = "DEFAULT_TELEMETRY_RATE_HZ";
constexpr std::string_view kAgentEnvMinTelemetryRateHz = "MIN_TELEMETRY_RATE_HZ";
constexpr std::string_view kAgentEnvRootCaCertPath = "ROOT_CA_CERT_PATH";
constexpr std::string_view kAgentEnvCertChainPath = "CERT_CHAIN_PATH";
constexpr std::string_view kAgentEnvPrivateKeyPath = "PRIVATE_KEY_PATH";
constexpr std::string_view kAgentEnvAllowedClientIds = "ALLOWED_CLIENT_IDS";

/// @brief Watcher poll interval while blocking inside WatchAuthority RPC.
constexpr auto kWatchPollInterval = std::chrono::milliseconds{100};
constexpr auto kTelemetryWaitTimeout = std::chrono::milliseconds{200};

/// @name Time helpers
/// @{

[[nodiscard]] std::int64_t NowUnixMs() {
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using std::chrono::system_clock;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

/// @}

/// @name Correlation ID resolution
/// @{

[[nodiscard]] std::string ResolveCorrelationId(grpc::ServerContext* ctx,
                                               std::string_view request_correlation_id,
                                               std::string_view fallback_prefix) {
    if (!request_correlation_id.empty()) {
        return std::string(request_correlation_id);
    }
    if (ctx != nullptr) {
        const auto kMetadataIter =
            ctx->client_metadata().find(std::string(kCorrelationMetadataKey));
        if (kMetadataIter != ctx->client_metadata().end()) {
            return std::string(kMetadataIter->second.data(), kMetadataIter->second.length());
        }
    }
    return MakeCorrelationId(fallback_prefix);
}

/// @}

/// @name Validation helpers
/// @{

[[nodiscard]] core::Result ValidateCommandContext(const CommandContext& context) {
    if (context.drone_id.empty()) {
        return core::Result::Rejected("ctx.drone_id must not be empty");
    }
    if (context.client_id.empty()) {
        return core::Result::Rejected("ctx.client_id must not be empty");
    }
    if (!IsValidPriority(context.priority)) {
        return core::Result::Rejected("ctx.priority is not a supported CommandPriority");
    }
    const auto kEpoch = std::chrono::system_clock::time_point{};
    if (context.deadline != kEpoch && context.deadline <= std::chrono::system_clock::now()) {
        return core::Result::Failed("command deadline already expired");
    }
    return core::Result::Success();
}

[[nodiscard]] core::Result ValidateLockRequest(const CommandContext& context, std::int64_t ttl_ms) {
    if (ttl_ms < 0) {
        return core::Result::Rejected("ttl_ms must be >= 0");
    }
    return ValidateCommandContext(context);
}

[[nodiscard]] core::Result ValidateTelemetryRequest(const swarmkit::v1::TelemetryRequest* req) {
    if (req == nullptr) {
        return core::Result::Success();
    }
    if (req->rate_hz() < 0) {
        return core::Result::Rejected("telemetry rate_hz must be >= 0");
    }
    return core::Result::Success();
}

[[nodiscard]] std::vector<std::string> SplitCsvList(std::string_view value) {
    std::vector<std::string> out;
    std::size_t start = 0;
    while (start < value.size()) {
        const std::size_t end = value.find(',', start);
        const std::string_view token =
            end == std::string_view::npos ? value.substr(start) : value.substr(start, end - start);
        const std::string kTrimmed = core::internal::TrimWhitespace(token);
        if (!kTrimmed.empty()) {
            out.push_back(kTrimmed);
        }
        if (end == std::string_view::npos) {
            break;
        }
        start = end + 1;
    }
    return out;
}

[[nodiscard]] std::optional<std::string> FindPeerAuthProperty(grpc::ServerContext* ctx,
                                                              std::string_view property_name) {
    if (ctx == nullptr) {
        return std::nullopt;
    }

    const auto kAuthContext = ctx->auth_context();
    if (!kAuthContext || !kAuthContext->IsPeerAuthenticated()) {
        return std::nullopt;
    }

    const auto kValues = kAuthContext->FindPropertyValues(std::string(property_name));
    if (kValues.empty()) {
        return std::nullopt;
    }
    return std::string(kValues.front().data(), kValues.front().size());
}

[[nodiscard]] std::optional<std::string> ResolvePeerIdentity(grpc::ServerContext* ctx) {
    if (const auto kCommonName = FindPeerAuthProperty(ctx, "x509_common_name");
        kCommonName.has_value()) {
        return kCommonName;
    }
    if (const auto kSubjectAltName = FindPeerAuthProperty(ctx, "x509_subject_alternative_name");
        kSubjectAltName.has_value()) {
        return kSubjectAltName;
    }
    return std::nullopt;
}

[[nodiscard]] core::Result AuthorizePeer(grpc::ServerContext* ctx,
                                         const AgentSecurityConfig& security,
                                         std::string* requested_client_id) {
    const auto kPeerIdentity = ResolvePeerIdentity(ctx);
    if (!kPeerIdentity.has_value()) {
        return core::Result::Rejected("authenticated peer identity is required");
    }

    if (!security.allowed_client_ids.empty() &&
        std::ranges::find(security.allowed_client_ids, *kPeerIdentity) ==
            security.allowed_client_ids.end()) {
        return core::Result::Rejected("peer identity '" + *kPeerIdentity + "' is not authorized");
    }

    if (requested_client_id == nullptr) {
        return core::Result::Success();
    }
    if (requested_client_id->empty()) {
        *requested_client_id = *kPeerIdentity;
        return core::Result::Success();
    }
    if (*requested_client_id != *kPeerIdentity) {
        return core::Result::Rejected(
            "request client_id must match the authenticated peer identity");
    }
    return core::Result::Success();
}

[[nodiscard]] std::shared_ptr<grpc::ServerCredentials> MakeServerCredentials(
    const AgentSecurityConfig& security, core::Result* out_error) {
    if (out_error != nullptr) {
        *out_error = core::Result::Success();
    }

    std::string cert_chain;
    if (const core::Result kResult =
            core::internal::ReadTextFile(security.cert_chain_path, &cert_chain);
        !kResult.IsOk()) {
        if (out_error != nullptr) {
            *out_error = kResult;
        }
        return {};
    }

    std::string private_key;
    if (const core::Result kResult =
            core::internal::ReadTextFile(security.private_key_path, &private_key);
        !kResult.IsOk()) {
        if (out_error != nullptr) {
            *out_error = kResult;
        }
        return {};
    }

    grpc::SslServerCredentialsOptions options;
    options.pem_key_cert_pairs.push_back(grpc::SslServerCredentialsOptions::PemKeyCertPair{
        .private_key = private_key,
        .cert_chain = cert_chain,
    });

    if (const core::Result kResult =
            core::internal::ReadTextFile(security.root_ca_cert_path, &options.pem_root_certs);
        !kResult.IsOk()) {
        if (out_error != nullptr) {
            *out_error = kResult;
        }
        return {};
    }
    options.client_certificate_request = GRPC_SSL_REQUEST_AND_REQUIRE_CLIENT_CERTIFICATE_AND_VERIFY;

    return grpc::SslServerCredentials(options);
}

/// @}

/// @name Proto conversion helpers
/// @{

[[nodiscard]] swarmkit::v1::ErrorCode ToProtoErrorCode(core::StatusCode code) {
    using ProtoCode = swarmkit::v1::ErrorCode;
    switch (code) {
        case core::StatusCode::kOk:
            return ProtoCode::ERROR_CODE_NONE;
        case core::StatusCode::kRejected:
            return ProtoCode::ERROR_CODE_REJECTED;
        case core::StatusCode::kFailed:
            return ProtoCode::ERROR_CODE_INTERNAL;
    }
    return ProtoCode::ERROR_CODE_INTERNAL;
}

[[nodiscard]] swarmkit::v1::CommandReply::Status ToProtoStatus(core::StatusCode code) {
    using ReplyStatus = swarmkit::v1::CommandReply::Status;
    switch (code) {
        case core::StatusCode::kOk:
            return ReplyStatus::CommandReply_Status_OK;
        case core::StatusCode::kRejected:
            return ReplyStatus::CommandReply_Status_REJECTED;
        case core::StatusCode::kFailed:
            return ReplyStatus::CommandReply_Status_FAILED;
    }
    return ReplyStatus::CommandReply_Status_STATUS_UNSPECIFIED;
}

[[nodiscard]] CommandContext ToCoreContext(const swarmkit::v1::CommandContext& proto) {
    CommandContext context;
    context.drone_id = proto.drone_id();
    context.client_id = proto.client_id();
    context.priority = static_cast<CommandPriority>(proto.priority());
    context.correlation_id = proto.correlation_id();

    if (proto.deadline_unix_ms() > 0) {
        context.deadline = std::chrono::system_clock::time_point{
            std::chrono::milliseconds{proto.deadline_unix_ms()}};
    }

    return context;
}

[[nodiscard]] std::string ProtoCommandName(const swarmkit::v1::Command& proto) {
    switch (proto.kind_case()) {
        case swarmkit::v1::Command::kArm:
            return "ARM";
        case swarmkit::v1::Command::kDisarm:
            return "DISARM";
        case swarmkit::v1::Command::kLand:
            return "LAND";
        case swarmkit::v1::Command::kSetMode:
            return "SET_MODE(" + proto.set_mode().mode() + ")";
        case swarmkit::v1::Command::kForceDisarm:
            return "FORCE_DISARM";
        case swarmkit::v1::Command::kFlightTerminate:
            return "FLIGHT_TERMINATE";
        case swarmkit::v1::Command::kTakeoff:
            return "TAKEOFF(" + std::to_string(static_cast<int>(proto.takeoff().alt_m())) + "m)";
        case swarmkit::v1::Command::kSetWaypoint:
            return "WAYPOINT(lat=" + std::to_string(proto.set_waypoint().lat_deg()) +
                   " lon=" + std::to_string(proto.set_waypoint().lon_deg()) +
                   " alt=" + std::to_string(static_cast<int>(proto.set_waypoint().alt_m())) + "m)";
        case swarmkit::v1::Command::kReturnHome:
            return "RETURN_HOME";
        case swarmkit::v1::Command::kHoldPosition:
            return "HOLD";
        case swarmkit::v1::Command::kSetSpeed:
            return "SET_SPEED";
        case swarmkit::v1::Command::kGotoPosition:
            return "GOTO";
        case swarmkit::v1::Command::kPause:
            return "PAUSE";
        case swarmkit::v1::Command::kResume:
            return "RESUME";
        case swarmkit::v1::Command::kSetYaw:
            return "SET_YAW";
        case swarmkit::v1::Command::kVelocity:
            return "VELOCITY";
        case swarmkit::v1::Command::kSetHome:
            return "SET_HOME";
        case swarmkit::v1::Command::kSetRole:
            return "SET_ROLE";
        case swarmkit::v1::Command::kSetFormation:
            return "SET_FORMATION";
        case swarmkit::v1::Command::kRunSequence:
            return "RUN_SEQUENCE";
        case swarmkit::v1::Command::kUploadMission:
            return "UPLOAD_MISSION";
        case swarmkit::v1::Command::kClearMission:
            return "CLEAR_MISSION";
        case swarmkit::v1::Command::kStartMission:
            return "START_MISSION";
        case swarmkit::v1::Command::kPauseMission:
            return "PAUSE_MISSION";
        case swarmkit::v1::Command::kResumeMission:
            return "RESUME_MISSION";
        case swarmkit::v1::Command::kSetCurrentMissionItem:
            return "SET_CURRENT_MISSION_ITEM";
        case swarmkit::v1::Command::kPhoto:
            return "PHOTO";
        case swarmkit::v1::Command::kPhotoIntervalStart:
            return "PHOTO_INTERVAL_START";
        case swarmkit::v1::Command::kPhotoIntervalStop:
            return "PHOTO_INTERVAL_STOP";
        case swarmkit::v1::Command::kVideoStart:
            return "VIDEO_START";
        case swarmkit::v1::Command::kVideoStop:
            return "VIDEO_STOP";
        case swarmkit::v1::Command::kGimbalPoint:
            return "GIMBAL_POINT";
        case swarmkit::v1::Command::kRoiLocation:
            return "ROI_LOCATION";
        case swarmkit::v1::Command::kRoiClear:
            return "ROI_CLEAR";
        case swarmkit::v1::Command::kServo:
            return "SERVO";
        case swarmkit::v1::Command::kRelay:
            return "RELAY";
        case swarmkit::v1::Command::kGripper:
            return "GRIPPER";
        default:
            return "UNKNOWN";
    }
}

[[nodiscard]] swarmkit::v1::AuthorityEvent::Kind ToProtoEventKind(AuthorityEvent::Kind kind) {
    using ProtoKind = swarmkit::v1::AuthorityEvent::Kind;
    switch (kind) {
        case AuthorityEvent::Kind::kGranted:
            return ProtoKind::AuthorityEvent_Kind_GRANTED;
        case AuthorityEvent::Kind::kPreempted:
            return ProtoKind::AuthorityEvent_Kind_PREEMPTED;
        case AuthorityEvent::Kind::kResumed:
            return ProtoKind::AuthorityEvent_Kind_RESUMED;
        case AuthorityEvent::Kind::kExpired:
            return ProtoKind::AuthorityEvent_Kind_EXPIRED;
    }
    return ProtoKind::AuthorityEvent_Kind_KIND_UNSPECIFIED;
}

[[nodiscard]] std::expected<Command, core::Result> ConvertProtoCommand(
    const swarmkit::v1::Command& proto) {
    switch (proto.kind_case()) {
        case swarmkit::v1::Command::kArm:
            return FlightCmd{CmdArm{}};
        case swarmkit::v1::Command::kDisarm:
            return FlightCmd{CmdDisarm{}};
        case swarmkit::v1::Command::kLand:
            return FlightCmd{CmdLand{}};
        case swarmkit::v1::Command::kTakeoff:
            return FlightCmd{CmdTakeoff{proto.takeoff().alt_m()}};
        case swarmkit::v1::Command::kSetMode:
            return FlightCmd{CmdSetMode{
                .mode = proto.set_mode().mode(),
                .custom_mode = proto.set_mode().custom_mode(),
            }};
        case swarmkit::v1::Command::kForceDisarm:
            return FlightCmd{CmdForceDisarm{}};
        case swarmkit::v1::Command::kFlightTerminate:
            return FlightCmd{CmdFlightTerminate{}};

        case swarmkit::v1::Command::kSetWaypoint: {
            CmdSetWaypoint waypoint;
            waypoint.lat_deg = proto.set_waypoint().lat_deg();
            waypoint.lon_deg = proto.set_waypoint().lon_deg();
            waypoint.alt_m = proto.set_waypoint().alt_m();
            waypoint.speed_mps = proto.set_waypoint().speed_mps();
            return NavCmd{waypoint};
        }
        case swarmkit::v1::Command::kReturnHome:
            return NavCmd{CmdReturnHome{}};
        case swarmkit::v1::Command::kHoldPosition:
            return NavCmd{CmdHoldPosition{}};
        case swarmkit::v1::Command::kSetSpeed:
            return NavCmd{CmdSetSpeed{proto.set_speed().ground_mps()}};
        case swarmkit::v1::Command::kGotoPosition: {
            CmdGoto go_to;
            go_to.lat_deg = proto.goto_position().lat_deg();
            go_to.lon_deg = proto.goto_position().lon_deg();
            go_to.alt_m = proto.goto_position().alt_m();
            go_to.speed_mps = proto.goto_position().speed_mps();
            go_to.yaw_deg = proto.goto_position().yaw_deg();
            go_to.use_yaw = proto.goto_position().use_yaw();
            return NavCmd{go_to};
        }
        case swarmkit::v1::Command::kPause:
            return NavCmd{CmdPause{}};
        case swarmkit::v1::Command::kResume:
            return NavCmd{CmdResume{}};
        case swarmkit::v1::Command::kSetYaw:
            return NavCmd{CmdSetYaw{
                .yaw_deg = proto.set_yaw().yaw_deg(),
                .rate_deg_s = proto.set_yaw().rate_deg_s(),
                .relative = proto.set_yaw().relative(),
            }};
        case swarmkit::v1::Command::kVelocity:
            return NavCmd{CmdVelocity{
                .vx_mps = proto.velocity().vx_mps(),
                .vy_mps = proto.velocity().vy_mps(),
                .vz_mps = proto.velocity().vz_mps(),
                .duration_ms = proto.velocity().duration_ms(),
                .body_frame = proto.velocity().body_frame(),
            }};
        case swarmkit::v1::Command::kSetHome: {
            CmdSetHome home;
            home.use_current = proto.set_home().use_current();
            home.lat_deg = proto.set_home().lat_deg();
            home.lon_deg = proto.set_home().lon_deg();
            home.alt_m = proto.set_home().alt_m();
            return NavCmd{home};
        }

        case swarmkit::v1::Command::kUploadMission: {
            CmdUploadMission upload;
            upload.items.reserve(static_cast<std::size_t>(proto.upload_mission().items_size()));
            for (const auto& proto_item : proto.upload_mission().items()) {
                MissionItem item;
                item.command = static_cast<std::uint16_t>(proto_item.command());
                item.lat_deg = proto_item.lat_deg();
                item.lon_deg = proto_item.lon_deg();
                item.alt_m = proto_item.alt_m();
                item.param1 = proto_item.param1();
                item.param2 = proto_item.param2();
                item.param3 = proto_item.param3();
                item.param4 = proto_item.param4();
                item.current = proto_item.current();
                item.autocontinue = proto_item.autocontinue();
                upload.items.push_back(item);
            }
            return MissionCmd{std::move(upload)};
        }
        case swarmkit::v1::Command::kClearMission:
            return MissionCmd{CmdClearMission{}};
        case swarmkit::v1::Command::kStartMission:
            return MissionCmd{CmdStartMission{
                .first_item = proto.start_mission().first_item(),
                .last_item = proto.start_mission().last_item(),
            }};
        case swarmkit::v1::Command::kPauseMission:
            return MissionCmd{CmdPauseMission{}};
        case swarmkit::v1::Command::kResumeMission:
            return MissionCmd{CmdResumeMission{}};
        case swarmkit::v1::Command::kSetCurrentMissionItem:
            return MissionCmd{CmdSetCurrentMissionItem{proto.set_current_mission_item().seq()}};

        case swarmkit::v1::Command::kSetRole:
            return SwarmCmd{CmdSetRole{proto.set_role().role()}};
        case swarmkit::v1::Command::kSetFormation: {
            CmdSetFormation formation;
            formation.formation_id = proto.set_formation().formation_id();
            formation.slot_index = proto.set_formation().slot_index();
            return SwarmCmd{std::move(formation)};
        }
        case swarmkit::v1::Command::kRunSequence: {
            CmdRunSequence sequence;
            sequence.sequence_id = proto.run_sequence().sequence_id();
            sequence.sync_unix_ms = proto.run_sequence().sync_unix_ms();
            return SwarmCmd{std::move(sequence)};
        }

        case swarmkit::v1::Command::kPhoto:
            return PayloadCmd{CmdPhoto{proto.photo().camera_id()}};
        case swarmkit::v1::Command::kPhotoIntervalStart:
            return PayloadCmd{CmdPhotoIntervalStart{
                .interval_s = proto.photo_interval_start().interval_s(),
                .count = proto.photo_interval_start().count(),
                .camera_id = proto.photo_interval_start().camera_id(),
            }};
        case swarmkit::v1::Command::kPhotoIntervalStop:
            return PayloadCmd{CmdPhotoIntervalStop{proto.photo_interval_stop().camera_id()}};
        case swarmkit::v1::Command::kVideoStart:
            return PayloadCmd{CmdVideoStart{
                .stream_id = proto.video_start().stream_id(),
                .camera_id = proto.video_start().camera_id(),
            }};
        case swarmkit::v1::Command::kVideoStop:
            return PayloadCmd{CmdVideoStop{
                .stream_id = proto.video_stop().stream_id(),
                .camera_id = proto.video_stop().camera_id(),
            }};
        case swarmkit::v1::Command::kGimbalPoint:
            return PayloadCmd{CmdGimbalPoint{
                .pitch_deg = proto.gimbal_point().pitch_deg(),
                .roll_deg = proto.gimbal_point().roll_deg(),
                .yaw_deg = proto.gimbal_point().yaw_deg(),
            }};
        case swarmkit::v1::Command::kRoiLocation: {
            CmdRoiLocation roi;
            roi.lat_deg = proto.roi_location().lat_deg();
            roi.lon_deg = proto.roi_location().lon_deg();
            roi.alt_m = proto.roi_location().alt_m();
            roi.gimbal_id = proto.roi_location().gimbal_id();
            return PayloadCmd{roi};
        }
        case swarmkit::v1::Command::kRoiClear:
            return PayloadCmd{CmdRoiClear{proto.roi_clear().gimbal_id()}};
        case swarmkit::v1::Command::kServo:
            return PayloadCmd{CmdServo{
                .servo = proto.servo().servo(),
                .pwm = proto.servo().pwm(),
            }};
        case swarmkit::v1::Command::kRelay:
            return PayloadCmd{CmdRelay{
                .relay = proto.relay().relay(),
                .enabled = proto.relay().enabled(),
            }};
        case swarmkit::v1::Command::kGripper:
            return PayloadCmd{CmdGripper{
                .gripper = proto.gripper().gripper(),
                .release = proto.gripper().release(),
            }};

        default:
            return std::unexpected(core::Result::Rejected("unknown command kind"));
    }
}

/// @}

[[nodiscard]] double DegToRad(double degrees) {
    return degrees * std::numbers::pi / 180.0;
}

[[nodiscard]] double DistanceMeters(double lat_a_deg, double lon_a_deg, double lat_b_deg,
                                    double lon_b_deg) {
    constexpr double kEarthRadiusMeters = 6371000.0;
    const double lat_a = DegToRad(lat_a_deg);
    const double lat_b = DegToRad(lat_b_deg);
    const double delta_lat = DegToRad(lat_b_deg - lat_a_deg);
    const double delta_lon = DegToRad(lon_b_deg - lon_a_deg);
    const double sin_lat = std::sin(delta_lat / 2.0);
    const double sin_lon = std::sin(delta_lon / 2.0);
    const double haversine =
        (sin_lat * sin_lat) + (std::cos(lat_a) * std::cos(lat_b) * sin_lon * sin_lon);
    return 2.0 * kEarthRadiusMeters * std::atan2(std::sqrt(haversine), std::sqrt(1.0 - haversine));
}

struct LocalPointMeters {
    double x{};
    double y{};
};

[[nodiscard]] LocalPointMeters ProjectMeters(const core::TelemetryFrame& origin, double lat_deg,
                                             double lon_deg) {
    constexpr double kMetersPerDegreeLat = 111320.0;
    const double meters_per_degree_lon = kMetersPerDegreeLat * std::cos(DegToRad(origin.lat_deg));
    return {
        .x = (lon_deg - origin.lon_deg) * meters_per_degree_lon,
        .y = (lat_deg - origin.lat_deg) * kMetersPerDegreeLat,
    };
}

[[nodiscard]] double CorridorDeviationMeters(const core::TelemetryFrame& origin,
                                             const core::TelemetryFrame& current,
                                             const swarmkit::v1::GeoPoint& target) {
    const LocalPointMeters target_point = ProjectMeters(origin, target.lat_deg(), target.lon_deg());
    const LocalPointMeters current_point = ProjectMeters(origin, current.lat_deg, current.lon_deg);
    const double segment_len_sq =
        (target_point.x * target_point.x) + (target_point.y * target_point.y);
    if (segment_len_sq <= 1.0) {
        return DistanceMeters(current.lat_deg, current.lon_deg, target.lat_deg(), target.lon_deg());
    }
    const double projection =
        ((current_point.x * target_point.x) + (current_point.y * target_point.y)) / segment_len_sq;
    const double clamped_projection = std::clamp(projection, 0.0, 1.0);
    const double closest_x = target_point.x * clamped_projection;
    const double closest_y = target_point.y * clamped_projection;
    const double delta_x = current_point.x - closest_x;
    const double delta_y = current_point.y - closest_y;
    return std::sqrt((delta_x * delta_x) + (delta_y * delta_y));
}

[[nodiscard]] std::int64_t ComputeGoalTimeoutMs(const VehicleProfile& profile,
                                                const swarmkit::v1::ActiveGoal& goal,
                                                const std::optional<core::TelemetryFrame>& frame) {
    if (goal.timeout_ms() > 0) {
        return goal.timeout_ms();
    }
    double travel_seconds = 30.0;
    if (frame.has_value()) {
        const double distance_m =
            DistanceMeters(frame->lat_deg, frame->lon_deg, goal.target().lat_deg(),
                           goal.target().lon_deg());
        const float speed_mps = goal.speed_mps() > 0.0F ? goal.speed_mps() : profile.cruise_speed_mps;
        const double horizontal_seconds = speed_mps > 0.0F ? distance_m / speed_mps : 0.0;
        const double alt_delta = goal.target().alt_m() - frame->rel_alt_m;
        const float vertical_speed =
            alt_delta >= 0.0 ? profile.climb_speed_mps : profile.descent_speed_mps;
        const double vertical_seconds =
            vertical_speed > 0.0F ? std::abs(alt_delta) / vertical_speed : 0.0;
        travel_seconds = std::max(horizontal_seconds, vertical_seconds);
    }
    const auto calculated_ms = static_cast<std::int64_t>((travel_seconds * 1000.0) +
                                                        profile.goal_margin_ms);
    return std::clamp<std::int64_t>(calculated_ms, profile.goal_margin_ms,
                                    profile.max_goal_timeout_ms);
}

class ReportQueue {
   public:
    void Push(swarmkit::v1::AgentReport report) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (shutdown_) {
                return;
            }
            queue_.push(std::move(report));
        }
        cv_.notify_one();
    }

    [[nodiscard]] bool Pop(swarmkit::v1::AgentReport* out, std::chrono::milliseconds timeout) {
        if (out == nullptr) {
            return false;
        }
        std::unique_lock<std::mutex> lock(mutex_);
        const bool ready =
            cv_.wait_for(lock, timeout, [this] { return !queue_.empty() || shutdown_; });
        if (!ready || queue_.empty()) {
            return false;
        }
        *out = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    void Shutdown() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            shutdown_ = true;
        }
        cv_.notify_all();
    }

   private:
    std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<swarmkit::v1::AgentReport> queue_;
    bool shutdown_{false};
};

struct ReportWatchToken {
    std::uint64_t watch_id{};
};

class ReportHub {
   public:
    ReportWatchToken Watch(std::string drone_id, std::uint64_t after_sequence,
                           const std::shared_ptr<ReportQueue>& queue) {
        std::vector<swarmkit::v1::AgentReport> backlog;
        ReportWatchToken token;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            token.watch_id = ++next_watch_id_;
            watchers_.emplace(token.watch_id, Watcher{
                                                  .drone_id = std::move(drone_id),
                                                  .queue = queue,
                                              });
            for (const auto& report : backlog_) {
                if (report.sequence() > after_sequence && Matches(watchers_.at(token.watch_id),
                                                                  report)) {
                    backlog.push_back(report);
                }
            }
        }
        for (auto& report : backlog) {
            if (queue) {
                queue->Push(std::move(report));
            }
        }
        return token;
    }

    void Unwatch(ReportWatchToken token) {
        std::lock_guard<std::mutex> lock(mutex_);
        watchers_.erase(token.watch_id);
    }

    void Publish(swarmkit::v1::AgentReport report) {
        std::vector<std::shared_ptr<ReportQueue>> queues;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            report.set_sequence(++next_sequence_);
            report.set_unix_time_ms(NowUnixMs());
            backlog_.push_back(report);
            constexpr std::size_t kMaxBacklog = 1000;
            while (backlog_.size() > kMaxBacklog) {
                backlog_.pop_front();
            }
            for (auto iter = watchers_.begin(); iter != watchers_.end();) {
                if (auto queue = iter->second.queue.lock()) {
                    if (Matches(iter->second, report)) {
                        queues.push_back(std::move(queue));
                    }
                    ++iter;
                } else {
                    iter = watchers_.erase(iter);
                }
            }
        }
        for (const auto& queue : queues) {
            queue->Push(report);
        }
    }

   private:
    struct Watcher {
        std::string drone_id;
        std::weak_ptr<ReportQueue> queue;
    };

    [[nodiscard]] static bool Matches(const Watcher& watcher,
                                      const swarmkit::v1::AgentReport& report) {
        return watcher.drone_id.empty() || watcher.drone_id == "all" ||
               watcher.drone_id == report.drone_id();
    }

    std::mutex mutex_;
    std::unordered_map<std::uint64_t, Watcher> watchers_;
    std::deque<swarmkit::v1::AgentReport> backlog_;
    std::uint64_t next_watch_id_{0};
    std::uint64_t next_sequence_{0};
};

class ActiveGoalSupervisor {
   public:
    ActiveGoalSupervisor(internal::TelemetryManager* telemetry, ReportHub* reports,
                         const AgentConfig* config)
        : telemetry_(telemetry), reports_(reports), config_(config) {}

    ~ActiveGoalSupervisor() noexcept {
        try {
            Shutdown();
        } catch (const std::exception& exc) {
            core::Logger::ErrorFmt("ActiveGoalSupervisor shutdown failed: {}", exc.what());
        } catch (...) {
            core::Logger::Error("ActiveGoalSupervisor shutdown failed with unknown exception");
        }
    }

    ActiveGoalSupervisor(const ActiveGoalSupervisor&) = delete;
    ActiveGoalSupervisor& operator=(const ActiveGoalSupervisor&) = delete;

    [[nodiscard]] std::int64_t StartGoal(swarmkit::v1::ActiveGoal goal,
                                         std::string correlation_id) {
        std::thread old_worker;
        std::shared_ptr<std::atomic<bool>> old_stop;
        std::int64_t out_timeout{};
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (auto iter = goals_.find(goal.drone_id()); iter != goals_.end()) {
                old_stop = iter->second.stop;
                PublishGoalReport(iter->second.goal, swarmkit::v1::GOAL_SUPERSEDED,
                                  swarmkit::v1::REPORT_INFO, 0.0, 0.0, 0.0,
                                  iter->second.computed_timeout_ms, iter->second.started_unix_ms,
                                  "superseded by newer active goal",
                                  iter->second.correlation_id);
                old_worker = std::move(iter->second.worker);
                goals_.erase(iter);
            }
            if (old_stop) {
                old_stop->store(true, std::memory_order_relaxed);
            }

            ActiveGoalRuntime runtime;
            runtime.goal = std::move(goal);
            runtime.correlation_id = std::move(correlation_id);
            runtime.started_unix_ms = NowUnixMs();
            runtime.computed_timeout_ms =
                ComputeGoalTimeoutMs(config_->vehicle_profile, runtime.goal, std::nullopt);
            runtime.status = swarmkit::v1::GOAL_ACTIVE;
            runtime.stop = std::make_shared<std::atomic<bool>>(false);
            const std::string drone_id = runtime.goal.drone_id();
            const auto stop = runtime.stop;
            const auto runtime_goal = runtime.goal;
            const std::int64_t timeout_ms = runtime.computed_timeout_ms;
            const std::string runtime_correlation_id = runtime.correlation_id;
            const std::int64_t started_ms = runtime.started_unix_ms;
            // NOLINTNEXTLINE(bugprone-exception-escape): the thread entry catches all exceptions.
            runtime.worker = std::thread([this, runtime_goal, timeout_ms, started_ms,
                                          runtime_correlation_id, stop] noexcept {
                try {
                    MonitorGoal(runtime_goal, timeout_ms, started_ms, runtime_correlation_id, stop);
                } catch (...) {
                    static_cast<void>(
                        std::fputs("Active goal monitor failed\n", stderr));
                }
            });
            PublishGoalReport(runtime.goal, swarmkit::v1::GOAL_ACTIVE, swarmkit::v1::REPORT_INFO,
                              0.0, 0.0, 0.0, runtime.computed_timeout_ms,
                              runtime.started_unix_ms, "active goal accepted",
                              runtime.correlation_id);
            out_timeout = runtime.computed_timeout_ms;
            goals_.emplace(drone_id, std::move(runtime));
        }
        if (old_worker.joinable()) {
            old_worker.join();
        }
        return out_timeout;
    }

    [[nodiscard]] core::Result CancelGoal(const std::string& drone_id, std::string_view goal_id,
                                          std::string_view correlation_id) {
        std::thread old_worker;
        std::shared_ptr<std::atomic<bool>> old_stop;
        swarmkit::v1::ActiveGoal old_goal;
        std::int64_t timeout_ms{};
        std::int64_t started_ms{};
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto iter = goals_.find(drone_id);
            if (iter == goals_.end()) {
                return core::Result::Rejected("no active goal for drone");
            }
            if (!goal_id.empty() && iter->second.goal.goal_id() != goal_id) {
                return core::Result::Rejected("active goal id does not match cancel request");
            }
            old_goal = iter->second.goal;
            timeout_ms = iter->second.computed_timeout_ms;
            started_ms = iter->second.started_unix_ms;
            old_stop = iter->second.stop;
            old_worker = std::move(iter->second.worker);
            goals_.erase(iter);
        }
        if (old_stop) {
            old_stop->store(true, std::memory_order_relaxed);
        }
        if (old_worker.joinable()) {
            old_worker.join();
        }
        PublishGoalReport(old_goal, swarmkit::v1::GOAL_CANCELLED, swarmkit::v1::REPORT_INFO, 0.0,
                          0.0, 0.0, timeout_ms, started_ms, "goal cancelled",
                          std::string(correlation_id));
        return core::Result::Success("goal cancelled");
    }

    [[nodiscard]] std::optional<std::pair<swarmkit::v1::ActiveGoal, std::int64_t>> GetGoal(
        const std::string& drone_id, swarmkit::v1::GoalStatus* status) const {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto iter = goals_.find(drone_id);
        if (iter == goals_.end()) {
            return std::nullopt;
        }
        if (status != nullptr) {
            *status = iter->second.status;
        }
        return std::make_pair(iter->second.goal, iter->second.computed_timeout_ms);
    }

    void Shutdown() {
        std::vector<std::thread> workers;
        std::vector<std::shared_ptr<std::atomic<bool>>> stops;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (auto& [_, runtime] : goals_) {
                stops.push_back(runtime.stop);
                workers.push_back(std::move(runtime.worker));
            }
            goals_.clear();
        }
        for (const auto& stop : stops) {
            if (stop) {
                stop->store(true, std::memory_order_relaxed);
            }
        }
        for (auto& worker : workers) {
            if (worker.joinable()) {
                worker.join();
            }
        }
    }

   private:
    struct ActiveGoalRuntime {
        swarmkit::v1::ActiveGoal goal;
        swarmkit::v1::GoalStatus status{swarmkit::v1::GOAL_STATUS_UNSPECIFIED};
        std::int64_t computed_timeout_ms{};
        std::int64_t started_unix_ms{};
        std::string correlation_id;
        std::shared_ptr<std::atomic<bool>> stop;
        std::thread worker;
    };

    void SetTerminalStatus(const std::string& drone_id, swarmkit::v1::GoalStatus status) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (auto iter = goals_.find(drone_id); iter != goals_.end()) {
            iter->second.status = status;
        }
    }

    void PublishGoalReport(const swarmkit::v1::ActiveGoal& goal, swarmkit::v1::GoalStatus status,
                           swarmkit::v1::ReportSeverity severity, double distance_to_goal_m,
                           double deviation_m, double altitude_error_m,
                           std::int64_t timeout_ms, std::int64_t started_ms,
                           std::string_view message, std::string_view correlation_id) {
        if (reports_ == nullptr) {
            return;
        }
        swarmkit::v1::AgentReport report;
        report.set_drone_id(goal.drone_id());
        report.set_correlation_id(std::string(correlation_id));
        report.set_type(swarmkit::v1::GOAL_REPORT);
        report.set_severity(severity);
        report.set_message(std::string(message));
        auto* goal_report = report.mutable_goal();
        goal_report->set_drone_id(goal.drone_id());
        goal_report->set_goal_id(goal.goal_id());
        goal_report->set_revision(goal.revision());
        goal_report->set_status(status);
        goal_report->set_distance_to_goal_m(distance_to_goal_m);
        goal_report->set_deviation_m(deviation_m);
        goal_report->set_altitude_error_m(altitude_error_m);
        goal_report->set_acceptance_radius_m(goal.acceptance_radius_m());
        goal_report->set_deviation_radius_m(goal.deviation_radius_m());
        goal_report->set_elapsed_ms(std::max<std::int64_t>(0, NowUnixMs() - started_ms));
        goal_report->set_timeout_ms(timeout_ms);
        goal_report->set_message(std::string(message));
        reports_->Publish(std::move(report));
    }

    void MonitorGoal(const swarmkit::v1::ActiveGoal& goal, std::int64_t configured_timeout_ms,
                     std::int64_t started_ms, const std::string& correlation_id,
                     const std::shared_ptr<std::atomic<bool>>& stop) {
        internal::TelemetryLease lease;
        const core::Result acquire_result = telemetry_->AcquireLease(
            goal.drone_id(), std::max(1, config_->default_telemetry_rate_hz), &lease);
        if (!acquire_result.IsOk()) {
            PublishGoalReport(goal, swarmkit::v1::GOAL_FAILED, swarmkit::v1::REPORT_ERROR, 0.0, 0.0,
                              0.0, configured_timeout_ms, started_ms,
                              "telemetry unavailable: " + acquire_result.message, correlation_id);
            SetTerminalStatus(goal.drone_id(), swarmkit::v1::GOAL_FAILED);
            return;
        }

        std::uint64_t last_sequence = 0;
        std::optional<core::TelemetryFrame> origin;
        bool reported_deviation = false;
        std::int64_t timeout_ms = configured_timeout_ms;

        while (stop && !stop->load(std::memory_order_relaxed)) {
            core::TelemetryFrame frame;
            if (!internal::TelemetryManager::WaitForFrame(lease, &last_sequence, &frame,
                                                          kTelemetryWaitTimeout)) {
                if (NowUnixMs() - started_ms >= timeout_ms) {
                    PublishGoalReport(goal, swarmkit::v1::GOAL_TIMEOUT,
                                      swarmkit::v1::REPORT_ERROR, 0.0, 0.0, 0.0, timeout_ms,
                                      started_ms, "goal timed out without fresh telemetry",
                                      correlation_id);
                    SetTerminalStatus(goal.drone_id(), swarmkit::v1::GOAL_TIMEOUT);
                    break;
                }
                continue;
            }
            if (!origin.has_value()) {
                origin = frame;
                timeout_ms = ComputeGoalTimeoutMs(config_->vehicle_profile, goal, origin);
            }

            const double distance_to_goal_m =
                DistanceMeters(frame.lat_deg, frame.lon_deg, goal.target().lat_deg(),
                               goal.target().lon_deg());
            const double deviation_m = CorridorDeviationMeters(*origin, frame, goal.target());
            const double altitude_error_m = std::abs(frame.rel_alt_m - goal.target().alt_m());
            const bool altitude_ok =
                altitude_error_m <= std::max(1.0F, goal.acceptance_radius_m());

            if (distance_to_goal_m <= goal.acceptance_radius_m() && altitude_ok) {
                PublishGoalReport(goal, swarmkit::v1::GOAL_REACHED, swarmkit::v1::REPORT_INFO,
                                  distance_to_goal_m, deviation_m, altitude_error_m, timeout_ms,
                                  started_ms, "goal reached", correlation_id);
                SetTerminalStatus(goal.drone_id(), swarmkit::v1::GOAL_REACHED);
                break;
            }

            if (goal.deviation_radius_m() > 0.0F && deviation_m > goal.deviation_radius_m()) {
                if (!reported_deviation) {
                    PublishGoalReport(goal, swarmkit::v1::GOAL_DEVIATING,
                                      swarmkit::v1::REPORT_WARNING, distance_to_goal_m,
                                      deviation_m, altitude_error_m, timeout_ms, started_ms,
                                      "outside goal deviation radius", correlation_id);
                    reported_deviation = true;
                }
            } else if (reported_deviation) {
                PublishGoalReport(goal, swarmkit::v1::GOAL_ACTIVE, swarmkit::v1::REPORT_INFO,
                                  distance_to_goal_m, deviation_m, altitude_error_m, timeout_ms,
                                  started_ms, "goal back inside deviation envelope",
                                  correlation_id);
                reported_deviation = false;
            }

            if (NowUnixMs() - started_ms >= timeout_ms) {
                PublishGoalReport(goal, swarmkit::v1::GOAL_TIMEOUT, swarmkit::v1::REPORT_ERROR,
                                  distance_to_goal_m, deviation_m, altitude_error_m, timeout_ms,
                                  started_ms, "goal timed out", correlation_id);
                SetTerminalStatus(goal.drone_id(), swarmkit::v1::GOAL_TIMEOUT);
                break;
            }
        }

        telemetry_->ReleaseLease(lease);
    }

    internal::TelemetryManager* telemetry_{nullptr};
    ReportHub* reports_{nullptr};
    const AgentConfig* config_{nullptr};
    mutable std::mutex mutex_;
    std::unordered_map<std::string, ActiveGoalRuntime> goals_;
};

/// @name AgentServiceImpl — gRPC service implementation
/// @{

/**
 * @brief Implements the AgentService gRPC service.
 *
 * Delegates telemetry management to TelemetryManager and runtime
 * counting to RuntimeCounters, keeping RPC handlers focused.
 */
class AgentServiceImpl final : public swarmkit::v1::AgentService::Service {
   public:
    AgentServiceImpl(AgentConfig config, DroneBackendPtr backend)
        : config_(std::move(config)),
          backend_(std::move(backend)),
          telemetry_(backend_.get(), config_.default_telemetry_rate_hz,
                     config_.min_telemetry_rate_hz),
          goals_(&telemetry_, &reports_, &config_) {
        if (const core::Result start_result = backend_->Start(); !start_result.IsOk()) {
            ready_.store(false, std::memory_order_relaxed);
            startup_error_ = start_result.message;
            core::Logger::ErrorFmt("Agent backend failed to start: {}", start_result.message);
        }
    }

    // -- Unary RPCs -----------------------------------------------------------

    grpc::Status Ping(grpc::ServerContext* ctx, const swarmkit::v1::PingRequest* /*req*/,
                      swarmkit::v1::PingReply* reply) override {
        if (reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null response");
        }
        if (const core::Result kAuthResult = AuthorizePeer(ctx, config_.security, nullptr);
            !kAuthResult.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, kAuthResult.message);
        }

        counters_.IncrementPingRequests();
        const std::string kCorrelationId = ResolveCorrelationId(ctx, "", "ping");

        reply->set_agent_id(config_.agent_id);
        reply->set_version(core::kSwarmkitVersionString);
        reply->set_unix_time_ms(NowUnixMs());
        reply->set_correlation_id(kCorrelationId);
        return grpc::Status::OK;
    }

    grpc::Status GetHealth(grpc::ServerContext* ctx, const swarmkit::v1::HealthRequest* /*req*/,
                           swarmkit::v1::HealthReply* reply) override {
        if (reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null response");
        }
        if (const core::Result kAuthResult = AuthorizePeer(ctx, config_.security, nullptr);
            !kAuthResult.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, kAuthResult.message);
        }

        counters_.IncrementHealthRequests();
        const std::string kCorrelationId = ResolveCorrelationId(ctx, "", "health");
        BackendHealth backend_health = backend_->GetHealth();
        if (!startup_error_.empty()) {
            backend_health.ready = false;
            backend_health.message = startup_error_;
        }
        const bool kIsReady = ready_.load(std::memory_order_relaxed) && backend_health.ready;

        reply->set_ok(true);
        reply->set_ready(kIsReady);
        reply->set_agent_id(config_.agent_id);
        reply->set_version(core::kSwarmkitVersionString);
        reply->set_unix_time_ms(NowUnixMs());
        reply->set_message(kIsReady ? backend_health.message
                                    : "not ready: " + backend_health.message);
        reply->set_correlation_id(kCorrelationId);
        return grpc::Status::OK;
    }

    grpc::Status GetRuntimeStats(grpc::ServerContext* ctx,
                                 const swarmkit::v1::RuntimeStatsRequest* /*req*/,
                                 swarmkit::v1::RuntimeStatsReply* reply) override {
        if (reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null response");
        }
        if (const core::Result kAuthResult = AuthorizePeer(ctx, config_.security, nullptr);
            !kAuthResult.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, kAuthResult.message);
        }

        counters_.IncrementRuntimeStatsRequests();

        // Sync telemetry counters from the manager into the atomic counters.
        counters_.SetTelemetryCounters(telemetry_.TotalSubscriptionCount(),
                                       telemetry_.ActiveStreamCount(), telemetry_.FramesSentTotal(),
                                       telemetry_.BackendFailureCount());

        const internal::CounterSnapshot kSnap = counters_.Snapshot();
        const std::string kCorrelationId = ResolveCorrelationId(ctx, "", "stats");

        reply->set_agent_id(config_.agent_id);
        reply->set_unix_time_ms(NowUnixMs());
        reply->set_correlation_id(kCorrelationId);
        reply->set_ping_requests_total(kSnap.ping_requests_total);
        reply->set_health_requests_total(kSnap.health_requests_total);
        reply->set_runtime_stats_requests_total(kSnap.runtime_stats_requests_total);
        reply->set_command_requests_total(kSnap.command_requests_total);
        reply->set_command_rejected_total(kSnap.command_rejected_total);
        reply->set_command_failed_total(kSnap.command_failed_total);
        reply->set_lock_requests_total(kSnap.lock_requests_total);
        reply->set_watch_requests_total(kSnap.watch_requests_total);
        reply->set_current_authority_watchers(kSnap.current_authority_watchers);
        reply->set_total_telemetry_subscriptions(kSnap.total_telemetry_subscriptions);
        reply->set_current_telemetry_streams(kSnap.current_telemetry_streams);
        reply->set_telemetry_frames_sent_total(kSnap.telemetry_frames_sent_total);
        reply->set_backend_failures_total(kSnap.backend_failures_total);
        reply->set_ready(ready_.load(std::memory_order_relaxed));
        return grpc::Status::OK;
    }

    // -- Command RPC ----------------------------------------------------------

    grpc::Status SendCommand(grpc::ServerContext* ctx, const swarmkit::v1::CommandRequest* req,
                             swarmkit::v1::CommandReply* reply) override {
        if (req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request/response");
        }

        counters_.IncrementCommandRequests();

        if (!req->has_cmd()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing cmd field");
        }

        CommandEnvelope envelope;
        if (req->has_ctx()) {
            envelope.context = ToCoreContext(req->ctx());
        }
        envelope.context.correlation_id =
            ResolveCorrelationId(ctx, envelope.context.correlation_id, "command");
        if (const core::Result kAuthResult =
                AuthorizePeer(ctx, config_.security, &envelope.context.client_id);
            !kAuthResult.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, kAuthResult.message);
        }

        if (const core::Result kValidation = ValidateCommandContext(envelope.context);
            !kValidation.IsOk()) {
            if (kValidation.code == core::StatusCode::kFailed) {
                reply->set_status(swarmkit::v1::CommandReply::FAILED);
                reply->set_message("command deadline already expired");
                reply->set_correlation_id(envelope.context.correlation_id);
                reply->set_error_code(swarmkit::v1::ERROR_CODE_DEADLINE_EXCEEDED);
                reply->set_debug_message(kValidation.message);
                counters_.IncrementCommandFailed();
                return grpc::Status::OK;
            }
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, kValidation.message);
        }

        const std::string kCmdName = ProtoCommandName(req->cmd());

        std::chrono::milliseconds ttl{config_.default_authority_ttl_ms};
        const auto kEpoch = std::chrono::system_clock::time_point{};
        if (envelope.context.deadline != kEpoch) {
            const auto kRemaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                envelope.context.deadline - std::chrono::system_clock::now());
            if (kRemaining.count() > 0) {
                ttl = kRemaining;
            }
        }

        const core::Result kArbiterResult = arbiter_.CheckAndGrant(envelope.context, ttl);
        if (!kArbiterResult.IsOk()) {
            counters_.IncrementCommandRejected();
            core::Logger::WarnFmt(
                "rpc=SendCommand corr={} agent={} drone={} client={} priority={} peer={} "
                "result=rejected reason={}",
                envelope.context.correlation_id, config_.agent_id, envelope.context.drone_id,
                envelope.context.client_id, static_cast<int>(envelope.context.priority),
                ctx->peer(), kArbiterResult.message);
            reply->set_status(ToProtoStatus(kArbiterResult.code));
            reply->set_message(kArbiterResult.message);
            reply->set_correlation_id(envelope.context.correlation_id);
            reply->set_error_code(ToProtoErrorCode(kArbiterResult.code));
            reply->set_debug_message(kArbiterResult.message);
            return grpc::Status::OK;
        }

        core::Logger::InfoFmt(
            "rpc=SendCommand corr={} agent={} drone={} client={} priority={} peer={} command={}",
            envelope.context.correlation_id, config_.agent_id, envelope.context.drone_id,
            envelope.context.client_id, static_cast<int>(envelope.context.priority), ctx->peer(),
            kCmdName);

        auto convert_result = ConvertProtoCommand(req->cmd());
        if (!convert_result.has_value()) {
            const auto& error = convert_result.error();
            counters_.IncrementCommandFailed();
            reply->set_status(swarmkit::v1::CommandReply::FAILED);
            reply->set_message("invalid command payload");
            reply->set_correlation_id(envelope.context.correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_INVALID_ARGUMENT);
            reply->set_debug_message(error.message);
            return grpc::Status::OK;
        }
        envelope.command = std::move(convert_result.value());

        const core::Result kExecResult = backend_->Execute(envelope);
        reply->set_correlation_id(envelope.context.correlation_id);
        reply->set_error_code(ToProtoErrorCode(kExecResult.code));
        if (kExecResult.IsOk()) {
            reply->set_status(swarmkit::v1::CommandReply::OK);
            reply->set_message(kExecResult.message);
        } else {
            counters_.IncrementCommandFailed();
            counters_.IncrementBackendFailures();
            core::Logger::ErrorFmt(
                "rpc=SendCommand corr={} agent={} drone={} client={} result=backend_failure "
                "detail={}",
                envelope.context.correlation_id, config_.agent_id, envelope.context.drone_id,
                envelope.context.client_id, kExecResult.message);
            reply->set_status(swarmkit::v1::CommandReply::FAILED);
            reply->set_message(kExecResult.message.empty() ? "command execution failed"
                                                           : kExecResult.message);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_BACKEND_FAILURE);
            reply->set_debug_message(kExecResult.message);
        }
        return grpc::Status::OK;
    }

    grpc::Status SetActiveGoal(grpc::ServerContext* ctx,
                               const swarmkit::v1::SetActiveGoalRequest* req,
                               swarmkit::v1::SetActiveGoalReply* reply) override {
        if (req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request/response");
        }
        if (!req->has_ctx() || !req->has_goal()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing ctx or goal field");
        }

        CommandContext context = ToCoreContext(req->ctx());
        context.correlation_id = ResolveCorrelationId(ctx, context.correlation_id, "goal");
        if (const core::Result auth_result = AuthorizePeer(ctx, config_.security, &context.client_id);
            !auth_result.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth_result.message);
        }
        if (context.drone_id.empty()) {
            context.drone_id = req->goal().drone_id();
        }
        if (const core::Result validation = ValidateCommandContext(context); !validation.IsOk()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, validation.message);
        }

        swarmkit::v1::ActiveGoal goal = req->goal();
        if (goal.drone_id().empty()) {
            goal.set_drone_id(context.drone_id);
        }
        if (goal.drone_id() != context.drone_id) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "goal.drone_id must match ctx.drone_id");
        }
        if (const core::Result validation = ValidateActiveGoal(goal); !validation.IsOk()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, validation.message);
        }

        const core::Result arbiter_result =
            arbiter_.CheckAndGrant(context, std::chrono::milliseconds{config_.default_authority_ttl_ms});
        if (!arbiter_result.IsOk()) {
            reply->set_ok(false);
            reply->set_message(arbiter_result.message);
            reply->set_correlation_id(context.correlation_id);
            reply->set_error_code(ToProtoErrorCode(arbiter_result.code));
            reply->set_debug_message(arbiter_result.message);
            PublishSimpleReport(context.drone_id, context.correlation_id,
                                swarmkit::v1::AUTHORITY_REJECTED,
                                swarmkit::v1::REPORT_WARNING, arbiter_result.message);
            return grpc::Status::OK;
        }

        CommandEnvelope envelope;
        envelope.context = context;
        envelope.command = NavCmd{CmdGoto{
            .lat_deg = goal.target().lat_deg(),
            .lon_deg = goal.target().lon_deg(),
            .alt_m = goal.target().alt_m(),
            .speed_mps = goal.speed_mps(),
        }};

        const core::Result exec_result = backend_->Execute(envelope);
        if (!exec_result.IsOk()) {
            counters_.IncrementBackendFailures();
            reply->set_ok(false);
            reply->set_message(exec_result.message.empty() ? "active goal command failed"
                                                           : exec_result.message);
            reply->set_correlation_id(context.correlation_id);
            reply->set_error_code(swarmkit::v1::ERROR_CODE_BACKEND_FAILURE);
            reply->set_debug_message(exec_result.message);
            *reply->mutable_goal() = goal;
            PublishGoalFailure(goal, context.correlation_id, exec_result.message);
            return grpc::Status::OK;
        }

        const std::int64_t computed_timeout_ms = goals_.StartGoal(goal, context.correlation_id);
        reply->set_ok(true);
        reply->set_message(exec_result.message);
        reply->set_correlation_id(context.correlation_id);
        reply->set_error_code(swarmkit::v1::ERROR_CODE_NONE);
        *reply->mutable_goal() = goal;
        reply->set_computed_timeout_ms(computed_timeout_ms);
        return grpc::Status::OK;
    }

    grpc::Status CancelGoal(grpc::ServerContext* ctx,
                            const swarmkit::v1::CancelGoalRequest* req,
                            swarmkit::v1::CancelGoalReply* reply) override {
        if (req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request/response");
        }
        if (!req->has_ctx()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing ctx field");
        }

        CommandContext context = ToCoreContext(req->ctx());
        context.correlation_id = ResolveCorrelationId(ctx, context.correlation_id, "cancel-goal");
        if (const core::Result auth_result = AuthorizePeer(ctx, config_.security, &context.client_id);
            !auth_result.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth_result.message);
        }
        if (const core::Result validation = ValidateCommandContext(context); !validation.IsOk()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, validation.message);
        }

        const core::Result arbiter_result =
            arbiter_.CheckAndGrant(context, std::chrono::milliseconds{config_.default_authority_ttl_ms});
        if (!arbiter_result.IsOk()) {
            reply->set_ok(false);
            reply->set_message(arbiter_result.message);
            reply->set_correlation_id(context.correlation_id);
            reply->set_error_code(ToProtoErrorCode(arbiter_result.code));
            reply->set_debug_message(arbiter_result.message);
            PublishSimpleReport(context.drone_id, context.correlation_id,
                                swarmkit::v1::AUTHORITY_REJECTED,
                                swarmkit::v1::REPORT_WARNING, arbiter_result.message);
            return grpc::Status::OK;
        }

        const core::Result result =
            goals_.CancelGoal(context.drone_id, req->goal_id(), context.correlation_id);
        reply->set_ok(result.IsOk());
        reply->set_message(result.message);
        reply->set_correlation_id(context.correlation_id);
        reply->set_error_code(ToProtoErrorCode(result.code));
        reply->set_debug_message(result.message);
        return grpc::Status::OK;
    }

    grpc::Status GetActiveGoal(grpc::ServerContext* ctx,
                               const swarmkit::v1::GetActiveGoalRequest* req,
                               swarmkit::v1::GetActiveGoalReply* reply) override {
        if (req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request/response");
        }
        if (const core::Result auth_result = AuthorizePeer(ctx, config_.security, nullptr);
            !auth_result.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth_result.message);
        }
        if (req->drone_id().empty()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "drone_id must not be empty");
        }

        swarmkit::v1::GoalStatus status = swarmkit::v1::GOAL_STATUS_UNSPECIFIED;
        const auto goal = goals_.GetGoal(req->drone_id(), &status);
        if (!goal.has_value()) {
            reply->set_goal_present(false);
            reply->set_status(swarmkit::v1::GOAL_STATUS_UNSPECIFIED);
            reply->set_message("no active goal");
            return grpc::Status::OK;
        }
        reply->set_goal_present(true);
        *reply->mutable_goal() = goal->first;
        reply->set_status(status);
        reply->set_computed_timeout_ms(goal->second);
        reply->set_message("goal found");
        return grpc::Status::OK;
    }

    // -- Authority RPCs -------------------------------------------------------

    grpc::Status LockAuthority(grpc::ServerContext* ctx,
                               const swarmkit::v1::LockAuthorityRequest* req,
                               swarmkit::v1::LockAuthorityReply* reply) override {
        if (req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request/response");
        }
        if (!req->has_ctx()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing ctx field");
        }

        counters_.IncrementLockRequests();

        CommandContext lock_context = ToCoreContext(req->ctx());
        lock_context.correlation_id =
            ResolveCorrelationId(ctx, lock_context.correlation_id, "lock");
        if (const core::Result kAuthResult =
                AuthorizePeer(ctx, config_.security, &lock_context.client_id);
            !kAuthResult.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, kAuthResult.message);
        }
        const auto kTtlDuration = std::chrono::milliseconds{req->ttl_ms()};

        if (const core::Result kValidation = ValidateLockRequest(lock_context, req->ttl_ms());
            !kValidation.IsOk()) {
            if (kValidation.code == core::StatusCode::kFailed) {
                reply->set_ok(false);
                reply->set_message("command deadline already expired");
                reply->set_correlation_id(lock_context.correlation_id);
                reply->set_error_code(swarmkit::v1::ERROR_CODE_DEADLINE_EXCEEDED);
                reply->set_debug_message(kValidation.message);
                return grpc::Status::OK;
            }
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, kValidation.message);
        }

        const core::Result kResult = arbiter_.CheckAndGrant(lock_context, kTtlDuration);

        reply->set_ok(kResult.IsOk());
        reply->set_message(kResult.message);
        reply->set_correlation_id(lock_context.correlation_id);
        reply->set_error_code(ToProtoErrorCode(kResult.code));
        reply->set_debug_message(kResult.message);

        if (kResult.IsOk()) {
            core::Logger::InfoFmt(
                "rpc=LockAuthority corr={} agent={} drone={} client={} priority={} ttl_ms={} "
                "result=granted",
                lock_context.correlation_id, config_.agent_id, lock_context.drone_id,
                lock_context.client_id, static_cast<int>(lock_context.priority), req->ttl_ms());
        } else {
            core::Logger::WarnFmt(
                "rpc=LockAuthority corr={} agent={} drone={} client={} priority={} ttl_ms={} "
                "result=rejected reason={}",
                lock_context.correlation_id, config_.agent_id, lock_context.drone_id,
                lock_context.client_id, static_cast<int>(lock_context.priority), req->ttl_ms(),
                kResult.message);
        }
        return grpc::Status::OK;
    }

    grpc::Status ReleaseAuthority(grpc::ServerContext* ctx,
                                  const swarmkit::v1::ReleaseAuthorityRequest* req,
                                  swarmkit::v1::ReleaseAuthorityReply* /*reply*/) override {
        if (req == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request");
        }
        std::string client_id = req->client_id();
        if (const core::Result kAuthResult = AuthorizePeer(ctx, config_.security, &client_id);
            !kAuthResult.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, kAuthResult.message);
        }
        if (req->drone_id().empty() || client_id.empty()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "drone_id and client_id must not be empty");
        }
        const std::string kCorrelationId = ResolveCorrelationId(ctx, "", "unlock");
        arbiter_.Release(req->drone_id(), client_id);
        core::Logger::InfoFmt(
            "rpc=ReleaseAuthority corr={} agent={} drone={} client={} result=released",
            kCorrelationId, config_.agent_id, req->drone_id(), client_id);
        return grpc::Status::OK;
    }

    // -- Streaming RPCs -------------------------------------------------------

    grpc::Status StreamTelemetry(
        grpc::ServerContext* ctx, const swarmkit::v1::TelemetryRequest* req,
        grpc::ServerWriter<swarmkit::v1::TelemetryFrame>* writer) override {
        if (ctx == nullptr || writer == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/writer");
        }
        if (const core::Result kAuthResult = AuthorizePeer(ctx, config_.security, nullptr);
            !kAuthResult.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, kAuthResult.message);
        }
        if (const core::Result kValidation = ValidateTelemetryRequest(req); !kValidation.IsOk()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, kValidation.message);
        }

        const std::string kPeer = ctx->peer();
        const std::string kStreamId = ResolveCorrelationId(ctx, "", "telemetry");
        const std::string kDroneId =
            (req != nullptr && !req->drone_id().empty()) ? req->drone_id() : "default";
        const int kRequestedRateHz = (req != nullptr) ? req->rate_hz() : 0;
        const int kEffectiveRateHz =
            std::max(config_.min_telemetry_rate_hz,
                     kRequestedRateHz <= 0 ? config_.default_telemetry_rate_hz : kRequestedRateHz);
        const auto kEmitPeriod =
            std::chrono::milliseconds(kMillisecondsPerSecond / kEffectiveRateHz);

        internal::TelemetryLease lease;
        const core::Result kAcquireResult =
            telemetry_.AcquireLease(kDroneId, kEffectiveRateHz, &lease);
        if (!kAcquireResult.IsOk()) {
            core::Logger::ErrorFmt(
                "rpc=StreamTelemetry corr={} agent={} drone={} peer={} rate_hz={} "
                "result=acquire_failed detail={}",
                kStreamId, config_.agent_id, kDroneId, kPeer, kEffectiveRateHz,
                kAcquireResult.message);
            return grpc::Status(grpc::StatusCode::UNAVAILABLE, kAcquireResult.message);
        }

        core::Logger::InfoFmt(
            "rpc=StreamTelemetry corr={} agent={} drone={} peer={} rate_hz={} result=connected",
            kStreamId, config_.agent_id, kDroneId, kPeer, kEffectiveRateHz);

        std::uint64_t last_sequence = 0;
        auto next_emit_at = std::chrono::steady_clock::now();

        while (!ctx->IsCancelled()) {
            core::TelemetryFrame frame;
            if (!internal::TelemetryManager::WaitForFrame(lease, &last_sequence, &frame,
                                                          kTelemetryWaitTimeout)) {
                continue;
            }

            const auto kNow = std::chrono::steady_clock::now();
            if (kNow < next_emit_at) {
                continue;
            }

            swarmkit::v1::TelemetryFrame out;
            out.set_drone_id(frame.drone_id);
            out.set_unix_time_ms(frame.unix_time_ms);
            out.set_lat_deg(frame.lat_deg);
            out.set_lon_deg(frame.lon_deg);
            out.set_rel_alt_m(frame.rel_alt_m);
            out.set_battery_percent(frame.battery_percent);
            out.set_mode(frame.mode);
            out.set_stream_id(kStreamId);

            if (!writer->Write(out)) {
                break;
            }
            telemetry_.IncrementFramesSent();
            next_emit_at = kNow + kEmitPeriod;
        }

        telemetry_.ReleaseLease(lease);
        core::Logger::InfoFmt(
            "rpc=StreamTelemetry corr={} agent={} drone={} peer={} result=disconnected", kStreamId,
            config_.agent_id, kDroneId, kPeer);
        return grpc::Status::OK;
    }

    grpc::Status WatchAuthority(grpc::ServerContext* ctx,
                                const swarmkit::v1::WatchAuthorityRequest* req,
                                grpc::ServerWriter<swarmkit::v1::AuthorityEvent>* writer) override {
        if (ctx == nullptr || req == nullptr || writer == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/writer");
        }

        const std::string& drone_id = req->drone_id();
        std::string client_id = req->client_id();
        const auto kPriority = static_cast<CommandPriority>(req->priority());
        const std::string kStreamId = ResolveCorrelationId(ctx, "", "watch");
        if (const core::Result kAuthResult = AuthorizePeer(ctx, config_.security, &client_id);
            !kAuthResult.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, kAuthResult.message);
        }
        if (drone_id.empty() || client_id.empty() || !IsValidPriority(kPriority)) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "invalid watch authority request");
        }

        counters_.IncrementWatchRequests();
        counters_.IncrementAuthorityWatchers();
        auto queue = std::make_shared<EventQueue>();
        const WatchToken kToken = arbiter_.Watch(drone_id, client_id, kPriority, queue);

        core::Logger::InfoFmt(
            "rpc=WatchAuthority corr={} agent={} drone={} client={} priority={} result=connected",
            kStreamId, config_.agent_id, drone_id, client_id, static_cast<int>(kPriority));

        while (!ctx->IsCancelled()) {
            AuthorityEvent event;
            if (!queue->Pop(event, kWatchPollInterval)) {
                continue;
            }

            swarmkit::v1::AuthorityEvent proto_event;
            proto_event.set_kind(ToProtoEventKind(event.kind));
            proto_event.set_drone_id(event.drone_id);
            proto_event.set_holder_client_id(event.holder_client_id);
            proto_event.set_holder_priority(static_cast<int>(event.holder_priority));
            proto_event.set_correlation_id(kStreamId);

            if (!writer->Write(proto_event)) {
                break;
            }
        }

        queue->Shutdown();
        arbiter_.Unwatch(kToken);
        counters_.DecrementAuthorityWatchers();

        core::Logger::InfoFmt(
            "rpc=WatchAuthority corr={} agent={} drone={} client={} result=disconnected", kStreamId,
            config_.agent_id, drone_id, client_id);
        return grpc::Status::OK;
    }

    grpc::Status SubscribeReports(
        grpc::ServerContext* ctx, const swarmkit::v1::ReportSubscription* req,
        grpc::ServerWriter<swarmkit::v1::AgentReport>* writer) override {
        if (ctx == nullptr || req == nullptr || writer == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/request/writer");
        }
        std::string client_id = req->client_id();
        if (const core::Result auth_result = AuthorizePeer(ctx, config_.security, &client_id);
            !auth_result.IsOk()) {
            return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, auth_result.message);
        }

        const std::string stream_id = ResolveCorrelationId(ctx, "", "reports");
        auto queue = std::make_shared<ReportQueue>();
        const ReportWatchToken token =
            reports_.Watch(req->drone_id(), req->after_sequence(), queue);
        core::Logger::InfoFmt(
            "rpc=SubscribeReports corr={} agent={} drone={} client={} result=connected", stream_id,
            config_.agent_id, req->drone_id(), client_id);

        while (!ctx->IsCancelled()) {
            swarmkit::v1::AgentReport report;
            if (!queue->Pop(&report, kWatchPollInterval)) {
                continue;
            }
            if (!writer->Write(report)) {
                break;
            }
        }

        queue->Shutdown();
        reports_.Unwatch(token);
        core::Logger::InfoFmt(
            "rpc=SubscribeReports corr={} agent={} drone={} client={} result=disconnected",
            stream_id, config_.agent_id, req->drone_id(), client_id);
        return grpc::Status::OK;
    }

   private:
    [[nodiscard]] static core::Result ValidateActiveGoal(const swarmkit::v1::ActiveGoal& goal) {
        if (goal.drone_id().empty()) {
            return core::Result::Rejected("goal.drone_id must not be empty");
        }
        if (goal.goal_id().empty()) {
            return core::Result::Rejected("goal.goal_id must not be empty");
        }
        if (!goal.has_target()) {
            return core::Result::Rejected("goal.target must be set");
        }
        if (goal.target().lat_deg() < -90.0 || goal.target().lat_deg() > 90.0 ||
            goal.target().lon_deg() < -180.0 || goal.target().lon_deg() > 180.0) {
            return core::Result::Rejected("goal target latitude/longitude out of range");
        }
        if (goal.speed_mps() < 0.0F) {
            return core::Result::Rejected("goal.speed_mps must be >= 0");
        }
        if (goal.acceptance_radius_m() <= 0.0F) {
            return core::Result::Rejected("goal.acceptance_radius_m must be > 0");
        }
        if (goal.deviation_radius_m() < 0.0F) {
            return core::Result::Rejected("goal.deviation_radius_m must be >= 0");
        }
        if (goal.timeout_ms() < 0) {
            return core::Result::Rejected("goal.timeout_ms must be >= 0");
        }
        return core::Result::Success();
    }

    void PublishSimpleReport(std::string_view drone_id, std::string_view correlation_id,
                             swarmkit::v1::AgentReportType type,
                             swarmkit::v1::ReportSeverity severity, std::string_view message) {
        swarmkit::v1::AgentReport report;
        report.set_drone_id(std::string(drone_id));
        report.set_correlation_id(std::string(correlation_id));
        report.set_type(type);
        report.set_severity(severity);
        report.set_message(std::string(message));
        reports_.Publish(std::move(report));
    }

    void PublishGoalFailure(const swarmkit::v1::ActiveGoal& goal, std::string_view correlation_id,
                            std::string_view message) {
        swarmkit::v1::AgentReport report;
        report.set_drone_id(goal.drone_id());
        report.set_correlation_id(std::string(correlation_id));
        report.set_type(swarmkit::v1::GOAL_REPORT);
        report.set_severity(swarmkit::v1::REPORT_ERROR);
        report.set_message(std::string(message));
        auto* goal_report = report.mutable_goal();
        goal_report->set_drone_id(goal.drone_id());
        goal_report->set_goal_id(goal.goal_id());
        goal_report->set_revision(goal.revision());
        goal_report->set_status(swarmkit::v1::GOAL_FAILED);
        goal_report->set_acceptance_radius_m(goal.acceptance_radius_m());
        goal_report->set_deviation_radius_m(goal.deviation_radius_m());
        goal_report->set_timeout_ms(goal.timeout_ms());
        goal_report->set_message(std::string(message));
        reports_.Publish(std::move(report));
    }

    AgentConfig config_;
    DroneBackendPtr backend_;
    CommandArbiter arbiter_;
    internal::TelemetryManager telemetry_;
    ReportHub reports_;
    ActiveGoalSupervisor goals_;
    internal::RuntimeCounters counters_;
    std::atomic<bool> ready_{true};
    std::string startup_error_;
};

/// @}

}  // namespace

// ---------------------------------------------------------------------------
// AgentConfig
// ---------------------------------------------------------------------------

core::Result VehicleProfile::Validate() const {
    if (profile_id.empty()) {
        return core::Result::Rejected("vehicle_profile.profile_id must not be empty");
    }
    if (cruise_speed_mps <= 0.0F) {
        return core::Result::Rejected("vehicle_profile.cruise_speed_mps must be > 0");
    }
    if (climb_speed_mps <= 0.0F) {
        return core::Result::Rejected("vehicle_profile.climb_speed_mps must be > 0");
    }
    if (descent_speed_mps <= 0.0F) {
        return core::Result::Rejected("vehicle_profile.descent_speed_mps must be > 0");
    }
    if (goal_margin_ms < 0) {
        return core::Result::Rejected("vehicle_profile.goal_margin_ms must be >= 0");
    }
    if (max_goal_timeout_ms <= 0) {
        return core::Result::Rejected("vehicle_profile.max_goal_timeout_ms must be > 0");
    }
    return core::Result::Success();
}

core::Result AgentSecurityConfig::Validate() const {
    if (!core::internal::FileExists(cert_chain_path)) {
        return core::Result::Rejected(
            "security.cert_chain_path must point to an existing file for mTLS");
    }
    if (!core::internal::FileExists(private_key_path)) {
        return core::Result::Rejected(
            "security.private_key_path must point to an existing file for mTLS");
    }
    if (!core::internal::FileExists(root_ca_cert_path)) {
        return core::Result::Rejected(
            "security.root_ca_cert_path must point to an existing file for mTLS");
    }
    return core::Result::Success();
}

core::Result AgentConfig::Validate() const {
    if (agent_id.empty()) {
        return core::Result::Rejected("agent_id must not be empty");
    }
    if (!LooksLikeAddress(bind_addr)) {
        return core::Result::Rejected("bind_addr must be in host:port format");
    }
    if (default_authority_ttl_ms <= 0) {
        return core::Result::Rejected("default_authority_ttl_ms must be > 0");
    }
    if (default_telemetry_rate_hz <= 0) {
        return core::Result::Rejected("default_telemetry_rate_hz must be > 0");
    }
    if (min_telemetry_rate_hz <= 0) {
        return core::Result::Rejected("min_telemetry_rate_hz must be > 0");
    }
    if (min_telemetry_rate_hz > default_telemetry_rate_hz) {
        return core::Result::Rejected("min_telemetry_rate_hz must be <= default_telemetry_rate_hz");
    }
    if (const core::Result vehicle_result = vehicle_profile.Validate(); !vehicle_result.IsOk()) {
        return vehicle_result;
    }
    return security.Validate();
}

void AgentConfig::ApplyEnvironment(std::string_view prefix) {
    using core::internal::ApplyIntEnv;
    using core::internal::ApplyStringEnv;

    ApplyStringEnv(prefix, kAgentEnvId, &agent_id);
    ApplyStringEnv(prefix, kAgentEnvBindAddr, &bind_addr);
    ApplyIntEnv(prefix, kAgentEnvDefaultAuthorityTtlMs, &default_authority_ttl_ms);
    ApplyIntEnv(prefix, kAgentEnvDefaultTelemetryRateHz, &default_telemetry_rate_hz);
    ApplyIntEnv(prefix, kAgentEnvMinTelemetryRateHz, &min_telemetry_rate_hz);

    ApplyStringEnv(prefix, kAgentEnvRootCaCertPath, &security.root_ca_cert_path);
    ApplyStringEnv(prefix, kAgentEnvCertChainPath, &security.cert_chain_path);
    ApplyStringEnv(prefix, kAgentEnvPrivateKeyPath, &security.private_key_path);
    if (const auto kValue =
            GetEnvValue(std::string(prefix) + std::string(kAgentEnvAllowedClientIds));
        kValue.has_value()) {
        security.allowed_client_ids = SplitCsvList(*kValue);
    }
}

std::expected<AgentConfig, core::Result> LoadAgentConfigFromFile(const std::string& path) {
    const auto loaded_yaml = core::yaml::LoadYamlFile(path);
    if (!loaded_yaml.has_value()) {
        return std::unexpected(loaded_yaml.error());
    }

    AgentConfig config;
    const YAML::Node root = core::yaml::SelectSection(*loaded_yaml, "agent");
    if (!root || !root.IsMap()) {
        return std::unexpected(core::Result::Rejected("agent YAML config must be a map"));
    }

    const auto agent_id = core::yaml::ReadOptionalScalar<std::string>(root, "agent_id");
    if (!agent_id.has_value()) {
        return std::unexpected(agent_id.error());
    }
    if (agent_id->has_value()) {
        config.agent_id = agent_id->value_or(config.agent_id);
    }

    const auto bind_addr = core::yaml::ReadOptionalScalar<std::string>(root, "bind_addr");
    if (!bind_addr.has_value()) {
        return std::unexpected(bind_addr.error());
    }
    if (bind_addr->has_value()) {
        config.bind_addr = bind_addr->value_or(config.bind_addr);
    }

    const auto default_authority_ttl_ms =
        core::yaml::ReadOptionalScalar<int>(root, "default_authority_ttl_ms");
    if (!default_authority_ttl_ms.has_value()) {
        return std::unexpected(default_authority_ttl_ms.error());
    }
    if (default_authority_ttl_ms->has_value()) {
        config.default_authority_ttl_ms =
            default_authority_ttl_ms->value_or(config.default_authority_ttl_ms);
    }

    const auto default_telemetry_rate_hz =
        core::yaml::ReadOptionalScalar<int>(root, "default_telemetry_rate_hz");
    if (!default_telemetry_rate_hz.has_value()) {
        return std::unexpected(default_telemetry_rate_hz.error());
    }
    if (default_telemetry_rate_hz->has_value()) {
        config.default_telemetry_rate_hz =
            default_telemetry_rate_hz->value_or(config.default_telemetry_rate_hz);
    }

    const auto min_telemetry_rate_hz =
        core::yaml::ReadOptionalScalar<int>(root, "min_telemetry_rate_hz");
    if (!min_telemetry_rate_hz.has_value()) {
        return std::unexpected(min_telemetry_rate_hz.error());
    }
    if (min_telemetry_rate_hz->has_value()) {
        config.min_telemetry_rate_hz =
            min_telemetry_rate_hz->value_or(config.min_telemetry_rate_hz);
    }

    if (const YAML::Node vehicle_profile = root["vehicle_profile"]; vehicle_profile) {
        if (!vehicle_profile.IsMap()) {
            return std::unexpected(core::Result::Rejected("agent.vehicle_profile must be a map"));
        }

        const auto profile_id =
            core::yaml::ReadOptionalScalar<std::string>(vehicle_profile, "profile_id");
        if (!profile_id.has_value()) {
            return std::unexpected(profile_id.error());
        }
        if (profile_id->has_value()) {
            config.vehicle_profile.profile_id =
                profile_id->value_or(config.vehicle_profile.profile_id);
        }

        const auto cruise_speed_mps =
            core::yaml::ReadOptionalScalar<float>(vehicle_profile, "cruise_speed_mps");
        if (!cruise_speed_mps.has_value()) {
            return std::unexpected(cruise_speed_mps.error());
        }
        if (cruise_speed_mps->has_value()) {
            config.vehicle_profile.cruise_speed_mps =
                cruise_speed_mps->value_or(config.vehicle_profile.cruise_speed_mps);
        }

        const auto climb_speed_mps =
            core::yaml::ReadOptionalScalar<float>(vehicle_profile, "climb_speed_mps");
        if (!climb_speed_mps.has_value()) {
            return std::unexpected(climb_speed_mps.error());
        }
        if (climb_speed_mps->has_value()) {
            config.vehicle_profile.climb_speed_mps =
                climb_speed_mps->value_or(config.vehicle_profile.climb_speed_mps);
        }

        const auto descent_speed_mps =
            core::yaml::ReadOptionalScalar<float>(vehicle_profile, "descent_speed_mps");
        if (!descent_speed_mps.has_value()) {
            return std::unexpected(descent_speed_mps.error());
        }
        if (descent_speed_mps->has_value()) {
            config.vehicle_profile.descent_speed_mps =
                descent_speed_mps->value_or(config.vehicle_profile.descent_speed_mps);
        }

        const auto goal_margin_ms =
            core::yaml::ReadOptionalScalar<int>(vehicle_profile, "goal_margin_ms");
        if (!goal_margin_ms.has_value()) {
            return std::unexpected(goal_margin_ms.error());
        }
        if (goal_margin_ms->has_value()) {
            config.vehicle_profile.goal_margin_ms =
                goal_margin_ms->value_or(config.vehicle_profile.goal_margin_ms);
        }

        const auto max_goal_timeout_ms =
            core::yaml::ReadOptionalScalar<int>(vehicle_profile, "max_goal_timeout_ms");
        if (!max_goal_timeout_ms.has_value()) {
            return std::unexpected(max_goal_timeout_ms.error());
        }
        if (max_goal_timeout_ms->has_value()) {
            config.vehicle_profile.max_goal_timeout_ms =
                max_goal_timeout_ms->value_or(config.vehicle_profile.max_goal_timeout_ms);
        }
    }

    if (const YAML::Node security = root["security"]; security) {
        if (!security.IsMap()) {
            return std::unexpected(core::Result::Rejected("agent.security must be a map"));
        }

        const auto root_ca_cert_path =
            core::yaml::ReadOptionalScalar<std::string>(security, "root_ca_cert_path");
        if (!root_ca_cert_path.has_value()) {
            return std::unexpected(root_ca_cert_path.error());
        }
        if (root_ca_cert_path->has_value()) {
            config.security.root_ca_cert_path = core::internal::ResolveConfigRelativePath(
                path, root_ca_cert_path->value_or(std::string{}));
        }

        const auto cert_chain_path =
            core::yaml::ReadOptionalScalar<std::string>(security, "cert_chain_path");
        if (!cert_chain_path.has_value()) {
            return std::unexpected(cert_chain_path.error());
        }
        if (cert_chain_path->has_value()) {
            config.security.cert_chain_path = core::internal::ResolveConfigRelativePath(
                path, cert_chain_path->value_or(std::string{}));
        }

        const auto private_key_path =
            core::yaml::ReadOptionalScalar<std::string>(security, "private_key_path");
        if (!private_key_path.has_value()) {
            return std::unexpected(private_key_path.error());
        }
        if (private_key_path->has_value()) {
            config.security.private_key_path = core::internal::ResolveConfigRelativePath(
                path, private_key_path->value_or(std::string{}));
        }

        const YAML::Node allowed_client_ids = security["allowed_client_ids"];
        if (allowed_client_ids) {
            if (!allowed_client_ids.IsSequence()) {
                return std::unexpected(
                    core::Result::Rejected("agent.security.allowed_client_ids must be a sequence"));
            }
            config.security.allowed_client_ids.clear();
            config.security.allowed_client_ids.reserve(allowed_client_ids.size());
            for (const auto& entry : allowed_client_ids) {
                if (!entry.IsScalar()) {
                    return std::unexpected(core::Result::Rejected(
                        "agent.security.allowed_client_ids entries must be scalars"));
                }
                config.security.allowed_client_ids.push_back(entry.as<std::string>());
            }
        }
    }

    if (const core::Result kValidation = config.Validate(); !kValidation.IsOk()) {
        return std::unexpected(kValidation);
    }
    return config;
}

// ---------------------------------------------------------------------------
// RunAgentServer
// ---------------------------------------------------------------------------

int RunAgentServer(const AgentConfig& config, DroneBackendPtr backend) {
    if (!backend) {
        core::Logger::Error("RunAgentServer: backend is null");
        return 1;
    }
    if (const core::Result kValidation = config.Validate(); !kValidation.IsOk()) {
        core::Logger::ErrorFmt("RunAgentServer: invalid config - {}", kValidation.message);
        return 1;
    }

    grpc::ServerBuilder builder;
    core::Result credentials_error = core::Result::Success();
    auto credentials = MakeServerCredentials(config.security, &credentials_error);
    if (!credentials) {
        core::Logger::ErrorFmt("Failed to configure server credentials: {}",
                               credentials_error.message);
        return 1;
    }
    builder.AddListeningPort(config.bind_addr, credentials);

    auto service = internal::MakeAgentServiceForTesting(config, std::move(backend));
    builder.RegisterService(service.get());

    std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
    if (!server) {
        core::Logger::ErrorFmt("Failed to start gRPC server on {}", config.bind_addr);
        return 1;
    }

    core::Logger::InfoFmt("Agent '{}' listening on {}", config.agent_id, config.bind_addr);
    server->Wait();
    return 0;
}

std::unique_ptr<grpc::Service> internal::MakeAgentServiceForTesting(const AgentConfig& config,
                                                                    DroneBackendPtr backend) {
    return std::make_unique<AgentServiceImpl>(config, std::move(backend));
}

}  // namespace swarmkit::agent
