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
#include <cstdint>
#include <expected>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <utility>

#include "config_yaml.h"
#include "env_utils.h"
#include "runtime_counters.h"
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

/// @brief Watcher poll interval while blocking inside WatchAuthority RPC.
constexpr auto kWatchPollInterval = std::chrono::milliseconds{100};

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
        case swarmkit::v1::Command::kSetRole:
            return "SET_ROLE";
        case swarmkit::v1::Command::kSetFormation:
            return "SET_FORMATION";
        case swarmkit::v1::Command::kRunSequence:
            return "RUN_SEQUENCE";
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

        default:
            return std::unexpected(core::Result::Rejected("unknown command kind"));
    }
}

/// @}

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
                     config_.min_telemetry_rate_hz) {}

    // -- Unary RPCs -----------------------------------------------------------

    grpc::Status Ping(grpc::ServerContext* ctx, const swarmkit::v1::PingRequest* /*req*/,
                      swarmkit::v1::PingReply* reply) override {
        if (reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null response");
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

        counters_.IncrementHealthRequests();
        const std::string kCorrelationId = ResolveCorrelationId(ctx, "", "health");
        const bool kIsReady = ready_.load(std::memory_order_relaxed);

        reply->set_ok(true);
        reply->set_ready(kIsReady);
        reply->set_agent_id(config_.agent_id);
        reply->set_version(core::kSwarmkitVersionString);
        reply->set_unix_time_ms(NowUnixMs());
        reply->set_message(kIsReady ? "ready" : "not ready");
        reply->set_correlation_id(kCorrelationId);
        return grpc::Status::OK;
    }

    grpc::Status GetRuntimeStats(grpc::ServerContext* ctx,
                                 const swarmkit::v1::RuntimeStatsRequest* /*req*/,
                                 swarmkit::v1::RuntimeStatsReply* reply) override {
        if (reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null response");
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
            reply->set_message("command execution failed");
            reply->set_error_code(swarmkit::v1::ERROR_CODE_BACKEND_FAILURE);
            reply->set_debug_message(kExecResult.message);
        }
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
        if (req->drone_id().empty() || req->client_id().empty()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "drone_id and client_id must not be empty");
        }
        const std::string kCorrelationId = ResolveCorrelationId(ctx, "", "unlock");
        arbiter_.Release(req->drone_id(), req->client_id());
        core::Logger::InfoFmt(
            "rpc=ReleaseAuthority corr={} agent={} drone={} client={} result=released",
            kCorrelationId, config_.agent_id, req->drone_id(), req->client_id());
        return grpc::Status::OK;
    }

    // -- Streaming RPCs -------------------------------------------------------

    grpc::Status StreamTelemetry(
        grpc::ServerContext* ctx, const swarmkit::v1::TelemetryRequest* req,
        grpc::ServerWriter<swarmkit::v1::TelemetryFrame>* writer) override {
        if (ctx == nullptr || writer == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/writer");
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
        const auto kSleepPeriod =
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

        while (!ctx->IsCancelled()) {
            core::TelemetryFrame frame;
            if (internal::TelemetryManager::ReadFrame(lease, &last_sequence, &frame)) {
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
            }

            std::this_thread::sleep_for(kSleepPeriod);
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
        const std::string& client_id = req->client_id();
        const auto kPriority = static_cast<CommandPriority>(req->priority());
        const std::string kStreamId = ResolveCorrelationId(ctx, "", "watch");
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

   private:
    AgentConfig config_;
    DroneBackendPtr backend_;
    CommandArbiter arbiter_;
    internal::TelemetryManager telemetry_;
    internal::RuntimeCounters counters_;
    std::atomic<bool> ready_{true};
};

/// @}

}  // namespace

// ---------------------------------------------------------------------------
// AgentConfig
// ---------------------------------------------------------------------------

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
    return core::Result::Success();
}

void AgentConfig::ApplyEnvironment(std::string_view prefix) {
    using core::internal::ApplyIntEnv;
    using core::internal::ApplyStringEnv;

    ApplyStringEnv(prefix, kAgentEnvId, &agent_id);
    ApplyStringEnv(prefix, kAgentEnvBindAddr, &bind_addr);
    ApplyIntEnv(prefix, kAgentEnvDefaultAuthorityTtlMs, &default_authority_ttl_ms);
    ApplyIntEnv(prefix, kAgentEnvDefaultTelemetryRateHz, &default_telemetry_rate_hz);
    ApplyIntEnv(prefix, kAgentEnvMinTelemetryRateHz, &min_telemetry_rate_hz);
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

    if (const auto kValue = core::yaml::ReadOptionalScalar<std::string>(root, "agent_id");
        !kValue.has_value()) {
        return std::unexpected(kValue.error());
    } else if (kValue->has_value()) {
        config.agent_id = **kValue;
    }

    if (const auto kValue = core::yaml::ReadOptionalScalar<std::string>(root, "bind_addr");
        !kValue.has_value()) {
        return std::unexpected(kValue.error());
    } else if (kValue->has_value()) {
        config.bind_addr = **kValue;
    }

    if (const auto kValue = core::yaml::ReadOptionalScalar<int>(root, "default_authority_ttl_ms");
        !kValue.has_value()) {
        return std::unexpected(kValue.error());
    } else if (kValue->has_value()) {
        config.default_authority_ttl_ms = **kValue;
    }

    if (const auto kValue = core::yaml::ReadOptionalScalar<int>(root, "default_telemetry_rate_hz");
        !kValue.has_value()) {
        return std::unexpected(kValue.error());
    } else if (kValue->has_value()) {
        config.default_telemetry_rate_hz = **kValue;
    }

    if (const auto kValue = core::yaml::ReadOptionalScalar<int>(root, "min_telemetry_rate_hz");
        !kValue.has_value()) {
        return std::unexpected(kValue.error());
    } else if (kValue->has_value()) {
        config.min_telemetry_rate_hz = **kValue;
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
    builder.AddListeningPort(config.bind_addr, grpc::InsecureServerCredentials());

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
