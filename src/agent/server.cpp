// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "swarmkit/agent/server.h"
#include "server_test_support.h"

#include <grpcpp/grpcpp.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <expected>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "swarmkit/agent/arbiter.h"
#include "config_yaml.h"
#include "swarmkit/core/logger.h"
#include "swarmkit/core/version.h"
#include "swarmkit/v1/swarmkit.grpc.pb.h"
#include "swarmkit/v1/swarmkit.pb.h"

namespace swarmkit::agent {

using namespace swarmkit::commands;  // NOLINT(google-build-using-namespace)

namespace {
using grpc::Status;

constexpr int kDefaultTelemetryRateHz = 5;
constexpr int kMinTelemetryRateHz = 1;
constexpr int kMillisecondsPerSecond = 1000;
constexpr std::string_view kCorrelationMetadataKey = "x-correlation-id";
constexpr std::string_view kAgentEnvId = "AGENT_ID";
constexpr std::string_view kAgentEnvBindAddr = "BIND_ADDR";
constexpr std::string_view kAgentEnvDefaultAuthorityTtlMs = "DEFAULT_AUTHORITY_TTL_MS";
constexpr std::string_view kAgentEnvDefaultTelemetryRateHz = "DEFAULT_TELEMETRY_RATE_HZ";
constexpr std::string_view kAgentEnvMinTelemetryRateHz = "MIN_TELEMETRY_RATE_HZ";
constexpr std::uint64_t kInitialCorrelationSequence = 1;

[[nodiscard]] std::int64_t NowUnixMs();

class CorrelationIdGenerator {
   public:
    [[nodiscard]] std::string Next(std::string_view prefix) {
        const std::uint64_t kSequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
        return std::string(prefix) + "-" + std::to_string(NowUnixMs()) + "-" +
               std::to_string(kSequence);
    }

   private:
    std::atomic<std::uint64_t> next_sequence_{kInitialCorrelationSequence};
};

/// @brief Watcher poll interval while blocking inside WatchAuthority RPC.
constexpr auto kWatchPollInterval = std::chrono::milliseconds{100};

/// @name Helpers
/// @{

/// @brief Returns the current Unix time in milliseconds.
[[nodiscard]] std::int64_t NowUnixMs() {
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using std::chrono::system_clock;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

[[nodiscard]] CorrelationIdGenerator& GetCorrelationIdGenerator() {
    static CorrelationIdGenerator generator;
    return generator;
}

[[nodiscard]] std::string MakeCorrelationId(std::string_view prefix) {
    return GetCorrelationIdGenerator().Next(prefix);
}

[[nodiscard]] std::optional<std::string> GetEnvValue(std::string_view key) {
    const char* value = std::getenv(std::string(key).c_str());
    if (value == nullptr) {
        return std::nullopt;
    }
    return std::string(value);
}

[[nodiscard]] std::expected<int, core::Result> ParseIntValue(std::string_view value,
                                                             std::string_view field_name) {
    try {
        return std::stoi(std::string(value));
    } catch (const std::exception&) {
        return std::unexpected(
            core::Result::Rejected("invalid integer for '" + std::string(field_name) + "'"));
    }
}

[[nodiscard]] bool LooksLikeAddress(std::string_view address) {
    return !address.empty() && address.find(':') != std::string_view::npos;
}

[[nodiscard]] bool IsSupportedPriority(CommandPriority priority) {
    switch (priority) {
        case CommandPriority::kOperator:
        case CommandPriority::kSupervisor:
        case CommandPriority::kOverride:
        case CommandPriority::kEmergency:
            return true;
    }
    return false;
}

/// @brief Clamp telemetry rate to a valid positive value.
[[nodiscard]] int NormalizeTelemetryRate(int requested_rate_hz) {
    if (requested_rate_hz <= 0) {
        return kDefaultTelemetryRateHz;
    }
    return std::max(kMinTelemetryRateHz, requested_rate_hz);
}

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

/// @brief Map a core::StatusCode to the protobuf CommandReply::Status enum.
[[nodiscard]] swarmkit::v1::CommandReply::Status ToProtoStatus(core::StatusCode code) {
    using Status = swarmkit::v1::CommandReply::Status;
    switch (code) {
        case core::StatusCode::kOk:
            return Status::CommandReply_Status_OK;
        case core::StatusCode::kRejected:
            return Status::CommandReply_Status_REJECTED;
        case core::StatusCode::kFailed:
            return Status::CommandReply_Status_FAILED;
    }
    return Status::CommandReply_Status_STATUS_UNSPECIFIED;
}

/// @brief Convert a proto CommandContext to the C++ CommandContext struct.
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

[[nodiscard]] core::Result ValidateCommandContext(const CommandContext& context) {
    if (context.drone_id.empty()) {
        return core::Result::Rejected("ctx.drone_id must not be empty");
    }
    if (context.client_id.empty()) {
        return core::Result::Rejected("ctx.client_id must not be empty");
    }
    if (!IsSupportedPriority(context.priority)) {
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

/// @brief Return a short human-readable name for a proto Command oneof.
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

/// @brief Convert a C++ AuthorityEvent::Kind to the proto enum.
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

/**
 * @brief Convert a proto Command oneof into the nested C++ Command variant.
 *
 * Proto field numbers are grouped by category to match the C++ variant
 * structure (FlightCmd / NavCmd / SwarmCmd / PayloadCmd).
 *
 * @returns The converted Command on success, or a core::Result error for
 *          unimplemented or unknown kinds.
 */
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
 * Owns the drone backend, per-drone telemetry caches, and the CommandArbiter.
 * All RPC handlers are called from gRPC's thread pool; shared state is
 * protected by dedicated mutexes.
 */
class AgentServiceImpl final : public swarmkit::v1::AgentService::Service {
   public:
    AgentServiceImpl(AgentConfig config, DroneBackendPtr backend)
        : config_(std::move(config)), backend_(std::move(backend)) {}

    ~AgentServiceImpl() override {
        std::vector<std::string> drone_ids;
        {
            std::lock_guard<std::mutex> lock(telemetry_states_mutex_);
            drone_ids.reserve(telemetry_states_.size());
            for (const auto& [drone_id, state] : telemetry_states_) {
                static_cast<void>(state);
                drone_ids.push_back(drone_id);
            }
        }

        for (const auto& drone_id : drone_ids) {
            const core::Result kStopResult = backend_->StopTelemetry(drone_id);
            if (!kStopResult.IsOk()) {
                core::Logger::WarnFmt(
                    "AgentServiceImpl: StopTelemetry('{}') failed during "
                    "shutdown: {}",
                    drone_id, kStopResult.message);
            }
        }
    }

    grpc::Status Ping(grpc::ServerContext* ctx, const swarmkit::v1::PingRequest* /*req*/,
                      swarmkit::v1::PingReply* reply) override {
        if (reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null response");
        }

        counters_.ping_requests_total.fetch_add(1, std::memory_order_relaxed);
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

        counters_.health_requests_total.fetch_add(1, std::memory_order_relaxed);
        const std::string kCorrelationId = ResolveCorrelationId(ctx, "", "health");
        reply->set_ok(true);
        reply->set_ready(ready_.load(std::memory_order_relaxed));
        reply->set_agent_id(config_.agent_id);
        reply->set_version(core::kSwarmkitVersionString);
        reply->set_unix_time_ms(NowUnixMs());
        reply->set_message(ready_.load(std::memory_order_relaxed) ? "ready" : "not ready");
        reply->set_correlation_id(kCorrelationId);
        return grpc::Status::OK;
    }

    grpc::Status GetRuntimeStats(grpc::ServerContext* ctx,
                                 const swarmkit::v1::RuntimeStatsRequest* /*req*/,
                                 swarmkit::v1::RuntimeStatsReply* reply) override {
        if (reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null response");
        }

        counters_.runtime_stats_requests_total.fetch_add(1, std::memory_order_relaxed);
        const std::string kCorrelationId = ResolveCorrelationId(ctx, "", "stats");

        reply->set_agent_id(config_.agent_id);
        reply->set_unix_time_ms(NowUnixMs());
        reply->set_correlation_id(kCorrelationId);
        reply->set_ping_requests_total(
            counters_.ping_requests_total.load(std::memory_order_relaxed));
        reply->set_health_requests_total(
            counters_.health_requests_total.load(std::memory_order_relaxed));
        reply->set_runtime_stats_requests_total(
            counters_.runtime_stats_requests_total.load(std::memory_order_relaxed));
        reply->set_command_requests_total(
            counters_.command_requests_total.load(std::memory_order_relaxed));
        reply->set_command_rejected_total(
            counters_.command_rejected_total.load(std::memory_order_relaxed));
        reply->set_command_failed_total(
            counters_.command_failed_total.load(std::memory_order_relaxed));
        reply->set_lock_requests_total(
            counters_.lock_requests_total.load(std::memory_order_relaxed));
        reply->set_watch_requests_total(
            counters_.watch_requests_total.load(std::memory_order_relaxed));
        reply->set_current_authority_watchers(
            counters_.current_authority_watchers.load(std::memory_order_relaxed));
        reply->set_total_telemetry_subscriptions(
            counters_.total_telemetry_subscriptions.load(std::memory_order_relaxed));
        reply->set_current_telemetry_streams(
            counters_.current_telemetry_streams.load(std::memory_order_relaxed));
        reply->set_telemetry_frames_sent_total(
            counters_.telemetry_frames_sent_total.load(std::memory_order_relaxed));
        reply->set_backend_failures_total(
            counters_.backend_failures_total.load(std::memory_order_relaxed));
        reply->set_ready(ready_.load(std::memory_order_relaxed));
        return grpc::Status::OK;
    }

    grpc::Status SendCommand(grpc::ServerContext* ctx, const swarmkit::v1::CommandRequest* req,
                             swarmkit::v1::CommandReply* reply) override {
        if (req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request/response");
        }

        counters_.command_requests_total.fetch_add(1, std::memory_order_relaxed);

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
                counters_.command_failed_total.fetch_add(1, std::memory_order_relaxed);
                return grpc::Status::OK;
            }
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, kValidation.message);
        }

        const std::string kCmdName = ProtoCommandName(req->cmd());

        std::chrono::milliseconds ttl = std::chrono::milliseconds{config_.default_authority_ttl_ms};
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
            counters_.command_rejected_total.fetch_add(1, std::memory_order_relaxed);
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
            counters_.command_failed_total.fetch_add(1, std::memory_order_relaxed);
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
            counters_.command_failed_total.fetch_add(1, std::memory_order_relaxed);
            counters_.backend_failures_total.fetch_add(1, std::memory_order_relaxed);
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

    grpc::Status LockAuthority(grpc::ServerContext* ctx,
                               const swarmkit::v1::LockAuthorityRequest* req,
                               swarmkit::v1::LockAuthorityReply* reply) override {
        if (req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request/response");
        }
        if (!req->has_ctx()) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing ctx field");
        }

        counters_.lock_requests_total.fetch_add(1, std::memory_order_relaxed);

        CommandContext kContext = ToCoreContext(req->ctx());
        kContext.correlation_id = ResolveCorrelationId(ctx, kContext.correlation_id, "lock");
        const auto kTtlDuration = std::chrono::milliseconds{req->ttl_ms()};
        if (const core::Result kValidation = ValidateLockRequest(kContext, req->ttl_ms());
            !kValidation.IsOk()) {
            if (kValidation.code == core::StatusCode::kFailed) {
                reply->set_ok(false);
                reply->set_message("command deadline already expired");
                reply->set_correlation_id(kContext.correlation_id);
                reply->set_error_code(swarmkit::v1::ERROR_CODE_DEADLINE_EXCEEDED);
                reply->set_debug_message(kValidation.message);
                return grpc::Status::OK;
            }
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, kValidation.message);
        }
        const core::Result kResult = arbiter_.CheckAndGrant(kContext, kTtlDuration);

        reply->set_ok(kResult.IsOk());
        reply->set_message(kResult.message);
        reply->set_correlation_id(kContext.correlation_id);
        reply->set_error_code(ToProtoErrorCode(kResult.code));
        reply->set_debug_message(kResult.message);

        if (kResult.IsOk()) {
            core::Logger::InfoFmt(
                "rpc=LockAuthority corr={} agent={} drone={} client={} priority={} ttl_ms={} "
                "result=granted",
                kContext.correlation_id, config_.agent_id, kContext.drone_id, kContext.client_id,
                static_cast<int>(kContext.priority), req->ttl_ms());
        } else {
            core::Logger::WarnFmt(
                "rpc=LockAuthority corr={} agent={} drone={} client={} priority={} ttl_ms={} "
                "result=rejected reason={}",
                kContext.correlation_id, config_.agent_id, kContext.drone_id, kContext.client_id,
                static_cast<int>(kContext.priority), req->ttl_ms(), kResult.message);
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

        TelemetryLease telemetry_lease;
        const core::Result kAcquireResult =
            AcquireTelemetryLease(kDroneId, kEffectiveRateHz, &telemetry_lease);
        if (!kAcquireResult.IsOk()) {
            counters_.backend_failures_total.fetch_add(1, std::memory_order_relaxed);
            core::Logger::ErrorFmt(
                "rpc=StreamTelemetry corr={} agent={} drone={} peer={} rate_hz={} "
                "result=acquire_failed detail={}",
                kStreamId, config_.agent_id, kDroneId, kPeer, kEffectiveRateHz,
                kAcquireResult.message);
            return grpc::Status(grpc::StatusCode::UNAVAILABLE, kAcquireResult.message);
        }

        counters_.current_telemetry_streams.fetch_add(1, std::memory_order_relaxed);
        counters_.total_telemetry_subscriptions.fetch_add(1, std::memory_order_relaxed);
        core::Logger::InfoFmt(
            "rpc=StreamTelemetry corr={} agent={} drone={} peer={} rate_hz={} result=connected",
            kStreamId, config_.agent_id, kDroneId, kPeer, kEffectiveRateHz);

        std::uint64_t last_sequence = 0;

        while (!ctx->IsCancelled()) {
            std::optional<core::TelemetryFrame> maybe_frame;

            {
                std::lock_guard<std::mutex> lock(telemetry_lease.state->data_mutex);
                if (telemetry_lease.state->last_frame.has_value() &&
                    telemetry_lease.state->sequence != last_sequence) {
                    maybe_frame = telemetry_lease.state->last_frame;
                    last_sequence = telemetry_lease.state->sequence;
                }
            }

            if (maybe_frame.has_value()) {
                const auto& src = *maybe_frame;

                swarmkit::v1::TelemetryFrame out;
                out.set_drone_id(src.drone_id);
                out.set_unix_time_ms(src.unix_time_ms);
                out.set_lat_deg(src.lat_deg);
                out.set_lon_deg(src.lon_deg);
                out.set_rel_alt_m(src.rel_alt_m);
                out.set_battery_percent(src.battery_percent);
                out.set_mode(src.mode);
                out.set_stream_id(kStreamId);

                if (!writer->Write(out)) {
                    break;
                }
                counters_.telemetry_frames_sent_total.fetch_add(1, std::memory_order_relaxed);
            }

            std::this_thread::sleep_for(kSleepPeriod);
        }

        ReleaseTelemetryLease(telemetry_lease);
        counters_.current_telemetry_streams.fetch_sub(1, std::memory_order_relaxed);
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
        if (drone_id.empty() || client_id.empty() || !IsSupportedPriority(kPriority)) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "invalid watch authority request");
        }

        counters_.watch_requests_total.fetch_add(1, std::memory_order_relaxed);
        counters_.current_authority_watchers.fetch_add(1, std::memory_order_relaxed);
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
        counters_.current_authority_watchers.fetch_sub(1, std::memory_order_relaxed);

        core::Logger::InfoFmt(
            "rpc=WatchAuthority corr={} agent={} drone={} client={} result=disconnected", kStreamId,
            config_.agent_id, drone_id, client_id);
        return grpc::Status::OK;
    }

   private:
    struct RuntimeCounters {
        std::atomic<std::uint64_t> ping_requests_total{0};
        std::atomic<std::uint64_t> health_requests_total{0};
        std::atomic<std::uint64_t> runtime_stats_requests_total{0};
        std::atomic<std::uint64_t> command_requests_total{0};
        std::atomic<std::uint64_t> command_rejected_total{0};
        std::atomic<std::uint64_t> command_failed_total{0};
        std::atomic<std::uint64_t> lock_requests_total{0};
        std::atomic<std::uint64_t> watch_requests_total{0};
        std::atomic<std::uint64_t> current_authority_watchers{0};
        std::atomic<std::uint64_t> total_telemetry_subscriptions{0};
        std::atomic<std::uint64_t> current_telemetry_streams{0};
        std::atomic<std::uint64_t> telemetry_frames_sent_total{0};
        std::atomic<std::uint64_t> backend_failures_total{0};
    };

    struct TelemetryState {
        std::mutex control_mutex;
        std::mutex data_mutex;
        std::optional<core::TelemetryFrame> last_frame;
        std::uint64_t sequence{0};
        std::unordered_map<std::uint64_t, int> subscriber_rates_hz;
        bool backend_running{false};
        int backend_rate_hz{0};
    };

    struct TelemetryLease {
        std::shared_ptr<TelemetryState> state;
        std::string drone_id;
        std::uint64_t subscriber_id{0};
    };

    [[nodiscard]] std::shared_ptr<TelemetryState> GetOrCreateTelemetryState(
        const std::string& drone_id) {
        std::lock_guard<std::mutex> lock(telemetry_states_mutex_);
        auto& state = telemetry_states_[drone_id];
        if (!state) {
            state = std::make_shared<TelemetryState>();
        }
        return state;
    }

    static void PublishTelemetryFrame(const std::shared_ptr<TelemetryState>& state,
                                      const core::TelemetryFrame& frame) {
        std::lock_guard<std::mutex> lock(state->data_mutex);
        state->last_frame = frame;
        ++state->sequence;
    }

    [[nodiscard]] core::Result AcquireTelemetryLease(const std::string& drone_id,
                                                     int requested_rate_hz,
                                                     TelemetryLease* telemetry_lease) {
        if (telemetry_lease == nullptr) {
            return core::Result::Failed("telemetry lease output is null");
        }

        const int kNormalizedRateHz = NormalizeTelemetryRate(requested_rate_hz);
        const int kFloorRateHz = std::max(1, config_.min_telemetry_rate_hz);
        const int kDefaultRateHz = std::max(kFloorRateHz, config_.default_telemetry_rate_hz);
        auto state = GetOrCreateTelemetryState(drone_id);
        const std::uint64_t kSubscriberId =
            next_telemetry_subscriber_id_.fetch_add(1, std::memory_order_relaxed);

        std::lock_guard<std::mutex> lock(state->control_mutex);
        state->subscriber_rates_hz[kSubscriberId] =
            std::max(kFloorRateHz, requested_rate_hz <= 0 ? kDefaultRateHz : kNormalizedRateHz);

        int desired_backend_rate_hz = state->subscriber_rates_hz[kSubscriberId];
        for (const auto& [entry_id, rate_hz] : state->subscriber_rates_hz) {
            static_cast<void>(entry_id);
            desired_backend_rate_hz = std::max(desired_backend_rate_hz, rate_hz);
        }

        if (!state->backend_running) {
            const core::Result kStartResult = backend_->StartTelemetry(
                drone_id, desired_backend_rate_hz, [state](const core::TelemetryFrame& frame) {
                    PublishTelemetryFrame(state, frame);
                });
            if (!kStartResult.IsOk()) {
                counters_.backend_failures_total.fetch_add(1, std::memory_order_relaxed);
                state->subscriber_rates_hz.erase(kSubscriberId);
                return core::Result::Failed("backend StartTelemetry failed: " +
                                            kStartResult.message);
            }

            state->backend_running = true;
            state->backend_rate_hz = desired_backend_rate_hz;
        } else if (desired_backend_rate_hz > state->backend_rate_hz) {
            const int kPreviousBackendRateHz = state->backend_rate_hz;

            const core::Result kStopResult = backend_->StopTelemetry(drone_id);
            if (!kStopResult.IsOk()) {
                counters_.backend_failures_total.fetch_add(1, std::memory_order_relaxed);
                state->subscriber_rates_hz.erase(kSubscriberId);
                return core::Result::Failed("backend StopTelemetry failed during reconfigure: " +
                                            kStopResult.message);
            }

            const core::Result kStartResult = backend_->StartTelemetry(
                drone_id, desired_backend_rate_hz, [state](const core::TelemetryFrame& frame) {
                    PublishTelemetryFrame(state, frame);
                });
            if (!kStartResult.IsOk()) {
                counters_.backend_failures_total.fetch_add(1, std::memory_order_relaxed);
                core::Logger::ErrorFmt(
                    "AgentServiceImpl: failed to raise telemetry rate for "
                    "drone '{}' from {}Hz to {}Hz: {}",
                    drone_id, kPreviousBackendRateHz, desired_backend_rate_hz,
                    kStartResult.message);

                const core::Result kRestoreResult = backend_->StartTelemetry(
                    drone_id, kPreviousBackendRateHz, [state](const core::TelemetryFrame& frame) {
                        PublishTelemetryFrame(state, frame);
                    });
                if (kRestoreResult.IsOk()) {
                    state->backend_running = true;
                    state->backend_rate_hz = kPreviousBackendRateHz;
                } else {
                    state->backend_running = false;
                    state->backend_rate_hz = 0;
                }

                state->subscriber_rates_hz.erase(kSubscriberId);
                return core::Result::Failed("backend telemetry reconfigure failed: " +
                                            kStartResult.message);
            }

            state->backend_running = true;
            state->backend_rate_hz = desired_backend_rate_hz;
        }

        telemetry_lease->state = std::move(state);
        telemetry_lease->drone_id = drone_id;
        telemetry_lease->subscriber_id = kSubscriberId;
        return core::Result::Success();
    }

    void ReleaseTelemetryLease(const TelemetryLease& telemetry_lease) {
        if (!telemetry_lease.state) {
            return;
        }

        bool should_erase_state = false;

        {
            std::lock_guard<std::mutex> lock(telemetry_lease.state->control_mutex);
            telemetry_lease.state->subscriber_rates_hz.erase(telemetry_lease.subscriber_id);

            if (telemetry_lease.state->subscriber_rates_hz.empty()) {
                if (telemetry_lease.state->backend_running) {
                    const core::Result kStopResult =
                        backend_->StopTelemetry(telemetry_lease.drone_id);
                    if (!kStopResult.IsOk()) {
                        counters_.backend_failures_total.fetch_add(1, std::memory_order_relaxed);
                        core::Logger::WarnFmt("AgentServiceImpl: StopTelemetry('{}') failed: {}",
                                              telemetry_lease.drone_id, kStopResult.message);
                    }
                    telemetry_lease.state->backend_running = false;
                    telemetry_lease.state->backend_rate_hz = 0;
                }

                should_erase_state = true;
            }
        }

        if (should_erase_state) {
            {
                std::lock_guard<std::mutex> data_lock(telemetry_lease.state->data_mutex);
                telemetry_lease.state->last_frame.reset();
                telemetry_lease.state->sequence = 0;
            }

            std::lock_guard<std::mutex> map_lock(telemetry_states_mutex_);
            auto iter = telemetry_states_.find(telemetry_lease.drone_id);
            if (iter != telemetry_states_.end() && iter->second == telemetry_lease.state) {
                telemetry_states_.erase(iter);
            }
        }
    }

    AgentConfig config_;
    DroneBackendPtr backend_;
    CommandArbiter arbiter_;
    std::atomic<bool> ready_{true};
    RuntimeCounters counters_{};
    std::mutex telemetry_states_mutex_;
    std::unordered_map<std::string, std::shared_ptr<TelemetryState>> telemetry_states_;
    std::atomic<std::uint64_t> next_telemetry_subscriber_id_{1};
};

/// @}

}  // namespace

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
    const auto ApplyIntEnv = [&](std::string_view suffix, int* out) {
        const auto kValue = GetEnvValue(std::string(prefix) + std::string(suffix));
        if (!kValue.has_value()) {
            return;
        }
        const auto kParsed = ParseIntValue(*kValue, suffix);
        if (kParsed.has_value()) {
            *out = *kParsed;
        }
    };

    const auto kAgentId = GetEnvValue(std::string(prefix) + std::string(kAgentEnvId));
    if (kAgentId.has_value()) {
        agent_id = *kAgentId;
    }

    const auto kBindAddr = GetEnvValue(std::string(prefix) + std::string(kAgentEnvBindAddr));
    if (kBindAddr.has_value()) {
        bind_addr = *kBindAddr;
    }

    ApplyIntEnv(kAgentEnvDefaultAuthorityTtlMs, &default_authority_ttl_ms);
    ApplyIntEnv(kAgentEnvDefaultTelemetryRateHz, &default_telemetry_rate_hz);
    ApplyIntEnv(kAgentEnvMinTelemetryRateHz, &min_telemetry_rate_hz);
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

    if (const auto kValue =
            core::yaml::ReadOptionalScalar<int>(root, "default_authority_ttl_ms");
        !kValue.has_value()) {
        return std::unexpected(kValue.error());
    } else if (kValue->has_value()) {
        config.default_authority_ttl_ms = **kValue;
    }

    if (const auto kValue =
            core::yaml::ReadOptionalScalar<int>(root, "default_telemetry_rate_hz");
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
