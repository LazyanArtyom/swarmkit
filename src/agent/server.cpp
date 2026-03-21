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
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include "crc32.h"
#include "swarmkit/agent/arbiter.h"
#include "swarmkit/core/logger.h"
#include "swarmkit/core/version.h"
#include "swarmkit/v1/swarmkit.grpc.pb.h"
#include "swarmkit/v1/swarmkit.pb.h"

namespace swarmkit::agent {

using namespace swarmkit::commands;  // NOLINT(google-build-using-namespace)

namespace {
using grpc::Status;

namespace fs = std::filesystem;

constexpr int kDefaultTelemetryRateHz = 5;
constexpr int kMinTelemetryRateHz = 1;
constexpr int kMillisecondsPerSecond = 1000;

/// @brief Watcher poll interval while blocking inside WatchAuthority RPC.
constexpr auto kWatchPollInterval = std::chrono::milliseconds{100};

/// @brief How long a client keeps authority after a single command when no
/// explicit deadline is set. After this window lower-priority clients can
/// reclaim the drone on their next attempt.
constexpr auto kDefaultAuthorityTtl = std::chrono::seconds{5};

/// @name Helpers
/// @{

/// @brief Returns the current Unix time in milliseconds.
[[nodiscard]] std::int64_t NowUnixMs() {
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using std::chrono::system_clock;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

/// @brief Clamp telemetry rate to a valid positive value.
[[nodiscard]] int NormalizeTelemetryRate(int requested_rate_hz) {
    if (requested_rate_hz <= 0) {
        return kDefaultTelemetryRateHz;
    }
    return std::max(kMinTelemetryRateHz, requested_rate_hz);
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

        case swarmkit::v1::Command::kSendData:
            return std::unexpected(
                core::Result::Rejected("send_data is not a flight command; use the Transfer RPC"));

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
        : config_(std::move(config)), backend_(std::move(backend)) {
        try {
            fs::create_directories(config_.inbox_dir);
        } catch (const fs::filesystem_error& err) {
            core::Logger::WarnFmt("AgentServiceImpl: failed to create inbox_dir '{}': {}",
                                  config_.inbox_dir, err.what());
        }
    }

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

    grpc::Status Ping(grpc::ServerContext* /*ctx*/, const swarmkit::v1::PingRequest* /*req*/,
                      swarmkit::v1::PingReply* reply) override {
        reply->set_agent_id(config_.agent_id);
        reply->set_version(core::kSwarmkitVersionString);
        reply->set_unix_time_ms(NowUnixMs());
        return grpc::Status::OK;
    }

    grpc::Status SendCommand(grpc::ServerContext* ctx, const swarmkit::v1::CommandRequest* req,
                             swarmkit::v1::CommandReply* reply) override {
        if (req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request/response");
        }

        if (!req->has_cmd()) {
            reply->set_status(swarmkit::v1::CommandReply::FAILED);
            reply->set_message("missing cmd field");
            return grpc::Status::OK;
        }

        CommandEnvelope envelope;
        if (req->has_ctx()) {
            envelope.context = ToCoreContext(req->ctx());
        }

        const std::string kCmdName = ProtoCommandName(req->cmd());

        std::chrono::milliseconds ttl = kDefaultAuthorityTtl;
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
            core::Logger::WarnFmt(
                "Agent '{}' [{}]: REJECTED {} from '{}' (priority={}) - {}", config_.agent_id,
                envelope.context.drone_id, kCmdName, envelope.context.client_id,
                static_cast<int>(envelope.context.priority), kArbiterResult.message);
            reply->set_status(ToProtoStatus(kArbiterResult.code));
            reply->set_message(kArbiterResult.message);
            return grpc::Status::OK;
        }

        core::Logger::InfoFmt("Agent '{}' [{}]: executing {} from '{}' (priority={}) peer={}",
                              config_.agent_id, envelope.context.drone_id, kCmdName,
                              envelope.context.client_id,
                              static_cast<int>(envelope.context.priority), ctx->peer());

        auto convert_result = ConvertProtoCommand(req->cmd());
        if (!convert_result.has_value()) {
            const auto& error = convert_result.error();
            reply->set_status(ToProtoStatus(error.code));
            reply->set_message(error.message);
            return grpc::Status::OK;
        }
        envelope.command = std::move(convert_result.value());

        const core::Result kExecResult = backend_->Execute(envelope);
        reply->set_status(ToProtoStatus(kExecResult.code));
        reply->set_message(kExecResult.message);
        return grpc::Status::OK;
    }

    grpc::Status LockAuthority(grpc::ServerContext* /*ctx*/,
                               const swarmkit::v1::LockAuthorityRequest* req,
                               swarmkit::v1::LockAuthorityReply* reply) override {
        if (req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request/response");
        }
        if (!req->has_ctx()) {
            reply->set_ok(false);
            reply->set_message("missing ctx field");
            return grpc::Status::OK;
        }

        const CommandContext kContext = ToCoreContext(req->ctx());
        const auto kTtlDuration = std::chrono::milliseconds{req->ttl_ms()};
        const core::Result kResult = arbiter_.CheckAndGrant(kContext, kTtlDuration);

        reply->set_ok(kResult.IsOk());
        reply->set_message(kResult.message);

        if (kResult.IsOk()) {
            core::Logger::InfoFmt("Agent '{}' [{}]: LOCKED by '{}' (priority={}) ttl={}ms",
                                  config_.agent_id, kContext.drone_id, kContext.client_id,
                                  static_cast<int>(kContext.priority), req->ttl_ms());
        } else {
            core::Logger::WarnFmt("Agent '{}' [{}]: LOCK REJECTED for '{}' (priority={}) - {}",
                                  config_.agent_id, kContext.drone_id, kContext.client_id,
                                  static_cast<int>(kContext.priority), kResult.message);
        }
        return grpc::Status::OK;
    }

    grpc::Status ReleaseAuthority(grpc::ServerContext* /*ctx*/,
                                  const swarmkit::v1::ReleaseAuthorityRequest* req,
                                  swarmkit::v1::ReleaseAuthorityReply* /*reply*/) override {
        if (req == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request");
        }
        arbiter_.Release(req->drone_id(), req->client_id());
        core::Logger::InfoFmt("Agent '{}' [{}]: authority released by '{}'", config_.agent_id,
                              req->drone_id(), req->client_id());
        return grpc::Status::OK;
    }

    grpc::Status StreamTelemetry(
        grpc::ServerContext* ctx, const swarmkit::v1::TelemetryRequest* req,
        grpc::ServerWriter<swarmkit::v1::TelemetryFrame>* writer) override {
        if (ctx == nullptr || writer == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/writer");
        }

        const std::string kPeer = ctx->peer();
        const std::string kDroneId =
            (req != nullptr && !req->drone_id().empty()) ? req->drone_id() : "default";
        const int kRequestedRateHz = (req != nullptr) ? req->rate_hz() : 0;
        const int kEffectiveRateHz = NormalizeTelemetryRate(kRequestedRateHz);
        const auto kSleepPeriod =
            std::chrono::milliseconds(kMillisecondsPerSecond / kEffectiveRateHz);

        TelemetryLease telemetry_lease;
        const core::Result kAcquireResult =
            AcquireTelemetryLease(kDroneId, kEffectiveRateHz, &telemetry_lease);
        if (!kAcquireResult.IsOk()) {
            core::Logger::ErrorFmt(
                "Agent '{}': failed to start telemetry for drone '{}' "
                "peer={} rate={}Hz: {}",
                config_.agent_id, kDroneId, kPeer, kEffectiveRateHz, kAcquireResult.message);
            return grpc::Status(grpc::StatusCode::UNAVAILABLE, kAcquireResult.message);
        }

        core::Logger::InfoFmt(
            "Agent '{}': telemetry subscriber connected peer={} drone='{}' "
            "rate={}Hz",
            config_.agent_id, kPeer, kDroneId, kEffectiveRateHz);

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

                if (!writer->Write(out)) {
                    break;
                }
            }

            std::this_thread::sleep_for(kSleepPeriod);
        }

        ReleaseTelemetryLease(telemetry_lease);
        core::Logger::InfoFmt("Agent '{}': telemetry subscriber disconnected peer={} drone='{}'",
                              config_.agent_id, kPeer, kDroneId);
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

        auto queue = std::make_shared<EventQueue>();
        const WatchToken kToken = arbiter_.Watch(drone_id, client_id, kPriority, queue);

        core::Logger::InfoFmt("WatchAuthority: '{}' watching drone '{}' (priority={})", client_id,
                              drone_id, static_cast<int>(kPriority));

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

            if (!writer->Write(proto_event)) {
                break;
            }
        }

        queue->Shutdown();
        arbiter_.Unwatch(kToken);

        core::Logger::InfoFmt("WatchAuthority: '{}' disconnected from drone '{}'", client_id,
                              drone_id);
        return grpc::Status::OK;
    }

    grpc::Status Transfer(
        grpc::ServerContext* ctx,
        grpc::ServerReaderWriter<swarmkit::v1::DataAck, swarmkit::v1::DataChunk>* stream) override {
        if (ctx == nullptr || stream == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/stream");
        }

        swarmkit::v1::DataChunk chunk;
        std::string transfer_id;
        fs::path output_path;
        std::fstream output_file;
        std::uint64_t expected_offset = 0;

        auto send_ack = [&](swarmkit::v1::DataAck::Status ack_status, std::uint64_t next_offset,
                            std::string message) {
            swarmkit::v1::DataAck ack;
            ack.set_transfer_id(transfer_id);
            ack.set_next_offset(next_offset);
            ack.set_status(ack_status);
            ack.set_message(std::move(message));
            static_cast<void>(stream->Write(ack));
        };

        while (!ctx->IsCancelled() && stream->Read(&chunk)) {
            if (transfer_id.empty()) {
                transfer_id = chunk.transfer_id();
                if (transfer_id.empty()) {
                    return {grpc::StatusCode::INVALID_ARGUMENT, "missing transfer_id"};
                }

                output_path = fs::path(config_.inbox_dir) / (transfer_id + ".bin");
                fs::create_directories(output_path.parent_path());

                output_file.open(output_path, std::ios::in | std::ios::out | std::ios::binary);
                if (!output_file.is_open()) {
                    std::ofstream create(output_path, std::ios::binary);
                    create.close();
                    output_file.open(output_path, std::ios::in | std::ios::out | std::ios::binary);
                }
                if (!output_file.is_open()) {
                    return {grpc::StatusCode::INTERNAL, "failed to open output file"};
                }

                output_file.seekg(0, std::ios::end);
                expected_offset = static_cast<std::uint64_t>(output_file.tellg());

                send_ack(swarmkit::v1::DataAck::OK, expected_offset,
                         "ready, resume_offset=" + std::to_string(expected_offset));
            }

            if (chunk.offset() != expected_offset) {
                send_ack(swarmkit::v1::DataAck::BAD_OFFSET, expected_offset,
                         "bad offset, expected=" + std::to_string(expected_offset) +
                             " got=" + std::to_string(chunk.offset()));
                continue;
            }

            const std::string& payload = chunk.data();
            const std::uint32_t kComputedCrc =
                core::Crc32Bytes(payload.data(), static_cast<std::size_t>(payload.size()));

            if (kComputedCrc != chunk.crc32()) {
                send_ack(swarmkit::v1::DataAck::BAD_CHECKSUM, expected_offset,
                         "bad checksum at offset=" + std::to_string(expected_offset));
                continue;
            }

            output_file.seekp(static_cast<std::streamoff>(expected_offset), std::ios::beg);
            output_file.write(payload.data(), static_cast<std::streamsize>(payload.size()));
            if (!output_file.good()) {
                send_ack(swarmkit::v1::DataAck::FAILED, expected_offset, "write failed");
                continue;
            }

            expected_offset += static_cast<std::uint64_t>(payload.size());
            send_ack(swarmkit::v1::DataAck::OK, expected_offset, "ok");

            if (chunk.eof()) {
                core::Logger::InfoFmt("Transfer complete: id={} bytes={} path={}", transfer_id,
                                      expected_offset, output_path.string());
                break;
            }
        }

        return {grpc::StatusCode::OK, ""};
    }

   private:
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
        auto state = GetOrCreateTelemetryState(drone_id);
        const std::uint64_t kSubscriberId =
            next_telemetry_subscriber_id_.fetch_add(1, std::memory_order_relaxed);

        std::lock_guard<std::mutex> lock(state->control_mutex);
        state->subscriber_rates_hz[kSubscriberId] = kNormalizedRateHz;

        int desired_backend_rate_hz = kNormalizedRateHz;
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
                state->subscriber_rates_hz.erase(kSubscriberId);
                return core::Result::Failed("backend StopTelemetry failed during reconfigure: " +
                                            kStopResult.message);
            }

            const core::Result kStartResult = backend_->StartTelemetry(
                drone_id, desired_backend_rate_hz, [state](const core::TelemetryFrame& frame) {
                    PublishTelemetryFrame(state, frame);
                });
            if (!kStartResult.IsOk()) {
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
    std::mutex telemetry_states_mutex_;
    std::unordered_map<std::string, std::shared_ptr<TelemetryState>> telemetry_states_;
    std::atomic<std::uint64_t> next_telemetry_subscriber_id_{1};
};

/// @}

}  // namespace

int RunAgentServer(const AgentConfig& config, DroneBackendPtr backend) {
    if (!backend) {
        core::Logger::Error("RunAgentServer: backend is null");
        return 1;
    }

    grpc::ServerBuilder builder;
    builder.AddListeningPort(config.bind_addr, grpc::InsecureServerCredentials());

    AgentServiceImpl service(config, std::move(backend));
    builder.RegisterService(&service);

    std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
    if (!server) {
        core::Logger::ErrorFmt("Failed to start gRPC server on {}", config.bind_addr);
        return 1;
    }

    core::Logger::InfoFmt("Agent '{}' listening on {}", config.agent_id, config.bind_addr);
    server->Wait();
    return 0;
}

}  // namespace swarmkit::agent
