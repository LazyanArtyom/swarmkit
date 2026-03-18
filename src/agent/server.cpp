#include "swarmkit/agent/server.h"

#include <grpcpp/grpcpp.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "crc32.h"
#include "swarmkit/agent/arbiter.h"
#include "swarmkit/core/logger.h"
#include "swarmkit/core/version.h"
#include "swarmkit/v1/swarmkit.grpc.pb.h"
#include "swarmkit/v1/swarmkit.pb.h"

namespace swarmkit::agent {
namespace {

namespace fs = std::filesystem;

constexpr int kDefaultTelemetryRateHz = 5;
constexpr int kMinTelemetryRateHz     = 1;

/// @brief Watcher poll interval while blocking inside WatchAuthority RPC.
constexpr auto kWatchPollInterval = std::chrono::milliseconds{100};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

std::int64_t NowUnixMs() {
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using std::chrono::system_clock;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

swarmkit::v1::CommandReply::Status ToProtoStatus(core::StatusCode code) {
    using Status = swarmkit::v1::CommandReply::Status;
    switch (code) {
        case core::StatusCode::kOk:       return Status::CommandReply_Status_OK;
        case core::StatusCode::kRejected: return Status::CommandReply_Status_REJECTED;
        case core::StatusCode::kFailed:   return Status::CommandReply_Status_FAILED;
    }
    return Status::CommandReply_Status_STATUS_UNSPECIFIED;
}

/// @brief Convert a proto CommandContext to the C++ CommandContext struct.
CommandContext ToCoreContext(const swarmkit::v1::CommandContext& proto) {
    CommandContext ctx;
    ctx.drone_id       = proto.drone_id();
    ctx.client_id      = proto.client_id();
    ctx.priority       = static_cast<CommandPriority>(proto.priority());
    ctx.correlation_id = proto.correlation_id();

    if (proto.deadline_unix_ms() > 0) {
        ctx.deadline = std::chrono::system_clock::time_point{
            std::chrono::milliseconds{proto.deadline_unix_ms()}};
    }

    return ctx;
}

/// @brief Convert a proto AuthorityEvent::Kind to the C++ enum.
swarmkit::v1::AuthorityEvent::Kind ToProtoEventKind(AuthorityEvent::Kind kind) {
    using ProtoKind = swarmkit::v1::AuthorityEvent::Kind;
    switch (kind) {
        case AuthorityEvent::Kind::kGranted:   return ProtoKind::AuthorityEvent_Kind_GRANTED;
        case AuthorityEvent::Kind::kPreempted: return ProtoKind::AuthorityEvent_Kind_PREEMPTED;
        case AuthorityEvent::Kind::kResumed:   return ProtoKind::AuthorityEvent_Kind_RESUMED;
        case AuthorityEvent::Kind::kExpired:   return ProtoKind::AuthorityEvent_Kind_EXPIRED;
    }
    return ProtoKind::AuthorityEvent_Kind_KIND_UNSPECIFIED;
}

/**
 * @brief Convert a proto Command oneof into the nested C++ Command variant.
 *
 * Proto field numbers are grouped by category to match the C++ variant
 * structure (FlightCmd / NavCmd / SwarmCmd / PayloadCmd).
 *
 * @returns Ok on success, Rejected for unimplemented or unknown kinds.
 */
core::Result ConvertProtoCommand(const swarmkit::v1::Command& proto,
                                 Command*                     out_command) {
    if (out_command == nullptr) {
        return core::Result::Failed("null out_command");
    }

    switch (proto.kind_case()) {

        // ---- Flight --------------------------------------------------------
        case swarmkit::v1::Command::kArm:
            *out_command = FlightCmd{CmdArm{}};
            return core::Result::Ok();

        case swarmkit::v1::Command::kDisarm:
            *out_command = FlightCmd{CmdDisarm{}};
            return core::Result::Ok();

        case swarmkit::v1::Command::kLand:
            *out_command = FlightCmd{CmdLand{}};
            return core::Result::Ok();

        case swarmkit::v1::Command::kTakeoff:
            *out_command = FlightCmd{CmdTakeoff{proto.takeoff().alt_m()}};
            return core::Result::Ok();

        // ---- Navigation ----------------------------------------------------
        case swarmkit::v1::Command::kSetWaypoint: {
            CmdSetWaypoint wp;
            wp.lat_deg   = proto.set_waypoint().lat_deg();
            wp.lon_deg   = proto.set_waypoint().lon_deg();
            wp.alt_m     = proto.set_waypoint().alt_m();
            wp.speed_mps = proto.set_waypoint().speed_mps();
            *out_command = NavCmd{std::move(wp)};
            return core::Result::Ok();
        }

        case swarmkit::v1::Command::kReturnHome:
            *out_command = NavCmd{CmdReturnHome{}};
            return core::Result::Ok();

        case swarmkit::v1::Command::kHoldPosition:
            *out_command = NavCmd{CmdHoldPosition{}};
            return core::Result::Ok();

        // ---- Swarm ---------------------------------------------------------
        case swarmkit::v1::Command::kSetRole:
            *out_command = SwarmCmd{CmdSetRole{proto.set_role().role()}};
            return core::Result::Ok();

        case swarmkit::v1::Command::kSetFormation: {
            CmdSetFormation fmt;
            fmt.formation_id = proto.set_formation().formation_id();
            fmt.slot_index   = proto.set_formation().slot_index();
            *out_command     = SwarmCmd{std::move(fmt)};
            return core::Result::Ok();
        }

        case swarmkit::v1::Command::kRunSequence: {
            CmdRunSequence seq;
            seq.sequence_id  = proto.run_sequence().sequence_id();
            seq.sync_unix_ms = proto.run_sequence().sync_unix_ms();
            *out_command     = SwarmCmd{std::move(seq)};
            return core::Result::Ok();
        }

        // ---- Transfer routing (not a flight command) -----------------------
        case swarmkit::v1::Command::kSendData:
            return core::Result::Rejected(
                "send_data is not a flight command; use the Transfer RPC");

        default:
            return core::Result::Rejected("unknown command kind");
    }
}

// ---------------------------------------------------------------------------
// AgentServiceImpl — gRPC service implementation
// ---------------------------------------------------------------------------

/**
 * @brief Implements the AgentService gRPC service.
 *
 * Owns the drone backend, the telemetry buffer, and the CommandArbiter.
 * All RPC handlers are called from gRPC's thread pool; all shared state
 * is protected by the relevant mutexes.
 */
class AgentServiceImpl final : public swarmkit::v1::AgentService::Service {
   public:
    AgentServiceImpl(AgentConfig config, DroneBackendPtr backend)
        : config_(std::move(config)), backend_(std::move(backend)) {
        fs::create_directories(config_.inbox_dir);

        const core::Result kStartResult = backend_->StartTelemetry(
            "default", kDefaultTelemetryRateHz,
            [this](const core::TelemetryFrame& frame) {
                std::lock_guard<std::mutex> lock(telemetry_mutex_);
                last_telemetry_ = frame;
                telemetry_seq_.fetch_add(1, std::memory_order_relaxed);
            });

        if (!kStartResult.ok()) {
            core::Logger::WarnFmt("AgentServiceImpl: StartTelemetry failed: {}",
                                  kStartResult.message);
        }
    }

    // -----------------------------------------------------------------------
    // RPC: Ping
    // -----------------------------------------------------------------------

    grpc::Status Ping(grpc::ServerContext* /*ctx*/,
                      const swarmkit::v1::PingRequest* /*req*/,
                      swarmkit::v1::PingReply* reply) override {
        reply->set_agent_id(config_.agent_id);
        reply->set_version(core::kSwarmkitVersionString);
        reply->set_unix_time_ms(NowUnixMs());
        return grpc::Status::OK;
    }

    // -----------------------------------------------------------------------
    // RPC: SendCommand
    // -----------------------------------------------------------------------

    grpc::Status SendCommand(grpc::ServerContext* /*ctx*/,
                             const swarmkit::v1::CommandRequest* req,
                             swarmkit::v1::CommandReply*          reply) override {
        if (req == nullptr || reply == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "null request/response");
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

        // Check arbitration before touching the backend.
        const core::Result kArbResult = arbiter_.CheckAndGrant(envelope.context);
        if (!kArbResult.ok()) {
            reply->set_status(ToProtoStatus(kArbResult.code));
            reply->set_message(kArbResult.message);
            return grpc::Status::OK;
        }

        const core::Result kConvertResult =
            ConvertProtoCommand(req->cmd(), &envelope.command);
        if (!kConvertResult.ok()) {
            reply->set_status(ToProtoStatus(kConvertResult.code));
            reply->set_message(kConvertResult.message);
            return grpc::Status::OK;
        }

        const core::Result kExecResult = backend_->Execute(envelope);
        reply->set_status(ToProtoStatus(kExecResult.code));
        reply->set_message(kExecResult.message);
        return grpc::Status::OK;
    }

    // -----------------------------------------------------------------------
    // RPC: StreamTelemetry (server streaming)
    // -----------------------------------------------------------------------

    grpc::Status StreamTelemetry(
        grpc::ServerContext*                               ctx,
        const swarmkit::v1::TelemetryRequest*              req,
        grpc::ServerWriter<swarmkit::v1::TelemetryFrame>*  writer) override {
        if (ctx == nullptr || writer == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "null context/writer");
        }

        const int kRequestedRate = (req != nullptr) ? req->rate_hz() : 0;
        const int kEffectiveRate = (kRequestedRate > 0) ? kRequestedRate
                                                        : kDefaultTelemetryRateHz;
        const int kClampedRate   = std::max(kMinTelemetryRateHz, kEffectiveRate);
        const auto kSleepPeriod  = std::chrono::milliseconds(1000 / kClampedRate);

        std::uint64_t last_seq = 0;

        while (!ctx->IsCancelled()) {
            std::optional<core::TelemetryFrame> maybe_frame;
            std::uint64_t current_seq = 0;

            {
                std::lock_guard<std::mutex> lock(telemetry_mutex_);
                current_seq = telemetry_seq_.load(std::memory_order_relaxed);
                if (last_telemetry_.has_value() && current_seq != last_seq) {
                    maybe_frame = last_telemetry_;
                    last_seq    = current_seq;
                }
            }

            if (maybe_frame.has_value()) {
                const core::TelemetryFrame& src = maybe_frame.value();

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

        return grpc::Status::OK;
    }

    // -----------------------------------------------------------------------
    // RPC: WatchAuthority (server streaming)
    // -----------------------------------------------------------------------

    /**
     * @brief Streams AuthorityEvent messages to the caller.
     *
     * Registers the client with the CommandArbiter so it receives
     * notifications when its authority is preempted or restored.
     * The stream stays open until the client disconnects or the server shuts
     * down.  The handler polls the EventQueue at kWatchPollInterval so that
     * client cancellation is detected promptly.
     */
    grpc::Status WatchAuthority(
        grpc::ServerContext*                                  ctx,
        const swarmkit::v1::WatchAuthorityRequest*            req,
        grpc::ServerWriter<swarmkit::v1::AuthorityEvent>*     writer) override {
        if (ctx == nullptr || req == nullptr || writer == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "null context/request/writer");
        }

        const std::string kDroneId  = req->drone_id();
        const std::string kClientId = req->client_id();
        const auto kPriority =
            static_cast<CommandPriority>(req->priority());

        auto queue = std::make_shared<EventQueue>();
        const WatchToken kToken =
            arbiter_.Watch(kDroneId, kClientId, kPriority, queue);

        core::Logger::InfoFmt(
            "WatchAuthority: '{}' watching drone '{}' (priority={})",
            kClientId, kDroneId, static_cast<int>(kPriority));

        while (!ctx->IsCancelled()) {
            AuthorityEvent event;
            if (!queue->Pop(event, kWatchPollInterval)) {
                continue;  // Timeout — check IsCancelled() and loop.
            }

            swarmkit::v1::AuthorityEvent proto_event;
            proto_event.set_kind(ToProtoEventKind(event.kind));
            proto_event.set_drone_id(event.drone_id);
            proto_event.set_holder_client_id(event.holder_client_id);
            proto_event.set_holder_priority(
                static_cast<int>(event.holder_priority));

            if (!writer->Write(proto_event)) {
                break;
            }
        }

        queue->Shutdown();
        arbiter_.Unwatch(kToken);

        core::Logger::InfoFmt(
            "WatchAuthority: '{}' disconnected from drone '{}'",
            kClientId, kDroneId);

        return grpc::Status::OK;
    }

    // -----------------------------------------------------------------------
    // RPC: Transfer (bidirectional streaming — resumable file upload)
    // -----------------------------------------------------------------------

    grpc::Status Transfer(
        grpc::ServerContext*                                        ctx,
        grpc::ServerReaderWriter<swarmkit::v1::DataAck,
                                 swarmkit::v1::DataChunk>* stream) override {
        if (ctx == nullptr || stream == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "null context/stream");
        }

        swarmkit::v1::DataChunk chunk;
        std::string             transfer_id;
        fs::path                output_path;
        std::fstream            output_file;
        std::uint64_t           expected_offset = 0;

        auto SendAck = [&](swarmkit::v1::DataAck::Status ack_status,
                           std::uint64_t                  next_offset,
                           std::string                    message) {
            swarmkit::v1::DataAck ack;
            ack.set_transfer_id(transfer_id);
            ack.set_next_offset(next_offset);
            ack.set_status(ack_status);
            ack.set_message(std::move(message));
            (void)stream->Write(ack);
        };

        while (!ctx->IsCancelled() && stream->Read(&chunk)) {
            if (transfer_id.empty()) {
                transfer_id = chunk.transfer_id();
                if (transfer_id.empty()) {
                    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                       "missing transfer_id");
                }

                output_path = fs::path(config_.inbox_dir) / (transfer_id + ".bin");
                fs::create_directories(output_path.parent_path());

                output_file.open(output_path,
                                 std::ios::in | std::ios::out | std::ios::binary);
                if (!output_file.is_open()) {
                    std::ofstream create(output_path, std::ios::binary);
                    create.close();
                    output_file.open(output_path,
                                     std::ios::in | std::ios::out | std::ios::binary);
                }
                if (!output_file.is_open()) {
                    return grpc::Status(grpc::StatusCode::INTERNAL,
                                       "failed to open output file");
                }

                output_file.seekg(0, std::ios::end);
                expected_offset =
                    static_cast<std::uint64_t>(output_file.tellg());

                SendAck(swarmkit::v1::DataAck::OK, expected_offset,
                        "ready, resume_offset=" + std::to_string(expected_offset));
            }

            if (chunk.offset() != expected_offset) {
                SendAck(swarmkit::v1::DataAck::BAD_OFFSET, expected_offset,
                        "bad offset, expected=" + std::to_string(expected_offset) +
                        " got=" + std::to_string(chunk.offset()));
                continue;
            }

            const std::string& payload      = chunk.data();
            const std::uint32_t kComputedCrc = core::Crc32Bytes(
                payload.data(), static_cast<std::size_t>(payload.size()));

            if (kComputedCrc != chunk.crc32()) {
                SendAck(swarmkit::v1::DataAck::BAD_CHECKSUM, expected_offset,
                        "bad checksum at offset=" +
                        std::to_string(expected_offset));
                continue;
            }

            output_file.seekp(static_cast<std::streamoff>(expected_offset),
                              std::ios::beg);
            output_file.write(payload.data(),
                              static_cast<std::streamsize>(payload.size()));
            if (!output_file.good()) {
                SendAck(swarmkit::v1::DataAck::FAILED, expected_offset,
                        "write failed");
                continue;
            }

            expected_offset += static_cast<std::uint64_t>(payload.size());
            SendAck(swarmkit::v1::DataAck::OK, expected_offset, "ok");

            if (chunk.eof()) {
                core::Logger::InfoFmt(
                    "Transfer complete: id={} bytes={} path={}",
                    transfer_id, expected_offset, output_path.string());
                break;
            }
        }

        return grpc::Status::OK;
    }

   private:
    AgentConfig                         config_;
    DroneBackendPtr                     backend_;
    CommandArbiter                      arbiter_;

    std::mutex                           telemetry_mutex_;
    std::optional<core::TelemetryFrame>  last_telemetry_;
    std::atomic<std::uint64_t>           telemetry_seq_{0};
};

}  // namespace

// ---------------------------------------------------------------------------
// RunAgentServer
// ---------------------------------------------------------------------------

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

    core::Logger::InfoFmt("Agent '{}' listening on {}", config.agent_id,
                          config.bind_addr);
    server->Wait();
    return 0;
}

}  // namespace swarmkit::agent
