#include "swarmkit/agent/agent_server.h"

#include <grpcpp/grpcpp.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "swarmkit/core/crc32.h"
#include "swarmkit/core/drone_backend.h"
#include "swarmkit/core/logger.h"
#include "swarmkit/core/version.h"
#include "swarmkit/v1/swarmkit.grpc.pb.h"
#include "swarmkit/v1/swarmkit.pb.h"

namespace swarmkit::agent {
namespace {

namespace filesystem = std::filesystem;

constexpr int kDefaultTelemetryRateHz = 5;
constexpr int kMinimumTelemetryRateHz = 1;

std::int64_t NowUnixMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

swarmkit::v1::CommandReply::Status ToProtoStatus(swarmkit::core::StatusCode code) {
    using Status = swarmkit::v1::CommandReply::Status;

    switch (code) {
        case swarmkit::core::StatusCode::kOk:
            return Status::CommandReply_Status_OK;
        case swarmkit::core::StatusCode::kRejected:
            return Status::CommandReply_Status_REJECTED;
        case swarmkit::core::StatusCode::kFailed:
            return Status::CommandReply_Status_FAILED;
    }

    return Status::CommandReply_Status_STATUS_UNSPECIFIED;
}

swarmkit::core::CommandContext ToCoreContext(const swarmkit::v1::CommandContext& context) {
    swarmkit::core::CommandContext converted;

    converted.drone_id = context.drone_id();
    converted.priority = context.priority();
    converted.correlation_id = context.correlation_id();

    if (context.deadline_unix_ms() > 0) {
        converted.deadline = std::chrono::system_clock::time_point{
            std::chrono::milliseconds{context.deadline_unix_ms()}};
    }

    return converted;
}

swarmkit::core::Result ConvertProtoCommand(const swarmkit::v1::Command& command,
                                           swarmkit::core::Command* out_command) {
    if (out_command == nullptr) {
        return swarmkit::core::Result::Failed("null out_command");
    }

    switch (command.kind_case()) {
        case swarmkit::v1::Command::kArm:
            *out_command = swarmkit::core::CmdArm{};
            return swarmkit::core::Result::Ok();

        case swarmkit::v1::Command::kDisarm:
            *out_command = swarmkit::core::CmdDisarm{};
            return swarmkit::core::Result::Ok();

        case swarmkit::v1::Command::kLand:
            *out_command = swarmkit::core::CmdLand{};
            return swarmkit::core::Result::Ok();

        case swarmkit::v1::Command::kTakeoff: {
            swarmkit::core::CmdTakeoff takeoff;
            takeoff.alt_m = command.takeoff().alt_m();
            *out_command = std::move(takeoff);
            return swarmkit::core::Result::Ok();
        }

        case swarmkit::v1::Command::kSetRole: {
            swarmkit::core::CmdSetRole set_role;
            set_role.role = command.set_role().role();
            *out_command = std::move(set_role);
            return swarmkit::core::Result::Ok();
        }

        case swarmkit::v1::Command::kSendData: {
            swarmkit::core::CmdSendData send_data;
            send_data.ref.transfer_id = command.send_data().ref().transfer_id();
            send_data.ref.size_bytes = command.send_data().ref().size_bytes();
            send_data.ref.mime_type = command.send_data().ref().mime_type();
            send_data.dst_agent_id = command.send_data().dst_agent_id();
            *out_command = std::move(send_data);
            return swarmkit::core::Result::Ok();
        }

        default:
            return swarmkit::core::Result::Rejected("unknown command kind");
    }
}

class AgentServiceImpl final : public swarmkit::v1::AgentService::Service {
   public:
    AgentServiceImpl(AgentConfig config, std::unique_ptr<swarmkit::core::IDroneBackend> backend)
        : config_(std::move(config)), backend_(std::move(backend)) {
        filesystem::create_directories(config_.inbox_dir);

        backend_->SetTelemetryCallback([this](const swarmkit::core::TelemetryFrame& frame) {
            std::lock_guard<std::mutex> lock(telemetry_mutex_);
            last_telemetry_ = frame;
            telemetry_sequence_.fetch_add(1, std::memory_order_relaxed);
        });

        (void)backend_->StartTelemetry("default", kDefaultTelemetryRateHz);
    }

    grpc::Status Ping(grpc::ServerContext* server_context, const swarmkit::v1::PingRequest* request,
                      swarmkit::v1::PingReply* response) override {
        (void)server_context;
        (void)request;

        response->set_agent_id(config_.agent_id);
        response->set_version(swarmkit::core::kSwarmkitVersionString);
        response->set_unix_time_ms(NowUnixMs());
        return grpc::Status::OK;
    }

    grpc::Status SendCommand(grpc::ServerContext* server_context,
                             const swarmkit::v1::CommandRequest* request,
                             swarmkit::v1::CommandReply* response) override {
        (void)server_context;

        if (request == nullptr || response == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null request/response");
        }

        if (!request->has_cmd()) {
            response->set_status(swarmkit::v1::CommandReply::FAILED);
            response->set_message("missing cmd");
            return grpc::Status::OK;
        }

        swarmkit::core::CommandContext command_context;
        if (request->has_ctx()) {
            command_context = ToCoreContext(request->ctx());
        }

        swarmkit::core::Command core_command;
        const swarmkit::core::Result convert_result =
            ConvertProtoCommand(request->cmd(), &core_command);
        if (!convert_result.ok()) {
            response->set_status(ToProtoStatus(convert_result.code));
            response->set_message(convert_result.message);
            return grpc::Status::OK;
        }

        const swarmkit::core::Result execution_result =
            backend_->Execute(command_context, core_command);
        response->set_status(ToProtoStatus(execution_result.code));
        response->set_message(execution_result.message);

        return grpc::Status::OK;
    }

    grpc::Status StreamTelemetry(
        grpc::ServerContext* server_context, const swarmkit::v1::TelemetryRequest* request,
        grpc::ServerWriter<swarmkit::v1::TelemetryFrame>* writer) override {
        if (server_context == nullptr || writer == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/writer");
        }

        const int requested_rate_hz = (request != nullptr) ? request->rate_hz() : 0;
        const int effective_rate_hz =
            (requested_rate_hz > 0) ? requested_rate_hz : kDefaultTelemetryRateHz;
        const int clamped_rate_hz = std::max(kMinimumTelemetryRateHz, effective_rate_hz);

        const auto sleep_duration = std::chrono::milliseconds(1000 / clamped_rate_hz);

        std::uint64_t last_seen_sequence = 0;

        while (!server_context->IsCancelled()) {
            std::optional<swarmkit::core::TelemetryFrame> maybe_frame;
            std::uint64_t current_sequence = 0;

            {
                std::lock_guard<std::mutex> lock(telemetry_mutex_);
                current_sequence = telemetry_sequence_.load(std::memory_order_relaxed);
                if (last_telemetry_.has_value() && current_sequence != last_seen_sequence) {
                    maybe_frame = last_telemetry_;
                    last_seen_sequence = current_sequence;
                }
            }

            if (maybe_frame.has_value()) {
                const swarmkit::core::TelemetryFrame& frame = maybe_frame.value();

                swarmkit::v1::TelemetryFrame out_frame;
                out_frame.set_drone_id(frame.drone_id);
                out_frame.set_unix_time_ms(frame.unix_time_ms);
                out_frame.set_lat_deg(frame.lat_deg);
                out_frame.set_lon_deg(frame.lon_deg);
                out_frame.set_rel_alt_m(frame.rel_alt_m);
                out_frame.set_battery_percent(frame.battery_percent);
                out_frame.set_mode(frame.mode);

                if (!writer->Write(out_frame)) {
                    break;
                }
            }

            std::this_thread::sleep_for(sleep_duration);
        }

        return grpc::Status::OK;
    }

    grpc::Status Transfer(
        grpc::ServerContext* server_context,
        grpc::ServerReaderWriter<swarmkit::v1::DataAck, swarmkit::v1::DataChunk>* stream) override {
        if (server_context == nullptr || stream == nullptr) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "null context/stream");
        }

        swarmkit::v1::DataChunk chunk;
        std::string transfer_id;
        filesystem::path output_path;
        std::fstream output_file;
        std::uint64_t expected_offset = 0;

        auto SendAck = [&](swarmkit::v1::DataAck::Status status, std::uint64_t next_offset,
                           std::string message) {
            swarmkit::v1::DataAck ack;
            ack.set_transfer_id(transfer_id);
            ack.set_next_offset(next_offset);
            ack.set_status(status);
            ack.set_message(std::move(message));
            (void)stream->Write(ack);
        };

        while (!server_context->IsCancelled() && stream->Read(&chunk)) {
            if (transfer_id.empty()) {
                transfer_id = chunk.transfer_id();
                if (transfer_id.empty()) {
                    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing transfer_id");
                }

                output_path = filesystem::path(config_.inbox_dir) / (transfer_id + ".bin");
                filesystem::create_directories(output_path.parent_path());

                output_file.open(output_path, std::ios::in | std::ios::out | std::ios::binary);
                if (!output_file.is_open()) {
                    std::ofstream create_file(output_path, std::ios::binary);
                    create_file.close();
                    output_file.open(output_path, std::ios::in | std::ios::out | std::ios::binary);
                }
                if (!output_file.is_open()) {
                    return grpc::Status(grpc::StatusCode::INTERNAL, "failed to open output file");
                }

                output_file.seekg(0, std::ios::end);
                expected_offset = static_cast<std::uint64_t>(output_file.tellg());

                SendAck(swarmkit::v1::DataAck::OK, expected_offset,
                        "ready, resume_offset=" + std::to_string(expected_offset));
            }

            if (chunk.offset() != expected_offset) {
                SendAck(swarmkit::v1::DataAck::BAD_OFFSET, expected_offset,
                        "bad offset, expected=" + std::to_string(expected_offset) +
                            ", got=" + std::to_string(chunk.offset()));
                continue;
            }

            const std::string& payload = chunk.data();
            const std::uint32_t computed_crc = swarmkit::core::Crc32Bytes(
                payload.data(), static_cast<std::size_t>(payload.size()));

            if (computed_crc != chunk.crc32()) {
                SendAck(swarmkit::v1::DataAck::BAD_CHECKSUM, expected_offset,
                        "bad checksum at offset=" + std::to_string(expected_offset));
                continue;
            }

            output_file.seekp(static_cast<std::streamoff>(expected_offset), std::ios::beg);
            output_file.write(payload.data(), static_cast<std::streamsize>(payload.size()));
            if (!output_file.good()) {
                SendAck(swarmkit::v1::DataAck::FAILED, expected_offset, "write failed");
                continue;
            }

            expected_offset += static_cast<std::uint64_t>(payload.size());
            SendAck(swarmkit::v1::DataAck::OK, expected_offset, "ok");

            if (chunk.eof()) {
                swarmkit::core::Logger::InfoFmt("Transfer complete: id={} bytes={} path={}",
                                                transfer_id, expected_offset, output_path.string());
                break;
            }
        }

        return grpc::Status::OK;
    }

   private:
    AgentConfig config_;
    std::unique_ptr<swarmkit::core::IDroneBackend> backend_;

    std::mutex telemetry_mutex_;
    std::optional<swarmkit::core::TelemetryFrame> last_telemetry_;
    std::atomic<std::uint64_t> telemetry_sequence_{0};
};

}  // namespace

int RunAgentServer(const AgentConfig& config,
                   std::unique_ptr<swarmkit::core::IDroneBackend> backend) {
    if (!backend) {
        swarmkit::core::Logger::Error("RunAgentServer: backend is null");
        return 1;
    }

    grpc::ServerBuilder builder;
    builder.AddListeningPort(config.bind_addr, grpc::InsecureServerCredentials());

    AgentServiceImpl service(config, std::move(backend));
    builder.RegisterService(&service);

    std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
    if (!server) {
        swarmkit::core::Logger::ErrorFmt("Failed to start AgentService on {}", config.bind_addr);
        return 1;
    }

    swarmkit::core::Logger::InfoFmt("AgentService started: agent_id={} bind={}", config.agent_id,
                                    config.bind_addr);

    server->Wait();
    return 0;
}

}  // namespace swarmkit::agent