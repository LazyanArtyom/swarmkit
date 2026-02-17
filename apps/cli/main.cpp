#include <grpcpp/grpcpp.h>

#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <random>
#include <string>

#include "swarmkit.grpc.pb.h"
#include "swarmkit.pb.h"
#include "swarmkit/core/crc32.h"
#include "swarmkit/core/logger.h"

namespace {
constexpr const char* kDefaultAgentAddr = "127.0.0.1:50061";
constexpr std::int64_t kDefaultDeadlineMs = 30'000;
constexpr std::size_t kDefaultChunkSizeBytes = 65536;

std::string GetArgValue(int argc, char** argv, const std::string& key, const std::string& def) {
    for (int arg_index = 1; arg_index + 1 < argc; ++arg_index) {
        if (std::string(argv[arg_index]) == key) {
            return std::string(argv[arg_index + 1]);
        }
    }
    return def;
}

bool HasFlag(int argc, char** argv, const std::string& flag) {
    for (int arg_index = 1; arg_index < argc; ++arg_index) {
        if (std::string(argv[arg_index]) == flag) {
            return true;
        }
    }
    return false;
}

bool StartsWithPrefix(const std::string& value, const std::string& prefix) {
    return value.size() >= prefix.size() && value.compare(0, prefix.size(), prefix) == 0;
}

std::string RandomHexString(std::size_t length) {
    static std::mt19937_64 generator{std::random_device{}()};
    static constexpr char kHex[] = "0123456789abcdef";

    std::string output;
    output.reserve(length);

    for (std::size_t index = 0; index < length; ++index) {
        const std::size_t digit = static_cast<std::size_t>(generator() % 16ULL);
        output.push_back(kHex[digit]);
    }
    return output;
}

std::int64_t NowUnixMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

void PrintUsage() {
    std::cout << "swarmkit_cli --addr HOST:PORT <command> [args]\n\n"
              << "Commands:\n"
              << "  ping\n"
              << "  arm --drone ID\n"
              << "  disarm --drone ID\n"
              << "  takeoff --drone ID --alt 10\n"
              << "  land --drone ID\n"
              << "  tele --drone ID [--rate 5]\n"
              << "  send-file --file PATH [--transfer-id ID] [--chunk 65536]\n";
}

swarmkit::v1::CommandRequest MakeCommandRequest(const std::string& drone_id,
                                                const swarmkit::v1::Command& command) {
    swarmkit::v1::CommandRequest request;
    request.mutable_ctx()->set_drone_id(drone_id);
    request.mutable_ctx()->set_priority(0);
    request.mutable_ctx()->set_deadline_unix_ms(NowUnixMs() + kDefaultDeadlineMs);
    request.mutable_ctx()->set_correlation_id(RandomHexString(12));
    *request.mutable_cmd() = command;
    return request;
}

}  // namespace

int main(int argc, char** argv) {
    swarmkit::core::LoggerConfig logger_config;
    logger_config.sink_type = swarmkit::core::LogSinkType::kStdout;
    logger_config.level = swarmkit::core::LogLevel::kInfo;
    swarmkit::core::Logger::Init(logger_config);

    const std::string agent_addr = GetArgValue(argc, argv, "--addr", kDefaultAgentAddr);

    if (argc < 2 || HasFlag(argc, argv, "--help")) {
        PrintUsage();
        return 0;
    }

    int verb_index = 1;
    for (; verb_index < argc; ++verb_index) {
        const std::string arg_value = std::string(argv[verb_index]);
        if (arg_value == "--addr") {
            ++verb_index;
            continue;
        }
        if (!StartsWithPrefix(arg_value, "--")) {
            break;
        }
    }

    if (verb_index >= argc) {
        PrintUsage();
        return 1;
    }

    const std::string verb = std::string(argv[verb_index]);

    auto channel = grpc::CreateChannel(agent_addr, grpc::InsecureChannelCredentials());
    auto stub = swarmkit::v1::AgentService::NewStub(channel);

    if (verb == "ping") {
        grpc::ClientContext context;
        swarmkit::v1::PingRequest request;
        request.set_agent_id("cli");

        swarmkit::v1::PingReply reply;
        const grpc::Status status = stub->Ping(&context, request, &reply);
        if (!status.ok()) {
            swarmkit::core::Logger::ErrorFmt("Ping failed: {}", status.error_message());
            return 1;
        }

        swarmkit::core::Logger::InfoFmt("agent_id={} version={} time_ms={}", reply.agent_id(),
                                        reply.version(), reply.unix_time_ms());
        return 0;
    }

    if (verb == "arm" || verb == "disarm" || verb == "land" || verb == "takeoff") {
        const std::string drone_id = GetArgValue(argc, argv, "--drone", "default");

        swarmkit::v1::Command command;
        if (verb == "arm") {
            command.mutable_arm();
        } else if (verb == "disarm") {
            command.mutable_disarm();
        } else if (verb == "land") {
            command.mutable_land();
        } else {
            const double altitude_meters = std::stod(GetArgValue(argc, argv, "--alt", "10"));
            command.mutable_takeoff()->set_alt_m(altitude_meters);
        }

        const swarmkit::v1::CommandRequest request = MakeCommandRequest(drone_id, command);

        grpc::ClientContext context;
        swarmkit::v1::CommandReply reply;
        const grpc::Status status = stub->SendCommand(&context, request, &reply);
        if (!status.ok()) {
            swarmkit::core::Logger::ErrorFmt("SendCommand failed: {}", status.error_message());
            return 1;
        }

        swarmkit::core::Logger::InfoFmt("status={} message={}",
                                        swarmkit::v1::CommandReply_Status_Name(reply.status()),
                                        reply.message());
        return 0;
    }

    if (verb == "tele") {
        const std::string drone_id = GetArgValue(argc, argv, "--drone", "default");
        const int rate_hz = std::stoi(GetArgValue(argc, argv, "--rate", "5"));

        grpc::ClientContext context;
        swarmkit::v1::TelemetryRequest request;
        request.set_drone_id(drone_id);
        request.set_rate_hz(rate_hz);

        auto reader = stub->StreamTelemetry(&context, request);

        swarmkit::v1::TelemetryFrame frame;
        while (reader->Read(&frame)) {
            swarmkit::core::Logger::InfoFmt(
                "tele: t={} drone={} lat={} lon={} alt={} batt={} mode={}", frame.unix_time_ms(),
                frame.drone_id(), frame.lat_deg(), frame.lon_deg(), frame.rel_alt_m(),
                frame.battery_percent(), frame.mode());
        }

        const grpc::Status finish_status = reader->Finish();
        if (!finish_status.ok()) {
            swarmkit::core::Logger::Warn(std::string("telemetry stream ended: ") +
                                         finish_status.error_message());
        }
        return 0;
    }

    if (verb == "send-file") {
        const std::string file_path = GetArgValue(argc, argv, "--file", "");
        if (file_path.empty()) {
            swarmkit::core::Logger::Error("send-file requires --file PATH");
            return 1;
        }

        const std::string transfer_id =
            GetArgValue(argc, argv, "--transfer-id", RandomHexString(24));
        const std::size_t chunk_size = static_cast<std::size_t>(std::stoull(
            GetArgValue(argc, argv, "--chunk", std::to_string(kDefaultChunkSizeBytes))));

        std::ifstream input_file(file_path, std::ios::binary);
        if (!input_file.is_open()) {
            swarmkit::core::Logger::ErrorFmt("failed to open file: {}", file_path);
            return 1;
        }

        grpc::ClientContext context;
        auto stream = stub->Transfer(&context);

        swarmkit::v1::DataChunk hello;
        hello.set_transfer_id(transfer_id);
        hello.set_src_agent_id("cli");
        hello.set_dst_agent_id("agent");
        hello.set_offset(0);
        hello.set_data("");
        hello.set_crc32(swarmkit::core::Crc32Bytes("", 0));
        hello.set_eof(false);
        hello.set_filename(file_path);

        if (!stream->Write(hello)) {
            swarmkit::core::Logger::Error("transfer: failed to write hello chunk");
            return 1;
        }

        swarmkit::v1::DataAck ack;
        if (!stream->Read(&ack)) {
            swarmkit::core::Logger::Error("transfer: no ack from server");
            return 1;
        }

        std::uint64_t offset = ack.next_offset();
        swarmkit::core::Logger::InfoFmt(
            "transfer_id={} resume_offset={} status={} msg={}", transfer_id, offset,
            swarmkit::v1::DataAck_Status_Name(ack.status()), ack.message());

        input_file.seekg(static_cast<std::streamoff>(offset), std::ios::beg);

        std::string buffer(chunk_size, '\0');
        while (input_file.good()) {
            input_file.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
            const std::size_t bytes_read = static_cast<std::size_t>(input_file.gcount());
            if (bytes_read == 0) {
                break;
            }

            swarmkit::v1::DataChunk chunk;
            chunk.set_transfer_id(transfer_id);
            chunk.set_src_agent_id("cli");
            chunk.set_dst_agent_id("agent");
            chunk.set_offset(offset);
            chunk.set_data(buffer.data(), bytes_read);
            chunk.set_crc32(swarmkit::core::Crc32Bytes(buffer.data(), bytes_read));
            chunk.set_eof(false);
            chunk.set_filename(file_path);

            if (!stream->Write(chunk)) {
                swarmkit::core::Logger::Error("transfer: write failed");
                break;
            }

            if (!stream->Read(&ack)) {
                swarmkit::core::Logger::Error("transfer: ack read failed");
                break;
            }

            if (ack.status() != swarmkit::v1::DataAck::OK) {
                swarmkit::core::Logger::Warn("transfer: server requested resume, status=" +
                                             swarmkit::v1::DataAck_Status_Name(ack.status()) +
                                             " msg=" + ack.message());

                offset = ack.next_offset();
                input_file.clear();
                input_file.seekg(static_cast<std::streamoff>(offset), std::ios::beg);
                continue;
            }

            offset = ack.next_offset();
        }

        swarmkit::v1::DataChunk end_chunk;
        end_chunk.set_transfer_id(transfer_id);
        end_chunk.set_src_agent_id("cli");
        end_chunk.set_dst_agent_id("agent");
        end_chunk.set_offset(offset);
        end_chunk.set_data("");
        end_chunk.set_crc32(swarmkit::core::Crc32Bytes("", 0));
        end_chunk.set_eof(true);
        end_chunk.set_filename(file_path);

        (void)stream->Write(end_chunk);
        (void)stream->Read(&ack);

        stream->WritesDone();
        const grpc::Status finish_status = stream->Finish();
        if (!finish_status.ok()) {
            swarmkit::core::Logger::ErrorFmt("transfer finished with error: {}",
                                             finish_status.error_message());
            return 1;
        }

        swarmkit::core::Logger::InfoFmt("transfer done: id={} bytes_sent={}", transfer_id, offset);
        return 0;
    }

    PrintUsage();
    return 1;
}