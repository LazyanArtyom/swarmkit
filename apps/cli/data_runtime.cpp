// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "data_runtime.h"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <expected>
#include <fstream>
#include <iostream>
#include <iterator>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

#include "common/arg_utils.h"
#include "constants.h"
#include "options.h"
#include "swarmkit/client/client.h"

namespace swarmkit::apps::cli::internal {
namespace {

using swarmkit::client::Client;

[[nodiscard]] volatile std::sig_atomic_t& StopRequestedFlag() {
    static volatile std::sig_atomic_t stop_requested = 0;
    return stop_requested;
}

void OnDataSignal(int /*sig*/) {
    StopRequestedFlag() = 1;
}

[[nodiscard]] bool IsStopRequested() {
    return StopRequestedFlag() != 0;
}

void ResetStopRequested() {
    StopRequestedFlag() = 0;
}

[[nodiscard]] std::vector<std::string> FindActionsAfterCommand(int argc, char** argv,
                                                               std::string_view command_name) {
    int command_index = -1;
    for (int index = 1; index < argc; ++index) {
        const std::string_view current_arg = argv[index];
        if (IsOptionWithValue(current_arg)) {
            ++index;
            continue;
        }
        if (current_arg == command_name) {
            command_index = index;
            break;
        }
    }
    if (command_index < 0) {
        return {};
    }
    std::vector<std::string> actions;
    for (int index = command_index + 1; index < argc; ++index) {
        const std::string_view current_arg = argv[index];
        if (IsOptionWithValue(current_arg)) {
            ++index;
            continue;
        }
        if (current_arg.starts_with("-")) {
            continue;
        }
        actions.emplace_back(current_arg);
    }
    return actions;
}

[[nodiscard]] std::vector<std::string> CollectOptionValues(int argc, char** argv,
                                                           std::string_view option_name) {
    std::vector<std::string> values;
    for (int index = 1; index + 1 < argc; ++index) {
        if (std::string_view(argv[index]) == option_name) {
            values.emplace_back(argv[++index]);
        }
    }
    return values;
}

[[nodiscard]] std::expected<std::string, std::string> ReadSmallPayload(int argc, char** argv) {
    if (const std::string payload = common::GetOptionValue(argc, argv, "--payload");
        !payload.empty()) {
        return payload;
    }
    const std::string file_path = common::GetOptionValue(argc, argv, "--file");
    if (file_path.empty()) {
        return {};
    }
    std::ifstream input(file_path, std::ios::binary);
    if (!input.is_open()) {
        return std::unexpected("failed to open payload file '" + file_path + "'");
    }
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

[[nodiscard]] std::expected<std::int64_t, std::string> ParseDurationMs(int argc, char** argv) {
    const auto duration = ParseIntArg(
        common::GetOptionValue(argc, argv, "--duration-ms", kDefaultZero), "--duration-ms");
    if (!duration.has_value()) {
        return std::unexpected(duration.error());
    }
    if (*duration < 0) {
        return std::unexpected("--duration-ms must be >= 0");
    }
    return static_cast<std::int64_t>(*duration);
}

[[nodiscard]] std::expected<std::unordered_map<std::string, std::string>, std::string>
ParseKeyValueLabels(int argc, char** argv, std::string_view option_name) {
    std::unordered_map<std::string, std::string> labels;
    for (int index = 1; index < argc; ++index) {
        if (std::string_view(argv[index]) != option_name) {
            continue;
        }
        if (index + 1 >= argc) {
            return std::unexpected(std::string(option_name) + " requires KEY=VALUE");
        }
        const std::string entry = argv[++index];
        const std::size_t separator = entry.find('=');
        if (separator == std::string::npos || separator == 0) {
            return std::unexpected(std::string(option_name) + " requires KEY=VALUE");
        }
        labels[entry.substr(0, separator)] = entry.substr(separator + 1);
    }
    return labels;
}

}  // namespace

int RunMessage(Client& client, int argc, char** argv) {
    const auto actions = FindActionsAfterCommand(argc, argv, "message");
    if (actions.empty()) {
        std::cerr << "message requires publish|send|subscribe|peers\n";
        return EXIT_FAILURE;
    }

    if (actions[0] == "peers") {
        const bool refresh = common::HasFlag(argc, argv, "--refresh");
        const auto result = refresh ? client.RefreshDataPeers() : client.ListDataPeers(false);
        std::cout << "Data peers: " << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
        const auto state_name = [](swarmkit::client::DataPeerState state) {
            switch (state) {
                case swarmkit::client::DataPeerState::kReady:
                    return "ready";
                case swarmkit::client::DataPeerState::kUnreachable:
                    return "unreachable";
                case swarmkit::client::DataPeerState::kMisconfigured:
                    return "misconfigured";
                case swarmkit::client::DataPeerState::kUnknown:
                default:
                    return "unknown";
            }
        };
        for (const auto& peer : result.peers) {
            std::cout << "peer=" << peer.drone_id << " address=" << peer.address
                      << " security=" << peer.transport_security
                      << " state=" << state_name(peer.state)
                      << " rtt_ms=" << peer.round_trip_ms;
            if (!peer.message.empty()) {
                std::cout << " message=" << peer.message;
            }
            std::cout << "\n";
        }
        return result.ok ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    if (actions[0] == "publish" || actions[0] == "send") {
        const std::string topic = common::GetOptionValue(argc, argv, "--topic");
        if (topic.empty()) {
            std::cerr << "message " << actions[0] << " requires --topic TOPIC\n";
            return EXIT_FAILURE;
        }
        auto labels = ParseKeyValueLabels(argc, argv, "--label");
        if (!labels.has_value()) {
            std::cerr << labels.error() << "\n";
            return EXIT_FAILURE;
        }
        auto payload = ReadSmallPayload(argc, argv);
        if (!payload.has_value()) {
            std::cerr << payload.error() << "\n";
            return EXIT_FAILURE;
        }
        swarmkit::client::DataMessage message;
        message.topic = topic;
        message.target_id = common::GetOptionValue(argc, argv, "--target");
        const auto ttl_ms = ParseDurationMs(argc, argv);
        if (!ttl_ms.has_value()) {
            std::cerr << ttl_ms.error() << "\n";
            return EXIT_FAILURE;
        }
        message.ttl_ms = *ttl_ms;
        message.labels = std::move(*labels);
        message.payload = std::move(*payload);

        const bool routed_send = actions[0] == "send";
        if (routed_send && message.target_id.empty()) {
            std::cerr << "message send requires --target DRONE_ID\n";
            return EXIT_FAILURE;
        }
        const auto result =
            routed_send ? client.SendMessageToDrone(std::move(message))
                        : client.PublishMessage(std::move(message));
        std::cout << (routed_send ? "Message send: " : "Message publish: ")
                  << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (result.sequence > 0) {
            std::cout << " seq=" << result.sequence;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
        return result.ok ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    if (actions[0] == "subscribe") {
        const auto duration_ms = ParseDurationMs(argc, argv);
        if (!duration_ms.has_value()) {
            std::cerr << duration_ms.error() << "\n";
            return EXIT_FAILURE;
        }
        const auto after_sequence =
            ParseIntArg(common::GetOptionValue(argc, argv, "--after-sequence", kDefaultZero),
                        "--after-sequence");
        if (!after_sequence.has_value() || *after_sequence < 0) {
            std::cerr << "Invalid --after-sequence\n";
            return EXIT_FAILURE;
        }

        ResetStopRequested();
        std::signal(SIGINT, OnDataSignal);
        std::signal(SIGTERM, OnDataSignal);

        swarmkit::client::MessageSubscription subscription;
        subscription.target_id = common::GetOptionValue(argc, argv, "--target");
        subscription.topics = CollectOptionValues(argc, argv, "--topic");
        subscription.after_sequence = static_cast<std::uint64_t>(*after_sequence);
        if (subscription.target_id.empty()) {
            std::cerr << "Note: no --target set; targeted messages for another id may not appear.\n";
        }

        std::mutex output_mutex;
        auto stream = client.StartMessages(
            std::move(subscription),
            [&](const swarmkit::client::DataMessage& message) {
                std::lock_guard<std::mutex> lock(output_mutex);
                std::cout << "MESSAGE seq=" << message.sequence << " topic=" << message.topic
                          << " source=" << message.source_id;
                if (!message.target_id.empty()) {
                    std::cout << " target=" << message.target_id;
                }
                if (!message.message_id.empty()) {
                    std::cout << " id=" << message.message_id;
                }
                std::cout << " payload=" << message.payload << "\n";
            },
            [](const std::string& error) { std::cerr << "message stream error: " << error << "\n"; });
        if (!stream.has_value()) {
            std::cerr << "Message subscribe failed: " << stream.error().user_message << "\n";
            return EXIT_FAILURE;
        }

        const auto start = std::chrono::steady_clock::now();
        while (!IsStopRequested()) {
            if (*duration_ms > 0 &&
                std::chrono::steady_clock::now() - start >= std::chrono::milliseconds(*duration_ms)) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{100});
        }
        stream->Stop();
        return EXIT_SUCCESS;
    }

    std::cerr << "Unknown message action: " << actions[0] << "\n";
    return EXIT_FAILURE;
}

int RunArtifact(Client& client, int argc, char** argv) {
    const auto actions = FindActionsAfterCommand(argc, argv, "artifact");
    if (actions.empty()) {
        std::cerr << "artifact requires upload|send|start|status|cancel|list|info|download|announce\n";
        return EXIT_FAILURE;
    }

    const auto print_descriptor = [](const swarmkit::client::ArtifactDescriptor& descriptor,
                                     std::string_view indent) {
        std::cout << indent << "artifact_id : " << descriptor.artifact_id << "\n"
                  << indent << "source_id   : " << descriptor.source_id << "\n"
                  << indent << "target_id   : " << descriptor.target_id << "\n"
                  << indent << "content_type: " << descriptor.content_type << "\n"
                  << indent << "size_bytes  : " << descriptor.size_bytes << "\n"
                  << indent << "created_ms  : " << descriptor.created_unix_ms << "\n"
                  << indent << "ttl_ms      : " << descriptor.ttl_ms << "\n"
                  << indent << "sha256      : " << descriptor.sha256_hex << "\n"
                  << indent << "filename    : " << descriptor.filename << "\n";
        if (!descriptor.labels.empty()) {
            std::cout << indent << "labels:\n";
            for (const auto& [key, value] : descriptor.labels) {
                std::cout << indent << "  " << key << "=" << value << "\n";
            }
        }
    };
    const auto transfer_state_name = [](swarmkit::client::ArtifactTransferState state) {
        switch (state) {
            case swarmkit::client::ArtifactTransferState::kQueued:
                return "queued";
            case swarmkit::client::ArtifactTransferState::kRunning:
                return "running";
            case swarmkit::client::ArtifactTransferState::kCompleted:
                return "completed";
            case swarmkit::client::ArtifactTransferState::kFailed:
                return "failed";
            case swarmkit::client::ArtifactTransferState::kCancelled:
                return "cancelled";
            case swarmkit::client::ArtifactTransferState::kUnknown:
            default:
                return "unknown";
        }
    };
    const auto print_transfer = [&](std::string_view prefix,
                                    const swarmkit::client::ArtifactTransferStatusResult& result) {
        std::cout << prefix << ": " << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.transfer.transfer_id.empty()) {
            std::cout << " transfer_id=" << result.transfer.transfer_id
                      << " state=" << transfer_state_name(result.transfer.state)
                      << " bytes=" << result.transfer.bytes_transferred << "/"
                      << result.transfer.bytes_total;
        }
        if (!result.transfer.descriptor.artifact_id.empty()) {
            std::cout << " artifact_id=" << result.transfer.descriptor.artifact_id;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
    };

    if (actions[0] == "upload" || actions[0] == "send") {
        const std::string file_path = common::GetOptionValue(argc, argv, "--file");
        if (file_path.empty()) {
            std::cerr << "artifact " << actions[0] << " requires --file PATH\n";
            return EXIT_FAILURE;
        }
        auto labels = ParseKeyValueLabels(argc, argv, "--label");
        if (!labels.has_value()) {
            std::cerr << labels.error() << "\n";
            return EXIT_FAILURE;
        }
        swarmkit::client::ArtifactUpload upload;
        upload.file_path = file_path;
        upload.descriptor.target_id = common::GetOptionValue(argc, argv, "--target");
        upload.descriptor.content_type =
            common::GetOptionValue(argc, argv, "--content-type", "application/octet-stream");
        const auto ttl_ms = ParseDurationMs(argc, argv);
        if (!ttl_ms.has_value()) {
            std::cerr << ttl_ms.error() << "\n";
            return EXIT_FAILURE;
        }
        upload.descriptor.ttl_ms = *ttl_ms;
        upload.descriptor.labels = std::move(*labels);
        const bool routed_send = actions[0] == "send";
        if (routed_send && upload.descriptor.target_id.empty()) {
            std::cerr << "artifact send requires --target DRONE_ID\n";
            return EXIT_FAILURE;
        }
        if (const std::string chunk_bytes = common::GetOptionValue(argc, argv, "--chunk-bytes");
            !chunk_bytes.empty()) {
            const auto parsed = ParseIntArg(chunk_bytes, "--chunk-bytes");
            if (!parsed.has_value() || *parsed <= 0) {
                std::cerr << "Invalid --chunk-bytes\n";
                return EXIT_FAILURE;
            }
            upload.chunk_bytes = static_cast<std::size_t>(*parsed);
        }

        const auto result =
            routed_send ? client.SendArtifactToDrone(upload) : client.UploadArtifact(upload);
        std::cout << (routed_send ? "Artifact send: " : "Artifact upload: ")
                  << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.descriptor.artifact_id.empty()) {
            std::cout << " artifact_id=" << result.descriptor.artifact_id;
        }
        if (!result.descriptor.sha256_hex.empty()) {
            std::cout << " sha256=" << result.descriptor.sha256_hex;
        }
        if (result.ok || result.descriptor.size_bytes > 0) {
            std::cout << " size=" << result.descriptor.size_bytes;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
        return result.ok ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    if (actions[0] == "start") {
        const std::string file_path = common::GetOptionValue(argc, argv, "--file");
        if (file_path.empty()) {
            std::cerr << "artifact start requires --file PATH\n";
            return EXIT_FAILURE;
        }
        auto labels = ParseKeyValueLabels(argc, argv, "--label");
        if (!labels.has_value()) {
            std::cerr << labels.error() << "\n";
            return EXIT_FAILURE;
        }
        swarmkit::client::ArtifactUpload upload;
        upload.file_path = file_path;
        upload.descriptor.target_id = common::GetOptionValue(argc, argv, "--target");
        upload.descriptor.content_type =
            common::GetOptionValue(argc, argv, "--content-type", "application/octet-stream");
        const auto ttl_ms = ParseDurationMs(argc, argv);
        if (!ttl_ms.has_value()) {
            std::cerr << ttl_ms.error() << "\n";
            return EXIT_FAILURE;
        }
        upload.descriptor.ttl_ms = *ttl_ms;
        upload.descriptor.labels = std::move(*labels);
        if (const std::string chunk_bytes = common::GetOptionValue(argc, argv, "--chunk-bytes");
            !chunk_bytes.empty()) {
            const auto parsed = ParseIntArg(chunk_bytes, "--chunk-bytes");
            if (!parsed.has_value() || *parsed <= 0) {
                std::cerr << "Invalid --chunk-bytes\n";
                return EXIT_FAILURE;
            }
            upload.chunk_bytes = static_cast<std::size_t>(*parsed);
        }
        const bool route = common::HasFlag(argc, argv, "--route") ||
                           !upload.descriptor.target_id.empty();
        if (route && upload.descriptor.target_id.empty()) {
            std::cerr << "artifact start --route requires --target DRONE_ID\n";
            return EXIT_FAILURE;
        }
        const auto result = client.StartArtifactTransfer(upload, route);
        print_transfer("Artifact transfer start", result);
        return result.ok ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    if (actions[0] == "status" || actions[0] == "cancel") {
        const std::string transfer_id = common::GetOptionValue(argc, argv, "--transfer-id");
        if (transfer_id.empty()) {
            std::cerr << "artifact " << actions[0] << " requires --transfer-id ID\n";
            return EXIT_FAILURE;
        }
        const auto result = actions[0] == "cancel" ? client.CancelArtifactTransfer(transfer_id)
                                                   : client.GetArtifactTransfer(transfer_id);
        print_transfer(actions[0] == "cancel" ? "Artifact transfer cancel"
                                              : "Artifact transfer status",
                       result);
        return result.ok ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    if (actions[0] == "list") {
        std::string source_id = common::GetOptionValue(argc, argv, "--source-id");
        if (source_id.empty()) {
            source_id = common::GetOptionValue(argc, argv, "--source");
        }
        const std::string target_id = common::GetOptionValue(argc, argv, "--target");
        const bool include_expired = common::HasFlag(argc, argv, "--include-expired");
        swarmkit::client::ArtifactListOptions options;
        options.source_id = source_id;
        options.target_id = target_id;
        options.include_expired = include_expired;
        options.page_token = common::GetOptionValue(argc, argv, "--page-token");
        if (const std::string page_size = common::GetOptionValue(argc, argv, "--page-size");
            !page_size.empty()) {
            const auto parsed = ParseIntArg(page_size, "--page-size");
            if (!parsed.has_value() || *parsed <= 0) {
                std::cerr << "Invalid --page-size\n";
                return EXIT_FAILURE;
            }
            options.page_size = *parsed;
        }
        const auto result = client.ListArtifacts(options);
        std::cout << "Artifact list: " << (result.ok ? "OK" : "FAILED")
                  << " count=" << result.artifacts.size()
                  << " total=" << result.total_count;
        if (!result.next_page_token.empty()) {
            std::cout << " next_page_token=" << result.next_page_token;
        }
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
        for (const auto& descriptor : result.artifacts) {
            std::cout << "  " << descriptor.artifact_id << " source=" << descriptor.source_id
                      << " target=" << descriptor.target_id
                      << " content_type=" << descriptor.content_type
                      << " size=" << descriptor.size_bytes
                      << " sha256=" << descriptor.sha256_hex
                      << " file=" << descriptor.filename << "\n";
        }
        return result.ok ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    if (actions[0] == "info") {
        const std::string artifact_id = common::GetOptionValue(argc, argv, "--artifact-id");
        if (artifact_id.empty()) {
            std::cerr << "artifact info requires --artifact-id ID\n";
            return EXIT_FAILURE;
        }
        const auto result = client.GetArtifact(artifact_id);
        std::cout << "Artifact info: " << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
        if (result.ok) {
            print_descriptor(result.descriptor, "  ");
        }
        return result.ok ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    if (actions[0] == "download") {
        const std::string artifact_id = common::GetOptionValue(argc, argv, "--artifact-id");
        const std::string file_path = common::GetOptionValue(argc, argv, "--file");
        if (artifact_id.empty() || file_path.empty()) {
            std::cerr << "artifact download requires --artifact-id ID --file PATH\n";
            return EXIT_FAILURE;
        }
        const auto result = client.DownloadArtifact(artifact_id, file_path);
        std::cout << "Artifact download: " << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.descriptor.artifact_id.empty()) {
            std::cout << " artifact_id=" << result.descriptor.artifact_id;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
        return result.ok ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    if (actions[0] == "announce") {
        const std::string artifact_id = common::GetOptionValue(argc, argv, "--artifact-id");
        if (artifact_id.empty()) {
            std::cerr << "artifact announce requires --artifact-id ID\n";
            return EXIT_FAILURE;
        }
        auto labels = ParseKeyValueLabels(argc, argv, "--label");
        if (!labels.has_value()) {
            std::cerr << labels.error() << "\n";
            return EXIT_FAILURE;
        }
        swarmkit::client::ArtifactDescriptor descriptor;
        descriptor.artifact_id = artifact_id;
        descriptor.target_id = common::GetOptionValue(argc, argv, "--target");
        descriptor.content_type =
            common::GetOptionValue(argc, argv, "--content-type", "application/octet-stream");
        descriptor.filename = common::GetOptionValue(argc, argv, "--file");
        const auto ttl_ms = ParseDurationMs(argc, argv);
        if (!ttl_ms.has_value()) {
            std::cerr << ttl_ms.error() << "\n";
            return EXIT_FAILURE;
        }
        descriptor.ttl_ms = *ttl_ms;
        descriptor.labels = std::move(*labels);
        const auto result = client.AnnounceArtifact(std::move(descriptor));
        std::cout << "Artifact announce: " << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
        return result.ok ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    std::cerr << "Unknown artifact action: " << actions[0] << "\n";
    return EXIT_FAILURE;
}
}  // namespace swarmkit::apps::cli::internal
