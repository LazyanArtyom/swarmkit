// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "data_runtime.h"

#include "data_runtime_common.h"

namespace swarmkit::apps::cli::internal {

using swarmkit::client::Client;
using namespace data_runtime_detail;

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
