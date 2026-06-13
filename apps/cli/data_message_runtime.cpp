// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "data_runtime.h"
#include "data_runtime_common.h"

namespace swarmkit::apps::cli::internal {

using data_runtime_detail::CollectOptionValues;
using data_runtime_detail::FindActionsAfterCommand;
using data_runtime_detail::IsStopRequested;
using data_runtime_detail::OnDataSignal;
using data_runtime_detail::ParseDurationMs;
using data_runtime_detail::ParseKeyValueLabels;
using data_runtime_detail::ReadSmallPayload;
using data_runtime_detail::ResetStopRequested;
using swarmkit::client::Client;

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
                      << " state=" << state_name(peer.state) << " rtt_ms=" << peer.round_trip_ms;
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
        const auto result = routed_send ? client.SendMessageToDrone(std::move(message))
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
            std::cerr
                << "Note: no --target set; targeted messages for another id may not appear.\n";
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
            [](const std::string& error) {
                std::cerr << "message stream error: " << error << "\n";
            });
        if (!stream.has_value()) {
            std::cerr << "Message subscribe failed: " << stream.error().user_message << "\n";
            return EXIT_FAILURE;
        }

        const auto start = std::chrono::steady_clock::now();
        while (!IsStopRequested()) {
            if (*duration_ms > 0 && std::chrono::steady_clock::now() - start >=
                                        std::chrono::milliseconds(*duration_ms)) {
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

}  // namespace swarmkit::apps::cli::internal
