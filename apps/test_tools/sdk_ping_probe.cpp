// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>

#include "swarmkit/client/client.h"
#include "swarmkit/core/security.h"

namespace {

[[nodiscard]] std::string OptionValue(int argc, char** argv, std::string_view name,
                                      std::string_view fallback = {}) {
    for (int index = 1; index + 1 < argc; ++index) {
        if (std::string_view{argv[index]} == name) {
            return argv[index + 1];
        }
    }
    return std::string{fallback};
}

[[nodiscard]] bool HasFlag(int argc, char** argv, std::string_view name) {
    for (int index = 1; index < argc; ++index) {
        if (std::string_view{argv[index]} == name) {
            return true;
        }
    }
    return false;
}

void PrintUsage(const char* program) {
    std::cerr << "Usage: " << program << " [--addr HOST:PORT] [--client-id ID] [--insecure]\n"
              << "       " << program
              << " --addr 127.0.0.1:50061 --insecure\n\n"
              << "Options:\n"
              << "  --addr HOST:PORT       SwarmKit agent address. Default: 127.0.0.1:50061\n"
              << "  --client-id ID         Client identity. Default: sdk-ping-probe\n"
              << "  --deadline-ms N        Unary RPC deadline. Default: 5000\n"
              << "  --insecure             Use plaintext gRPC transport.\n"
              << "  --ca-cert PATH         Root CA certificate for TLS.\n"
              << "  --client-cert PATH     Client certificate for mTLS.\n"
              << "  --client-key PATH      Client private key for mTLS.\n"
              << "  --server-name NAME     TLS authority override.\n";
}

[[nodiscard]] int ParseDeadlineMs(int argc, char** argv) {
    const std::string value = OptionValue(argc, argv, "--deadline-ms", "5000");
    try {
        return std::stoi(value);
    } catch (const std::exception&) {
        return -1;
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (HasFlag(argc, argv, "--help") || HasFlag(argc, argv, "-h")) {
        PrintUsage(argv[0]);
        return EXIT_SUCCESS;
    }

    swarmkit::client::ClientConfig config;
    config.address = OptionValue(argc, argv, "--addr", "127.0.0.1:50061");
    config.client_id = OptionValue(argc, argv, "--client-id", "sdk-ping-probe");
    config.deadline_ms = ParseDeadlineMs(argc, argv);
    if (config.deadline_ms < 0) {
        std::cerr << "Invalid --deadline-ms\n";
        return EXIT_FAILURE;
    }

    if (HasFlag(argc, argv, "--insecure")) {
        config.security.transport_security =
            swarmkit::core::TransportSecurityMode::kInsecure;
    } else {
        config.security.root_ca_cert_path = OptionValue(argc, argv, "--ca-cert");
        config.security.cert_chain_path = OptionValue(argc, argv, "--client-cert");
        config.security.private_key_path = OptionValue(argc, argv, "--client-key");
        config.security.server_authority_override = OptionValue(argc, argv, "--server-name");
    }

    auto validation = config.Validate();
    if (!validation.IsOk()) {
        std::cerr << "Invalid client config: " << validation.message << "\n";
        return EXIT_FAILURE;
    }

    swarmkit::client::Client client{config};
    const auto result = client.Ping();
    if (!result.ok) {
        std::cerr << "Ping FAILED: " << result.error_message;
        if (!result.correlation_id.empty()) {
            std::cerr << " [corr=" << result.correlation_id << "]";
        }
        if (!result.error.debug_message.empty()) {
            std::cerr << "\nDebug: " << result.error.debug_message;
        }
        std::cerr << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "Ping OK\n"
              << "  agent_id : " << result.agent_id << "\n"
              << "  version  : " << result.version << "\n"
              << "  time_ms  : " << result.unix_time_ms << "\n";
    return EXIT_SUCCESS;
}
