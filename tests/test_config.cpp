// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <fstream>

#include "swarmkit/agent/server.h"
#include "swarmkit/client/client.h"
#include "swarmkit/client/swarm_client.h"

namespace fs = std::filesystem;

TEST_CASE("ClientConfig validates retry and reconnect policies", "[client][config]") {
    swarmkit::client::ClientConfig config;
    CHECK(config.Validate().IsOk());

    config.address = "invalid-address";
    CHECK_FALSE(config.Validate().IsOk());

    config.address = "127.0.0.1:50061";
    config.retry_policy.max_attempts = 0;
    CHECK_FALSE(config.Validate().IsOk());
}

TEST_CASE("LoadClientConfigFromFile parses YAML config", "[client][config]") {
    const fs::path kConfigPath = fs::temp_directory_path() / "swarmkit_client_test.yaml";
    std::ofstream output(kConfigPath);
    REQUIRE(output.is_open());
    output << "client:\n";
    output << "  address: 10.0.0.5:50061\n";
    output << "  client_id: test-client\n";
    output << "  deadline_ms: 2500\n";
    output << "  priority: override\n";
    output << "  retry:\n";
    output << "    max_attempts: 4\n";
    output << "  stream_reconnect:\n";
    output << "    enabled: false\n";
    output.close();

    const auto kLoaded = swarmkit::client::LoadClientConfigFromFile(kConfigPath.string());
    REQUIRE(kLoaded.has_value());
    CHECK(kLoaded->address == "10.0.0.5:50061");
    CHECK(kLoaded->client_id == "test-client");
    CHECK(kLoaded->deadline_ms == 2500);
    CHECK(kLoaded->priority == swarmkit::commands::CommandPriority::kOverride);
    CHECK(kLoaded->retry_policy.max_attempts == 4);
    CHECK_FALSE(kLoaded->stream_reconnect_policy.enabled);

    fs::remove(kConfigPath);
}

TEST_CASE("LoadClientConfigFromFile rejects invalid priority strings", "[client][config]") {
    const fs::path kConfigPath = fs::temp_directory_path() / "swarmkit_client_invalid.yaml";
    std::ofstream output(kConfigPath);
    REQUIRE(output.is_open());
    output << "client:\n";
    output << "  address: 10.0.0.5:50061\n";
    output << "  client_id: test-client\n";
    output << "  priority: invalid-priority\n";
    output.close();

    const auto kLoaded = swarmkit::client::LoadClientConfigFromFile(kConfigPath.string());
    CHECK_FALSE(kLoaded.has_value());

    fs::remove(kConfigPath);
}

TEST_CASE("AgentConfig validates rates and bind address", "[agent][config]") {
    swarmkit::agent::AgentConfig config;
    CHECK(config.Validate().IsOk());

    config.bind_addr = "";
    CHECK_FALSE(config.Validate().IsOk());

    config.bind_addr = "0.0.0.0:50061";
    config.min_telemetry_rate_hz = 10;
    config.default_telemetry_rate_hz = 5;
    CHECK_FALSE(config.Validate().IsOk());
}

TEST_CASE("LoadAgentConfigFromFile parses YAML values", "[agent][config]") {
    const fs::path kConfigPath = fs::temp_directory_path() / "swarmkit_agent_test.yaml";
    std::ofstream output(kConfigPath);
    REQUIRE(output.is_open());
    output << "agent:\n";
    output << "  agent_id: agent-prod\n";
    output << "  bind_addr: 192.168.1.10:50061\n";
    output << "  default_authority_ttl_ms: 7000\n";
    output << "  default_telemetry_rate_hz: 8\n";
    output << "  min_telemetry_rate_hz: 2\n";
    output.close();

    const auto kLoaded = swarmkit::agent::LoadAgentConfigFromFile(kConfigPath.string());
    REQUIRE(kLoaded.has_value());
    CHECK(kLoaded->agent_id == "agent-prod");
    CHECK(kLoaded->bind_addr == "192.168.1.10:50061");
    CHECK(kLoaded->default_authority_ttl_ms == 7000);
    CHECK(kLoaded->default_telemetry_rate_hz == 8);
    CHECK(kLoaded->min_telemetry_rate_hz == 2);

    fs::remove(kConfigPath);
}

TEST_CASE("LoadSwarmConfigFromFile parses client defaults and drone topology", "[swarm][config]") {
    const fs::path kConfigPath = fs::temp_directory_path() / "swarmkit_swarm_test.yaml";
    std::ofstream output(kConfigPath);
    REQUIRE(output.is_open());
    output << "client:\n";
    output << "  client_id: test-gcs\n";
    output << "  deadline_ms: 3500\n";
    output << "  priority: override\n";
    output << "swarm:\n";
    output << "  drones:\n";
    output << "    - drone_id: drone-1\n";
    output << "      address: 192.168.1.101:50061\n";
    output << "      local_address: 127.0.0.1:50061\n";
    output << "    - drone_id: drone-2\n";
    output << "      address: 192.168.1.102:50061\n";
    output.close();

    const auto kLoaded = swarmkit::client::LoadSwarmConfigFromFile(kConfigPath.string());
    REQUIRE(kLoaded.has_value());
    CHECK(kLoaded->default_client_config.client_id == "test-gcs");
    CHECK(kLoaded->default_client_config.deadline_ms == 3500);
    CHECK(kLoaded->default_client_config.priority ==
          swarmkit::commands::CommandPriority::kOverride);
    REQUIRE(kLoaded->drones.size() == 2);
    CHECK(kLoaded->drones[0].drone_id == "drone-1");
    CHECK(kLoaded->drones[0].address == "192.168.1.101:50061");
    CHECK(kLoaded->drones[0].local_address == "127.0.0.1:50061");
    CHECK(kLoaded->drones[1].drone_id == "drone-2");
    CHECK(kLoaded->drones[1].address == "192.168.1.102:50061");
    CHECK(kLoaded->drones[1].local_address.empty());

    fs::remove(kConfigPath);
}

TEST_CASE("LoadSwarmConfigFromFile rejects duplicate drone ids", "[swarm][config]") {
    const fs::path kConfigPath = fs::temp_directory_path() / "swarmkit_swarm_duplicate.yaml";
    std::ofstream output(kConfigPath);
    REQUIRE(output.is_open());
    output << "swarm:\n";
    output << "  drones:\n";
    output << "    - drone_id: drone-1\n";
    output << "      address: 192.168.1.101:50061\n";
    output << "    - drone_id: drone-1\n";
    output << "      address: 192.168.1.102:50061\n";
    output.close();

    const auto kLoaded = swarmkit::client::LoadSwarmConfigFromFile(kConfigPath.string());
    CHECK_FALSE(kLoaded.has_value());

    fs::remove(kConfigPath);
}

TEST_CASE("LoadSwarmConfigFromFile rejects invalid local addresses", "[swarm][config]") {
    const fs::path kConfigPath = fs::temp_directory_path() / "swarmkit_swarm_invalid_local.yaml";
    std::ofstream output(kConfigPath);
    REQUIRE(output.is_open());
    output << "swarm:\n";
    output << "  drones:\n";
    output << "    - drone_id: drone-1\n";
    output << "      address: 192.168.1.101:50061\n";
    output << "      local_address: invalid-local-address\n";
    output.close();

    const auto kLoaded = swarmkit::client::LoadSwarmConfigFromFile(kConfigPath.string());
    CHECK_FALSE(kLoaded.has_value());

    fs::remove(kConfigPath);
}
