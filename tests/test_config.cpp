// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "swarmkit/agent/mavlink_backend.h"
#include "swarmkit/agent/server.h"
#include "swarmkit/client/client.h"
#include "swarmkit/client/swarm_client.h"

namespace fs = std::filesystem;

namespace {

[[nodiscard]] fs::path WriteTempFile(std::string_view name, std::string_view contents) {
    const fs::path kPath = fs::temp_directory_path() / std::string(name);
    std::ofstream output(kPath);
    if (!output.is_open()) {
        throw std::runtime_error("failed to open temp file for test");
    }
    output << contents;
    output.close();
    return kPath;
}

[[nodiscard]] fs::path RepoRootPath() {
    return fs::path(__FILE__).parent_path().parent_path();
}

[[nodiscard]] fs::path DevCertsPath() {
    return RepoRootPath() / "testdata" / "certs";
}

void ApplyDevClientSecurity(swarmkit::client::ClientConfig* config,
                            std::string_view client_cert_name = "swarmkit-cli") {
    REQUIRE(config != nullptr);
    const fs::path kCertsPath = DevCertsPath();
    config->security.root_ca_cert_path = (kCertsPath / "ca.pem").string();
    config->security.cert_chain_path =
        (kCertsPath / (std::string(client_cert_name) + ".pem")).string();
    config->security.private_key_path =
        (kCertsPath / (std::string(client_cert_name) + ".key")).string();
    config->security.server_authority_override = "localhost";
}

void ApplyDevAgentSecurity(swarmkit::agent::AgentConfig* config) {
    REQUIRE(config != nullptr);
    const fs::path kCertsPath = DevCertsPath();
    config->security.root_ca_cert_path = (kCertsPath / "ca.pem").string();
    config->security.cert_chain_path = (kCertsPath / "agent.pem").string();
    config->security.private_key_path = (kCertsPath / "agent.key").string();
}

}  // namespace

TEST_CASE("ClientConfig validates retry and reconnect policies", "[client][config]") {
    swarmkit::client::ClientConfig config;
    ApplyDevClientSecurity(&config);
    CHECK(config.Validate().IsOk());

    config.address = "invalid-address";
    CHECK_FALSE(config.Validate().IsOk());

    config.address = "127.0.0.1:50061";
    config.retry_policy.max_attempts = 0;
    CHECK_FALSE(config.Validate().IsOk());
}

TEST_CASE("ClientConfig validates mTLS security settings", "[client][config][security]") {
    const fs::path kRootCa = WriteTempFile("swarmkit_test_root_ca.pem", "root-ca");
    const fs::path kClientCert = WriteTempFile("swarmkit_test_client_cert.pem", "client-cert");
    const fs::path kClientKey = WriteTempFile("swarmkit_test_client_key.pem", "client-key");

    swarmkit::client::ClientConfig config;
    CHECK(config.Validate().IsOk());
    CHECK(config.security.EffectiveTransportSecurity() ==
          swarmkit::core::TransportSecurityMode::kInsecure);

    config.security.root_ca_cert_path = kRootCa.string();
    CHECK(config.Validate().IsOk());
    CHECK(config.security.EffectiveTransportSecurity() ==
          swarmkit::core::TransportSecurityMode::kTls);

    config.security.transport_security = swarmkit::core::TransportSecurityMode::kMutualTls;
    CHECK_FALSE(config.Validate().IsOk());

    config.security.cert_chain_path = kClientCert.string();
    config.security.private_key_path = kClientKey.string();
    CHECK(config.Validate().IsOk());
    CHECK(config.security.EffectiveTransportSecurity() ==
          swarmkit::core::TransportSecurityMode::kMutualTls);

    fs::remove(kRootCa);
    fs::remove(kClientCert);
    fs::remove(kClientKey);
}

TEST_CASE("LoadClientConfigFromFile parses YAML config", "[client][config]") {
    const fs::path kCertsPath = DevCertsPath();
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
    output << "  security:\n";
    output << "    transport_security: mtls\n";
    output << "    root_ca_cert_path: " << (kCertsPath / "ca.pem").string() << "\n";
    output << "    cert_chain_path: " << (kCertsPath / "swarmkit-cli.pem").string() << "\n";
    output << "    private_key_path: " << (kCertsPath / "swarmkit-cli.key").string() << "\n";
    output.close();

    const auto kLoaded = swarmkit::client::LoadClientConfigFromFile(kConfigPath.string());
    REQUIRE(kLoaded.has_value());
    CHECK(kLoaded->address == "10.0.0.5:50061");
    CHECK(kLoaded->client_id == "test-client");
    CHECK(kLoaded->deadline_ms == 2500);
    CHECK(kLoaded->priority == swarmkit::commands::CommandPriority::kOverride);
    CHECK(kLoaded->retry_policy.max_attempts == 4);
    CHECK_FALSE(kLoaded->stream_reconnect_policy.enabled);
    CHECK(kLoaded->security.transport_security ==
          swarmkit::core::TransportSecurityMode::kMutualTls);

    fs::remove(kConfigPath);
}

TEST_CASE("LoadClientConfigFromFile rejects invalid priority strings", "[client][config]") {
    const fs::path kCertsPath = DevCertsPath();
    const fs::path kConfigPath = fs::temp_directory_path() / "swarmkit_client_invalid.yaml";
    std::ofstream output(kConfigPath);
    REQUIRE(output.is_open());
    output << "client:\n";
    output << "  address: 10.0.0.5:50061\n";
    output << "  client_id: test-client\n";
    output << "  priority: invalid-priority\n";
    output << "  security:\n";
    output << "    transport_security: mtls\n";
    output << "    root_ca_cert_path: " << (kCertsPath / "ca.pem").string() << "\n";
    output << "    cert_chain_path: " << (kCertsPath / "swarmkit-cli.pem").string() << "\n";
    output << "    private_key_path: " << (kCertsPath / "swarmkit-cli.key").string() << "\n";
    output.close();

    const auto kLoaded = swarmkit::client::LoadClientConfigFromFile(kConfigPath.string());
    CHECK_FALSE(kLoaded.has_value());

    fs::remove(kConfigPath);
}

TEST_CASE("AgentConfig validates rates and bind address", "[agent][config]") {
    swarmkit::agent::AgentConfig config;
    CHECK(config.Validate().IsOk());
    CHECK(config.security.EffectiveTransportSecurity() ==
          swarmkit::core::TransportSecurityMode::kInsecure);

    ApplyDevAgentSecurity(&config);
    CHECK(config.Validate().IsOk());
    CHECK(config.security.EffectiveTransportSecurity() ==
          swarmkit::core::TransportSecurityMode::kMutualTls);

    config.bind_addr = "";
    CHECK_FALSE(config.Validate().IsOk());

    config.bind_addr = "0.0.0.0:50061";
    config.min_telemetry_rate_hz = 10;
    config.default_telemetry_rate_hz = 5;
    CHECK_FALSE(config.Validate().IsOk());
}

TEST_CASE("LoadAgentConfigFromFile parses YAML values", "[agent][config]") {
    const fs::path kCertsPath = DevCertsPath();
    const fs::path kConfigPath = fs::temp_directory_path() / "swarmkit_agent_test.yaml";
    std::ofstream output(kConfigPath);
    REQUIRE(output.is_open());
    output << "agent:\n";
    output << "  agent_id: agent-prod\n";
    output << "  bind_addr: 192.168.1.10:50061\n";
    output << "  default_authority_ttl_ms: 7000\n";
    output << "  default_telemetry_rate_hz: 8\n";
    output << "  min_telemetry_rate_hz: 2\n";
    output << "  report_log_file: /tmp/swarmkit-agent-reports.jsonl\n";
    output << "  safety:\n";
    output << "    allow_unsafe_bench_commands: true\n";
    output << "  report_persistence:\n";
    output << "    sequence_state_file: /tmp/swarmkit-agent-reports.seq\n";
    output << "    backlog_size: 2048\n";
    output << "    max_log_file_size_bytes: 4096\n";
    output << "    max_log_files: 7\n";
    output << "    flush_each_write: true\n";
    output << "    fsync_each_write: false\n";
    output << "    replay_from_log: true\n";
    output << "  data:\n";
    output << "    artifact_dir: /tmp/swarmkit-artifacts-test\n";
    output << "    message_backlog_size: 99\n";
    output << "    max_message_payload_bytes: 12345\n";
    output << "    artifact_chunk_bytes: 4096\n";
    output << "    max_artifact_bytes: 98765\n";
    output << "    max_concurrent_artifact_transfers: 3\n";
    output << "    max_queued_artifact_transfers: 11\n";
    output << "    max_concurrent_artifact_transfers_per_peer: 2\n";
    output << "    artifact_peer_bytes_per_second: 8192\n";
    output << "    peers:\n";
    output << "      - drone_id: drone-2\n";
    output << "        address: 127.0.0.1:50062\n";
    output << "        transport_security: insecure\n";
    output << "  vehicle_profile:\n";
    output << "    profile_id: heavy-copter\n";
    output << "    cruise_speed_mps: 6.5\n";
    output << "    climb_speed_mps: 2.0\n";
    output << "    descent_speed_mps: 1.2\n";
    output << "    max_altitude_m: 150\n";
    output << "    min_battery_percent: 25\n";
    output << "    battery_reserve_percent: 12\n";
    output << "    tracking_tolerance_m: 1.5\n";
    output << "    goal_margin_ms: 25000\n";
    output << "    takeoff_timeout_margin_ms: 22000\n";
    output << "    land_timeout_margin_ms: 33000\n";
    output << "    max_goal_timeout_ms: 420000\n";
    output << "    min_gps_fix_type: 4\n";
    output << "    min_satellites_visible: 9\n";
    output << "    max_gps_hdop: 1.8\n";
    output << "  security:\n";
    output << "    transport_security: mtls\n";
    output << "    root_ca_cert_path: " << (kCertsPath / "ca.pem").string() << "\n";
    output << "    cert_chain_path: " << (kCertsPath / "agent.pem").string() << "\n";
    output << "    private_key_path: " << (kCertsPath / "agent.key").string() << "\n";
    output.close();

    const auto kLoaded = swarmkit::agent::LoadAgentConfigFromFile(kConfigPath.string());
    REQUIRE(kLoaded.has_value());
    CHECK(kLoaded->agent_id == "agent-prod");
    CHECK(kLoaded->bind_addr == "192.168.1.10:50061");
    CHECK(kLoaded->default_authority_ttl_ms == 7000);
    CHECK(kLoaded->default_telemetry_rate_hz == 8);
    CHECK(kLoaded->min_telemetry_rate_hz == 2);
    CHECK(kLoaded->reports.log_file == "/tmp/swarmkit-agent-reports.jsonl");
    CHECK(kLoaded->reports.sequence_state_file == "/tmp/swarmkit-agent-reports.seq");
    CHECK(kLoaded->reports.backlog_size == 2048);
    CHECK(kLoaded->reports.max_log_file_size_bytes == 4096);
    CHECK(kLoaded->reports.max_log_files == 7);
    CHECK(kLoaded->reports.flush_each_write);
    CHECK_FALSE(kLoaded->reports.fsync_each_write);
    CHECK(kLoaded->reports.replay_from_log);
    CHECK(kLoaded->data.artifact_dir == "/tmp/swarmkit-artifacts-test");
    CHECK(kLoaded->data.message_backlog_size == 99);
    CHECK(kLoaded->data.max_message_payload_bytes == 12345);
    CHECK(kLoaded->data.artifact_chunk_bytes == 4096);
    CHECK(kLoaded->data.max_artifact_bytes == 98765);
    CHECK(kLoaded->data.max_concurrent_artifact_transfers == 3);
    CHECK(kLoaded->data.max_queued_artifact_transfers == 11);
    CHECK(kLoaded->data.max_concurrent_artifact_transfers_per_peer == 2);
    CHECK(kLoaded->data.artifact_peer_bytes_per_second == 8192);
    REQUIRE(kLoaded->data.peers.size() == 1);
    CHECK(kLoaded->data.peers.front().drone_id == "drone-2");
    CHECK(kLoaded->data.peers.front().address == "127.0.0.1:50062");
    CHECK(kLoaded->data.peers.front().transport_security ==
          swarmkit::core::TransportSecurityMode::kInsecure);
    CHECK(kLoaded->safety.allow_unsafe_bench_commands);
    CHECK(kLoaded->vehicle_profile.profile_id == "heavy-copter");
    CHECK(kLoaded->vehicle_profile.cruise_speed_mps == 6.5F);
    CHECK(kLoaded->vehicle_profile.climb_speed_mps == 2.0F);
    CHECK(kLoaded->vehicle_profile.descent_speed_mps == 1.2F);
    CHECK(kLoaded->vehicle_profile.max_altitude_m == 150.0F);
    CHECK(kLoaded->vehicle_profile.min_battery_percent == 25.0F);
    CHECK(kLoaded->vehicle_profile.battery_reserve_percent == 12.0F);
    CHECK(kLoaded->vehicle_profile.tracking_tolerance_m == 1.5F);
    CHECK(kLoaded->vehicle_profile.goal_margin_ms == 25000);
    CHECK(kLoaded->vehicle_profile.takeoff_timeout_margin_ms == 22000);
    CHECK(kLoaded->vehicle_profile.land_timeout_margin_ms == 33000);
    CHECK(kLoaded->vehicle_profile.max_goal_timeout_ms == 420000);
    CHECK(kLoaded->vehicle_profile.min_gps_fix_type == 4);
    CHECK(kLoaded->vehicle_profile.min_satellites_visible == 9);
    CHECK(kLoaded->vehicle_profile.max_gps_hdop == 1.8F);
    CHECK(kLoaded->security.transport_security ==
          swarmkit::core::TransportSecurityMode::kMutualTls);

    fs::remove(kConfigPath);
}

TEST_CASE("MAVLink autopilot profile parser supports production profiles",
          "[agent][config][mavlink]") {
    const auto copter = swarmkit::agent::ParseMavlinkAutopilotProfile("ardupilot-copter");
    REQUIRE(copter.has_value());
    CHECK(*copter == swarmkit::agent::MavlinkAutopilotProfile::kArdupilotCopter);

    const auto plane = swarmkit::agent::ParseMavlinkAutopilotProfile("plane");
    REQUIRE(plane.has_value());
    CHECK(*plane == swarmkit::agent::MavlinkAutopilotProfile::kArdupilotPlane);

    CHECK_FALSE(swarmkit::agent::ParseMavlinkAutopilotProfile("px4").has_value());
    CHECK_FALSE(swarmkit::agent::ParseMavlinkAutopilotProfile("unknown").has_value());
}

TEST_CASE("LoadAgentConfigFromFile parses agent security config", "[agent][config][security]") {
    const fs::path kRootCa = WriteTempFile("swarmkit_agent_root_ca.pem", "root-ca");
    const fs::path kServerCert = WriteTempFile("swarmkit_agent_cert.pem", "server-cert");
    const fs::path kServerKey = WriteTempFile("swarmkit_agent_key.pem", "server-key");
    const fs::path kConfigPath = fs::temp_directory_path() / "swarmkit_agent_security.yaml";

    std::ofstream output(kConfigPath);
    REQUIRE(output.is_open());
    output << "agent:\n";
    output << "  agent_id: secure-agent\n";
    output << "  bind_addr: 127.0.0.1:50061\n";
    output << "  security:\n";
    output << "    root_ca_cert_path: " << kRootCa.string() << "\n";
    output << "    cert_chain_path: " << kServerCert.string() << "\n";
    output << "    private_key_path: " << kServerKey.string() << "\n";
    output << "    allowed_client_ids:\n";
    output << "      - gcs-1\n";
    output.close();

    const auto kLoaded = swarmkit::agent::LoadAgentConfigFromFile(kConfigPath.string());
    REQUIRE(kLoaded.has_value());
    CHECK(kLoaded->security.root_ca_cert_path == kRootCa.string());
    CHECK(kLoaded->security.cert_chain_path == kServerCert.string());
    CHECK(kLoaded->security.private_key_path == kServerKey.string());
    REQUIRE(kLoaded->security.allowed_client_ids.size() == 1);
    CHECK(kLoaded->security.allowed_client_ids.front() == "gcs-1");

    fs::remove(kRootCa);
    fs::remove(kServerCert);
    fs::remove(kServerKey);
    fs::remove(kConfigPath);
}

TEST_CASE("AgentConfig rejects identity ACL without mTLS", "[agent][config][security]") {
    swarmkit::agent::AgentConfig config;
    config.security.transport_security = swarmkit::core::TransportSecurityMode::kInsecure;
    config.security.allowed_client_ids = {"gcs-1"};

    const auto result = config.Validate();

    CHECK_FALSE(result.IsOk());
    CHECK(result.message.find("allowed_client_ids") != std::string::npos);
}

TEST_CASE("LoadClientConfigFromFile resolves relative mTLS cert paths",
          "[client][config][security]") {
    const fs::path kConfigDir = fs::temp_directory_path() / "swarmkit_client_relative_security";
    fs::create_directories(kConfigDir / "certs");

    const fs::path kRootCa = WriteTempFile("swarmkit_client_relative_ca.pem", "root-ca");
    const fs::path kClientCert = WriteTempFile("swarmkit_client_relative_cert.pem", "client-cert");
    const fs::path kClientKey = WriteTempFile("swarmkit_client_relative_key.pem", "client-key");
    fs::rename(kRootCa, kConfigDir / "certs" / "ca.pem");
    fs::rename(kClientCert, kConfigDir / "certs" / "client.pem");
    fs::rename(kClientKey, kConfigDir / "certs" / "client.key");

    const fs::path kConfigPath = kConfigDir / "client.yaml";
    std::ofstream output(kConfigPath);
    REQUIRE(output.is_open());
    output << "client:\n";
    output << "  address: 127.0.0.1:50061\n";
    output << "  client_id: test-client\n";
    output << "  security:\n";
    output << "    root_ca_cert_path: certs/ca.pem\n";
    output << "    cert_chain_path: certs/client.pem\n";
    output << "    private_key_path: certs/client.key\n";
    output.close();

    const auto kLoaded = swarmkit::client::LoadClientConfigFromFile(kConfigPath.string());
    REQUIRE(kLoaded.has_value());
    CHECK(kLoaded->security.root_ca_cert_path == (kConfigDir / "certs" / "ca.pem").string());
    CHECK(kLoaded->security.cert_chain_path == (kConfigDir / "certs" / "client.pem").string());
    CHECK(kLoaded->security.private_key_path == (kConfigDir / "certs" / "client.key").string());

    fs::remove_all(kConfigDir);
}

TEST_CASE("LoadSwarmConfigFromFile parses client defaults and drone topology", "[swarm][config]") {
    const fs::path kCertsPath = DevCertsPath();
    const fs::path kConfigPath = fs::temp_directory_path() / "swarmkit_swarm_test.yaml";
    std::ofstream output(kConfigPath);
    REQUIRE(output.is_open());
    output << "client:\n";
    output << "  client_id: test-gcs\n";
    output << "  deadline_ms: 3500\n";
    output << "  priority: override\n";
    output << "  security:\n";
    output << "    root_ca_cert_path: " << (kCertsPath / "ca.pem").string() << "\n";
    output << "    cert_chain_path: " << (kCertsPath / "test-server.pem").string() << "\n";
    output << "    private_key_path: " << (kCertsPath / "test-server.key").string() << "\n";
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
    const fs::path kCertsPath = DevCertsPath();
    const fs::path kConfigPath = fs::temp_directory_path() / "swarmkit_swarm_duplicate.yaml";
    std::ofstream output(kConfigPath);
    REQUIRE(output.is_open());
    output << "client:\n";
    output << "  client_id: test-gcs\n";
    output << "  security:\n";
    output << "    root_ca_cert_path: " << (kCertsPath / "ca.pem").string() << "\n";
    output << "    cert_chain_path: " << (kCertsPath / "test-server.pem").string() << "\n";
    output << "    private_key_path: " << (kCertsPath / "test-server.key").string() << "\n";
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
    const fs::path kCertsPath = DevCertsPath();
    const fs::path kConfigPath = fs::temp_directory_path() / "swarmkit_swarm_invalid_local.yaml";
    std::ofstream output(kConfigPath);
    REQUIRE(output.is_open());
    output << "client:\n";
    output << "  client_id: test-gcs\n";
    output << "  security:\n";
    output << "    root_ca_cert_path: " << (kCertsPath / "ca.pem").string() << "\n";
    output << "    cert_chain_path: " << (kCertsPath / "test-server.pem").string() << "\n";
    output << "    private_key_path: " << (kCertsPath / "test-server.key").string() << "\n";
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
