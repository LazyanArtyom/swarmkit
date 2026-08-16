// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>

extern "C" {
#include "ardupilotmega/mavlink.h"
}

#include "swarmkit/agent/backend.h"
#include "swarmkit/agent/mavlink_backend.h"
#include "swarmkit/agent/sim_backend.h"

namespace swarmkit::agent {
namespace {

using namespace std::chrono_literals;

[[nodiscard]] std::optional<std::string> AvailableLoopbackUdpAddress() {
    const int socket_handle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_handle < 0) {
        return std::nullopt;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    if (bind(socket_handle, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
        close(socket_handle);
        return std::nullopt;
    }

    socklen_t address_size = sizeof(address);
    if (getsockname(socket_handle, reinterpret_cast<sockaddr*>(&address), &address_size) != 0) {
        close(socket_handle);
        return std::nullopt;
    }
    const std::uint16_t port = ntohs(address.sin_port);
    close(socket_handle);
    return "127.0.0.1:" + std::to_string(port);
}

[[nodiscard]] bool SendMavlinkGlobalPosition(const std::string& destination) {
    const std::size_t colon = destination.rfind(':');
    if (colon == std::string::npos) {
        return false;
    }
    const auto port = static_cast<std::uint16_t>(std::stoi(destination.substr(colon + 1)));

    mavlink_message_t message{};
    mavlink_msg_global_position_int_pack(1, 1, &message, 1000, 401811000, 445136000, 1000, 1000, 0,
                                         0, 0, 0);
    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> bytes{};
    const std::uint16_t length = mavlink_msg_to_send_buffer(bytes.data(), &message);

    const int socket_handle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_handle < 0) {
        return false;
    }
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(port);
    const auto sent = sendto(socket_handle, bytes.data(), length, 0,
                             reinterpret_cast<const sockaddr*>(&address), sizeof(address));
    close(socket_handle);
    return sent == length;
}

void VerifyCallbackQuiescenceContract(IDroneBackend* backend, std::string_view backend_name,
                                      std::string_view drone_id,
                                      const std::function<void()>& produce_frame) {
    INFO("backend=" << backend_name);
    REQUIRE(backend != nullptr);

    std::promise<void> callback_entered;
    auto callback_entered_future = callback_entered.get_future();
    std::promise<void> release_callback;
    std::shared_future<void> release = release_callback.get_future().share();
    REQUIRE(backend
                ->StartTelemetry(std::string(drone_id), 5,
                                 [&](const core::TelemetryFrame&) {
                                     callback_entered.set_value();
                                     release.wait();
                                 })
                .IsOk());

    std::jthread producer([&produce_frame] {
        try {
            produce_frame();
        } catch (...) {
            static_cast<void>(0);
        }
    });
    if (callback_entered_future.wait_for(1s) != std::future_status::ready) {
        release_callback.set_value();
        FAIL("backend did not invoke its telemetry callback");
    }

    std::promise<void> stop_started;
    auto stop_started_future = stop_started.get_future();
    std::promise<core::Result> stop_completed;
    auto stop_completed_future = stop_completed.get_future();
    std::jthread stop([&] {
        stop_started.set_value();
        stop_completed.set_value(backend->StopTelemetry(std::string(drone_id)));
    });
    if (stop_started_future.wait_for(1s) != std::future_status::ready) {
        release_callback.set_value();
        FAIL("backend stop thread did not start");
    }
    CHECK(stop_completed_future.wait_for(20ms) == std::future_status::timeout);

    release_callback.set_value();
    REQUIRE(stop_completed_future.wait_for(1s) == std::future_status::ready);
    CHECK(stop_completed_future.get().IsOk());
}

void VerifyBackendLifecycleContract(DroneBackendPtr backend, std::string_view backend_name,
                                    std::string_view drone_id) {
    INFO("backend=" << backend_name);
    REQUIRE(backend != nullptr);

    CHECK(backend->StopTelemetry(std::string(drone_id)).IsOk());
    CHECK(backend->StopTelemetry(std::string(drone_id)).IsOk());

    const auto callback = [](const core::TelemetryFrame&) {};
    CHECK(backend->StartTelemetry(std::string(drone_id), 1, {}).code ==
          core::StatusCode::kRejected);
    CHECK(backend->StartTelemetry(std::string(drone_id), 0, callback).code ==
          core::StatusCode::kRejected);

    REQUIRE(backend->Start().IsOk());
    CHECK(backend->Start().IsOk());
    REQUIRE(backend->StartTelemetry(std::string(drone_id), 5, callback).IsOk());
    CHECK(backend->StartTelemetry(std::string(drone_id), 5, callback).code ==
          core::StatusCode::kRejected);
    REQUIRE(backend->StopTelemetry(std::string(drone_id)).IsOk());
    CHECK(backend->StopTelemetry(std::string(drone_id)).IsOk());

    REQUIRE(backend->StartTelemetry(std::string(drone_id), 5, callback).IsOk());
    CHECK(backend->StopTelemetry(std::string(drone_id)).IsOk());

    const BackendHealth health = backend->GetHealth();
    CHECK_FALSE(health.backend_name.empty());
    CHECK_FALSE(health.message.empty());
}

TEST_CASE("Simulator and MAVLink backends satisfy the same lifecycle contract",
          "[agent][backend][contract][sim][mavlink]") {
    SECTION("simulator") {
        SimBackendConfig config;
        config.clock_mode = SimulationClockMode::kManual;
        auto simulator = MakeSimBackend(config);
        REQUIRE(simulator.has_value());
        VerifyBackendLifecycleContract(std::move(simulator->backend), "simulator", "drone-1");
    }

    SECTION("MAVLink") {
        const auto bind_address = AvailableLoopbackUdpAddress();
        REQUIRE(bind_address.has_value());
        MavlinkBackendConfig config;
        config.bind_addr = *bind_address;
        VerifyBackendLifecycleContract(MakeMavlinkBackend(config), "MAVLink", config.drone_id);
    }
}

TEST_CASE("Simulator and MAVLink stop waits for in-flight callbacks",
          "[agent][backend][contract][sim][mavlink][shutdown]") {
    SECTION("simulator") {
        SimBackendConfig config;
        config.clock_mode = SimulationClockMode::kManual;
        auto simulator = MakeSimBackend(config);
        REQUIRE(simulator.has_value());
        VerifyCallbackQuiescenceContract(simulator->backend.get(), "simulator", "drone-1", [&] {
            static_cast<void>(simulator->control->Advance("drone-1", 1s));
        });
    }

    SECTION("MAVLink UDP ingress") {
        const auto bind_address = AvailableLoopbackUdpAddress();
        REQUIRE(bind_address.has_value());
        MavlinkBackendConfig config;
        config.bind_addr = *bind_address;
        DroneBackendPtr backend = MakeMavlinkBackend(config);
        VerifyCallbackQuiescenceContract(backend.get(), "MAVLink", config.drone_id, [&] {
            static_cast<void>(SendMavlinkGlobalPosition(*bind_address));
        });
    }
}

}  // namespace
}  // namespace swarmkit::agent
