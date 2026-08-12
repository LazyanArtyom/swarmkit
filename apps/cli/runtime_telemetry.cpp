// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "runtime_telemetry.h"

#include "runtime_common.h"

namespace swarmkit::apps::cli::internal {

[[nodiscard]] std::expected<int, std::string> ParseTelemetryRate(int argc, char** argv) {
    try {
        return std::stoi(common::GetOptionValue(argc, argv, "--rate", kDefaultTelemetryRate));
    } catch (const std::exception& exc) {
        return std::unexpected("Invalid --rate value '" +
                               common::GetOptionValue(argc, argv, "--rate", kDefaultTelemetryRate) +
                               "': " + exc.what());
    }
}

int RunTelemetry(Client& client, std::string_view drone_id, int rate_hz, int argc, char** argv) {
    auto sink = TelemetrySink::FromArgs(argc, argv, false);
    if (!sink.has_value()) {
        std::cerr << sink.error() << "\n";
        return EXIT_FAILURE;
    }
    auto telemetry_sink = std::move(*sink);
    const auto duration_ms = ParseDurationMs(argc, argv);
    if (!duration_ms.has_value()) {
        std::cerr << duration_ms.error() << "\n";
        return EXIT_FAILURE;
    }

    ResetStopRequested();
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    std::cout << "Subscribing to telemetry: drone=" << drone_id << " rate=" << rate_hz << " Hz\n"
              << "Press Ctrl+C to stop.\n\n";

    swarmkit::client::TelemetrySubscription subscription;
    subscription.drone_id = std::string(drone_id);
    subscription.rate_hertz = rate_hz;

    if (telemetry_sink->WritesFiles()) {
        std::cout << "Telemetry CSV logging enabled.\n";
    }

    std::atomic<bool> stream_failed{false};
    std::string stream_error;
    std::mutex stream_error_mutex;
    auto telemetry_stream = client.StartTelemetry(
        subscription,
        [&telemetry_sink](const swarmkit::client::TelemetryObservation& observation) {
            if (const auto* frame =
                    std::get_if<swarmkit::client::TelemetryFrameObservation>(&observation)) {
                telemetry_sink->Write(frame->delivery);
            }
        },
        [&](const std::string& error_msg) {
            {
                std::lock_guard<std::mutex> lock(stream_error_mutex);
                stream_error = error_msg;
            }
            stream_failed.store(true, std::memory_order_relaxed);
            std::cerr << "Telemetry stream error: " << error_msg << "\n";
        });
    if (!telemetry_stream.has_value()) {
        std::cerr << "Failed to start telemetry stream: " << telemetry_stream.error().user_message
                  << "\n";
        return EXIT_FAILURE;
    }

    WaitForStop(*duration_ms);

    telemetry_stream->Stop();
    std::cout << "\nStopped.\n";
    if (stream_failed.load(std::memory_order_relaxed)) {
        std::lock_guard<std::mutex> lock(stream_error_mutex);
        std::cerr << "Telemetry FAILED: " << stream_error << "\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

}  // namespace swarmkit::apps::cli::internal
