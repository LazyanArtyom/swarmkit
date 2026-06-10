// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "runtime_reports.h"

#include "runtime_common.h"

namespace swarmkit::apps::cli::internal {

int RunReports(Client& client, std::string_view drone_id, int argc, char** argv) {
    const std::string format = common::GetOptionValue(argc, argv, "--format", "text");
    if (format != "text" && format != "jsonl") {
        std::cerr << "--format must be text or jsonl\n";
        return EXIT_FAILURE;
    }
    std::unique_ptr<std::ofstream> report_file;
    std::ostream* out = &std::cout;
    if (const std::string path = common::GetOptionValue(argc, argv, "--report-file");
        !path.empty()) {
        report_file = std::make_unique<std::ofstream>(path, std::ios::app);
        if (!report_file->is_open()) {
            std::cerr << "failed to open report file: " << path << "\n";
            return EXIT_FAILURE;
        }
        out = report_file.get();
    }

    const auto after_sequence = ParseIntArg(
        common::GetOptionValue(argc, argv, "--after-sequence", kDefaultZero), "--after-sequence");
    if (!after_sequence.has_value()) {
        std::cerr << after_sequence.error() << "\n";
        return EXIT_FAILURE;
    }
    if (*after_sequence < 0) {
        std::cerr << "--after-sequence must be >= 0\n";
        return EXIT_FAILURE;
    }

    ResetStopRequested();
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    swarmkit::client::ReportSubscription subscription;
    subscription.drone_id = std::string(drone_id);
    subscription.after_sequence = static_cast<std::uint64_t>(*after_sequence);
    auto report_stream = client.StartReports(
        subscription,
        [out, &format](const swarmkit::client::AgentReport& report) {
            if (format == "jsonl") {
                PrintReportJsonl(report, *out);
            } else {
                PrintReportText(report, *out);
            }
            out->flush();
        },
        [](const std::string& error_msg) {
            std::cerr << "Report stream error: " << error_msg << "\n";
        });
    if (!report_stream.has_value()) {
        std::cerr << "Failed to start report stream: " << report_stream.error().user_message
                  << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "Subscribed to reports: drone=" << drone_id << " format=" << format << "\n"
              << "Press Ctrl+C to stop.\n";
    WaitForStop(ParseDurationMs(argc, argv).value_or(0));
    report_stream->Stop();
    std::cout << "\nStopped.\n";
    return EXIT_SUCCESS;
}

}  // namespace swarmkit::apps::cli::internal
