// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include <cstdlib>
#include <iostream>
#include <string_view>

#include "swarmkit/evidence/execution_log.h"

int main(int argc, char** argv) {
    if (argc == 2 && (std::string_view{argv[1]} == "--help" ||
                      std::string_view{argv[1]} == "-h")) {
        std::cout << "usage: " << argv[0] << " EXECUTION_LOG\n";
        return EXIT_SUCCESS;
    }
    if (argc != 2) {
        std::cerr << "usage: " << argv[0] << " EXECUTION_LOG\n";
        return EXIT_FAILURE;
    }
    const auto log = swarmkit::evidence::ReadExecutionLog(argv[1]);
    if (!log.has_value()) {
        std::cerr << "invalid execution log: " << log.error().message
                  << " path=" << log.error().path << " record=" << log.error().record_index << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "session=" << log->agent_session_id << " run=" << log->metadata.run_id()
              << " scenario=" << log->metadata.scenario_id() << " events=" << log->events.size()
              << " complete=" << (log->has_clean_completion ? "true" : "false") << '\n';
    return EXIT_SUCCESS;
}
