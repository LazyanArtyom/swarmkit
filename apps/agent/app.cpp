// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "app.h"

#include <cstdlib>
#include <utility>

#include "common/arg_utils.h"
#include "configuration.h"
#include "swarmkit/agent/backend_factory.h"
#include "swarmkit/agent/server.h"
#include "swarmkit/core/logger.h"
#include "usage.h"

namespace swarmkit::apps::agent {

int RunAgentApp(int argc, char** argv) {
    if (common::HasFlag(argc, argv, "--help")) {
        internal::PrintUsage();
        return EXIT_SUCCESS;
    }

    const auto kLoggerConfig = internal::BuildAgentLoggerConfig(argc, argv);
    if (!kLoggerConfig.has_value()) {
        return kLoggerConfig.error();
    }

    swarmkit::core::Logger::Init(*kLoggerConfig);
    swarmkit::core::Logger::InfoFmt("Logger initialized: sink={} level={} file={}",
                                    swarmkit::core::ToString(kLoggerConfig->sink_type),
                                    swarmkit::core::ToString(kLoggerConfig->level),
                                    kLoggerConfig->log_file_path);

    const auto kAgentConfig = internal::BuildAgentConfig(argc, argv);
    if (!kAgentConfig.has_value()) {
        return kAgentConfig.error();
    }

    const auto kBackendSelection = internal::BuildBackendSelection(argc, argv);
    if (!kBackendSelection.has_value()) {
        return kBackendSelection.error();
    }

    swarmkit::agent::BackendRegistry registry;
    swarmkit::agent::RegisterBuiltinBackends(&registry);
    auto backend_result = registry.Create(kBackendSelection->request);
    if (!backend_result.has_value()) {
        swarmkit::core::Logger::ErrorFmt("Failed to create backend '{}': {}",
                                         kBackendSelection->request.backend_name,
                                         backend_result.error().message);
        return EXIT_FAILURE;
    }

    return swarmkit::agent::RunAgentServer(*kAgentConfig, std::move(*backend_result));
}

}  // namespace swarmkit::apps::agent
