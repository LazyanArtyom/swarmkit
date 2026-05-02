// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "app.h"

#include <cstdlib>

#include "common/arg_utils.h"
#include "options.h"
#include "runtime.h"
#include "swarmkit/client/client.h"
#include "swarmkit/core/logger.h"
#include "usage.h"

namespace swarmkit::apps::cli {

int RunCliApp(int argc, char** argv) {
    if (common::HasFlag(argc, argv, "--help") || common::HasFlag(argc, argv, "-h")) {
        internal::PrintUsage();
        return EXIT_SUCCESS;
    }

    const auto kInvocation = internal::ParseInvocation(argc, argv);
    if (!kInvocation.has_value()) {
        return kInvocation.error();
    }

    const auto kLoggerConfig = internal::BuildCliLoggerConfig(argc, argv);
    if (!kLoggerConfig.has_value()) {
        return kLoggerConfig.error();
    }
    swarmkit::core::Logger::Init(*kLoggerConfig);

    const auto kClientConfig = internal::BuildCliClientConfig(*kInvocation, argc, argv);
    if (!kClientConfig.has_value()) {
        return kClientConfig.error();
    }

    client::Client client(*kClientConfig);
    return internal::DispatchCommand(*kInvocation, *kClientConfig, client, argc, argv);
}

}  // namespace swarmkit::apps::cli
