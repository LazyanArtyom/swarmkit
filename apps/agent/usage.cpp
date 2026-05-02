// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "usage.h"

#include <iostream>

namespace swarmkit::apps::agent::internal {

void PrintUsage() {
    std::cout << "Usage: swarmkit-agent [OPTIONS]\n"
                 "\n"
                 "Options:\n"
                 "  --id        AGENT_ID      Agent identifier (default: agent-1)\n"
                 "  --bind      HOST:PORT     Bind address    (default: 0.0.0.0:50061)\n"
                 "  --config    PATH          Load agent config from YAML file\n"
                 "  --backend   sim|mavlink   Vehicle backend (default: sim)\n"
                 "  --mavlink-bind HOST:PORT  UDP MAVLink listen address\n"
                 "  --mavlink-drone ID        SwarmKit drone id for MAVLink backend\n"
                 "  --mavlink-target-system N MAVLink target system id\n"
                 "  --mavlink-target-component N\n"
                 "                             MAVLink target component id\n"
                 "  --ca-cert   PATH          mTLS CA certificate path\n"
                 "  --server-cert PATH        mTLS server certificate path\n"
                 "  --server-key PATH         mTLS server private key path\n"
                 "  --log-sink  stdout|file|both\n"
                 "                             Logger sink type (default: stdout)\n"
                 "  --log-file  PATH          Rotating log file path when file logging is used\n"
                 "  --log-level LEVEL         trace|debug|info|warn|error|critical|off\n"
                 "  --help                    Print this message\n";
}

}  // namespace swarmkit::apps::agent::internal
