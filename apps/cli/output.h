// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <iosfwd>
#include <string>
#include <string_view>

#include "swarmkit/client/client.h"

namespace swarmkit::apps::cli::internal {

[[nodiscard]] std::string JsonEscape(std::string_view value);
[[nodiscard]] std::string_view GoalStatusName(swarmkit::client::GoalStatus status);
[[nodiscard]] std::string_view ReportTypeName(swarmkit::client::AgentReportType type);
[[nodiscard]] std::string_view SeverityName(swarmkit::client::ReportSeverity severity);

void PrintReportText(const swarmkit::client::AgentReport& report, std::ostream& out);
void PrintReportJsonl(const swarmkit::client::AgentReport& report, std::ostream& out);

}  // namespace swarmkit::apps::cli::internal
