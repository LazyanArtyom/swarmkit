// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "output.h"

#include <ostream>

namespace swarmkit::apps::cli::internal {

std::string JsonEscape(std::string_view value) {
    std::string out;
    out.reserve(value.size());
    for (const char character : value) {
        switch (character) {
            case '"':
                out += "\\\"";
                break;
            case '\\':
                out += "\\\\";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                out += character;
                break;
        }
    }
    return out;
}

std::string_view GoalStatusName(swarmkit::client::GoalStatus status) {
    using swarmkit::client::GoalStatus;
    switch (status) {
        case GoalStatus::kActive:
            return "active";
        case GoalStatus::kReached:
            return "reached";
        case GoalStatus::kDeviating:
            return "deviating";
        case GoalStatus::kTimeout:
            return "timeout";
        case GoalStatus::kCancelled:
            return "cancelled";
        case GoalStatus::kSuperseded:
            return "superseded";
        case GoalStatus::kFailed:
            return "failed";
        case GoalStatus::kUnspecified:
            return "unspecified";
    }
    return "unspecified";
}

std::string_view ReportTypeName(swarmkit::client::AgentReportType type) {
    using swarmkit::client::AgentReportType;
    switch (type) {
        case AgentReportType::kCommandAccepted:
            return "command_accepted";
        case AgentReportType::kCommandRejected:
            return "command_rejected";
        case AgentReportType::kCommandAcked:
            return "command_acked";
        case AgentReportType::kCommandFailed:
            return "command_failed";
        case AgentReportType::kGoalReport:
            return "goal_report";
        case AgentReportType::kTelemetryStale:
            return "telemetry_stale";
        case AgentReportType::kHeartbeatLost:
            return "heartbeat_lost";
        case AgentReportType::kHealthChanged:
            return "health_changed";
        case AgentReportType::kAuthorityLocked:
            return "authority_locked";
        case AgentReportType::kAuthorityRejected:
            return "authority_rejected";
        case AgentReportType::kAuthorityReleased:
            return "authority_released";
        case AgentReportType::kUnspecified:
            return "unspecified";
    }
    return "unspecified";
}

std::string_view SeverityName(swarmkit::client::ReportSeverity severity) {
    using swarmkit::client::ReportSeverity;
    switch (severity) {
        case ReportSeverity::kInfo:
            return "info";
        case ReportSeverity::kWarning:
            return "warning";
        case ReportSeverity::kError:
            return "error";
        case ReportSeverity::kCritical:
            return "critical";
    }
    return "info";
}

void PrintReportText(const swarmkit::client::AgentReport& report, std::ostream& out) {
    out << "[" << report.sequence << "] drone=" << report.drone_id
        << " type=" << ReportTypeName(report.type) << " severity=" << SeverityName(report.severity);
    if (report.goal.has_value()) {
        const auto& goal = *report.goal;
        out << " goal=" << goal.goal_id << " rev=" << goal.revision
            << " status=" << GoalStatusName(goal.status)
            << " dist=" << goal.distance_to_goal_m << "m dev=" << goal.deviation_m
            << "m alt_err=" << goal.altitude_error_m << "m";
    }
    if (!report.message.empty()) {
        out << " msg=" << report.message;
    }
    out << "\n";
}

void PrintReportJsonl(const swarmkit::client::AgentReport& report, std::ostream& out) {
    out << R"({"sequence":)" << report.sequence << R"(,"unix_time_ms":)" << report.unix_time_ms
        << R"(,"drone_id":")" << JsonEscape(report.drone_id) << R"(","type":")"
        << ReportTypeName(report.type) << R"(","severity":")" << SeverityName(report.severity)
        << R"(","correlation_id":")" << JsonEscape(report.correlation_id)
        << R"(","message":")" << JsonEscape(report.message) << "\"";
    if (report.goal.has_value()) {
        const auto& goal = *report.goal;
        out << R"(,"goal":{"goal_id":")" << JsonEscape(goal.goal_id) << R"(","revision":)"
            << goal.revision << R"(,"status":")" << GoalStatusName(goal.status)
            << R"(","distance_to_goal_m":)" << goal.distance_to_goal_m
            << ",\"deviation_m\":" << goal.deviation_m
            << ",\"altitude_error_m\":" << goal.altitude_error_m
            << ",\"acceptance_radius_m\":" << goal.acceptance_radius_m
            << ",\"deviation_radius_m\":" << goal.deviation_radius_m
            << ",\"elapsed_ms\":" << goal.elapsed_ms << ",\"timeout_ms\":" << goal.timeout_ms
            << R"(,"message":")" << JsonEscape(goal.message) << "\"}";
    }
    out << "}\n";
}

}  // namespace swarmkit::apps::cli::internal
