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
        case AgentReportType::kEdgeReport:
            return "edge_report";
        case AgentReportType::kUnspecified:
            return "unspecified";
    }
    return "unspecified";
}

std::string_view EdgePhaseName(swarmkit::edge::Phase phase) {
    using swarmkit::edge::Phase;
    switch (phase) {
        case Phase::kMoving: return "moving";
        case Phase::kDeviating: return "deviating";
        case Phase::kRecovering: return "recovering";
        case Phase::kFulfilled: return "fulfilled";
        case Phase::kInterrupted: return "interrupted";
        case Phase::kUnknown: return "unknown";
    }
    return "unknown";
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
    if (report.edge) {
        const auto& edge = *report.edge;
        out << " edge=" << edge.edge_id << " attempt=" << edge.attempt_id
            << " phase=" << EdgePhaseName(edge.status.phase)
            << " at_b=" << edge.status.at_b_verified
            << " survey=" << edge.status.survey_verified
            << " covered_m=" << edge.status.survey_covered_m
            << " missing_intervals=" << edge.status.survey_missing.size();
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
    if (report.edge) {
        const auto& edge = *report.edge;
        out << R"(,"edge":{"mission_id":")" << JsonEscape(edge.mission_id)
            << R"(","mission_revision":)" << edge.mission_revision
            << R"(,"edge_id":")" << JsonEscape(edge.edge_id)
            << R"(","attempt_id":")" << JsonEscape(edge.attempt_id)
            << R"(","phase":")" << EdgePhaseName(edge.status.phase)
            << R"(","at_b_verified":)" << (edge.status.at_b_verified ? "true" : "false")
            << R"(,"traversal_verified":)" << (edge.status.traversal_verified ? "true" : "false")
            << R"(,"survey_verified":)" << (edge.status.survey_verified ? "true" : "false")
            << R"(,"survey_covered_m":)" << edge.status.survey_covered_m
            << R"(,"survey_prefix_m":)" << edge.status.survey_prefix_m
            << R"(,"survey_missing_m":[)";
        for (std::size_t i = 0; i < edge.status.survey_missing.size(); ++i) {
            if (i > 0) out << ',';
            out << '[' << edge.status.survey_missing[i].begin_m << ','
                << edge.status.survey_missing[i].end_m << ']';
        }
        out << "]}";
    }
    out << "}\n";
}

}  // namespace swarmkit::apps::cli::internal
