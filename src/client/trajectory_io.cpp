// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "swarmkit/client/trajectory_io.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

namespace swarmkit::client {
namespace {

[[nodiscard]] std::string Trim(std::string_view value) {
    const auto* begin = value.begin();
    while (begin != value.end() && std::isspace(static_cast<unsigned char>(*begin)) != 0) {
        ++begin;
    }
    const auto* end = value.end();
    while (end != begin && std::isspace(static_cast<unsigned char>(*(end - 1))) != 0) {
        --end;
    }
    return std::string(begin, end);
}

[[nodiscard]] std::string Lower(std::string value) {
    std::ranges::transform(value, value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

[[nodiscard]] bool HasAny(const YAML::Node& node, std::initializer_list<std::string_view> keys) {
    return std::ranges::any_of(keys, [&node](std::string_view key) {
        const YAML::Node value = node[std::string(key)];
        return value && value.IsDefined() && !value.IsNull();
    });
}

template <typename T>
void AssignIfPresent(const YAML::Node& node, std::initializer_list<std::string_view> keys, T* out) {
    for (std::string_view key : keys) {
        if (YAML::Node value = node[std::string(key)]) {
            if (!value.IsDefined() || value.IsNull()) {
                continue;
            }
            try {
                *out = value.as<T>();
            } catch (const YAML::Exception&) {
                throw YAML::RepresentationException(
                    value.Mark(), "invalid trajectory scalar '" + std::string(key) + "'");
            }
            return;
        }
    }
}

[[nodiscard]] std::optional<Geofence> ParseGeofenceNode(const YAML::Node& node) {
    if (!node || !node.IsMap()) {
        return std::nullopt;
    }
    Geofence fence;
    AssignIfPresent(node, {"min_lat_deg", "min_lat"}, &fence.min_lat_deg);
    AssignIfPresent(node, {"max_lat_deg", "max_lat"}, &fence.max_lat_deg);
    AssignIfPresent(node, {"min_lon_deg", "min_lon"}, &fence.min_lon_deg);
    AssignIfPresent(node, {"max_lon_deg", "max_lon"}, &fence.max_lon_deg);
    AssignIfPresent(node, {"min_alt_m", "min_alt", "floor_m"}, &fence.min_alt_m);
    AssignIfPresent(node, {"max_alt_m", "max_alt", "ceiling_m"}, &fence.max_alt_m);
    return fence;
}

void ApplyValidationNode(const YAML::Node& node, TrajectoryValidationPolicy* validation) {
    if (!node || !node.IsMap()) {
        return;
    }
    AssignIfPresent(node, {"min_battery_percent", "battery_min_percent"},
                    &validation->min_battery_percent);
    AssignIfPresent(node, {"min_spacing_m", "spacing_m"}, &validation->min_spacing_m);
    AssignIfPresent(node, {"require_gps"}, &validation->require_gps);
    AssignIfPresent(node, {"require_ekf_ok", "require_ekf"}, &validation->require_ekf_ok);
    AssignIfPresent(node, {"max_horizontal_speed_mps", "max_speed_mps"},
                    &validation->max_horizontal_speed_mps);
    AssignIfPresent(node, {"max_climb_speed_mps", "max_climb_mps"},
                    &validation->max_climb_speed_mps);
    AssignIfPresent(node, {"max_descent_speed_mps", "max_descent_mps"},
                    &validation->max_descent_speed_mps);
    AssignIfPresent(node, {"max_altitude_m", "max_alt_m"}, &validation->max_altitude_m);
    AssignIfPresent(node, {"tracking_tolerance_m", "tolerance_m"},
                    &validation->tracking_tolerance_m);
    if (const auto fence = ParseGeofenceNode(node["geofence"]); fence.has_value()) {
        validation->geofence = *fence;
    }
}

[[nodiscard]] TrajectoryPoint ParseTrajectoryPointNode(const YAML::Node& node) {
    TrajectoryPoint point;
    AssignIfPresent(node, {"time_offset_ms", "t_ms", "time_ms", "t"}, &point.time_offset_ms);
    AssignIfPresent(node, {"unix_time_ms", "unix_ms"}, &point.unix_time_ms);
    AssignIfPresent(node, {"use_local_position", "local"}, &point.use_local_position);

    point.has_position =
        HasAny(node, {"lat", "latitude", "lat_deg", "lon", "longitude", "lon_deg", "alt", "alt_m"});
    if (point.has_position) {
        AssignIfPresent(node, {"lat", "latitude", "lat_deg"}, &point.position.lat_deg);
        AssignIfPresent(node, {"lon", "longitude", "lon_deg"}, &point.position.lon_deg);
        AssignIfPresent(node, {"alt", "alt_m"}, &point.position.alt_m);
    }
    point.has_local_position = HasAny(node, {"x", "x_m", "y", "y_m", "z", "z_m"});
    if (point.has_local_position) {
        AssignIfPresent(node, {"x", "x_m"}, &point.local_position.x_m);
        AssignIfPresent(node, {"y", "y_m"}, &point.local_position.y_m);
        AssignIfPresent(node, {"z", "z_m"}, &point.local_position.z_m);
        point.use_local_position = true;
    }
    point.has_velocity = HasAny(node, {"vx", "vx_mps", "vy", "vy_mps", "vz", "vz_mps"});
    AssignIfPresent(node, {"vx", "vx_mps"}, &point.vx_mps);
    AssignIfPresent(node, {"vy", "vy_mps"}, &point.vy_mps);
    AssignIfPresent(node, {"vz", "vz_mps"}, &point.vz_mps);
    point.has_yaw = HasAny(node, {"yaw", "yaw_deg"});
    AssignIfPresent(node, {"yaw", "yaw_deg"}, &point.yaw_deg);

    return point;
}

[[nodiscard]] TrajectoryPlan MakeDefaultPlan(const TrajectoryLoadOptions& options) {
    TrajectoryPlan plan;
    plan.execution_id = options.default_execution_id;
    plan.revision = options.default_revision;
    plan.drone_id = options.fallback_drone_id;
    plan.frame = options.default_frame;
    plan.validation = options.default_validation;
    plan.labels = options.default_labels;
    return plan;
}

void ApplyPlanMetadataNode(const YAML::Node& node, TrajectoryPlan* plan) {
    AssignIfPresent(node, {"execution_id", "id"}, &plan->execution_id);
    AssignIfPresent(node, {"revision"}, &plan->revision);
    AssignIfPresent(node, {"drone_id", "drone"}, &plan->drone_id);
    AssignIfPresent(node, {"frame"}, &plan->frame);
    ApplyValidationNode(node["validation"], &plan->validation);
    if (const YAML::Node labels = node["labels"]; labels && labels.IsMap()) {
        for (const auto& label : labels) {
            plan->labels[label.first.as<std::string>()] = label.second.as<std::string>();
        }
    }
}

void SkipJsonWhitespace(std::string_view text, std::size_t* pos) {
    while (*pos < text.size() && std::isspace(static_cast<unsigned char>(text[*pos])) != 0) {
        ++(*pos);
    }
}

[[nodiscard]] std::expected<std::string, std::string> ParseJsonString(std::string_view text,
                                                                      std::size_t* pos) {
    if (*pos >= text.size() || text[*pos] != '"') {
        return std::unexpected("expected JSON string");
    }
    ++(*pos);
    std::string out;
    while (*pos < text.size()) {
        const char character = text[*pos];
        ++(*pos);
        if (character == '"') {
            return out;
        }
        if (character == '\\') {
            if (*pos >= text.size()) {
                return std::unexpected("unterminated JSON escape");
            }
            const char escaped = text[*pos];
            ++(*pos);
            switch (escaped) {
                case '"':
                case '\\':
                case '/':
                    out.push_back(escaped);
                    break;
                case 'b':
                    out.push_back('\b');
                    break;
                case 'f':
                    out.push_back('\f');
                    break;
                case 'n':
                    out.push_back('\n');
                    break;
                case 'r':
                    out.push_back('\r');
                    break;
                case 't':
                    out.push_back('\t');
                    break;
                default:
                    return std::unexpected("unsupported JSON escape");
            }
            continue;
        }
        out.push_back(character);
    }
    return std::unexpected("unterminated JSON string");
}

[[nodiscard]] std::expected<std::string, std::string> ParseJsonScalarValue(std::string_view text,
                                                                           std::size_t* pos) {
    SkipJsonWhitespace(text, pos);
    if (*pos >= text.size()) {
        return std::unexpected("expected JSON value");
    }
    if (text[*pos] == '"') {
        return ParseJsonString(text, pos);
    }
    const std::size_t begin = *pos;
    while (*pos < text.size() && text[*pos] != ',' && text[*pos] != '}') {
        ++(*pos);
    }
    return Trim(text.substr(begin, *pos - begin));
}

[[nodiscard]] std::expected<std::unordered_map<std::string, std::string>, std::string>
ParseJsonFlatObject(std::string_view text) {
    std::unordered_map<std::string, std::string> record;
    std::size_t pos = 0;
    SkipJsonWhitespace(text, &pos);
    if (pos >= text.size() || text[pos] != '{') {
        return std::unexpected("expected JSON object");
    }
    ++pos;
    while (pos < text.size()) {
        SkipJsonWhitespace(text, &pos);
        if (pos < text.size() && text[pos] == '}') {
            ++pos;
            SkipJsonWhitespace(text, &pos);
            if (pos != text.size()) {
                return std::unexpected("unexpected content after JSON object");
            }
            return record;
        }
        auto key = ParseJsonString(text, &pos);
        if (!key.has_value()) {
            return std::unexpected(key.error());
        }
        SkipJsonWhitespace(text, &pos);
        if (pos >= text.size() || text[pos] != ':') {
            return std::unexpected("expected ':' after JSON key");
        }
        ++pos;
        auto value = ParseJsonScalarValue(text, &pos);
        if (!value.has_value()) {
            return std::unexpected(value.error());
        }
        record[Lower(*key)] = *value;
        SkipJsonWhitespace(text, &pos);
        if (pos < text.size() && text[pos] == ',') {
            ++pos;
            continue;
        }
        if (pos < text.size() && text[pos] == '}') {
            continue;
        }
        return std::unexpected("expected ',' or '}' after JSON value");
    }
    return std::unexpected("unterminated JSON object");
}

void ApplyCsvMetadata(const std::unordered_map<std::string, std::string>& record,
                      TrajectoryPlan* plan);

[[nodiscard]] bool CsvHasAny(const std::unordered_map<std::string, std::string>& record,
                             std::initializer_list<std::string_view> keys);

[[nodiscard]] TrajectoryPoint ParseCsvPoint(
    const std::unordered_map<std::string, std::string>& record);

[[nodiscard]] std::expected<TrajectoryPlan, std::string> LoadYamlTrajectoryPlan(
    std::istream& input, const TrajectoryLoadOptions& options) {
    try {
        const YAML::Node root = YAML::Load(input);
        const YAML::Node plan_node = root["trajectory"] ? root["trajectory"] : root;
        if (!plan_node || !plan_node.IsMap()) {
            return std::unexpected("trajectory YAML must contain a mapping");
        }

        TrajectoryPlan plan = MakeDefaultPlan(options);
        ApplyPlanMetadataNode(plan_node, &plan);
        const YAML::Node points = plan_node["points"];
        if (!points || !points.IsSequence()) {
            return std::unexpected("trajectory file must contain points: sequence");
        }
        for (const auto& node : points) {
            plan.points.push_back(ParseTrajectoryPointNode(node));
        }
        return plan;
    } catch (const YAML::Exception& exc) {
        return std::unexpected(std::string("failed to parse trajectory YAML: ") + exc.what());
    }
}

[[nodiscard]] std::expected<TrajectoryPlan, std::string> LoadJsonLinesTrajectoryPlan(
    std::istream& input, const TrajectoryLoadOptions& options) {
    TrajectoryPlan plan = MakeDefaultPlan(options);
    std::string line;
    int line_number = 0;
    try {
        while (std::getline(input, line)) {
            ++line_number;
            const std::string trimmed = Trim(line);
            if (trimmed.empty() || trimmed.starts_with('#')) {
                continue;
            }
            auto record = ParseJsonFlatObject(trimmed);
            if (!record.has_value()) {
                return std::unexpected("JSONL trajectory line " + std::to_string(line_number) +
                                       ": " + record.error());
            }
            ApplyCsvMetadata(*record, &plan);
            if (CsvHasAny(*record, {"time_offset_ms",
                                    "t_ms",
                                    "time_ms",
                                    "t",
                                    "unix_time_ms",
                                    "lat",
                                    "latitude",
                                    "lat_deg",
                                    "lon",
                                    "longitude",
                                    "lon_deg",
                                    "alt",
                                    "alt_m",
                                    "x",
                                    "x_m",
                                    "y",
                                    "y_m",
                                    "z",
                                    "z_m",
                                    "vx",
                                    "vx_mps",
                                    "vy",
                                    "vy_mps",
                                    "vz",
                                    "vz_mps",
                                    "yaw",
                                    "yaw_deg"})) {
                plan.points.push_back(ParseCsvPoint(*record));
                continue;
            }
        }
    } catch (const YAML::Exception& exc) {
        return std::unexpected("failed to parse trajectory JSONL line " +
                               std::to_string(line_number) + ": " + exc.what());
    }

    if (plan.points.empty()) {
        return std::unexpected("JSONL trajectory must contain at least one point object");
    }
    return plan;
}

[[nodiscard]] std::vector<std::string> ParseCsvRow(std::string_view line) {
    std::vector<std::string> fields;
    std::string field;
    bool in_quotes = false;
    for (std::size_t idx = 0; idx < line.size(); ++idx) {
        const char character = line[idx];
        if (character == '"') {
            if (in_quotes && idx + 1 < line.size() && line[idx + 1] == '"') {
                field.push_back('"');
                ++idx;
            } else {
                in_quotes = !in_quotes;
            }
            continue;
        }
        if (character == ',' && !in_quotes) {
            fields.push_back(Trim(field));
            field.clear();
            continue;
        }
        field.push_back(character);
    }
    fields.push_back(Trim(field));
    return fields;
}

[[nodiscard]] std::unordered_map<std::string, std::string> CsvRecord(
    const std::vector<std::string>& header, const std::vector<std::string>& row) {
    std::unordered_map<std::string, std::string> record;
    for (std::size_t idx = 0; idx < header.size() && idx < row.size(); ++idx) {
        if (!header[idx].empty() && !row[idx].empty()) {
            record[Lower(header[idx])] = row[idx];
        }
    }
    return record;
}

template <typename T>
void AssignCsv(const std::unordered_map<std::string, std::string>& record,
               std::initializer_list<std::string_view> keys, T* out) {
    for (std::string_view key : keys) {
        if (const auto iter = record.find(std::string(key)); iter != record.end()) {
            if constexpr (std::is_same_v<T, bool>) {
                const std::string value = Lower(iter->second);
                *out = value == "true" || value == "1" || value == "yes";
                return;
            }
            std::istringstream stream(iter->second);
            stream >> *out;
            return;
        }
    }
}

[[nodiscard]] bool CsvHasAny(const std::unordered_map<std::string, std::string>& record,
                             std::initializer_list<std::string_view> keys) {
    return std::ranges::any_of(
        keys, [&record](std::string_view key) { return record.contains(std::string(key)); });
}

void ApplyCsvMetadata(const std::unordered_map<std::string, std::string>& record,
                      TrajectoryPlan* plan) {
    if (const auto iter = record.find("execution_id"); iter != record.end()) {
        plan->execution_id = iter->second;
    }
    if (const auto iter = record.find("id"); iter != record.end()) {
        plan->execution_id = iter->second;
    }
    if (const auto iter = record.find("drone_id"); iter != record.end()) {
        plan->drone_id = iter->second;
    }
    if (const auto iter = record.find("drone"); iter != record.end()) {
        plan->drone_id = iter->second;
    }
    if (const auto iter = record.find("frame"); iter != record.end()) {
        plan->frame = iter->second;
    }
    AssignCsv(record, {"revision"}, &plan->revision);
    AssignCsv(record, {"min_battery_percent", "battery_min_percent"},
              &plan->validation.min_battery_percent);
    AssignCsv(record, {"min_spacing_m", "spacing_m"}, &plan->validation.min_spacing_m);
    AssignCsv(record, {"require_gps"}, &plan->validation.require_gps);
    AssignCsv(record, {"require_ekf_ok", "require_ekf"}, &plan->validation.require_ekf_ok);
    AssignCsv(record, {"max_horizontal_speed_mps", "max_speed_mps"},
              &plan->validation.max_horizontal_speed_mps);
    AssignCsv(record, {"max_climb_speed_mps", "max_climb_mps"},
              &plan->validation.max_climb_speed_mps);
    AssignCsv(record, {"max_descent_speed_mps", "max_descent_mps"},
              &plan->validation.max_descent_speed_mps);
    AssignCsv(record, {"max_altitude_m", "max_alt_m"}, &plan->validation.max_altitude_m);
    AssignCsv(record, {"tracking_tolerance_m", "tolerance_m"},
              &plan->validation.tracking_tolerance_m);
    if (CsvHasAny(record, {"min_lat_deg", "min_lat", "max_lat_deg", "max_lat", "min_lon_deg",
                           "min_lon", "max_lon_deg", "max_lon", "min_alt_m", "max_alt_m"})) {
        Geofence fence = plan->validation.geofence.value_or(Geofence{});
        AssignCsv(record, {"min_lat_deg", "min_lat"}, &fence.min_lat_deg);
        AssignCsv(record, {"max_lat_deg", "max_lat"}, &fence.max_lat_deg);
        AssignCsv(record, {"min_lon_deg", "min_lon"}, &fence.min_lon_deg);
        AssignCsv(record, {"max_lon_deg", "max_lon"}, &fence.max_lon_deg);
        AssignCsv(record, {"min_alt_m"}, &fence.min_alt_m);
        AssignCsv(record, {"max_alt_m"}, &fence.max_alt_m);
        plan->validation.geofence = fence;
    }
}

[[nodiscard]] TrajectoryPoint ParseCsvPoint(
    const std::unordered_map<std::string, std::string>& record) {
    TrajectoryPoint point;
    AssignCsv(record, {"time_offset_ms", "t_ms", "time_ms", "t"}, &point.time_offset_ms);
    AssignCsv(record, {"unix_time_ms", "unix_ms"}, &point.unix_time_ms);
    point.has_position = CsvHasAny(
        record, {"lat", "latitude", "lat_deg", "lon", "longitude", "lon_deg", "alt", "alt_m"});
    if (point.has_position) {
        AssignCsv(record, {"lat", "latitude", "lat_deg"}, &point.position.lat_deg);
        AssignCsv(record, {"lon", "longitude", "lon_deg"}, &point.position.lon_deg);
        AssignCsv(record, {"alt", "alt_m"}, &point.position.alt_m);
    }
    point.has_local_position = CsvHasAny(record, {"x", "x_m", "y", "y_m", "z", "z_m"});
    if (point.has_local_position) {
        AssignCsv(record, {"x", "x_m"}, &point.local_position.x_m);
        AssignCsv(record, {"y", "y_m"}, &point.local_position.y_m);
        AssignCsv(record, {"z", "z_m"}, &point.local_position.z_m);
        point.use_local_position = true;
    }
    AssignCsv(record, {"use_local_position", "local"}, &point.use_local_position);
    point.has_velocity = CsvHasAny(record, {"vx", "vx_mps", "vy", "vy_mps", "vz", "vz_mps"});
    AssignCsv(record, {"vx", "vx_mps"}, &point.vx_mps);
    AssignCsv(record, {"vy", "vy_mps"}, &point.vy_mps);
    AssignCsv(record, {"vz", "vz_mps"}, &point.vz_mps);
    point.has_yaw = CsvHasAny(record, {"yaw", "yaw_deg"});
    AssignCsv(record, {"yaw", "yaw_deg"}, &point.yaw_deg);
    return point;
}

[[nodiscard]] std::expected<TrajectoryPlan, std::string> LoadCsvTrajectoryPlan(
    std::istream& input, const TrajectoryLoadOptions& options) {
    TrajectoryPlan plan = MakeDefaultPlan(options);
    std::string line;
    std::vector<std::string> header;
    int line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        if (line_number == 1 && line.starts_with("\xEF\xBB\xBF")) {
            line.erase(0, 3);
        }
        const std::string trimmed = Trim(line);
        if (trimmed.empty() || trimmed.starts_with('#')) {
            continue;
        }
        if (header.empty()) {
            header = ParseCsvRow(trimmed);
            for (auto& name : header) {
                name = Lower(Trim(name));
            }
            continue;
        }
        const std::vector<std::string> row = ParseCsvRow(trimmed);
        const auto record = CsvRecord(header, row);
        ApplyCsvMetadata(record, &plan);
        plan.points.push_back(ParseCsvPoint(record));
    }
    if (header.empty()) {
        return std::unexpected("CSV trajectory requires a header row");
    }
    if (plan.points.empty()) {
        return std::unexpected("CSV trajectory must contain at least one point row");
    }
    return plan;
}

[[nodiscard]] std::string ExtensionOf(std::string_view path) {
    return Lower(std::filesystem::path(std::string(path)).extension().string());
}

}  // namespace

std::expected<TrajectoryFileFormat, std::string> ParseTrajectoryFileFormat(
    std::string_view format) {
    const std::string normalized = Lower(Trim(format));
    if (normalized.empty() || normalized == "auto") {
        return TrajectoryFileFormat::kAuto;
    }
    if (normalized == "yaml" || normalized == "yml") {
        return TrajectoryFileFormat::kYaml;
    }
    if (normalized == "jsonl" || normalized == "ndjson" || normalized == "json-lines") {
        return TrajectoryFileFormat::kJsonLines;
    }
    if (normalized == "csv") {
        return TrajectoryFileFormat::kCsv;
    }
    return std::unexpected("unknown trajectory format '" + std::string(format) +
                           "': expected auto|yaml|jsonl|csv");
}

TrajectoryFileFormat DetectTrajectoryFileFormat(std::string_view path) {
    const std::string extension = ExtensionOf(path);
    if (extension == ".yaml" || extension == ".yml") {
        return TrajectoryFileFormat::kYaml;
    }
    if (extension == ".jsonl" || extension == ".ndjson") {
        return TrajectoryFileFormat::kJsonLines;
    }
    if (extension == ".csv") {
        return TrajectoryFileFormat::kCsv;
    }
    return TrajectoryFileFormat::kYaml;
}

std::expected<TrajectoryPlan, std::string> LoadTrajectoryPlan(
    std::istream& input, TrajectoryFileFormat format, const TrajectoryLoadOptions& options) {
    switch (format) {
        case TrajectoryFileFormat::kYaml:
            return LoadYamlTrajectoryPlan(input, options);
        case TrajectoryFileFormat::kJsonLines:
            return LoadJsonLinesTrajectoryPlan(input, options);
        case TrajectoryFileFormat::kCsv:
            return LoadCsvTrajectoryPlan(input, options);
        case TrajectoryFileFormat::kAuto:
            return std::unexpected("stream trajectory loading requires an explicit format");
    }
    return std::unexpected("unsupported trajectory format");
}

std::expected<TrajectoryPlan, std::string> LoadTrajectoryPlanFile(
    std::string_view path, TrajectoryFileFormat format, const TrajectoryLoadOptions& options) {
    if (path.empty()) {
        return std::unexpected("trajectory requires a file path");
    }

    TrajectoryLoadOptions resolved_options = options;
    if (resolved_options.default_execution_id.empty()) {
        resolved_options.default_execution_id =
            std::filesystem::path(std::string(path)).stem().string();
    }

    std::ifstream input{std::string(path)};
    if (!input.is_open()) {
        return std::unexpected("failed to open trajectory file '" + std::string(path) + "'");
    }
    const TrajectoryFileFormat resolved_format =
        format == TrajectoryFileFormat::kAuto ? DetectTrajectoryFileFormat(path) : format;
    auto plan = LoadTrajectoryPlan(input, resolved_format, resolved_options);
    if (!plan.has_value()) {
        return std::unexpected("failed to load trajectory file '" + std::string(path) +
                               "': " + plan.error());
    }
    return plan;
}

}  // namespace swarmkit::client
