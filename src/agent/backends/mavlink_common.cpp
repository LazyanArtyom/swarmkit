// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "mavlink_common.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <exception>
#include <unordered_map>

namespace swarmkit::agent::mavlink {

namespace {

constexpr int kPx4MainModeManual = 1;
constexpr int kPx4MainModeAltctl = 2;
constexpr int kPx4MainModePosctl = 3;
constexpr int kPx4MainModeAuto = 4;
constexpr int kPx4MainModeOffboard = 6;
constexpr int kPx4SubModeAutoTakeoff = 2;
constexpr int kPx4SubModeAutoLoiter = 3;
constexpr int kPx4SubModeAutoMission = 4;
constexpr int kPx4SubModeAutoRtl = 5;
constexpr int kPx4SubModeAutoLand = 6;

[[nodiscard]] std::string CustomModeFallback(std::string_view backend, std::uint32_t custom_mode) {
    return std::string(backend) + "(custom=" + std::to_string(custom_mode) + ")";
}

[[nodiscard]] std::optional<std::string_view> ArduCopterModeName(std::uint32_t custom_mode) {
    static const std::unordered_map<std::uint32_t, std::string_view> kModeMap{
        {0, "STABILIZE"}, {1, "ACRO"},          {2, "ALT_HOLD"},     {3, "AUTO"},
        {4, "GUIDED"},    {5, "LOITER"},        {6, "RTL"},          {7, "CIRCLE"},
        {9, "LAND"},      {11, "DRIFT"},        {13, "SPORT"},       {14, "FLIP"},
        {15, "AUTOTUNE"}, {16, "POSHOLD"},      {17, "BRAKE"},       {18, "THROW"},
        {19, "AVOID_ADSB"},
        {20, "GUIDED_NOGPS"},
        {21, "SMART_RTL"},
        {22, "FLOWHOLD"},
        {23, "FOLLOW"},
        {24, "ZIGZAG"},
        {25, "SYSTEMID"},
        {26, "AUTOROTATE"},
        {27, "AUTO_RTL"},
        {28, "TURTLE"},
    };
    const auto iter = kModeMap.find(custom_mode);
    if (iter == kModeMap.end()) {
        return std::nullopt;
    }
    return iter->second;
}

[[nodiscard]] std::optional<std::string_view> ArduPlaneModeName(std::uint32_t custom_mode) {
    static const std::unordered_map<std::uint32_t, std::string_view> kModeMap{
        {0, "MANUAL"},      {1, "CIRCLE"},     {2, "STABILIZE"}, {3, "TRAINING"},
        {4, "ACRO"},        {5, "FBWA"},       {6, "FBWB"},      {7, "CRUISE"},
        {8, "AUTOTUNE"},    {10, "AUTO"},      {11, "RTL"},      {12, "LOITER"},
        {13, "TAKEOFF"},    {14, "AVOID_ADSB"},
        {15, "GUIDED"},     {16, "INITIALISING"},
        {17, "QSTABILIZE"}, {18, "QHOVER"},    {19, "QLOITER"},  {20, "QLAND"},
        {21, "QRTL"},       {22, "QAUTOTUNE"}, {23, "QACRO"},    {24, "THERMAL"},
        {25, "LOITERALTQLAND"},
    };
    const auto iter = kModeMap.find(custom_mode);
    if (iter == kModeMap.end()) {
        return std::nullopt;
    }
    return iter->second;
}

[[nodiscard]] std::string Px4ModeName(std::uint32_t custom_mode) {
    const int main_mode = static_cast<int>((custom_mode >> 16U) & 0xFFU);
    const int sub_mode = static_cast<int>((custom_mode >> 24U) & 0xFFU);

    switch (main_mode) {
        case kPx4MainModeManual:
            return "MANUAL";
        case kPx4MainModeAltctl:
            return "ALTCTL";
        case kPx4MainModePosctl:
            return "POSCTL";
        case kPx4MainModeOffboard:
            return "OFFBOARD";
        case kPx4MainModeAuto:
            switch (sub_mode) {
                case kPx4SubModeAutoTakeoff:
                    return "TAKEOFF";
                case kPx4SubModeAutoLoiter:
                    return "LOITER";
                case kPx4SubModeAutoMission:
                    return "MISSION";
                case kPx4SubModeAutoRtl:
                    return "RTL";
                case kPx4SubModeAutoLand:
                    return "LAND";
                default:
                    return "AUTO(sub=" + std::to_string(sub_mode) + ")";
            }
        default:
            return "PX4(main=" + std::to_string(main_mode) + ",sub=" + std::to_string(sub_mode) +
                   ",custom=" + std::to_string(custom_mode) + ")";
    }
}

}  // namespace

std::int64_t NowUnixMs() {
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using std::chrono::system_clock;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

std::string ToLower(std::string value) {
    std::ranges::transform(value, value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

std::optional<std::pair<std::string, std::uint16_t>> SplitHostPort(const std::string& value) {
    const std::size_t colon = value.rfind(':');
    if (colon == std::string::npos || colon == 0 || colon + 1 >= value.size()) {
        return std::nullopt;
    }

    const std::string host = value.substr(0, colon);
    const std::string port_text = value.substr(colon + 1);
    try {
        const int port = std::stoi(port_text);
        if (port <= 0 || port > kMaxUdpPort) {
            return std::nullopt;
        }
        return std::make_pair(host, static_cast<std::uint16_t>(port));
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

std::string ModeString(const mavlink_heartbeat_t& heartbeat, MavlinkAutopilotProfile profile) {
    switch (profile) {
        case MavlinkAutopilotProfile::kArdupilotCopter:
            if (const auto mode = ArduCopterModeName(heartbeat.custom_mode); mode.has_value()) {
                return std::string(*mode);
            }
            return CustomModeFallback("ARDUPILOT_COPTER", heartbeat.custom_mode);
        case MavlinkAutopilotProfile::kArdupilotPlane:
            if (const auto mode = ArduPlaneModeName(heartbeat.custom_mode); mode.has_value()) {
                return std::string(*mode);
            }
            return CustomModeFallback("ARDUPILOT_PLANE", heartbeat.custom_mode);
        case MavlinkAutopilotProfile::kPx4:
            return Px4ModeName(heartbeat.custom_mode);
    }
    return CustomModeFallback("MAVLINK", heartbeat.custom_mode);
}

std::optional<int> ArduCopterModeFromName(std::string mode) {
    static const std::unordered_map<std::string, int> kModeMap{
        {"stabilize", 0}, {"acro", 1},          {"alt-hold", 2},   {"althold", 2},
        {"auto", 3},      {"guided", 4},        {"loiter", 5},     {"rtl", 6},
        {"return", 6},    {"circle", 7},        {"land", 9},       {"drift", 11},
        {"sport", 13},    {"flip", 14},         {"auto-tune", 15}, {"autotune", 15},
        {"poshold", 16},  {"pos-hold", 16},     {"brake", 17},     {"throw", 18},
        {"avoid", 19},    {"guided-nogps", 20}, {"smart-rtl", 21}, {"smartrtl", 21},
    };
    const auto iter = kModeMap.find(ToLower(std::move(mode)));
    if (iter == kModeMap.end()) {
        return std::nullopt;
    }
    return iter->second;
}

std::optional<int> ArduPlaneModeFromName(std::string mode) {
    static const std::unordered_map<std::string, int> kModeMap{
        {"manual", 0},      {"circle", 1},  {"stabilize", 2}, {"training", 3}, {"acro", 4},
        {"fbwa", 5},        {"fbwb", 6},    {"cruise", 7},    {"autotune", 8}, {"auto", 10},
        {"rtl", 11},        {"loiter", 12}, {"takeoff", 13},  {"avoid", 15},   {"guided", 15},
        {"qstabilize", 17}, {"qhover", 18}, {"qloiter", 19},  {"qland", 20},   {"qrtl", 21},
    };
    const auto iter = kModeMap.find(ToLower(std::move(mode)));
    if (iter == kModeMap.end()) {
        return std::nullopt;
    }
    return iter->second;
}

std::optional<Px4Mode> Px4ModeFromName(std::string mode) {
    static const std::unordered_map<std::string, Px4Mode> kModeMap{
        {"manual", {.main_mode = kPx4MainModeManual}},
        {"altctl", {.main_mode = kPx4MainModeAltctl}},
        {"posctl", {.main_mode = kPx4MainModePosctl}},
        {"position", {.main_mode = kPx4MainModePosctl}},
        {"auto", {.main_mode = kPx4MainModeAuto, .sub_mode = kPx4SubModeAutoMission}},
        {"mission", {.main_mode = kPx4MainModeAuto, .sub_mode = kPx4SubModeAutoMission}},
        {"rtl", {.main_mode = kPx4MainModeAuto, .sub_mode = kPx4SubModeAutoRtl}},
        {"return", {.main_mode = kPx4MainModeAuto, .sub_mode = kPx4SubModeAutoRtl}},
        {"land", {.main_mode = kPx4MainModeAuto, .sub_mode = kPx4SubModeAutoLand}},
        {"loiter", {.main_mode = kPx4MainModeAuto, .sub_mode = kPx4SubModeAutoLoiter}},
        {"hold", {.main_mode = kPx4MainModeAuto, .sub_mode = kPx4SubModeAutoLoiter}},
        {"takeoff", {.main_mode = kPx4MainModeAuto, .sub_mode = kPx4SubModeAutoTakeoff}},
        {"offboard", {.main_mode = kPx4MainModeOffboard}},
    };
    const auto iter = kModeMap.find(ToLower(std::move(mode)));
    if (iter == kModeMap.end()) {
        return std::nullopt;
    }
    return iter->second;
}

std::vector<std::string> SupportedModes(MavlinkAutopilotProfile profile) {
    switch (profile) {
        case MavlinkAutopilotProfile::kArdupilotCopter:
            return {"stabilize", "alt-hold", "auto", "guided", "loiter", "rtl",
                    "land",      "brake",    "smart-rtl"};
        case MavlinkAutopilotProfile::kArdupilotPlane:
            return {"manual", "circle", "stabilize", "fbwa", "fbwb", "cruise",
                    "auto",   "rtl",    "loiter",    "takeoff", "guided"};
        case MavlinkAutopilotProfile::kPx4:
            return {"manual", "altctl", "posctl", "auto", "mission", "rtl",
                    "land",   "loiter", "takeoff", "offboard"};
    }
    return {};
}

std::string MavResultName(std::uint8_t result) {
    switch (result) {
        case MAV_RESULT_ACCEPTED:
            return "ACCEPTED";
        case MAV_RESULT_TEMPORARILY_REJECTED:
            return "TEMPORARILY_REJECTED";
        case MAV_RESULT_DENIED:
            return "DENIED";
        case MAV_RESULT_UNSUPPORTED:
            return "UNSUPPORTED";
        case MAV_RESULT_FAILED:
            return "FAILED";
        case MAV_RESULT_IN_PROGRESS:
            return "IN_PROGRESS";
        case MAV_RESULT_CANCELLED:
            return "CANCELLED";
        case MAV_RESULT_COMMAND_LONG_ONLY:
            return "COMMAND_LONG_ONLY";
        case MAV_RESULT_COMMAND_INT_ONLY:
            return "COMMAND_INT_ONLY";
        case MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME:
            return "UNSUPPORTED_MAV_FRAME";
        case MAV_RESULT_NOT_IN_CONTROL:
            return "NOT_IN_CONTROL";
        default:
            return "UNKNOWN(" + std::to_string(result) + ")";
    }
}

std::string MissionResultName(std::uint8_t result) {
    switch (result) {
        case MAV_MISSION_ACCEPTED:
            return "ACCEPTED";
        case MAV_MISSION_ERROR:
            return "ERROR";
        case MAV_MISSION_UNSUPPORTED_FRAME:
            return "UNSUPPORTED_FRAME";
        case MAV_MISSION_UNSUPPORTED:
            return "UNSUPPORTED";
        case MAV_MISSION_NO_SPACE:
            return "NO_SPACE";
        case MAV_MISSION_INVALID:
            return "INVALID";
        case MAV_MISSION_INVALID_PARAM1:
            return "INVALID_PARAM1";
        case MAV_MISSION_INVALID_PARAM2:
            return "INVALID_PARAM2";
        case MAV_MISSION_INVALID_PARAM3:
            return "INVALID_PARAM3";
        case MAV_MISSION_INVALID_PARAM4:
            return "INVALID_PARAM4";
        case MAV_MISSION_INVALID_PARAM5_X:
            return "INVALID_PARAM5_X";
        case MAV_MISSION_INVALID_PARAM6_Y:
            return "INVALID_PARAM6_Y";
        case MAV_MISSION_INVALID_PARAM7:
            return "INVALID_PARAM7";
        case MAV_MISSION_INVALID_SEQUENCE:
            return "INVALID_SEQUENCE";
        case MAV_MISSION_DENIED:
            return "DENIED";
        default:
            return "UNKNOWN(" + std::to_string(result) + ")";
    }
}

bool MavlinkCommandAckResult::IsUnsupported() const {
    return has_ack &&
           (ack.result == MAV_RESULT_UNSUPPORTED || ack.result == MAV_RESULT_COMMAND_INT_ONLY ||
            ack.result == MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME);
}

core::Result MavlinkCommandAckResult::ToCoreResult() const {
    if (!send_result.IsOk()) {
        return send_result;
    }
    if (!has_ack) {
        return core::Result::Success();
    }

    std::string detail = "COMMAND_ACK command=" + std::to_string(ack.command) +
                         " result=" + MavResultName(ack.result) +
                         " param2=" + std::to_string(ack.result_param2) +
                         " target=" + std::to_string(ack.target_system) + "/" +
                         std::to_string(ack.target_component);
    if (has_status_text && !status_text.text.empty()) {
        detail += " reason=\"" + status_text.text + "\"";
    }
    switch (ack.result) {
        case MAV_RESULT_ACCEPTED:
            return core::Result::Success(detail);
        case MAV_RESULT_TEMPORARILY_REJECTED:
        case MAV_RESULT_DENIED:
        case MAV_RESULT_UNSUPPORTED:
        case MAV_RESULT_CANCELLED:
        case MAV_RESULT_COMMAND_LONG_ONLY:
        case MAV_RESULT_COMMAND_INT_ONLY:
        case MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME:
        case MAV_RESULT_NOT_IN_CONTROL:
            return core::Result::Rejected(detail);
        case MAV_RESULT_FAILED:
        default:
            return core::Result::Failed(detail);
    }
}

}  // namespace swarmkit::agent::mavlink
