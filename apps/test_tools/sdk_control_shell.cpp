// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <expected>
#include <filesystem>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "swarmkit/client/client.h"
#include "swarmkit/commands.h"
#include "swarmkit/core/result.h"
#include "swarmkit/core/security.h"
#include "swarmkit/core/telemetry.h"

namespace {

using swarmkit::client::ActiveGoal;
using swarmkit::client::AgentReport;
using swarmkit::client::ArtifactDescriptor;
using swarmkit::client::ArtifactListOptions;
using swarmkit::client::ArtifactTransferState;
using swarmkit::client::ArtifactUpload;
using swarmkit::client::ArtifactUploadSession;
using swarmkit::client::AuthorityEventInfo;
using swarmkit::client::BackendCapabilities;
using swarmkit::client::Client;
using swarmkit::client::ClientConfig;
using swarmkit::client::CommandResult;
using swarmkit::client::CommandWaitOptions;
using swarmkit::client::DataMessage;
using swarmkit::client::DataPeerConfig;
using swarmkit::client::DataPeerState;
using swarmkit::client::GoalResult;
using swarmkit::client::GoalStatus;
using swarmkit::client::HealthStatus;
using swarmkit::client::ReleaseAuthorityResult;
using swarmkit::client::ReportSeverity;
using swarmkit::client::RuntimeStats;
using swarmkit::commands::BackendCmd;
using swarmkit::commands::CmdArm;
using swarmkit::commands::CmdBackendCommand;
using swarmkit::commands::CmdDisarm;
using swarmkit::commands::CmdFlightTerminate;
using swarmkit::commands::CmdForceArm;
using swarmkit::commands::CmdForceDisarm;
using swarmkit::commands::CmdGimbalPoint;
using swarmkit::commands::CmdGoto;
using swarmkit::commands::CmdGripper;
using swarmkit::commands::CmdHoldPosition;
using swarmkit::commands::CmdLand;
using swarmkit::commands::CmdPause;
using swarmkit::commands::CmdPhoto;
using swarmkit::commands::CmdPhotoIntervalStart;
using swarmkit::commands::CmdPhotoIntervalStop;
using swarmkit::commands::CmdRelay;
using swarmkit::commands::CmdResume;
using swarmkit::commands::CmdReturnHome;
using swarmkit::commands::CmdRoiClear;
using swarmkit::commands::CmdRoiLocation;
using swarmkit::commands::CmdServo;
using swarmkit::commands::CmdSetHome;
using swarmkit::commands::CmdSetMode;
using swarmkit::commands::CmdSetSpeed;
using swarmkit::commands::CmdSetWaypoint;
using swarmkit::commands::CmdSetYaw;
using swarmkit::commands::CmdTakeoff;
using swarmkit::commands::CmdVelocity;
using swarmkit::commands::CmdVideoStart;
using swarmkit::commands::CmdVideoStop;
using swarmkit::commands::Command;
using swarmkit::commands::CommandEnvelope;
using swarmkit::commands::CommandPriority;
using swarmkit::commands::FlightCmd;
using swarmkit::commands::NavCmd;
using swarmkit::commands::PayloadCmd;

struct AppOptions {
    ClientConfig config;
    std::string config_path;
    std::string drone_id{"drone-1"};
    double default_takeoff_alt_m{5.0};
    float controller_speed_mps{2.0F};
    float controller_climb_mps{1.0F};
    float controller_yaw_step_deg{15.0F};
    float controller_yaw_rate_deg_s{45.0F};
    int controller_pulse_ms{700};
};

[[nodiscard]] std::string OptionValue(int argc, char** argv, std::string_view name,
                                      std::string_view fallback = {}) {
    for (int index = 1; index + 1 < argc; ++index) {
        if (std::string_view{argv[index]} == name) {
            return argv[index + 1];
        }
    }
    return std::string{fallback};
}

[[nodiscard]] bool HasFlag(int argc, char** argv, std::string_view name) {
    for (int index = 1; index < argc; ++index) {
        if (std::string_view{argv[index]} == name) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] std::string Lower(std::string value) {
    std::ranges::transform(value, value.begin(),
                           [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return value;
}

[[nodiscard]] bool IsIntegerLike(std::string_view value) {
    if (value.empty()) {
        return false;
    }
    std::size_t index = 0;
    if (value.front() == '-' || value.front() == '+') {
        index = 1;
    }
    return index < value.size() &&
           std::all_of(
               value.begin() + static_cast<std::ptrdiff_t>(index), value.end(),
               [](unsigned char ch) { return std::isdigit(ch) != 0; });
}

template <typename T, typename Parser>
[[nodiscard]] std::expected<T, std::string> ParseNumber(std::string_view text,
                                                        std::string_view name, Parser parser) {
    try {
        std::size_t parsed = 0;
        std::string copy{text};
        T value = parser(copy, &parsed);
        if (parsed != copy.size()) {
            return std::unexpected("invalid " + std::string{name} + ": " + copy);
        }
        return value;
    } catch (const std::exception& exc) {
        return std::unexpected("invalid " + std::string{name} + ": " + std::string{text} + " (" +
                               exc.what() + ")");
    }
}

[[nodiscard]] std::expected<double, std::string> ParseDouble(std::string_view text,
                                                             std::string_view name) {
    return ParseNumber<double>(text, name, [](const std::string& copy, std::size_t* parsed) {
        return std::stod(copy, parsed);
    });
}

[[nodiscard]] std::expected<float, std::string> ParseFloat(std::string_view text,
                                                           std::string_view name) {
    return ParseNumber<float>(text, name, [](const std::string& copy, std::size_t* parsed) {
        return std::stof(copy, parsed);
    });
}

[[nodiscard]] std::expected<int, std::string> ParseInt(std::string_view text,
                                                       std::string_view name) {
    return ParseNumber<int>(text, name, [](const std::string& copy, std::size_t* parsed) {
        return std::stoi(copy, parsed);
    });
}

[[nodiscard]] std::expected<std::int64_t, std::string> ParseInt64(std::string_view text,
                                                                  std::string_view name) {
    return ParseNumber<std::int64_t>(text, name, [](const std::string& copy, std::size_t* parsed) {
        return std::stoll(copy, parsed);
    });
}

[[nodiscard]] std::expected<std::size_t, std::string> ParseSize(std::string_view text,
                                                                std::string_view name) {
    const auto parsed = ParseInt64(text, name);
    if (!parsed.has_value()) {
        return std::unexpected(parsed.error());
    }
    if (*parsed < 0) {
        return std::unexpected(std::string{name} + " cannot be negative");
    }
    return static_cast<std::size_t>(*parsed);
}

[[nodiscard]] std::expected<CommandPriority, std::string> ParsePriority(std::string_view value) {
    const std::string lower = Lower(std::string{value});
    if (lower == "operator" || lower == "op" || lower == "0") {
        return CommandPriority::kOperator;
    }
    if (lower == "supervisor" || lower == "sup" || lower == "10") {
        return CommandPriority::kSupervisor;
    }
    if (lower == "override" || lower == "20") {
        return CommandPriority::kOverride;
    }
    if (lower == "emergency" || lower == "100") {
        return CommandPriority::kEmergency;
    }
    return std::unexpected("priority must be operator, supervisor, override, or emergency");
}

[[nodiscard]] const char* PriorityName(CommandPriority priority) {
    switch (priority) {
        case CommandPriority::kOperator:
            return "operator";
        case CommandPriority::kSupervisor:
            return "supervisor";
        case CommandPriority::kOverride:
            return "override";
        case CommandPriority::kEmergency:
            return "emergency";
    }
    return "unknown";
}

[[nodiscard]] const char* BoolName(bool value) {
    return value ? "yes" : "no";
}

[[nodiscard]] bool HasToken(const std::vector<std::string>& tokens, std::string_view token) {
    return std::ranges::find(tokens, token) != tokens.end();
}

[[nodiscard]] std::optional<std::string> ValueAfter(const std::vector<std::string>& tokens,
                                                    std::string_view option) {
    for (std::size_t index = 0; index + 1 < tokens.size(); ++index) {
        if (tokens[index] == option) {
            return tokens[index + 1];
        }
    }
    return std::nullopt;
}

[[nodiscard]] std::expected<std::vector<std::string>, std::string> Tokenize(
    const std::string& line) {
    std::vector<std::string> tokens;
    std::string current;
    char quote = '\0';
    bool escaping = false;

    for (char ch : line) {
        if (escaping) {
            current.push_back(ch);
            escaping = false;
            continue;
        }
        if (ch == '\\') {
            escaping = true;
            continue;
        }
        if (quote != '\0') {
            if (ch == quote) {
                quote = '\0';
            } else {
                current.push_back(ch);
            }
            continue;
        }
        if (ch == '\'' || ch == '"') {
            quote = ch;
            continue;
        }
        if (std::isspace(static_cast<unsigned char>(ch)) != 0) {
            if (!current.empty()) {
                tokens.push_back(std::move(current));
                current.clear();
            }
            continue;
        }
        current.push_back(ch);
    }

    if (escaping) {
        current.push_back('\\');
    }
    if (quote != '\0') {
        return std::unexpected("unterminated quote");
    }
    if (!current.empty()) {
        tokens.push_back(std::move(current));
    }
    return tokens;
}

[[nodiscard]] std::string JoinTail(const std::vector<std::string>& tokens, std::size_t first) {
    std::string out;
    for (std::size_t index = first; index < tokens.size(); ++index) {
        if (!out.empty()) {
            out += ' ';
        }
        out += tokens[index];
    }
    return out;
}

[[nodiscard]] const char* GoalStatusName(GoalStatus status) {
    switch (status) {
        case GoalStatus::kUnspecified:
            return "unspecified";
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
    }
    return "unknown";
}

[[nodiscard]] const char* ReportSeverityName(ReportSeverity severity) {
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
    return "unknown";
}

[[nodiscard]] const char* PeerStateName(DataPeerState state) {
    switch (state) {
        case DataPeerState::kUnknown:
            return "unknown";
        case DataPeerState::kReady:
            return "ready";
        case DataPeerState::kUnreachable:
            return "unreachable";
        case DataPeerState::kMisconfigured:
            return "misconfigured";
    }
    return "unknown";
}

[[nodiscard]] const char* TransferStateName(ArtifactTransferState state) {
    switch (state) {
        case ArtifactTransferState::kUnknown:
            return "unknown";
        case ArtifactTransferState::kQueued:
            return "queued";
        case ArtifactTransferState::kRunning:
            return "running";
        case ArtifactTransferState::kCompleted:
            return "completed";
        case ArtifactTransferState::kFailed:
            return "failed";
        case ArtifactTransferState::kCancelled:
            return "cancelled";
    }
    return "unknown";
}

[[nodiscard]] std::string Basename(const std::string& path) {
    std::filesystem::path file_path{path};
    return file_path.filename().string();
}

void PrintUsage(const char* program) {
    std::cerr
        << "Usage: " << program
        << " [--addr HOST:PORT] [--config FILE] [--client-id ID] [--drone ID] [--insecure]\n"
        << "       " << program << " --addr 127.0.0.1:50061 --insecure --drone drone-1\n\n"
        << "Options:\n"
        << "  --config FILE          Load client YAML, then apply CLI overrides.\n"
        << "  --addr HOST:PORT       SwarmKit agent address. Default: 127.0.0.1:50061\n"
        << "  --client-id ID         Client identity. Default: sdk-control\n"
        << "  --drone ID             Default target drone. Default: drone-1\n"
        << "  --deadline-ms N        Unary RPC deadline. Default: 5000\n"
        << "  --priority LEVEL       operator|supervisor|override|emergency. Default: operator\n"
        << "  --insecure             Use plaintext gRPC transport.\n"
        << "  --ca-cert PATH         Root CA certificate for TLS.\n"
        << "  --client-cert PATH     Client certificate for mTLS.\n"
        << "  --client-key PATH      Client private key for mTLS.\n"
        << "  --server-name NAME     TLS authority override.\n"
        << "  --takeoff-alt M        Controller takeoff altitude. Default: 5\n"
        << "  --speed MPS            Controller horizontal speed. Default: 2\n"
        << "  --climb-speed MPS      Controller climb/descent speed. Default: 1\n"
        << "  --yaw-step DEG         Controller yaw step per key. Default: 15\n"
        << "  --yaw-rate DEG_S       Controller yaw rate. Default: 45\n"
        << "  --pulse-ms N           Controller velocity pulse duration. Default: 700\n";
}

class RawTerminal {
   public:
    explicit RawTerminal(int fd) : fd_(fd) {
        if (!isatty(fd_)) {
            error_ = "stdin is not a TTY";
            return;
        }
        if (tcgetattr(fd_, &original_) != 0) {
            error_ = std::string{"tcgetattr failed: "} + std::strerror(errno);
            return;
        }

        termios raw = original_;
        raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        raw.c_iflag &= static_cast<tcflag_t>(~(IXON | ICRNL));
        raw.c_cc[VMIN] = 1;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(fd_, TCSANOW, &raw) != 0) {
            error_ = std::string{"tcsetattr failed: "} + std::strerror(errno);
            return;
        }
        active_ = true;
    }

    ~RawTerminal() {
        if (active_) {
            tcsetattr(fd_, TCSANOW, &original_);
        }
    }

    RawTerminal(const RawTerminal&) = delete;
    RawTerminal& operator=(const RawTerminal&) = delete;

    [[nodiscard]] bool active() const {
        return active_;
    }

    [[nodiscard]] const std::string& error() const {
        return error_;
    }

   private:
    int fd_;
    termios original_{};
    bool active_{false};
    std::string error_;
};

enum class ControllerKey {
    kUnknown,
    kEscape,
    kHelp,
    kForward,
    kBackward,
    kStrafeLeft,
    kStrafeRight,
    kClimb,
    kDescend,
    kYawLeft,
    kYawRight,
    kStop,
    kHold,
    kArm,
    kDisarm,
    kGuided,
    kTakeoff,
    kLand,
    kSpeedUp,
    kSpeedDown,
};

[[nodiscard]] bool ReadByteWithTimeout(int fd, unsigned char* out, int timeout_ms) {
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(fd, &read_set);
    timeval timeout{};
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    const int ready = select(fd + 1, &read_set, nullptr, nullptr, &timeout);
    if (ready <= 0) {
        return false;
    }
    return read(fd, out, 1) == 1;
}

[[nodiscard]] std::optional<ControllerKey> ReadControllerKey(int fd) {
    unsigned char ch{};
    if (read(fd, &ch, 1) != 1) {
        return std::nullopt;
    }

    if (ch == 27) {
        unsigned char next{};
        if (!ReadByteWithTimeout(fd, &next, 30)) {
            return ControllerKey::kEscape;
        }
        if (next != '[') {
            return ControllerKey::kEscape;
        }
        unsigned char arrow{};
        if (!ReadByteWithTimeout(fd, &arrow, 30)) {
            return ControllerKey::kEscape;
        }
        switch (arrow) {
            case 'A':
                return ControllerKey::kForward;
            case 'B':
                return ControllerKey::kBackward;
            case 'C':
                return ControllerKey::kYawRight;
            case 'D':
                return ControllerKey::kYawLeft;
            default:
                return ControllerKey::kUnknown;
        }
    }

    switch (ch) {
        case '?':
            return ControllerKey::kHelp;
        case 'w':
        case 'W':
            return ControllerKey::kForward;
        case 's':
        case 'S':
            return ControllerKey::kBackward;
        case 'a':
        case 'A':
            return ControllerKey::kStrafeLeft;
        case 'd':
        case 'D':
            return ControllerKey::kStrafeRight;
        case 'r':
        case 'R':
            return ControllerKey::kClimb;
        case 'f':
        case 'F':
            return ControllerKey::kDescend;
        case 'q':
        case 'Q':
            return ControllerKey::kYawLeft;
        case 'e':
        case 'E':
            return ControllerKey::kYawRight;
        case ' ':
        case 'x':
        case 'X':
            return ControllerKey::kStop;
        case 'h':
        case 'H':
            return ControllerKey::kHold;
        case '1':
            return ControllerKey::kArm;
        case '0':
            return ControllerKey::kDisarm;
        case 'g':
        case 'G':
            return ControllerKey::kGuided;
        case 't':
        case 'T':
            return ControllerKey::kTakeoff;
        case 'l':
        case 'L':
            return ControllerKey::kLand;
        case '+':
        case '=':
            return ControllerKey::kSpeedUp;
        case '-':
        case '_':
            return ControllerKey::kSpeedDown;
        default:
            return ControllerKey::kUnknown;
    }
}

class ControlShell {
   public:
    explicit ControlShell(AppOptions options)
        : client_(options.config),
          config_address_(options.config.address),
          drone_id_(std::move(options.drone_id)),
          client_id_(options.config.client_id),
          priority_(options.config.priority),
          default_takeoff_alt_m_(options.default_takeoff_alt_m),
          controller_speed_mps_(options.controller_speed_mps),
          controller_climb_mps_(options.controller_climb_mps),
          controller_yaw_step_deg_(options.controller_yaw_step_deg),
          controller_yaw_rate_deg_s_(options.controller_yaw_rate_deg_s),
          controller_pulse_ms_(options.controller_pulse_ms) {}

    ~ControlShell() {
        StopStreams();
    }

    int Run() {
        PrintBanner();
        std::string line;
        while (true) {
            {
                std::lock_guard<std::mutex> lock(output_mutex_);
                std::cout << "swarmkit(" << drone_id_ << ")> " << std::flush;
            }
            if (!std::getline(std::cin, line)) {
                std::cout << "\n";
                break;
            }

            auto tokens = Tokenize(line);
            if (!tokens.has_value()) {
                PrintLine("ERROR: " + tokens.error());
                continue;
            }
            if (tokens->empty()) {
                continue;
            }
            if (!Dispatch(*tokens)) {
                break;
            }
        }
        StopStreams();
        return EXIT_SUCCESS;
    }

   private:
    void PrintLine(const std::string& line) const {
        std::lock_guard<std::mutex> lock(output_mutex_);
        std::cout << line << "\n";
    }

    void PrintBanner() const {
        std::lock_guard<std::mutex> lock(output_mutex_);
        std::cout
            << "SwarmKit SDK control shell\n"
            << "  agent  : " << config_address_ << "\n"
            << "  client : " << client_id_ << "\n"
            << "  drone  : " << drone_id_ << "\n"
            << "Type 'help' for commands, 'controller' for raw-key control, 'quit' to exit.\n";
    }

    void PrintHelp() const {
        std::lock_guard<std::mutex> lock(output_mutex_);
        std::cout
            << "Core:\n"
            << "  ping | health | stats | capabilities | config\n"
            << "  drone ID | priority operator|supervisor|override|emergency\n"
            << "  set speed MPS | set climb MPS | set pulse MS | set yaw-step DEG | set "
               "takeoff-alt M\n"
            << "\nFlight and nav:\n"
            << "  arm [--wait] | force-arm | disarm | force-disarm | terminate\n"
            << "  launch [ALT] | fly [ALT]   # guided + arm/wait + takeoff/wait; fly enters "
               "controller\n"
            << "  mode NAME [CUSTOM_MODE] | takeoff [ALT] [--wait] | land [--wait]\n"
            << "  rtl | hold | pause | resume | speed MPS | yaw DEG [RATE] [relative]\n"
            << "  velocity VX VY VZ [DURATION_MS] [body|local]\n"
            << "  forward [MPS] | back [MPS] | strafe-left [MPS] | strafe-right [MPS] | up [MPS] | "
               "down [MPS]\n"
            << "  goto LAT LON ALT [SPEED] [--wait] | waypoint LAT LON ALT [SPEED]\n"
            << "  home current | home LAT LON ALT\n"
            << "\nPayload:\n"
            << "  payload photo [CAMERA] | payload interval-start INTERVAL_S [COUNT] [CAMERA]\n"
            << "  payload interval-stop [CAMERA] | payload video-start [STREAM] [CAMERA]\n"
            << "  payload video-stop [STREAM] [CAMERA] | payload gimbal PITCH ROLL YAW\n"
            << "  payload roi LAT LON ALT [GIMBAL] | payload roi-clear [GIMBAL]\n"
            << "  payload servo SERVO PWM | payload relay RELAY on|off | payload gripper ID "
               "release|grab\n"
            << "\nSubscriptions and authority:\n"
            << "  telemetry start [RATE] [quiet] | telemetry stop | telemetry show\n"
            << "  reports start [AFTER_SEQUENCE] | reports stop\n"
            << "  watch start | watch stop | lock [TTL_MS] | unlock | lease [TTL_MS] | lease "
               "release\n"
            << "\nGoals, messages, peers, artifacts:\n"
            << "  goal set ID LAT LON ALT [SPEED] [ACCEPT_M] | goal get | goal cancel [ID]\n"
            << "  message publish TOPIC PAYLOAD [TTL_MS] | message send TARGET TOPIC PAYLOAD "
               "[TTL_MS]\n"
            << "  messages start [TOPIC...] | messages stop\n"
            << "  peers list [refresh] | peers refresh [ID...] | peer upsert ID ADDR [SECURITY] | "
               "peer remove ID\n"
            << "  artifact upload PATH [CONTENT_TYPE] | artifact send PATH TARGET [CONTENT_TYPE]\n"
            << "  artifact start PATH [TARGET] | artifact status ID | artifact cancel ID\n"
            << "  artifact list | artifact get ID | artifact download ID PATH | artifact resume ID "
               "PATH\n"
            << "  artifact announce ID [FILENAME] [CONTENT_TYPE]\n"
            << "  upload create ARTIFACT_ID SIZE [FILENAME] | upload chunk UPLOAD_ID OFFSET INDEX "
               "DATA\n"
            << "  upload status ID | upload commit ID | upload cancel ID\n"
            << "\nRaw-key controller:\n"
            << "  controller\n"
            << "  ArrowUp/W forward, ArrowDown/S back, ArrowLeft/Q yaw left, ArrowRight/E yaw "
               "right\n"
            << "  A/D strafe, R/F climb/descend, Space/X stop, H hold, 1 arm, 0 disarm\n"
            << "  G guided, T takeoff, L land, +/- speed, ? help, Esc exit\n";
    }

    void StopStreams() {
        if (telemetry_) {
            telemetry_->Stop();
            telemetry_.reset();
        }
        if (reports_) {
            reports_->Stop();
            reports_.reset();
        }
        if (authority_watch_) {
            authority_watch_->Stop();
            authority_watch_.reset();
        }
        if (messages_) {
            messages_->Stop();
            messages_.reset();
        }
        if (authority_session_) {
            const ReleaseAuthorityResult result = authority_session_->Release();
            PrintReleaseResult("lease release", result);
            authority_session_.reset();
        }
    }

    [[nodiscard]] CommandEnvelope MakeEnvelope(Command command) {
        CommandEnvelope envelope;
        envelope.context.drone_id = drone_id_;
        envelope.context.client_id = client_id_;
        envelope.context.priority = priority_;
        envelope.context.correlation_id = "sdk-control-" + std::to_string(++correlation_counter_);
        envelope.command = std::move(command);
        return envelope;
    }

    [[nodiscard]] bool SendCommand(Command command, std::string_view label, bool wait = false) {
        CommandEnvelope envelope = MakeEnvelope(std::move(command));
        CommandResult result = wait ? client_.SendCommandAndWait(envelope, wait_options_)
                                    : client_.SendCommand(envelope);
        PrintCommandResult(label, result);
        return result.ok;
    }

    void PrintCommandResult(std::string_view label, const CommandResult& result) const {
        std::lock_guard<std::mutex> lock(output_mutex_);
        std::cout << label << ": " << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        if (!result.error.debug_message.empty()) {
            std::cout << " debug=" << result.error.debug_message;
        }
        std::cout << "\n";
    }

    void PrintReleaseResult(std::string_view label, const ReleaseAuthorityResult& result) const {
        std::lock_guard<std::mutex> lock(output_mutex_);
        std::cout << label << ": " << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
    }

    [[nodiscard]] bool Dispatch(const std::vector<std::string>& tokens) {
        const std::string command = Lower(tokens[0]);
        if (command == "quit" || command == "exit") {
            return false;
        }
        if (command == "help" || command == "?") {
            PrintHelp();
            return true;
        }
        if (command == "ping") {
            DoPing();
            return true;
        }
        if (command == "health") {
            DoHealth();
            return true;
        }
        if (command == "stats") {
            DoStats();
            return true;
        }
        if (command == "capabilities" || command == "caps") {
            DoCapabilities();
            return true;
        }
        if (command == "config") {
            DoConfig();
            return true;
        }
        if (command == "drone") {
            DoDrone(tokens);
            return true;
        }
        if (command == "priority") {
            DoPriority(tokens);
            return true;
        }
        if (command == "set") {
            DoSet(tokens);
            return true;
        }
        if (command == "controller") {
            RunController();
            return true;
        }
        if (HandleFlight(tokens) || HandleNav(tokens) || HandlePayload(tokens) ||
            HandleSubscriptions(tokens) || HandleAuthority(tokens) || HandleGoal(tokens) ||
            HandleMessages(tokens) || HandlePeers(tokens) || HandleArtifacts(tokens) ||
            HandleUploadSession(tokens)) {
            return true;
        }

        PrintLine("unknown command: " + tokens[0] + " (type 'help')");
        return true;
    }

    void DoPing() {
        const auto result = client_.Ping();
        std::lock_guard<std::mutex> lock(output_mutex_);
        std::cout << "Ping: " << (result.ok ? "OK" : "FAILED");
        if (result.ok) {
            std::cout << " agent_id=" << result.agent_id << " version=" << result.version
                      << " time_ms=" << result.unix_time_ms;
        } else {
            std::cout << " " << result.error_message;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
    }

    void DoHealth() {
        const HealthStatus result = client_.GetHealth();
        std::lock_guard<std::mutex> lock(output_mutex_);
        std::cout << "Health: " << (result.ok ? "OK" : "FAILED")
                  << " ready=" << BoolName(result.ready) << " backend=" << result.backend_name
                  << " protocol=" << result.protocol << " armed=" << BoolName(result.armed)
                  << " landed=" << BoolName(result.landed) << " mode=" << result.mode
                  << " gps_ok=" << BoolName(result.gps_ok) << " sats=" << result.satellites_visible
                  << " ekf_ok=" << BoolName(result.ekf_ok);
        if (!result.message.empty()) {
            std::cout << " message=" << result.message;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
        for (const auto& blocker : result.arming_blockers) {
            std::cout << "  blocker: " << blocker << "\n";
        }
        for (const auto& check : result.readiness_checks) {
            std::cout << "  check " << check.name << ": " << (check.ok ? "ok" : "bad") << " "
                      << check.detail << "\n";
        }
    }

    void DoStats() {
        const RuntimeStats result = client_.GetRuntimeStats();
        std::lock_guard<std::mutex> lock(output_mutex_);
        std::cout << "Stats: " << (result.ok ? "OK" : "FAILED")
                  << " ready=" << BoolName(result.ready) << " pings=" << result.ping_requests_total
                  << " health=" << result.health_requests_total
                  << " commands=" << result.command_requests_total
                  << " rejected=" << result.command_rejected_total
                  << " failed=" << result.command_failed_total
                  << " telemetry_streams=" << result.current_telemetry_streams
                  << " telemetry_frames=" << result.telemetry_frames_sent_total
                  << " reports_watchers=" << result.current_authority_watchers
                  << " messages=" << result.data_messages_published_total
                  << " artifact_uploads=" << result.artifact_uploads_total
                  << " artifact_downloads=" << result.artifact_downloads_total;
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
    }

    void DoCapabilities() {
        const BackendCapabilities result = client_.GetCapabilities();
        std::lock_guard<std::mutex> lock(output_mutex_);
        std::cout << "Capabilities: " << (result.ok ? "OK" : "FAILED")
                  << " backend=" << result.backend_name << " protocol=" << result.protocol
                  << " vehicle=" << result.vehicle_class << " autopilot=" << result.autopilot_type
                  << " velocity=" << BoolName(result.supports_velocity_control)
                  << " payload=" << BoolName(result.supports_payload_control)
                  << " backend_cmds=" << BoolName(result.supports_backend_commands);
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
        PrintStringList("  modes", result.supported_modes);
        PrintStringList("  commands", result.supported_commands);
        PrintStringList("  telemetry", result.supported_telemetry_fields);
        PrintStringList("  backend", result.backend_command_names);
    }

    void PrintStringList(std::string_view label, const std::vector<std::string>& values) const {
        std::cout << label << ":";
        for (const auto& value : values) {
            std::cout << " " << value;
        }
        std::cout << "\n";
    }

    void DoConfig() const {
        std::lock_guard<std::mutex> lock(output_mutex_);
        std::cout << "agent=" << config_address_ << " client=" << client_id_
                  << " drone=" << drone_id_ << " priority=" << PriorityName(priority_)
                  << " takeoff_alt=" << default_takeoff_alt_m_ << " speed=" << controller_speed_mps_
                  << " climb=" << controller_climb_mps_ << " yaw_step=" << controller_yaw_step_deg_
                  << " yaw_rate=" << controller_yaw_rate_deg_s_
                  << " pulse_ms=" << controller_pulse_ms_ << "\n";
    }

    void DoDrone(const std::vector<std::string>& tokens) {
        if (tokens.size() < 2) {
            PrintLine("drone=" + drone_id_);
            return;
        }
        drone_id_ = tokens[1];
        PrintLine("drone set to " + drone_id_);
    }

    void DoPriority(const std::vector<std::string>& tokens) {
        if (tokens.size() < 2) {
            PrintLine(std::string{"priority="} + PriorityName(priority_));
            return;
        }
        const auto parsed = ParsePriority(tokens[1]);
        if (!parsed.has_value()) {
            PrintLine("ERROR: " + parsed.error());
            return;
        }
        priority_ = *parsed;
        PrintLine(std::string{"priority set to "} + PriorityName(priority_));
    }

    void DoSet(const std::vector<std::string>& tokens) {
        if (tokens.size() < 3) {
            PrintLine("usage: set speed|climb|pulse|yaw-step|yaw-rate|takeoff-alt VALUE");
            return;
        }
        const std::string key = Lower(tokens[1]);
        if (key == "speed") {
            SetFloat(tokens[2], "speed", &controller_speed_mps_);
            return;
        }
        if (key == "climb") {
            SetFloat(tokens[2], "climb", &controller_climb_mps_);
            return;
        }
        if (key == "yaw-step") {
            SetFloat(tokens[2], "yaw-step", &controller_yaw_step_deg_);
            return;
        }
        if (key == "yaw-rate") {
            SetFloat(tokens[2], "yaw-rate", &controller_yaw_rate_deg_s_);
            return;
        }
        if (key == "pulse") {
            const auto parsed = ParseInt(tokens[2], "pulse");
            if (!parsed.has_value() || *parsed <= 0) {
                PrintLine("ERROR: pulse must be a positive integer");
                return;
            }
            controller_pulse_ms_ = *parsed;
            PrintLine("pulse set");
            return;
        }
        if (key == "takeoff-alt") {
            const auto parsed = ParseDouble(tokens[2], "takeoff-alt");
            if (!parsed.has_value()) {
                PrintLine("ERROR: " + parsed.error());
                return;
            }
            default_takeoff_alt_m_ = *parsed;
            PrintLine("takeoff-alt set");
            return;
        }
        PrintLine("unknown set key: " + key);
    }

    void SetFloat(std::string_view text, std::string_view name, float* target) {
        const auto parsed = ParseFloat(text, name);
        if (!parsed.has_value()) {
            PrintLine("ERROR: " + parsed.error());
            return;
        }
        *target = *parsed;
        PrintLine(std::string{name} + " set");
    }

    [[nodiscard]] bool HandleFlight(const std::vector<std::string>& tokens) {
        const std::string command = Lower(tokens[0]);
        const bool wait = HasToken(tokens, "--wait");
        if (command == "launch" || command == "fly") {
            const auto alt = ParseTakeoffAltitude(tokens, 1);
            if (!alt.has_value()) {
                PrintLine("ERROR: " + alt.error());
                return true;
            }
            RunLaunch(*alt, command == "fly");
            return true;
        }
        if (command == "arm") {
            const CommandResult result =
                wait ? client_.ArmAndWait(drone_id_, wait_options_)
                     : client_.SendCommand(MakeEnvelope(FlightCmd{CmdArm{}}));
            PrintCommandResult("arm", result);
            return true;
        }
        if (command == "force-arm") {
            static_cast<void>(SendCommand(FlightCmd{CmdForceArm{}}, "force-arm"));
            return true;
        }
        if (command == "disarm") {
            static_cast<void>(SendCommand(FlightCmd{CmdDisarm{}}, "disarm", wait));
            return true;
        }
        if (command == "force-disarm") {
            static_cast<void>(SendCommand(FlightCmd{CmdForceDisarm{}}, "force-disarm"));
            return true;
        }
        if (command == "terminate") {
            static_cast<void>(SendCommand(FlightCmd{CmdFlightTerminate{}}, "terminate"));
            return true;
        }
        if (command == "land") {
            const CommandResult result =
                wait ? client_.LandAndWait(drone_id_, wait_options_)
                     : client_.SendCommand(MakeEnvelope(FlightCmd{CmdLand{}}));
            PrintCommandResult("land", result);
            return true;
        }
        if (command == "takeoff") {
            const auto alt = ParseTakeoffAltitude(tokens, 1);
            if (!alt.has_value()) {
                PrintLine("ERROR: " + alt.error());
                return true;
            }
            const CommandResult result =
                wait ? client_.TakeoffAndWait(drone_id_, *alt, wait_options_)
                     : client_.SendCommand(MakeEnvelope(FlightCmd{CmdTakeoff{.alt_m = *alt}}));
            PrintCommandResult("takeoff", result);
            return true;
        }
        if (command == "mode" || command == "set-mode") {
            if (tokens.size() < 2) {
                PrintLine("usage: mode NAME [CUSTOM_MODE]");
                return true;
            }
            CmdSetMode mode;
            mode.mode = tokens[1];
            if (tokens.size() >= 3) {
                const auto parsed = ParseInt(tokens[2], "custom_mode");
                if (!parsed.has_value()) {
                    PrintLine("ERROR: " + parsed.error());
                    return true;
                }
                mode.custom_mode = *parsed;
            }
            static_cast<void>(SendCommand(FlightCmd{mode}, "mode"));
            return true;
        }
        return false;
    }

    [[nodiscard]] std::expected<double, std::string> ParseTakeoffAltitude(
        const std::vector<std::string>& tokens, std::size_t first_positional) const {
        double alt = default_takeoff_alt_m_;
        if (const auto value = ValueAfter(tokens, "--alt"); value.has_value()) {
            const auto parsed = ParseDouble(*value, "--alt");
            if (!parsed.has_value()) {
                return std::unexpected(parsed.error());
            }
            alt = *parsed;
        } else if (tokens.size() > first_positional &&
                   !tokens[first_positional].starts_with("--")) {
            const auto parsed = ParseDouble(tokens[first_positional], "alt");
            if (!parsed.has_value()) {
                return std::unexpected(parsed.error());
            }
            alt = *parsed;
        }
        if (alt <= 0.0) {
            return std::unexpected("takeoff altitude must be positive");
        }
        return alt;
    }

    void RunLaunch(double alt_m, bool enter_controller) {
        if (!SendCommand(FlightCmd{CmdSetMode{.mode = "guided"}}, "guided")) {
            return;
        }
        const CommandResult arm = client_.ArmAndWait(drone_id_, wait_options_);
        PrintCommandResult("arm", arm);
        if (!arm.ok) {
            return;
        }
        const CommandResult takeoff = client_.TakeoffAndWait(drone_id_, alt_m, wait_options_);
        PrintCommandResult("takeoff", takeoff);
        if (!takeoff.ok) {
            return;
        }
        if (!telemetry_) {
            StartTelemetryStream(5, false);
        }
        if (enter_controller) {
            RunController();
        }
    }

    [[nodiscard]] bool HandleNav(const std::vector<std::string>& tokens) {
        const std::string command = Lower(tokens[0]);
        const bool wait = HasToken(tokens, "--wait");
        if (command == "rtl" || command == "return-home") {
            static_cast<void>(SendCommand(NavCmd{CmdReturnHome{}}, "rtl"));
            return true;
        }
        if (command == "hold") {
            static_cast<void>(SendCommand(NavCmd{CmdHoldPosition{}}, "hold"));
            return true;
        }
        if (command == "pause") {
            static_cast<void>(SendCommand(NavCmd{CmdPause{}}, "pause"));
            return true;
        }
        if (command == "resume") {
            static_cast<void>(SendCommand(NavCmd{CmdResume{}}, "resume"));
            return true;
        }
        if (command == "speed") {
            if (tokens.size() < 2) {
                PrintLine("usage: speed MPS");
                return true;
            }
            const auto speed = ParseFloat(tokens[1], "speed");
            if (!speed.has_value()) {
                PrintLine("ERROR: " + speed.error());
                return true;
            }
            static_cast<void>(SendCommand(NavCmd{CmdSetSpeed{.ground_mps = *speed}}, "speed"));
            return true;
        }
        if (command == "yaw") {
            if (tokens.size() < 2) {
                PrintLine("usage: yaw DEG [RATE] [relative]");
                return true;
            }
            const auto yaw = ParseFloat(tokens[1], "yaw");
            if (!yaw.has_value()) {
                PrintLine("ERROR: " + yaw.error());
                return true;
            }
            float rate = controller_yaw_rate_deg_s_;
            if (tokens.size() >= 3 && Lower(tokens[2]) != "relative") {
                const auto parsed = ParseFloat(tokens[2], "rate");
                if (!parsed.has_value()) {
                    PrintLine("ERROR: " + parsed.error());
                    return true;
                }
                rate = *parsed;
            }
            static_cast<void>(SendCommand(
                NavCmd{CmdSetYaw{
                    .yaw_deg = *yaw, .rate_deg_s = rate, .relative = HasToken(tokens, "relative")}},
                "yaw"));
            return true;
        }
        if (command == "velocity") {
            DoVelocity(tokens);
            return true;
        }
        if (command == "forward" || command == "back" || command == "strafe-left" ||
            command == "strafe-right" || command == "up" || command == "down") {
            DoDirectionalVelocity(command, tokens);
            return true;
        }
        if (command == "goto" || command == "waypoint") {
            DoGotoOrWaypoint(command, tokens, wait);
            return true;
        }
        if (command == "home" || command == "set-home") {
            DoSetHome(tokens);
            return true;
        }
        if (command == "backend") {
            DoBackendCommand(tokens);
            return true;
        }
        return false;
    }

    void DoVelocity(const std::vector<std::string>& tokens) {
        if (tokens.size() < 4) {
            PrintLine("usage: velocity VX VY VZ [DURATION_MS] [body|local]");
            return;
        }
        const auto vx = ParseFloat(tokens[1], "vx");
        const auto vy = ParseFloat(tokens[2], "vy");
        const auto vz = ParseFloat(tokens[3], "vz");
        if (!vx.has_value() || !vy.has_value() || !vz.has_value()) {
            PrintLine("ERROR: invalid velocity component");
            return;
        }
        int duration_ms = controller_pulse_ms_;
        if (tokens.size() >= 5 && IsIntegerLike(tokens[4])) {
            const auto parsed = ParseInt(tokens[4], "duration_ms");
            if (!parsed.has_value()) {
                PrintLine("ERROR: " + parsed.error());
                return;
            }
            duration_ms = *parsed;
        }
        const bool body_frame = !HasToken(tokens, "local");
        SendVelocity(*vx, *vy, *vz, duration_ms, body_frame, "velocity");
    }

    void DoDirectionalVelocity(const std::string& command, const std::vector<std::string>& tokens) {
        float speed =
            (command == "up" || command == "down") ? controller_climb_mps_ : controller_speed_mps_;
        if (tokens.size() >= 2) {
            const auto parsed = ParseFloat(tokens[1], "speed");
            if (!parsed.has_value()) {
                PrintLine("ERROR: " + parsed.error());
                return;
            }
            speed = *parsed;
        }
        float vx = 0.0F;
        float vy = 0.0F;
        float vz = 0.0F;
        if (command == "forward") {
            vx = speed;
        } else if (command == "back") {
            vx = -speed;
        } else if (command == "strafe-left") {
            vy = -speed;
        } else if (command == "strafe-right") {
            vy = speed;
        } else if (command == "up") {
            vz = -speed;
        } else if (command == "down") {
            vz = speed;
        }
        SendVelocity(vx, vy, vz, controller_pulse_ms_, true, command);
    }

    bool SendVelocity(float vx, float vy, float vz, int duration_ms, bool body_frame,
                      std::string_view label) {
        CmdVelocity velocity;
        velocity.vx_mps = vx;
        velocity.vy_mps = vy;
        velocity.vz_mps = vz;
        velocity.duration_ms = duration_ms;
        velocity.body_frame = body_frame;
        return SendCommand(NavCmd{velocity}, label);
    }

    void DoGotoOrWaypoint(const std::string& command, const std::vector<std::string>& tokens,
                          bool wait) {
        if (tokens.size() < 4) {
            PrintLine("usage: " + command + " LAT LON ALT [SPEED] [--wait]");
            return;
        }
        const auto lat = ParseDouble(tokens[1], "lat");
        const auto lon = ParseDouble(tokens[2], "lon");
        const auto alt = ParseDouble(tokens[3], "alt");
        if (!lat.has_value() || !lon.has_value() || !alt.has_value()) {
            PrintLine("ERROR: invalid coordinates");
            return;
        }
        float speed = controller_speed_mps_;
        if (tokens.size() >= 5 && tokens[4] != "--wait") {
            const auto parsed = ParseFloat(tokens[4], "speed");
            if (!parsed.has_value()) {
                PrintLine("ERROR: " + parsed.error());
                return;
            }
            speed = *parsed;
        }
        if (command == "goto") {
            const CommandResult result =
                wait ? client_.GotoAndWait(drone_id_, *lat, *lon, *alt, speed, wait_options_)
                     : client_.SendCommand(MakeEnvelope(NavCmd{CmdGoto{
                           .lat_deg = *lat,
                           .lon_deg = *lon,
                           .alt_m = *alt,
                           .speed_mps = speed,
                       }}));
            PrintCommandResult("goto", result);
            return;
        }
        static_cast<void>(
            SendCommand(NavCmd{CmdSetWaypoint{
                            .lat_deg = *lat, .lon_deg = *lon, .alt_m = *alt, .speed_mps = speed}},
                        "waypoint", wait));
    }

    void DoSetHome(const std::vector<std::string>& tokens) {
        if (tokens.size() < 2) {
            PrintLine("usage: home current | home LAT LON ALT");
            return;
        }
        CmdSetHome home;
        if (Lower(tokens[1]) == "current") {
            home.use_current = true;
        } else {
            if (tokens.size() < 4) {
                PrintLine("usage: home LAT LON ALT");
                return;
            }
            const auto lat = ParseDouble(tokens[1], "lat");
            const auto lon = ParseDouble(tokens[2], "lon");
            const auto alt = ParseDouble(tokens[3], "alt");
            if (!lat.has_value() || !lon.has_value() || !alt.has_value()) {
                PrintLine("ERROR: invalid home coordinates");
                return;
            }
            home.use_current = false;
            home.lat_deg = *lat;
            home.lon_deg = *lon;
            home.alt_m = *alt;
        }
        static_cast<void>(SendCommand(NavCmd{home}, "home"));
    }

    void DoBackendCommand(const std::vector<std::string>& tokens) {
        if (tokens.size() < 3) {
            PrintLine("usage: backend NAMESPACE NAME [key=value...]");
            return;
        }
        CmdBackendCommand backend;
        backend.backend_namespace = tokens[1];
        backend.name = tokens[2];
        for (std::size_t index = 3; index < tokens.size(); ++index) {
            const std::string& item = tokens[index];
            const std::size_t equals = item.find('=');
            if (equals == std::string::npos) {
                PrintLine("ERROR: backend params must be key=value");
                return;
            }
            backend.params[item.substr(0, equals)] = item.substr(equals + 1);
        }
        static_cast<void>(SendCommand(BackendCmd{backend}, "backend"));
    }

    [[nodiscard]] bool HandlePayload(const std::vector<std::string>& tokens) {
        if (Lower(tokens[0]) != "payload") {
            return false;
        }
        if (tokens.size() < 2) {
            PrintLine(
                "usage: payload photo|interval-start|interval-stop|video-start|video-stop|"
                "gimbal|roi|roi-clear|servo|relay|gripper ...");
            return true;
        }
        const std::string action = Lower(tokens[1]);
        if (action == "photo") {
            const auto camera = OptionalInt(tokens, 2, 0, "camera");
            if (!camera.has_value()) {
                PrintLine("ERROR: " + camera.error());
                return true;
            }
            static_cast<void>(SendCommand(PayloadCmd{CmdPhoto{.camera_id = *camera}}, "photo"));
            return true;
        }
        if (action == "interval-start") {
            if (tokens.size() < 3) {
                PrintLine("usage: payload interval-start INTERVAL_S [COUNT] [CAMERA]");
                return true;
            }
            const auto interval = ParseFloat(tokens[2], "interval_s");
            const auto count = OptionalInt(tokens, 3, 0, "count");
            const auto camera = OptionalInt(tokens, 4, 0, "camera");
            if (!interval.has_value() || !count.has_value() || !camera.has_value()) {
                PrintLine("ERROR: invalid interval-start argument");
                return true;
            }
            static_cast<void>(
                SendCommand(PayloadCmd{CmdPhotoIntervalStart{
                                .interval_s = *interval, .count = *count, .camera_id = *camera}},
                            "photo-interval-start"));
            return true;
        }
        if (action == "interval-stop") {
            const auto camera = OptionalInt(tokens, 2, 0, "camera");
            if (!camera.has_value()) {
                PrintLine("ERROR: " + camera.error());
                return true;
            }
            static_cast<void>(SendCommand(PayloadCmd{CmdPhotoIntervalStop{.camera_id = *camera}},
                                          "photo-interval-stop"));
            return true;
        }
        if (action == "video-start") {
            const auto stream = OptionalInt(tokens, 2, 0, "stream");
            const auto camera = OptionalInt(tokens, 3, 0, "camera");
            if (!stream.has_value() || !camera.has_value()) {
                PrintLine("ERROR: invalid video-start argument");
                return true;
            }
            static_cast<void>(
                SendCommand(PayloadCmd{CmdVideoStart{.stream_id = *stream, .camera_id = *camera}},
                            "video-start"));
            return true;
        }
        if (action == "video-stop") {
            const auto stream = OptionalInt(tokens, 2, 0, "stream");
            const auto camera = OptionalInt(tokens, 3, 0, "camera");
            if (!stream.has_value() || !camera.has_value()) {
                PrintLine("ERROR: invalid video-stop argument");
                return true;
            }
            static_cast<void>(
                SendCommand(PayloadCmd{CmdVideoStop{.stream_id = *stream, .camera_id = *camera}},
                            "video-stop"));
            return true;
        }
        if (action == "gimbal") {
            if (tokens.size() < 5) {
                PrintLine("usage: payload gimbal PITCH ROLL YAW");
                return true;
            }
            const auto pitch = ParseFloat(tokens[2], "pitch");
            const auto roll = ParseFloat(tokens[3], "roll");
            const auto yaw = ParseFloat(tokens[4], "yaw");
            if (!pitch.has_value() || !roll.has_value() || !yaw.has_value()) {
                PrintLine("ERROR: invalid gimbal argument");
                return true;
            }
            static_cast<void>(SendCommand(
                PayloadCmd{CmdGimbalPoint{.pitch_deg = *pitch, .roll_deg = *roll, .yaw_deg = *yaw}},
                "gimbal"));
            return true;
        }
        if (action == "roi") {
            if (tokens.size() < 5) {
                PrintLine("usage: payload roi LAT LON ALT [GIMBAL]");
                return true;
            }
            const auto lat = ParseDouble(tokens[2], "lat");
            const auto lon = ParseDouble(tokens[3], "lon");
            const auto alt = ParseDouble(tokens[4], "alt");
            const auto gimbal = OptionalInt(tokens, 5, 0, "gimbal");
            if (!lat.has_value() || !lon.has_value() || !alt.has_value() || !gimbal.has_value()) {
                PrintLine("ERROR: invalid roi argument");
                return true;
            }
            static_cast<void>(SendCommand(
                PayloadCmd{CmdRoiLocation{
                    .lat_deg = *lat, .lon_deg = *lon, .alt_m = *alt, .gimbal_id = *gimbal}},
                "roi"));
            return true;
        }
        if (action == "roi-clear") {
            const auto gimbal = OptionalInt(tokens, 2, 0, "gimbal");
            if (!gimbal.has_value()) {
                PrintLine("ERROR: " + gimbal.error());
                return true;
            }
            static_cast<void>(
                SendCommand(PayloadCmd{CmdRoiClear{.gimbal_id = *gimbal}}, "roi-clear"));
            return true;
        }
        if (action == "servo") {
            if (tokens.size() < 4) {
                PrintLine("usage: payload servo SERVO PWM");
                return true;
            }
            const auto servo = ParseInt(tokens[2], "servo");
            const auto pwm = ParseInt(tokens[3], "pwm");
            if (!servo.has_value() || !pwm.has_value()) {
                PrintLine("ERROR: invalid servo argument");
                return true;
            }
            static_cast<void>(
                SendCommand(PayloadCmd{CmdServo{.servo = *servo, .pwm = *pwm}}, "servo"));
            return true;
        }
        if (action == "relay") {
            if (tokens.size() < 4) {
                PrintLine("usage: payload relay RELAY on|off");
                return true;
            }
            const auto relay = ParseInt(tokens[2], "relay");
            const auto enabled = ParseOnOff(tokens[3]);
            if (!relay.has_value() || !enabled.has_value()) {
                PrintLine("ERROR: invalid relay argument");
                return true;
            }
            static_cast<void>(
                SendCommand(PayloadCmd{CmdRelay{.relay = *relay, .enabled = *enabled}}, "relay"));
            return true;
        }
        if (action == "gripper") {
            if (tokens.size() < 4) {
                PrintLine("usage: payload gripper ID release|grab");
                return true;
            }
            const auto gripper = ParseInt(tokens[2], "gripper");
            const auto release = ParseReleaseGrab(tokens[3]);
            if (!gripper.has_value() || !release.has_value()) {
                PrintLine("ERROR: invalid gripper argument");
                return true;
            }
            static_cast<void>(SendCommand(
                PayloadCmd{CmdGripper{.gripper = *gripper, .release = *release}}, "gripper"));
            return true;
        }

        PrintLine("unknown payload action: " + action);
        return true;
    }

    [[nodiscard]] std::expected<int, std::string> OptionalInt(
        const std::vector<std::string>& tokens, std::size_t index, int fallback,
        std::string_view name) const {
        if (tokens.size() <= index) {
            return fallback;
        }
        return ParseInt(tokens[index], name);
    }

    [[nodiscard]] std::expected<bool, std::string> ParseOnOff(std::string_view value) const {
        const std::string lower = Lower(std::string{value});
        if (lower == "on" || lower == "true" || lower == "1" || lower == "yes") {
            return true;
        }
        if (lower == "off" || lower == "false" || lower == "0" || lower == "no") {
            return false;
        }
        return std::unexpected("expected on/off");
    }

    [[nodiscard]] std::expected<bool, std::string> ParseReleaseGrab(std::string_view value) const {
        const std::string lower = Lower(std::string{value});
        if (lower == "release" || lower == "open" || lower == "drop") {
            return true;
        }
        if (lower == "grab" || lower == "close" || lower == "hold") {
            return false;
        }
        return std::unexpected("expected release/grab");
    }

    [[nodiscard]] bool HandleSubscriptions(const std::vector<std::string>& tokens) {
        const std::string command = Lower(tokens[0]);
        if (command == "telemetry") {
            DoTelemetry(tokens);
            return true;
        }
        if (command == "reports") {
            DoReports(tokens);
            return true;
        }
        if (command == "watch") {
            DoWatch(tokens);
            return true;
        }
        return false;
    }

    void DoTelemetry(const std::vector<std::string>& tokens) {
        if (tokens.size() < 2) {
            PrintLine("usage: telemetry start [RATE] [quiet] | telemetry stop | telemetry show");
            return;
        }
        const std::string action = Lower(tokens[1]);
        if (action == "start") {
            int rate = 5;
            if (tokens.size() >= 3 && IsIntegerLike(tokens[2])) {
                const auto parsed = ParseInt(tokens[2], "rate");
                if (!parsed.has_value() || *parsed <= 0) {
                    PrintLine("ERROR: rate must be positive");
                    return;
                }
                rate = *parsed;
            }
            StartTelemetryStream(rate, !HasToken(tokens, "quiet"));
            return;
        }
        if (action == "stop") {
            if (telemetry_) {
                telemetry_->Stop();
                telemetry_.reset();
            }
            PrintLine("telemetry stopped");
            return;
        }
        if (action == "show") {
            PrintLatestTelemetry();
            return;
        }
        PrintLine("unknown telemetry action: " + action);
    }

    bool StartTelemetryStream(int rate_hz, bool print_frames) {
        if (telemetry_) {
            telemetry_->Stop();
            telemetry_.reset();
        }
        telemetry_print_frames_ = print_frames;
        swarmkit::client::TelemetrySubscription subscription;
        subscription.drone_id = drone_id_;
        subscription.rate_hertz = rate_hz;
        auto stream = client_.StartTelemetry(
            subscription,
            [this](const swarmkit::core::TelemetryFrame& frame) {
                {
                    std::lock_guard<std::mutex> lock(telemetry_mutex_);
                    latest_frame_ = frame;
                    have_latest_frame_ = true;
                }
                if (telemetry_print_frames_) {
                    std::lock_guard<std::mutex> lock(output_mutex_);
                    std::cout << "[telemetry] " << TelemetrySummary(frame) << "\n";
                }
            },
            [this](const std::string& error) { PrintLine("[telemetry error] " + error); },
            [this](const swarmkit::client::SubscriptionEvent& event) {
                if (!event.message.empty()) {
                    PrintLine("[telemetry event] " + event.drone_id + " " + event.message);
                }
            });
        if (!stream.has_value()) {
            PrintLine("telemetry start FAILED: " + stream.error().ToString());
            return false;
        }
        telemetry_.emplace(std::move(*stream));
        PrintLine("telemetry started");
        return true;
    }

    void PrintLatestTelemetry() const {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        if (!have_latest_frame_) {
            std::lock_guard<std::mutex> output_lock(output_mutex_);
            std::cout << "no telemetry frame yet\n";
            return;
        }
        std::lock_guard<std::mutex> output_lock(output_mutex_);
        std::cout << TelemetrySummary(latest_frame_) << "\n";
    }

    [[nodiscard]] std::string LatestTelemetrySummary() const {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        if (!have_latest_frame_) {
            return "telemetry: none";
        }
        return TelemetrySummary(latest_frame_);
    }

    [[nodiscard]] static std::string TelemetrySummary(const swarmkit::core::TelemetryFrame& frame) {
        std::ostringstream out;
        out << "drone=" << frame.drone_id << " armed=" << BoolName(frame.armed)
            << " landed=" << BoolName(frame.landed) << " mode=" << frame.mode;
        if (frame.HasPosition()) {
            out << " lat=" << frame.lat_deg << " lon=" << frame.lon_deg;
        }
        if (frame.HasRelativeAltitude()) {
            out << " rel_alt=" << frame.rel_alt_m;
        }
        if (frame.HasVelocity()) {
            out << " vel=(" << frame.vx_mps << "," << frame.vy_mps << "," << frame.vz_mps << ")";
        }
        if (frame.HasAttitude()) {
            out << " yaw=" << frame.yaw_deg;
        }
        if (frame.HasBattery()) {
            out << " batt=" << frame.battery_percent;
        }
        return out.str();
    }

    void DoReports(const std::vector<std::string>& tokens) {
        if (tokens.size() < 2) {
            PrintLine("usage: reports start [AFTER_SEQUENCE] | reports stop");
            return;
        }
        const std::string action = Lower(tokens[1]);
        if (action == "stop") {
            if (reports_) {
                reports_->Stop();
                reports_.reset();
            }
            PrintLine("reports stopped");
            return;
        }
        if (action != "start") {
            PrintLine("unknown reports action: " + action);
            return;
        }
        std::uint64_t after_sequence = 0;
        if (tokens.size() >= 3) {
            const auto parsed = ParseInt64(tokens[2], "after_sequence");
            if (!parsed.has_value() || *parsed < 0) {
                PrintLine("ERROR: after_sequence must be non-negative");
                return;
            }
            after_sequence = static_cast<std::uint64_t>(*parsed);
        }
        if (reports_) {
            reports_->Stop();
            reports_.reset();
        }
        auto stream = client_.StartReports(
            {.drone_id = drone_id_, .after_sequence = after_sequence},
            [this](const AgentReport& report) { PrintReport(report); },
            [this](const std::string& error) { PrintLine("[report error] " + error); });
        if (!stream.has_value()) {
            PrintLine("reports start FAILED: " + stream.error().ToString());
            return;
        }
        reports_.emplace(std::move(*stream));
        PrintLine("reports started");
    }

    void PrintReport(const AgentReport& report) const {
        std::lock_guard<std::mutex> lock(output_mutex_);
        std::cout << "[report] seq=" << report.sequence << " drone=" << report.drone_id
                  << " severity=" << ReportSeverityName(report.severity)
                  << " message=" << report.message;
        if (!report.correlation_id.empty()) {
            std::cout << " corr=" << report.correlation_id;
        }
        if (report.goal.has_value()) {
            std::cout << " goal=" << report.goal->goal_id
                      << " status=" << GoalStatusName(report.goal->status)
                      << " dist=" << report.goal->distance_to_goal_m;
        }
        std::cout << "\n";
    }

    void DoWatch(const std::vector<std::string>& tokens) {
        if (tokens.size() < 2) {
            PrintLine("usage: watch start | watch stop");
            return;
        }
        const std::string action = Lower(tokens[1]);
        if (action == "stop") {
            if (authority_watch_) {
                authority_watch_->Stop();
                authority_watch_.reset();
            }
            PrintLine("authority watch stopped");
            return;
        }
        if (action != "start") {
            PrintLine("unknown watch action: " + action);
            return;
        }
        if (authority_watch_) {
            authority_watch_->Stop();
            authority_watch_.reset();
        }
        auto stream = client_.StartAuthorityWatch(
            {.drone_id = drone_id_, .priority = priority_},
            [this](const AuthorityEventInfo& event) { PrintAuthorityEvent(event); },
            [this](const std::string& error) { PrintLine("[authority watch error] " + error); });
        if (!stream.has_value()) {
            PrintLine("authority watch start FAILED: " + stream.error().ToString());
            return;
        }
        authority_watch_.emplace(std::move(*stream));
        PrintLine("authority watch started");
    }

    void PrintAuthorityEvent(const AuthorityEventInfo& event) const {
        std::lock_guard<std::mutex> lock(output_mutex_);
        std::cout << "[authority] drone=" << event.drone_id << " holder=" << event.holder_client_id
                  << " priority=" << PriorityName(event.holder_priority);
        if (!event.correlation_id.empty()) {
            std::cout << " corr=" << event.correlation_id;
        }
        std::cout << "\n";
    }

    [[nodiscard]] bool HandleAuthority(const std::vector<std::string>& tokens) {
        const std::string command = Lower(tokens[0]);
        if (command == "lock") {
            const std::int64_t ttl = ParseOptionalTtl(tokens);
            PrintCommandResult("lock", client_.LockAuthority(drone_id_, ttl));
            return true;
        }
        if (command == "unlock") {
            PrintReleaseResult("unlock", client_.ReleaseAuthority(drone_id_));
            return true;
        }
        if (command == "lease") {
            DoLease(tokens);
            return true;
        }
        return false;
    }

    [[nodiscard]] std::int64_t ParseOptionalTtl(const std::vector<std::string>& tokens) {
        if (tokens.size() < 2) {
            return 0;
        }
        const auto parsed = ParseInt64(tokens[1], "ttl_ms");
        if (!parsed.has_value()) {
            PrintLine("ERROR: " + parsed.error());
            return 0;
        }
        return *parsed;
    }

    void DoLease(const std::vector<std::string>& tokens) {
        if (tokens.size() >= 2 && Lower(tokens[1]) == "release") {
            if (!authority_session_) {
                PrintLine("no active lease");
                return;
            }
            PrintReleaseResult("lease release", authority_session_->Release());
            authority_session_.reset();
            return;
        }
        const std::int64_t ttl = ParseOptionalTtl(tokens);
        auto session = client_.AcquireAuthoritySession(drone_id_, ttl);
        if (!session.has_value()) {
            PrintCommandResult("lease", session.error());
            return;
        }
        authority_session_.emplace(std::move(*session));
        PrintLine("lease acquired");
    }

    [[nodiscard]] bool HandleGoal(const std::vector<std::string>& tokens) {
        const std::string command = Lower(tokens[0]);
        if (command != "goal") {
            return false;
        }
        if (tokens.size() < 2) {
            PrintLine(
                "usage: goal set ID LAT LON ALT [SPEED] [ACCEPT_M] | goal get | goal cancel [ID]");
            return true;
        }
        const std::string action = Lower(tokens[1]);
        if (action == "get") {
            const auto status = client_.GetActiveGoal(drone_id_);
            std::lock_guard<std::mutex> lock(output_mutex_);
            std::cout << "goal get: " << (status.error.IsOk() ? "OK" : "FAILED")
                      << " has_goal=" << BoolName(status.has_goal)
                      << " status=" << GoalStatusName(status.status)
                      << " timeout_ms=" << status.computed_timeout_ms
                      << " message=" << status.message << "\n";
            if (status.has_goal) {
                PrintGoal(status.goal);
            }
            return true;
        }
        if (action == "cancel") {
            const std::string goal_id = tokens.size() >= 3 ? tokens[2] : std::string{};
            PrintCommandResult("goal cancel", client_.CancelGoal(drone_id_, goal_id));
            return true;
        }
        if (action != "set") {
            PrintLine("unknown goal action: " + action);
            return true;
        }
        if (tokens.size() < 6) {
            PrintLine("usage: goal set ID LAT LON ALT [SPEED] [ACCEPT_M]");
            return true;
        }
        const auto lat = ParseDouble(tokens[3], "lat");
        const auto lon = ParseDouble(tokens[4], "lon");
        const auto alt = ParseDouble(tokens[5], "alt");
        if (!lat.has_value() || !lon.has_value() || !alt.has_value()) {
            PrintLine("ERROR: invalid goal coordinates");
            return true;
        }
        ActiveGoal goal;
        goal.drone_id = drone_id_;
        goal.goal_id = tokens[2];
        goal.revision = ++goal_revision_;
        goal.target.lat_deg = *lat;
        goal.target.lon_deg = *lon;
        goal.target.alt_m = *alt;
        goal.target_frame = "global";
        goal.speed_mps = controller_speed_mps_;
        if (tokens.size() >= 7) {
            const auto speed = ParseFloat(tokens[6], "speed");
            if (!speed.has_value()) {
                PrintLine("ERROR: " + speed.error());
                return true;
            }
            goal.speed_mps = *speed;
        }
        if (tokens.size() >= 8) {
            const auto acceptance = ParseFloat(tokens[7], "acceptance");
            if (!acceptance.has_value()) {
                PrintLine("ERROR: " + acceptance.error());
                return true;
            }
            goal.acceptance_radius_m = *acceptance;
        }
        const GoalResult result = client_.SetActiveGoal(goal);
        PrintGoalResult("goal set", result);
        return true;
    }

    void PrintGoalResult(std::string_view label, const GoalResult& result) const {
        std::lock_guard<std::mutex> lock(output_mutex_);
        std::cout << label << ": " << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << " computed_timeout_ms=" << result.computed_timeout_ms << "\n";
        if (result.ok) {
            PrintGoal(result.goal);
        }
    }

    void PrintGoal(const ActiveGoal& goal) const {
        std::cout << "  goal=" << goal.goal_id << " rev=" << goal.revision << " target=("
                  << goal.target.lat_deg << "," << goal.target.lon_deg << "," << goal.target.alt_m
                  << ")"
                  << " speed=" << goal.speed_mps << " accept=" << goal.acceptance_radius_m << "\n";
    }

    [[nodiscard]] bool HandleMessages(const std::vector<std::string>& tokens) {
        const std::string command = Lower(tokens[0]);
        if (command == "message") {
            DoMessage(tokens);
            return true;
        }
        if (command == "messages") {
            DoMessageSubscription(tokens);
            return true;
        }
        return false;
    }

    void DoMessage(const std::vector<std::string>& tokens) {
        if (tokens.size() < 4) {
            PrintLine(
                "usage: message publish TOPIC PAYLOAD [TTL_MS] | message send TARGET TOPIC PAYLOAD "
                "[TTL_MS]");
            return;
        }
        const std::string action = Lower(tokens[1]);
        DataMessage message;
        if (action == "publish") {
            message.topic = tokens[2];
            message.payload = tokens[3];
            if (tokens.size() >= 5) {
                const auto ttl = ParseInt64(tokens[4], "ttl_ms");
                if (!ttl.has_value()) {
                    PrintLine("ERROR: " + ttl.error());
                    return;
                }
                message.ttl_ms = *ttl;
            }
            const auto result = client_.PublishMessage(std::move(message));
            PrintPublishResult("message publish", result);
            return;
        }
        if (action == "send") {
            if (tokens.size() < 5) {
                PrintLine("usage: message send TARGET TOPIC PAYLOAD [TTL_MS]");
                return;
            }
            message.target_id = tokens[2];
            message.topic = tokens[3];
            message.payload = tokens[4];
            if (tokens.size() >= 6) {
                const auto ttl = ParseInt64(tokens[5], "ttl_ms");
                if (!ttl.has_value()) {
                    PrintLine("ERROR: " + ttl.error());
                    return;
                }
                message.ttl_ms = *ttl;
            }
            const auto result = client_.SendMessageToDrone(std::move(message));
            PrintPublishResult("message send", result);
            return;
        }
        PrintLine("unknown message action: " + action);
    }

    void PrintPublishResult(std::string_view label,
                            const swarmkit::client::PublishMessageResult& result) const {
        std::lock_guard<std::mutex> lock(output_mutex_);
        std::cout << label << ": " << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (result.sequence > 0) {
            std::cout << " seq=" << result.sequence;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
    }

    void DoMessageSubscription(const std::vector<std::string>& tokens) {
        if (tokens.size() < 2) {
            PrintLine("usage: messages start [TOPIC...] | messages stop");
            return;
        }
        const std::string action = Lower(tokens[1]);
        if (action == "stop") {
            if (messages_) {
                messages_->Stop();
                messages_.reset();
            }
            PrintLine("messages stopped");
            return;
        }
        if (action != "start") {
            PrintLine("unknown messages action: " + action);
            return;
        }
        if (messages_) {
            messages_->Stop();
            messages_.reset();
        }
        swarmkit::client::MessageSubscription subscription;
        subscription.subscriber_id = client_id_ + "-shell";
        subscription.target_id = drone_id_;
        for (std::size_t index = 2; index < tokens.size(); ++index) {
            subscription.topics.push_back(tokens[index]);
        }
        auto stream = client_.StartMessages(
            std::move(subscription),
            [this](const DataMessage& message) {
                std::lock_guard<std::mutex> lock(output_mutex_);
                std::cout << "[message] seq=" << message.sequence << " topic=" << message.topic
                          << " source=" << message.source_id << " target=" << message.target_id
                          << " payload=" << message.payload << "\n";
            },
            [this](const std::string& error) { PrintLine("[message error] " + error); });
        if (!stream.has_value()) {
            PrintLine("messages start FAILED: " + stream.error().ToString());
            return;
        }
        messages_.emplace(std::move(*stream));
        PrintLine("messages started");
    }

    [[nodiscard]] bool HandlePeers(const std::vector<std::string>& tokens) {
        const std::string command = Lower(tokens[0]);
        if (command == "peers") {
            DoPeers(tokens);
            return true;
        }
        if (command == "peer") {
            DoPeer(tokens);
            return true;
        }
        return false;
    }

    void DoPeers(const std::vector<std::string>& tokens) {
        const std::string action = tokens.size() >= 2 ? Lower(tokens[1]) : "list";
        if (action == "list") {
            PrintPeers(client_.ListDataPeers(tokens.size() >= 3 && Lower(tokens[2]) == "refresh"));
            return;
        }
        if (action == "refresh") {
            std::vector<std::string> ids;
            for (std::size_t index = 2; index < tokens.size(); ++index) {
                ids.push_back(tokens[index]);
            }
            PrintPeers(client_.RefreshDataPeers(ids));
            return;
        }
        PrintLine("usage: peers list [refresh] | peers refresh [ID...]");
    }

    void DoPeer(const std::vector<std::string>& tokens) {
        if (tokens.size() < 3) {
            PrintLine("usage: peer upsert ID ADDR [SECURITY] | peer remove ID");
            return;
        }
        const std::string action = Lower(tokens[1]);
        if (action == "remove") {
            PrintPeers(client_.RemoveDataPeer(tokens[2]));
            return;
        }
        if (action == "upsert") {
            if (tokens.size() < 4) {
                PrintLine("usage: peer upsert ID ADDR [SECURITY]");
                return;
            }
            DataPeerConfig peer;
            peer.drone_id = tokens[2];
            peer.address = tokens[3];
            if (tokens.size() >= 5) {
                peer.transport_security = tokens[4];
            }
            PrintPeers(client_.UpsertDataPeer(peer));
            return;
        }
        PrintLine("unknown peer action: " + action);
    }

    void PrintPeers(const swarmkit::client::DataPeerListResult& result) const {
        std::lock_guard<std::mutex> lock(output_mutex_);
        std::cout << "peers: " << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
        for (const auto& peer : result.peers) {
            std::cout << "  " << peer.drone_id << " addr=" << peer.address
                      << " security=" << peer.transport_security
                      << " state=" << PeerStateName(peer.state) << " rtt_ms=" << peer.round_trip_ms;
            if (!peer.message.empty()) {
                std::cout << " message=" << peer.message;
            }
            std::cout << "\n";
        }
    }

    [[nodiscard]] bool HandleArtifacts(const std::vector<std::string>& tokens) {
        if (Lower(tokens[0]) != "artifact") {
            return false;
        }
        if (tokens.size() < 2) {
            PrintLine(
                "usage: artifact "
                "upload|send|start|status|cancel|list|get|download|resume|announce");
            return true;
        }
        const std::string action = Lower(tokens[1]);
        if (action == "upload" || action == "send") {
            DoArtifactUpload(tokens, action == "send");
            return true;
        }
        if (action == "start") {
            DoArtifactStart(tokens);
            return true;
        }
        if (action == "status" || action == "cancel") {
            if (tokens.size() < 3) {
                PrintLine("usage: artifact " + action + " TRANSFER_ID");
                return true;
            }
            const auto result = action == "cancel" ? client_.CancelArtifactTransfer(tokens[2])
                                                   : client_.GetArtifactTransfer(tokens[2]);
            PrintTransferStatus("artifact " + action, result);
            return true;
        }
        if (action == "list") {
            ArtifactListOptions options;
            options.include_expired = HasToken(tokens, "--expired");
            PrintArtifactList(client_.ListArtifacts(options));
            return true;
        }
        if (action == "get" || action == "info") {
            if (tokens.size() < 3) {
                PrintLine("usage: artifact get ARTIFACT_ID");
                return true;
            }
            PrintArtifactResult("artifact get", client_.GetArtifact(tokens[2]), true);
            return true;
        }
        if (action == "download" || action == "resume") {
            if (tokens.size() < 4) {
                PrintLine("usage: artifact " + action + " ARTIFACT_ID PATH");
                return true;
            }
            const auto result = action == "resume"
                                    ? client_.ResumeArtifactDownload(tokens[2], tokens[3])
                                    : client_.DownloadArtifact(tokens[2], tokens[3]);
            PrintArtifactResult("artifact " + action, result, false);
            return true;
        }
        if (action == "announce") {
            DoArtifactAnnounce(tokens);
            return true;
        }
        PrintLine("unknown artifact action: " + action);
        return true;
    }

    void DoArtifactUpload(const std::vector<std::string>& tokens, bool routed) {
        if ((!routed && tokens.size() < 3) || (routed && tokens.size() < 4)) {
            PrintLine(routed ? "usage: artifact send PATH TARGET [CONTENT_TYPE]"
                             : "usage: artifact upload PATH [CONTENT_TYPE]");
            return;
        }
        ArtifactUpload upload;
        upload.file_path = tokens[2];
        upload.descriptor.filename = Basename(tokens[2]);
        upload.descriptor.content_type =
            routed && tokens.size() >= 5
                ? tokens[4]
                : (!routed && tokens.size() >= 4 ? tokens[3] : "application/octet-stream");
        if (routed) {
            upload.descriptor.target_id = tokens[3];
        }
        const auto result =
            routed ? client_.SendArtifactToDrone(upload) : client_.UploadArtifact(upload);
        PrintArtifactResult(routed ? "artifact send" : "artifact upload", result, true);
    }

    void DoArtifactStart(const std::vector<std::string>& tokens) {
        if (tokens.size() < 3) {
            PrintLine("usage: artifact start PATH [TARGET]");
            return;
        }
        ArtifactUpload upload;
        upload.file_path = tokens[2];
        upload.descriptor.filename = Basename(tokens[2]);
        upload.descriptor.content_type = "application/octet-stream";
        const bool route = tokens.size() >= 4;
        if (route) {
            upload.descriptor.target_id = tokens[3];
        }
        PrintTransferStatus("artifact start", client_.StartArtifactTransfer(upload, route));
    }

    void DoArtifactAnnounce(const std::vector<std::string>& tokens) {
        if (tokens.size() < 3) {
            PrintLine("usage: artifact announce ARTIFACT_ID [FILENAME] [CONTENT_TYPE]");
            return;
        }
        ArtifactDescriptor descriptor;
        descriptor.artifact_id = tokens[2];
        descriptor.source_id = client_id_;
        descriptor.filename = tokens.size() >= 4 ? tokens[3] : tokens[2];
        descriptor.content_type = tokens.size() >= 5 ? tokens[4] : "application/octet-stream";
        PrintArtifactResult("artifact announce", client_.AnnounceArtifact(std::move(descriptor)),
                            false);
    }

    void PrintArtifactResult(std::string_view label,
                             const swarmkit::client::ArtifactTransferResult& result,
                             bool print_descriptor) const {
        std::lock_guard<std::mutex> lock(output_mutex_);
        std::cout << label << ": " << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.descriptor.artifact_id.empty()) {
            std::cout << " artifact_id=" << result.descriptor.artifact_id;
        }
        if (!result.descriptor.sha256_hex.empty()) {
            std::cout << " sha256=" << result.descriptor.sha256_hex;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
        if (print_descriptor && !result.descriptor.artifact_id.empty()) {
            PrintDescriptor(result.descriptor);
        }
    }

    void PrintArtifactList(const swarmkit::client::ArtifactListResult& result) const {
        std::lock_guard<std::mutex> lock(output_mutex_);
        std::cout << "artifact list: " << (result.ok ? "OK" : "FAILED")
                  << " count=" << result.artifacts.size() << " total=" << result.total_count;
        if (!result.next_page_token.empty()) {
            std::cout << " next=" << result.next_page_token;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
        for (const auto& descriptor : result.artifacts) {
            PrintDescriptor(descriptor);
        }
    }

    void PrintDescriptor(const ArtifactDescriptor& descriptor) const {
        std::cout << "  artifact=" << descriptor.artifact_id << " source=" << descriptor.source_id
                  << " target=" << descriptor.target_id << " type=" << descriptor.content_type
                  << " size=" << descriptor.size_bytes << " file=" << descriptor.filename
                  << " sha256=" << descriptor.sha256_hex << "\n";
    }

    void PrintTransferStatus(std::string_view label,
                             const swarmkit::client::ArtifactTransferStatusResult& result) const {
        std::lock_guard<std::mutex> lock(output_mutex_);
        std::cout << label << ": " << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        std::cout << " transfer=" << result.transfer.transfer_id
                  << " state=" << TransferStateName(result.transfer.state)
                  << " bytes=" << result.transfer.bytes_transferred << "/"
                  << result.transfer.bytes_total;
        if (!result.transfer.descriptor.artifact_id.empty()) {
            std::cout << " artifact=" << result.transfer.descriptor.artifact_id;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
    }

    [[nodiscard]] bool HandleUploadSession(const std::vector<std::string>& tokens) {
        if (Lower(tokens[0]) != "upload") {
            return false;
        }
        if (tokens.size() < 2) {
            PrintLine("usage: upload create|chunk|status|commit|cancel ...");
            return true;
        }
        const std::string action = Lower(tokens[1]);
        if (action == "create") {
            if (tokens.size() < 4) {
                PrintLine("usage: upload create ARTIFACT_ID SIZE [FILENAME]");
                return true;
            }
            const auto size = ParseInt64(tokens[3], "size");
            if (!size.has_value()) {
                PrintLine("ERROR: " + size.error());
                return true;
            }
            ArtifactDescriptor descriptor;
            descriptor.artifact_id = tokens[2];
            descriptor.source_id = client_id_;
            descriptor.size_bytes = *size;
            descriptor.filename = tokens.size() >= 5 ? tokens[4] : tokens[2];
            descriptor.content_type = "application/octet-stream";
            PrintUploadSession("upload create", client_.CreateArtifactUpload(descriptor));
            return true;
        }
        if (action == "chunk") {
            if (tokens.size() < 6) {
                PrintLine("usage: upload chunk UPLOAD_ID OFFSET INDEX DATA");
                return true;
            }
            const auto offset = ParseInt64(tokens[3], "offset");
            const auto index = ParseInt(tokens[4], "index");
            if (!offset.has_value() || !index.has_value()) {
                PrintLine("ERROR: invalid chunk offset or index");
                return true;
            }
            PrintUploadSession(
                "upload chunk",
                client_.UploadArtifactChunk(tokens[2], JoinTail(tokens, 5), *offset, *index));
            return true;
        }
        if (action == "status" || action == "commit" || action == "cancel") {
            if (tokens.size() < 3) {
                PrintLine("usage: upload " + action + " UPLOAD_ID");
                return true;
            }
            if (action == "status") {
                PrintUploadSession("upload status", client_.GetUploadStatus(tokens[2]));
            } else if (action == "commit") {
                PrintUploadSession("upload commit", client_.CommitArtifactUpload(tokens[2]));
            } else {
                PrintUploadSession("upload cancel", client_.CancelUpload(tokens[2]));
            }
            return true;
        }
        PrintLine("unknown upload action: " + action);
        return true;
    }

    void PrintUploadSession(std::string_view label,
                            const swarmkit::client::ArtifactUploadSessionResult& result) const {
        std::lock_guard<std::mutex> lock(output_mutex_);
        const ArtifactUploadSession& upload = result.upload;
        std::cout << label << ": " << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        std::cout << " upload_id=" << upload.upload_id << " bytes=" << upload.bytes_received
                  << " next_chunk=" << upload.next_chunk_index
                  << " committed=" << BoolName(upload.committed);
        if (!result.descriptor.artifact_id.empty()) {
            std::cout << " artifact=" << result.descriptor.artifact_id;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
    }

    void RunController() {
        PrintControllerHelp();
        const bool had_telemetry = telemetry_.has_value();
        if (!had_telemetry) {
            StartTelemetryStream(2, false);
        }

        RawTerminal terminal{STDIN_FILENO};
        if (!terminal.active()) {
            PrintLine("controller FAILED: " + terminal.error());
            if (!had_telemetry && telemetry_) {
                telemetry_->Stop();
                telemetry_.reset();
            }
            return;
        }

        while (true) {
            const std::optional<ControllerKey> key = ReadControllerKey(STDIN_FILENO);
            if (!key.has_value()) {
                continue;
            }
            if (*key == ControllerKey::kEscape) {
                break;
            }
            HandleControllerKey(*key);
        }

        if (!had_telemetry && telemetry_) {
            telemetry_->Stop();
            telemetry_.reset();
        }
        PrintLine("controller exited");
    }

    void PrintControllerHelp() const {
        std::lock_guard<std::mutex> lock(output_mutex_);
        std::cout << "Controller mode. Esc exits.\n"
                  << "  ArrowUp/W forward, ArrowDown/S back\n"
                  << "  ArrowLeft/Q yaw left, ArrowRight/E yaw right\n"
                  << "  A/D strafe, R/F climb/descend, Space/X stop velocity, H hold\n"
                  << "  1 arm, 0 disarm, G guided, T takeoff(" << default_takeoff_alt_m_
                  << "m), L land, +/- speed\n";
    }

    void HandleControllerKey(ControllerKey key) {
        switch (key) {
            case ControllerKey::kHelp:
                PrintControllerHelp();
                return;
            case ControllerKey::kForward:
                SendVelocity(controller_speed_mps_, 0, 0, controller_pulse_ms_, true, "forward");
                PrintLine(LatestTelemetrySummary());
                return;
            case ControllerKey::kBackward:
                SendVelocity(-controller_speed_mps_, 0, 0, controller_pulse_ms_, true, "back");
                PrintLine(LatestTelemetrySummary());
                return;
            case ControllerKey::kStrafeLeft:
                SendVelocity(0, -controller_speed_mps_, 0, controller_pulse_ms_, true,
                             "strafe-left");
                PrintLine(LatestTelemetrySummary());
                return;
            case ControllerKey::kStrafeRight:
                SendVelocity(0, controller_speed_mps_, 0, controller_pulse_ms_, true,
                             "strafe-right");
                PrintLine(LatestTelemetrySummary());
                return;
            case ControllerKey::kClimb:
                SendVelocity(0, 0, -controller_climb_mps_, controller_pulse_ms_, true, "up");
                PrintLine(LatestTelemetrySummary());
                return;
            case ControllerKey::kDescend:
                SendVelocity(0, 0, controller_climb_mps_, controller_pulse_ms_, true, "down");
                PrintLine(LatestTelemetrySummary());
                return;
            case ControllerKey::kYawLeft:
                static_cast<void>(
                    SendCommand(NavCmd{CmdSetYaw{.yaw_deg = -controller_yaw_step_deg_,
                                                 .rate_deg_s = controller_yaw_rate_deg_s_,
                                                 .relative = true}},
                                "yaw-left"));
                PrintLine(LatestTelemetrySummary());
                return;
            case ControllerKey::kYawRight:
                static_cast<void>(
                    SendCommand(NavCmd{CmdSetYaw{.yaw_deg = controller_yaw_step_deg_,
                                                 .rate_deg_s = controller_yaw_rate_deg_s_,
                                                 .relative = true}},
                                "yaw-right"));
                PrintLine(LatestTelemetrySummary());
                return;
            case ControllerKey::kStop:
                SendVelocity(0, 0, 0, controller_pulse_ms_, true, "stop");
                PrintLine(LatestTelemetrySummary());
                return;
            case ControllerKey::kHold:
                static_cast<void>(SendCommand(NavCmd{CmdHoldPosition{}}, "hold"));
                PrintLine(LatestTelemetrySummary());
                return;
            case ControllerKey::kArm:
                PrintCommandResult("arm", client_.SendCommand(MakeEnvelope(FlightCmd{CmdArm{}})));
                return;
            case ControllerKey::kDisarm:
                PrintCommandResult("disarm",
                                   client_.SendCommand(MakeEnvelope(FlightCmd{CmdDisarm{}})));
                return;
            case ControllerKey::kGuided:
                PrintCommandResult(
                    "guided",
                    client_.SendCommand(MakeEnvelope(FlightCmd{CmdSetMode{.mode = "guided"}})));
                return;
            case ControllerKey::kTakeoff:
                PrintCommandResult("takeoff", client_.SendCommand(MakeEnvelope(FlightCmd{
                                                  CmdTakeoff{.alt_m = default_takeoff_alt_m_}})));
                return;
            case ControllerKey::kLand:
                PrintCommandResult("land", client_.SendCommand(MakeEnvelope(FlightCmd{CmdLand{}})));
                return;
            case ControllerKey::kSpeedUp:
                controller_speed_mps_ += 0.25F;
                PrintLine("speed=" + std::to_string(controller_speed_mps_));
                return;
            case ControllerKey::kSpeedDown:
                controller_speed_mps_ = std::max(0.25F, controller_speed_mps_ - 0.25F);
                PrintLine("speed=" + std::to_string(controller_speed_mps_));
                return;
            case ControllerKey::kUnknown:
            case ControllerKey::kEscape:
                return;
        }
    }

    Client client_;
    std::string config_address_{"127.0.0.1:50061"};
    std::string drone_id_;
    std::string client_id_;
    CommandPriority priority_{CommandPriority::kOperator};
    CommandWaitOptions wait_options_{};
    double default_takeoff_alt_m_{5.0};
    float controller_speed_mps_{2.0F};
    float controller_climb_mps_{1.0F};
    float controller_yaw_step_deg_{15.0F};
    float controller_yaw_rate_deg_s_{45.0F};
    int controller_pulse_ms_{700};
    std::uint64_t correlation_counter_{0};
    std::uint64_t goal_revision_{0};

    mutable std::mutex output_mutex_;
    mutable std::mutex telemetry_mutex_;
    swarmkit::core::TelemetryFrame latest_frame_;
    bool have_latest_frame_{false};
    bool telemetry_print_frames_{false};

    std::optional<swarmkit::client::Subscription> telemetry_;
    std::optional<swarmkit::client::Subscription> reports_;
    std::optional<swarmkit::client::Subscription> authority_watch_;
    std::optional<swarmkit::client::Subscription> messages_;
    std::optional<swarmkit::client::AuthoritySession> authority_session_;
};

[[nodiscard]] std::expected<AppOptions, std::string> ParseAppOptions(int argc, char** argv) {
    AppOptions options;
    options.config.client_id = "sdk-control";
    options.config.address = "127.0.0.1:50061";
    options.config.deadline_ms = 5000;

    options.config_path = OptionValue(argc, argv, "--config");
    if (!options.config_path.empty()) {
        auto loaded = swarmkit::client::LoadClientConfigFromFile(options.config_path);
        if (!loaded.has_value()) {
            return std::unexpected("failed to load client config: " + loaded.error().ToString());
        }
        options.config = std::move(*loaded);
    }

    options.config.ApplyEnvironment();

    if (const std::string address = OptionValue(argc, argv, "--addr"); !address.empty()) {
        options.config.address = address;
    }
    if (const std::string client_id = OptionValue(argc, argv, "--client-id"); !client_id.empty()) {
        options.config.client_id = client_id;
    }
    if (const std::string drone_id = OptionValue(argc, argv, "--drone"); !drone_id.empty()) {
        options.drone_id = drone_id;
    }
    if (const std::string deadline = OptionValue(argc, argv, "--deadline-ms"); !deadline.empty()) {
        const auto parsed = ParseInt(deadline, "--deadline-ms");
        if (!parsed.has_value()) {
            return std::unexpected(parsed.error());
        }
        options.config.deadline_ms = *parsed;
    }
    if (const std::string priority = OptionValue(argc, argv, "--priority"); !priority.empty()) {
        const auto parsed = ParsePriority(priority);
        if (!parsed.has_value()) {
            return std::unexpected(parsed.error());
        }
        options.config.priority = *parsed;
    }
    if (const std::string takeoff_alt = OptionValue(argc, argv, "--takeoff-alt");
        !takeoff_alt.empty()) {
        const auto parsed = ParseDouble(takeoff_alt, "--takeoff-alt");
        if (!parsed.has_value()) {
            return std::unexpected(parsed.error());
        }
        options.default_takeoff_alt_m = *parsed;
    }
    if (const std::string speed = OptionValue(argc, argv, "--speed"); !speed.empty()) {
        const auto parsed = ParseFloat(speed, "--speed");
        if (!parsed.has_value()) {
            return std::unexpected(parsed.error());
        }
        options.controller_speed_mps = *parsed;
    }
    if (const std::string climb = OptionValue(argc, argv, "--climb-speed"); !climb.empty()) {
        const auto parsed = ParseFloat(climb, "--climb-speed");
        if (!parsed.has_value()) {
            return std::unexpected(parsed.error());
        }
        options.controller_climb_mps = *parsed;
    }
    if (const std::string yaw_step = OptionValue(argc, argv, "--yaw-step"); !yaw_step.empty()) {
        const auto parsed = ParseFloat(yaw_step, "--yaw-step");
        if (!parsed.has_value()) {
            return std::unexpected(parsed.error());
        }
        options.controller_yaw_step_deg = *parsed;
    }
    if (const std::string yaw_rate = OptionValue(argc, argv, "--yaw-rate"); !yaw_rate.empty()) {
        const auto parsed = ParseFloat(yaw_rate, "--yaw-rate");
        if (!parsed.has_value()) {
            return std::unexpected(parsed.error());
        }
        options.controller_yaw_rate_deg_s = *parsed;
    }
    if (const std::string pulse = OptionValue(argc, argv, "--pulse-ms"); !pulse.empty()) {
        const auto parsed = ParseInt(pulse, "--pulse-ms");
        if (!parsed.has_value()) {
            return std::unexpected(parsed.error());
        }
        options.controller_pulse_ms = *parsed;
    }

    if (HasFlag(argc, argv, "--insecure")) {
        options.config.security.transport_security =
            swarmkit::core::TransportSecurityMode::kInsecure;
    } else {
        if (const std::string ca = OptionValue(argc, argv, "--ca-cert"); !ca.empty()) {
            options.config.security.root_ca_cert_path = ca;
        }
        if (const std::string cert = OptionValue(argc, argv, "--client-cert"); !cert.empty()) {
            options.config.security.cert_chain_path = cert;
        }
        if (const std::string key = OptionValue(argc, argv, "--client-key"); !key.empty()) {
            options.config.security.private_key_path = key;
        }
        if (const std::string server_name = OptionValue(argc, argv, "--server-name");
            !server_name.empty()) {
            options.config.security.server_authority_override = server_name;
        }
    }

    const auto validation = options.config.Validate();
    if (!validation.IsOk()) {
        return std::unexpected("invalid client config: " + validation.ToString());
    }
    if (options.drone_id.empty()) {
        return std::unexpected("--drone cannot be empty");
    }
    if (options.controller_pulse_ms <= 0) {
        return std::unexpected("--pulse-ms must be positive");
    }
    return options;
}

}  // namespace

int main(int argc, char** argv) {
    if (HasFlag(argc, argv, "--help") || HasFlag(argc, argv, "-h")) {
        PrintUsage(argv[0]);
        return EXIT_SUCCESS;
    }

    auto options = ParseAppOptions(argc, argv);
    if (!options.has_value()) {
        std::cerr << options.error() << "\n\n";
        PrintUsage(argv[0]);
        return EXIT_FAILURE;
    }

    ControlShell shell{std::move(*options)};
    return shell.Run();
}
