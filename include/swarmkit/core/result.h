#pragma once

#include <string>

namespace swarmkit::core {

enum class StatusCode {
    kOk,
    kRejected,
    kFailed,
};

struct Result {
    StatusCode code{StatusCode::kOk};
    std::string message;

    [[nodiscard]] static Result Ok(std::string msg = {}) { return {StatusCode::kOk, std::move(msg)}; }
    [[nodiscard]] static Result Rejected(std::string msg) { return {StatusCode::kRejected, std::move(msg)}; }
    [[nodiscard]] static Result Failed(std::string msg) { return {StatusCode::kFailed, std::move(msg)}; }

    [[nodiscard]] bool ok() const { return code == StatusCode::kOk; }
};

} // namespace swarmkit::core
