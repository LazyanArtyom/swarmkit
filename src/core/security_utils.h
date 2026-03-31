// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <filesystem>
#include <fstream>
#include <string>

#include "swarmkit/core/result.h"

namespace swarmkit::core::internal {

[[nodiscard]] inline Result ReadTextFile(const std::string& path, std::string* out_contents) {
    if (out_contents == nullptr) {
        return Result::Failed("file output is null");
    }

    std::ifstream input(path, std::ios::binary);
    if (!input.is_open()) {
        return Result::Failed("failed to open file '" + path + "'");
    }

    out_contents->assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
    if (input.bad()) {
        return Result::Failed("failed to read file '" + path + "'");
    }
    return Result::Success();
}

[[nodiscard]] inline bool FileExists(const std::string& path) {
    if (path.empty()) {
        return false;
    }
    std::error_code error_code;
    return std::filesystem::exists(path, error_code);
}

[[nodiscard]] inline std::string ResolveConfigRelativePath(const std::string& config_path,
                                                           const std::string& candidate_path) {
    if (candidate_path.empty()) {
        return {};
    }

    const std::filesystem::path kCandidate{candidate_path};
    if (kCandidate.is_absolute()) {
        return candidate_path;
    }

    const std::filesystem::path kConfigPath{config_path};
    const std::filesystem::path kResolved = kConfigPath.parent_path() / kCandidate;
    return kResolved.lexically_normal().string();
}

}  // namespace swarmkit::core::internal
