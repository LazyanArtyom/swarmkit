// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <yaml-cpp/yaml.h>

#include <expected>
#include <optional>
#include <string>
#include <string_view>

#include "swarmkit/core/result.h"

namespace swarmkit::core::yaml {

[[nodiscard]] inline std::expected<YAML::Node, Result> LoadYamlFile(const std::string& path) {
    try {
        return YAML::LoadFile(path);
    } catch (const YAML::Exception& error) {
        return std::unexpected(
            Result::Failed("failed to load YAML config '" + path + "': " + error.what()));
    }
}

[[nodiscard]] inline YAML::Node SelectSection(const YAML::Node& document,
                                              std::string_view section_name) {
    if (!document || !document.IsMap()) {
        return document;
    }

    const YAML::Node section = document[std::string(section_name)];
    return section ? section : document;
}

template <typename T>
[[nodiscard]] inline std::expected<std::optional<T>, Result> ReadOptionalScalar(
    const YAML::Node& node, std::string_view key) {
    if (!node || !node.IsMap()) {
        return std::optional<T>{};
    }

    YAML::Node child = node[std::string(key)];
    if (!child) {
        return std::optional<T>{};
    }

    try {
        return child.as<T>();
    } catch (const YAML::Exception& error) {
        return std::unexpected(
            Result::Rejected("invalid YAML field '" + std::string(key) + "': " + error.what()));
    }
}

[[nodiscard]] inline std::expected<YAML::Node, Result> ReadRequiredMap(const YAML::Node& node,
                                                                       std::string_view key) {
    if (!node || !node.IsMap()) {
        return std::unexpected(Result::Rejected("YAML root must be a map"));
    }

    YAML::Node child = node[std::string(key)];
    if (!child) {
        return std::unexpected(Result::Rejected("missing YAML map '" + std::string(key) + "'"));
    }
    if (!child.IsMap()) {
        return std::unexpected(
            Result::Rejected("YAML field '" + std::string(key) + "' must be a map"));
    }
    return child;
}

[[nodiscard]] inline std::expected<YAML::Node, Result> ReadRequiredSequence(const YAML::Node& node,
                                                                            std::string_view key) {
    if (!node || !node.IsMap()) {
        return std::unexpected(Result::Rejected("YAML root must be a map"));
    }

    YAML::Node child = node[std::string(key)];
    if (!child) {
        return std::unexpected(
            Result::Rejected("missing YAML sequence '" + std::string(key) + "'"));
    }
    if (!child.IsSequence()) {
        return std::unexpected(
            Result::Rejected("YAML field '" + std::string(key) + "' must be a sequence"));
    }
    return child;
}

}  // namespace swarmkit::core::yaml
