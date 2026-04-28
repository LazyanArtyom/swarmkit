// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "swarmkit/core/logger.h"

namespace fs = std::filesystem;

namespace {

class RecordingLogBackend final : public swarmkit::core::ILogBackend {
   public:
    [[nodiscard]] bool IsEnabled(swarmkit::core::LogLevel /*level*/) const override {
        return true;
    }

    void Log(swarmkit::core::LogLevel level, std::string_view message) override {
        std::lock_guard<std::mutex> lock(mutex_);
        entries_.emplace_back(level, message);
    }

    void Flush() override {
        ++flush_count_;
    }

    [[nodiscard]] std::size_t EntryCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return entries_.size();
    }

    [[nodiscard]] std::string MessageAt(std::size_t index) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return entries_.at(index).second;
    }

    [[nodiscard]] int FlushCount() const {
        return flush_count_;
    }

   private:
    mutable std::mutex mutex_;
    int flush_count_{0};
    std::vector<std::pair<swarmkit::core::LogLevel, std::string>> entries_;
};

}  // namespace

TEST_CASE("Logger validates config and writes to rotating file", "[core][logger]") {
    swarmkit::core::LoggerConfig invalid_config;
    invalid_config.sink_type = swarmkit::core::LogSinkType::kRotatingFile;
    invalid_config.log_file_path.clear();
    CHECK_FALSE(swarmkit::core::ValidateLoggerConfig(invalid_config).IsOk());

    const fs::path kLogPath = fs::temp_directory_path() / "swarmkit_test.log";

    swarmkit::core::LoggerConfig config;
    config.sink_type = swarmkit::core::LogSinkType::kRotatingFile;
    config.log_file_path = kLogPath.string();
    config.level = swarmkit::core::LogLevel::kInfo;
    config.flush_level = swarmkit::core::LogLevel::kInfo;

    swarmkit::core::Logger::Init(config);
    swarmkit::core::Logger::Info("test log entry");
    swarmkit::core::Logger::Flush();
    swarmkit::core::Logger::Shutdown();

    std::ifstream input(kLogPath);
    REQUIRE(input.is_open());
    std::string contents((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    CHECK(contents.find("test log entry") != std::string::npos);

    fs::remove(kLogPath);
}

TEST_CASE("Logger supports custom backends", "[core][logger]") {
    auto backend = std::make_shared<RecordingLogBackend>();
    swarmkit::core::Logger::SetBackend(backend);

    swarmkit::core::Logger::Info("captured info");
    swarmkit::core::Logger::WarnFmt("captured {}", "warning");
    swarmkit::core::Logger::Flush();
    swarmkit::core::Logger::Shutdown();

    CHECK(backend->EntryCount() == 2);
    CHECK(backend->MessageAt(0) == "captured info");
    CHECK(backend->MessageAt(1) == "captured warning");
    CHECK(backend->FlushCount() >= 1);
}
