// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <grpcpp/grpcpp.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "../src/agent/server_test_support.h"
#include "swarmkit/agent/backend.h"
#include "swarmkit/core/result.h"
#include "swarmkit/core/telemetry.h"

namespace swarmkit::testsupport {

template <typename Predicate>
[[nodiscard]] inline bool WaitUntil(Predicate&& predicate, std::chrono::milliseconds timeout,
                                    std::chrono::milliseconds poll_interval =
                                        std::chrono::milliseconds{10}) {
    const auto kDeadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < kDeadline) {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(poll_interval);
    }
    return predicate();
}

class RecordingBackend final : public agent::IDroneBackend {
   public:
    using ExecuteHandler = std::function<core::Result(const commands::CommandEnvelope&)>;

    struct ExecuteRecord {
        commands::CommandEnvelope envelope;
    };

    [[nodiscard]] core::Result Execute(const commands::CommandEnvelope& envelope) override {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            execute_records_.push_back(ExecuteRecord{.envelope = envelope});
        }
        if (execute_handler_) {
            return execute_handler_(envelope);
        }
        return core::Result::Success("executed");
    }

    [[nodiscard]] core::Result StartTelemetry(const std::string& drone_id, int rate_hertz,
                                              TelemetryCallback callback) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (telemetry_streams_.contains(drone_id)) {
            return core::Result::Rejected("telemetry already active");
        }

        TelemetryStream stream;
        stream.rate_hertz = rate_hertz;
        stream.callback = std::move(callback);
        telemetry_streams_.emplace(drone_id, std::move(stream));
        telemetry_start_count_[drone_id] += 1;
        return core::Result::Success();
    }

    [[nodiscard]] core::Result StopTelemetry(const std::string& drone_id) override {
        std::lock_guard<std::mutex> lock(mutex_);
        telemetry_streams_.erase(drone_id);
        telemetry_stop_count_[drone_id] += 1;
        return core::Result::Success();
    }

    void SetExecuteHandler(ExecuteHandler handler) {
        execute_handler_ = std::move(handler);
    }

    void EmitTelemetry(const std::string& drone_id, core::TelemetryFrame frame) {
        TelemetryCallback callback;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto iter = telemetry_streams_.find(drone_id);
            if (iter == telemetry_streams_.end()) {
                return;
            }
            callback = iter->second.callback;
        }
        callback(frame);
    }

    [[nodiscard]] std::size_t ExecuteCallCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return execute_records_.size();
    }

    [[nodiscard]] ExecuteRecord ExecuteCallAt(std::size_t index) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return execute_records_.at(index);
    }

    [[nodiscard]] bool HasTelemetryStream(const std::string& drone_id) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return telemetry_streams_.contains(drone_id);
    }

    [[nodiscard]] int TelemetryStartCount(const std::string& drone_id) const {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto iter = telemetry_start_count_.find(drone_id);
        return iter == telemetry_start_count_.end() ? 0 : iter->second;
    }

    [[nodiscard]] int TelemetryStopCount(const std::string& drone_id) const {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto iter = telemetry_stop_count_.find(drone_id);
        return iter == telemetry_stop_count_.end() ? 0 : iter->second;
    }

    [[nodiscard]] int TelemetryRate(const std::string& drone_id) const {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto iter = telemetry_streams_.find(drone_id);
        return iter == telemetry_streams_.end() ? 0 : iter->second.rate_hertz;
    }

   private:
    struct TelemetryStream {
        int rate_hertz{0};
        TelemetryCallback callback;
    };

    mutable std::mutex mutex_;
    std::vector<ExecuteRecord> execute_records_;
    std::unordered_map<std::string, TelemetryStream> telemetry_streams_;
    std::unordered_map<std::string, int> telemetry_start_count_;
    std::unordered_map<std::string, int> telemetry_stop_count_;
    ExecuteHandler execute_handler_;
};

class AgentServerHarness {
   public:
    AgentServerHarness()
        : AgentServerHarness(MakeDefaultConfig()) {}

    explicit AgentServerHarness(agent::AgentConfig config) {
        auto backend = std::make_unique<RecordingBackend>();
        backend_ = backend.get();
        service_ = agent::internal::MakeAgentServiceForTesting(config, std::move(backend));

        grpc::ServerBuilder builder;
        int selected_port = 0;
        builder.AddListeningPort("127.0.0.1:0", grpc::InsecureServerCredentials(), &selected_port);
        builder.RegisterService(service_.get());
        server_ = builder.BuildAndStart();
        address_ = "127.0.0.1:" + std::to_string(selected_port);
    }

    ~AgentServerHarness() {
        if (server_) {
            server_->Shutdown();
        }
    }

    AgentServerHarness(const AgentServerHarness&) = delete;
    AgentServerHarness& operator=(const AgentServerHarness&) = delete;

    [[nodiscard]] const std::string& address() const {
        return address_;
    }

    [[nodiscard]] RecordingBackend& backend() const {
        return *backend_;
    }

   private:
    [[nodiscard]] static agent::AgentConfig MakeDefaultConfig() {
        agent::AgentConfig config;
        config.agent_id = "test-agent";
        config.bind_addr = "127.0.0.1:0";
        return config;
    }

    RecordingBackend* backend_{nullptr};
    std::unique_ptr<grpc::Service> service_;
    std::unique_ptr<grpc::Server> server_;
    std::string address_;
};

}  // namespace swarmkit::testsupport
