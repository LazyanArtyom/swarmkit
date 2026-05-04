// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include "mavlink_common.h"
#include "swarmkit/core/result.h"

#include <netinet/in.h>
#include <sys/socket.h>

namespace swarmkit::agent::mavlink {

struct MavlinkDatagram {
    std::array<std::uint8_t, kUdpReceiveBufferBytes> bytes{};
    std::size_t size{};
    sockaddr_storage remote_addr{};
    socklen_t remote_len{};
};

class MavlinkUdpTransport {
   public:
    MavlinkUdpTransport() = default;
    ~MavlinkUdpTransport();

    MavlinkUdpTransport(const MavlinkUdpTransport&) = delete;
    MavlinkUdpTransport& operator=(const MavlinkUdpTransport&) = delete;

    [[nodiscard]] core::Result Bind(const std::string& bind_addr);
    void Close() noexcept;

    [[nodiscard]] std::optional<MavlinkDatagram> Receive();
    void RememberEndpoint(const sockaddr_storage& remote_addr, socklen_t remote_len);
    [[nodiscard]] bool WaitForEndpoint(std::chrono::milliseconds timeout);
    [[nodiscard]] core::Result Send(const std::uint8_t* bytes, std::uint16_t length);

   private:
    using SocketHandle = int;
    static constexpr SocketHandle kInvalidSocket = -1;

    [[nodiscard]] core::Result ConfigureReceiveTimeout() const;

    mutable std::mutex socket_mutex_;
    SocketHandle socket_{kInvalidSocket};

    std::mutex endpoint_mutex_;
    std::condition_variable endpoint_cv_;
    sockaddr_storage last_remote_addr_{};
    socklen_t last_remote_len_{};
    bool endpoint_known_{false};
};

}  // namespace swarmkit::agent::mavlink
