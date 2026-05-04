// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "mavlink_udp_transport.h"

#include <cstdint>
#include <string>

#include <arpa/inet.h>
#include <sys/time.h>
#include <unistd.h>

namespace swarmkit::agent::mavlink {

MavlinkUdpTransport::~MavlinkUdpTransport() {
    Close();
}

core::Result MavlinkUdpTransport::Bind(const std::string& bind_addr) {
    const auto host_port = SplitHostPort(bind_addr);
    if (!host_port.has_value()) {
        return core::Result::Rejected("mavlink bind_addr must be in host:port format");
    }

    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (socket_ != kInvalidSocket) {
        return core::Result::Success();
    }

    socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_ == kInvalidSocket) {
        return core::Result::Failed("failed to create MAVLink UDP socket");
    }

    if (const core::Result timeout_result = ConfigureReceiveTimeout(); !timeout_result.IsOk()) {
        close(socket_);
        socket_ = kInvalidSocket;
        return timeout_result;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(host_port->second);
    if (inet_pton(AF_INET, host_port->first.c_str(), &address.sin_addr) != 1) {
        close(socket_);
        socket_ = kInvalidSocket;
        return core::Result::Rejected("mavlink bind host must be an IPv4 address");
    }

    if (bind(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        close(socket_);
        socket_ = kInvalidSocket;
        return core::Result::Failed("failed to bind MAVLink UDP socket to " + bind_addr);
    }

    return core::Result::Success();
}

void MavlinkUdpTransport::Close() noexcept {
    SocketHandle socket_to_close = kInvalidSocket;
    {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        socket_to_close = socket_;
        socket_ = kInvalidSocket;
    }
    if (socket_to_close != kInvalidSocket) {
        close(socket_to_close);
    }
}

std::optional<MavlinkDatagram> MavlinkUdpTransport::Receive() {
    SocketHandle socket_snapshot = kInvalidSocket;
    {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        socket_snapshot = socket_;
    }
    if (socket_snapshot == kInvalidSocket) {
        return std::nullopt;
    }

    MavlinkDatagram datagram;
    datagram.remote_len = sizeof(datagram.remote_addr);
    const auto byte_count =
        recvfrom(socket_snapshot, datagram.bytes.data(), datagram.bytes.size(), 0,
                 reinterpret_cast<sockaddr*>(&datagram.remote_addr), &datagram.remote_len);
    if (byte_count <= 0) {
        return std::nullopt;
    }
    datagram.size = static_cast<std::size_t>(byte_count);
    return datagram;
}

void MavlinkUdpTransport::RememberEndpoint(const sockaddr_storage& remote_addr,
                                           socklen_t remote_len) {
    {
        std::lock_guard<std::mutex> lock(endpoint_mutex_);
        last_remote_addr_ = remote_addr;
        last_remote_len_ = remote_len;
        endpoint_known_ = true;
    }
    endpoint_cv_.notify_all();
}

bool MavlinkUdpTransport::WaitForEndpoint(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(endpoint_mutex_);
    return endpoint_cv_.wait_for(lock, timeout, [&] { return endpoint_known_; });
}

core::Result MavlinkUdpTransport::Send(const std::uint8_t* bytes, std::uint16_t length) {
    if (bytes == nullptr || length == 0) {
        return core::Result::Rejected("empty MAVLink packet");
    }

    SocketHandle socket_snapshot = kInvalidSocket;
    {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        socket_snapshot = socket_;
    }
    if (socket_snapshot == kInvalidSocket) {
        return core::Result::Rejected("MAVLink UDP socket is not open");
    }

    sockaddr_storage remote_addr{};
    socklen_t remote_len{};
    {
        std::lock_guard<std::mutex> lock(endpoint_mutex_);
        if (!endpoint_known_) {
            return core::Result::Rejected("no MAVLink peer endpoint known yet");
        }
        remote_addr = last_remote_addr_;
        remote_len = last_remote_len_;
    }

    const auto sent = sendto(socket_snapshot, reinterpret_cast<const char*>(bytes), length, 0,
                             reinterpret_cast<const sockaddr*>(&remote_addr), remote_len);
    if (sent != static_cast<ssize_t>(length)) {
        return core::Result::Failed("failed to send MAVLink UDP packet: sent=" +
                                    std::to_string(sent) +
                                    " expected=" + std::to_string(length));
    }
    return core::Result::Success();
}

core::Result MavlinkUdpTransport::ConfigureReceiveTimeout() const {
    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = static_cast<suseconds_t>(
        static_cast<std::int64_t>(kUdpReceiveTimeoutMs) *
        (kMicrosecondsPerSecond / kMillisecondsPerSecond));
    if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0) {
        return core::Result::Failed("failed to configure UDP receive timeout");
    }
    return core::Result::Success();
}

}  // namespace swarmkit::agent::mavlink
