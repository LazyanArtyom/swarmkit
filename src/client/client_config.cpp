// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "swarmkit/client/client.h"

#include <algorithm>
#include <string>
#include <string_view>

#include "env_utils.h"
#include "security_utils.h"

namespace swarmkit::client {
namespace {

using core::internal::GetEnvValue;
using core::internal::IsValidPriority;
using core::internal::LooksLikeAddress;
using core::internal::ParseBoolValue;
using core::internal::ParseIntValue;
using core::internal::ParsePriorityValue;

constexpr std::string_view kClientEnvAddress = "ADDRESS";
constexpr std::string_view kClientEnvId = "CLIENT_ID";
constexpr std::string_view kClientEnvDeadlineMs = "DEADLINE_MS";
constexpr std::string_view kClientEnvPriority = "PRIORITY";
constexpr std::string_view kClientEnvRetryMaxAttempts = "RETRY_MAX_ATTEMPTS";
constexpr std::string_view kClientEnvRetryInitialBackoffMs = "RETRY_INITIAL_BACKOFF_MS";
constexpr std::string_view kClientEnvRetryMaxBackoffMs = "RETRY_MAX_BACKOFF_MS";
constexpr std::string_view kClientEnvStreamReconnectEnabled = "STREAM_RECONNECT_ENABLED";
constexpr std::string_view kClientEnvStreamReconnectInitialBackoffMs =
    "STREAM_RECONNECT_INITIAL_BACKOFF_MS";
constexpr std::string_view kClientEnvStreamReconnectMaxBackoffMs =
    "STREAM_RECONNECT_MAX_BACKOFF_MS";
constexpr std::string_view kClientEnvStreamReconnectMaxAttempts = "STREAM_RECONNECT_MAX_ATTEMPTS";
constexpr std::string_view kClientEnvRootCaCertPath = "ROOT_CA_CERT_PATH";
constexpr std::string_view kClientEnvClientCertChainPath = "CLIENT_CERT_CHAIN_PATH";
constexpr std::string_view kClientEnvClientPrivateKeyPath = "CLIENT_PRIVATE_KEY_PATH";
constexpr std::string_view kClientEnvServerAuthorityOverride = "SERVER_AUTHORITY_OVERRIDE";
constexpr std::string_view kClientEnvTransportSecurity = "TRANSPORT_SECURITY";

}  // namespace

core::TransportSecurityMode ClientSecurityConfig::EffectiveTransportSecurity() const {
    if (transport_security != core::TransportSecurityMode::kAuto) {
        return transport_security;
    }
    if (root_ca_cert_path.empty() && cert_chain_path.empty() && private_key_path.empty()) {
        return core::TransportSecurityMode::kInsecure;
    }
    if (!root_ca_cert_path.empty() && cert_chain_path.empty() && private_key_path.empty()) {
        return core::TransportSecurityMode::kTls;
    }
    return core::TransportSecurityMode::kMutualTls;
}

core::Result ClientSecurityConfig::Validate() const {
    const core::TransportSecurityMode mode = EffectiveTransportSecurity();
    if (mode == core::TransportSecurityMode::kInsecure) {
        return core::Result::Success();
    }
    if (!core::internal::FileExists(root_ca_cert_path)) {
        return core::Result::Rejected(
            "security.root_ca_cert_path must point to an existing file for TLS/mTLS");
    }
    if (mode == core::TransportSecurityMode::kMutualTls) {
        if (!core::internal::FileExists(cert_chain_path)) {
            return core::Result::Rejected(
                "security.cert_chain_path must point to an existing file for mTLS");
        }
        if (!core::internal::FileExists(private_key_path)) {
            return core::Result::Rejected(
                "security.private_key_path must point to an existing file for mTLS");
        }
    }
    return core::Result::Success();
}

core::Result ClientConfig::Validate() const {
    if (!LooksLikeAddress(address)) {
        return core::Result::Rejected("client address must be in host:port format");
    }
    if (client_id.empty()) {
        return core::Result::Rejected("client_id must not be empty");
    }
    if (deadline_ms < 0) {
        return core::Result::Rejected("deadline_ms must be >= 0");
    }
    if (!IsValidPriority(priority)) {
        return core::Result::Rejected("priority is not a supported CommandPriority");
    }
    if (retry_policy.max_attempts <= 0) {
        return core::Result::Rejected("retry_policy.max_attempts must be > 0");
    }
    if (retry_policy.initial_backoff_ms <= 0 || retry_policy.max_backoff_ms <= 0) {
        return core::Result::Rejected("retry_policy backoff values must be > 0");
    }
    if (retry_policy.initial_backoff_ms > retry_policy.max_backoff_ms) {
        return core::Result::Rejected(
            "retry_policy.initial_backoff_ms must be <= retry_policy.max_backoff_ms");
    }
    if (stream_reconnect_policy.initial_backoff_ms <= 0 ||
        stream_reconnect_policy.max_backoff_ms <= 0) {
        return core::Result::Rejected("stream_reconnect_policy backoff values must be > 0");
    }
    if (stream_reconnect_policy.initial_backoff_ms > stream_reconnect_policy.max_backoff_ms) {
        return core::Result::Rejected(
            "stream_reconnect_policy.initial_backoff_ms must be <= "
            "stream_reconnect_policy.max_backoff_ms");
    }
    if (stream_reconnect_policy.max_attempts < 0) {
        return core::Result::Rejected("stream_reconnect_policy.max_attempts must be >= 0");
    }
    return security.Validate();
}

void ClientConfig::ApplyEnvironment(std::string_view prefix) {
    const auto kApplyIntEnv = [&](std::string_view suffix, int* out) {
        const auto kValue = GetEnvValue(std::string(prefix) + std::string(suffix));
        if (!kValue.has_value()) {
            return;
        }
        const auto kParsed = ParseIntValue(*kValue, suffix);
        if (kParsed.has_value()) {
            *out = *kParsed;
        }
    };

    const auto kAddress = GetEnvValue(std::string(prefix) + std::string(kClientEnvAddress));
    if (kAddress.has_value()) {
        address = *kAddress;
    }

    const auto kClientId = GetEnvValue(std::string(prefix) + std::string(kClientEnvId));
    if (kClientId.has_value()) {
        client_id = *kClientId;
    }

    const auto kPriority = GetEnvValue(std::string(prefix) + std::string(kClientEnvPriority));
    if (kPriority.has_value()) {
        const auto kParsed = ParsePriorityValue(*kPriority, kClientEnvPriority);
        if (kParsed.has_value()) {
            priority = *kParsed;
        }
    }

    const auto kApplyBoolEnv = [&](std::string_view suffix, bool* out) {
        const auto kValue = GetEnvValue(std::string(prefix) + std::string(suffix));
        if (!kValue.has_value()) {
            return;
        }
        const auto kParsed = ParseBoolValue(*kValue, suffix);
        if (kParsed.has_value()) {
            *out = *kParsed;
        }
    };

    kApplyIntEnv(kClientEnvDeadlineMs, &deadline_ms);
    kApplyIntEnv(kClientEnvRetryMaxAttempts, &retry_policy.max_attempts);
    kApplyIntEnv(kClientEnvRetryInitialBackoffMs, &retry_policy.initial_backoff_ms);
    kApplyIntEnv(kClientEnvRetryMaxBackoffMs, &retry_policy.max_backoff_ms);
    kApplyBoolEnv(kClientEnvStreamReconnectEnabled, &stream_reconnect_policy.enabled);
    kApplyIntEnv(kClientEnvStreamReconnectInitialBackoffMs,
                 &stream_reconnect_policy.initial_backoff_ms);
    kApplyIntEnv(kClientEnvStreamReconnectMaxBackoffMs, &stream_reconnect_policy.max_backoff_ms);
    kApplyIntEnv(kClientEnvStreamReconnectMaxAttempts, &stream_reconnect_policy.max_attempts);

    if (const auto kValue =
            GetEnvValue(std::string(prefix) + std::string(kClientEnvRootCaCertPath));
        kValue.has_value()) {
        security.root_ca_cert_path = *kValue;
    }
    if (const auto kValue =
            GetEnvValue(std::string(prefix) + std::string(kClientEnvClientCertChainPath));
        kValue.has_value()) {
        security.cert_chain_path = *kValue;
    }
    if (const auto kValue =
            GetEnvValue(std::string(prefix) + std::string(kClientEnvClientPrivateKeyPath));
        kValue.has_value()) {
        security.private_key_path = *kValue;
    }
    if (const auto kValue =
            GetEnvValue(std::string(prefix) + std::string(kClientEnvServerAuthorityOverride));
        kValue.has_value()) {
        security.server_authority_override = *kValue;
    }
    if (const auto kValue =
            GetEnvValue(std::string(prefix) + std::string(kClientEnvTransportSecurity));
        kValue.has_value()) {
        const auto parsed = core::ParseTransportSecurityMode(*kValue);
        if (parsed.has_value()) {
            security.transport_security = *parsed;
        }
    }
}
}  // namespace swarmkit::client
