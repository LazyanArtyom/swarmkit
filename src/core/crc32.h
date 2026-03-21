// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstddef>
#include <cstdint>

namespace swarmkit::core {

/// Compute CRC-32 over a byte buffer.  Uses zlib if available, otherwise a software fallback.
[[nodiscard]] std::uint32_t Crc32Bytes(const void* data, std::size_t size) noexcept;

}  // namespace swarmkit::core
