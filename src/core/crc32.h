#pragma once

#include <cstddef>
#include <cstdint>

namespace swarmkit::core {

/// Compute CRC-32 over a byte buffer.  Uses zlib if available, otherwise a software fallback.
[[nodiscard]] std::uint32_t Crc32Bytes(const void* data, std::size_t size) noexcept;

}  // namespace swarmkit::core
