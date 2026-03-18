#pragma once

#include <cstddef>
#include <cstdint>

namespace swarmkit::core {

// CRC-32 checksum.  Uses zlib if available, otherwise software fallback.
std::uint32_t Crc32Bytes(const void* data, std::size_t size) noexcept;

}  // namespace swarmkit::core
