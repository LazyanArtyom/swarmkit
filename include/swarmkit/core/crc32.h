#ifndef SWARMKIT_CORE_CRC32_H_
#define SWARMKIT_CORE_CRC32_H_

#include <cstddef>
#include <cstdint>

namespace swarmkit::core {

std::uint32_t Crc32Bytes(const void* data, std::size_t size) noexcept;

}  // namespace swarmkit::core
#endif  // SWARMKIT_CORE_CRC32_H_