#include "crc32.h"

#include <cstdint>

#if __has_include(<zlib.h>)
#include <zlib.h>
#endif

namespace swarmkit::core {

std::uint32_t Crc32Bytes(const void* data, std::size_t size) noexcept {
#if __has_include(<zlib.h>)
    return static_cast<std::uint32_t>(
        ::crc32(0u, static_cast<const Bytef*>(data), static_cast<uInt>(size)));
#else
    constexpr std::uint32_t kCrc32Initial = 0xFFFFFFFFu;
    constexpr std::uint32_t kCrc32Polynomial = 0xEDB88320u;

    const auto* bytes = static_cast<const std::uint8_t*>(data);
    std::uint32_t crc32_value = kCrc32Initial;
    for (std::size_t index = 0; index < size; ++index) {
        crc32_value ^= bytes[index];
        for (int bit_index = 0; bit_index < 8; ++bit_index) {
            const std::uint32_t mask = -(crc32_value & 1u);
            crc32_value = (crc32_value >> 1u) ^ (kCrc32Polynomial & mask);
        }
    }
    return ~crc32_value;
#endif
}

}  // namespace swarmkit::core