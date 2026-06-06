// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <string_view>

namespace swarmkit::core::internal {
namespace sha256_detail {

constexpr std::array<std::uint32_t, 64> kRoundConstants{
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU, 0x59f111f1U,
    0x923f82a4U, 0xab1c5ed5U, 0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
    0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U, 0xe49b69c1U, 0xefbe4786U,
    0x0fc19dc6U, 0x240ca1ccU, 0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
    0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U,
    0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
    0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U, 0xa2bfe8a1U, 0xa81a664bU,
    0xc24b8b70U, 0xc76c51a3U, 0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
    0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU,
    0x5b9cca4fU, 0x682e6ff3U, 0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
    0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};

[[nodiscard]] constexpr std::uint32_t Choose(std::uint32_t x, std::uint32_t y,
                                             std::uint32_t z) {
    return (x & y) ^ (~x & z);
}

[[nodiscard]] constexpr std::uint32_t Majority(std::uint32_t x, std::uint32_t y,
                                               std::uint32_t z) {
    return (x & y) ^ (x & z) ^ (y & z);
}

[[nodiscard]] constexpr std::uint32_t BigSigma0(std::uint32_t x) {
    return std::rotr(x, 2) ^ std::rotr(x, 13) ^ std::rotr(x, 22);
}

[[nodiscard]] constexpr std::uint32_t BigSigma1(std::uint32_t x) {
    return std::rotr(x, 6) ^ std::rotr(x, 11) ^ std::rotr(x, 25);
}

[[nodiscard]] constexpr std::uint32_t SmallSigma0(std::uint32_t x) {
    return std::rotr(x, 7) ^ std::rotr(x, 18) ^ (x >> 3);
}

[[nodiscard]] constexpr std::uint32_t SmallSigma1(std::uint32_t x) {
    return std::rotr(x, 17) ^ std::rotr(x, 19) ^ (x >> 10);
}

}  // namespace sha256_detail

class Sha256 {
   public:
    Sha256() = default;

    void Update(const void* input, std::size_t size) {
        const auto* bytes = static_cast<const std::uint8_t*>(input);
        for (std::size_t index = 0; index < size; ++index) {
            data_[data_len_++] = bytes[index];
            if (data_len_ == data_.size()) {
                Transform(data_.data());
                bit_len_ += 512;
                data_len_ = 0;
            }
        }
    }

    [[nodiscard]] std::array<std::uint8_t, 32> Final() {
        std::array<std::uint8_t, 32> hash{};
        std::size_t index = data_len_;

        data_[index++] = 0x80U;
        if (index > 56) {
            while (index < 64) {
                data_[index++] = 0;
            }
            Transform(data_.data());
            index = 0;
        }
        while (index < 56) {
            data_[index++] = 0;
        }

        bit_len_ += data_len_ * 8U;
        for (int shift = 56; shift >= 0; shift -= 8) {
            data_[index++] = static_cast<std::uint8_t>(bit_len_ >> shift);
        }
        Transform(data_.data());

        for (std::size_t word = 0; word < state_.size(); ++word) {
            hash[word * 4] = static_cast<std::uint8_t>(state_[word] >> 24);
            hash[(word * 4) + 1] = static_cast<std::uint8_t>(state_[word] >> 16);
            hash[(word * 4) + 2] = static_cast<std::uint8_t>(state_[word] >> 8);
            hash[(word * 4) + 3] = static_cast<std::uint8_t>(state_[word]);
        }
        return hash;
    }

   private:
    void Transform(const std::uint8_t* block) {
        using namespace sha256_detail;  // NOLINT(google-build-using-namespace)

        std::array<std::uint32_t, 64> schedule{};
        for (std::size_t index = 0; index < 16; ++index) {
            schedule[index] = (static_cast<std::uint32_t>(block[index * 4]) << 24) |
                              (static_cast<std::uint32_t>(block[(index * 4) + 1]) << 16) |
                              (static_cast<std::uint32_t>(block[(index * 4) + 2]) << 8) |
                              static_cast<std::uint32_t>(block[(index * 4) + 3]);
        }
        for (std::size_t index = 16; index < 64; ++index) {
            schedule[index] = SmallSigma1(schedule[index - 2]) + schedule[index - 7] +
                              SmallSigma0(schedule[index - 15]) + schedule[index - 16];
        }

        auto a = state_[0];
        auto b = state_[1];
        auto c = state_[2];
        auto d = state_[3];
        auto e = state_[4];
        auto f = state_[5];
        auto g = state_[6];
        auto h = state_[7];

        for (std::size_t index = 0; index < 64; ++index) {
            const std::uint32_t temp1 =
                h + BigSigma1(e) + Choose(e, f, g) + kRoundConstants[index] + schedule[index];
            const std::uint32_t temp2 = BigSigma0(a) + Majority(a, b, c);
            h = g;
            g = f;
            f = e;
            e = d + temp1;
            d = c;
            c = b;
            b = a;
            a = temp1 + temp2;
        }

        state_[0] += a;
        state_[1] += b;
        state_[2] += c;
        state_[3] += d;
        state_[4] += e;
        state_[5] += f;
        state_[6] += g;
        state_[7] += h;
    }

    std::array<std::uint8_t, 64> data_{};
    std::size_t data_len_{0};
    std::uint64_t bit_len_{0};
    std::array<std::uint32_t, 8> state_{0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
                                        0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U};
};

[[nodiscard]] inline std::string BytesToHex(const std::uint8_t* bytes, std::size_t size) {
    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (std::size_t index = 0; index < size; ++index) {
        out << std::setw(2) << static_cast<unsigned int>(bytes[index]);
    }
    return out.str();
}

[[nodiscard]] inline std::string Sha256Hex(std::string_view data) {
    Sha256 hasher;
    hasher.Update(data.data(), data.size());
    const auto hash = hasher.Final();
    return BytesToHex(hash.data(), hash.size());
}

[[nodiscard]] inline std::string Sha256FileHex(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open()) {
        return {};
    }
    Sha256 hasher;
    std::array<char, 64 * 1024> buffer{};
    while (input.good()) {
        input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const std::streamsize read = input.gcount();
        if (read > 0) {
            hasher.Update(buffer.data(), static_cast<std::size_t>(read));
        }
    }
    const auto hash = hasher.Final();
    return BytesToHex(hash.data(), hash.size());
}

}  // namespace swarmkit::core::internal
