#include <motioncam/Decoder.hpp>
#include <motioncam/Mc24.hpp>

#include <algorithm>
#include <array>
#include <cstring>
#include <utility>

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#include <nmmintrin.h>
#if defined(_MSC_VER)
#include <intrin.h>
#endif
#endif

namespace motioncam {
namespace {
uint16_t u16(const uint8_t *p) { return uint16_t(p[0]) | uint16_t(p[1]) << 8; }
uint32_t u32(const uint8_t *p) { return uint32_t(u16(p)) | uint32_t(u16(p + 2)) << 16; }
void require(bool ok, const char *message) {
    if (!ok)
        throw IOException(message);
}
} // namespace
namespace mc24 {
uint32_t crc32cPortable(const uint8_t *data, size_t size) {
    static const auto table = [] {
        std::array<std::array<uint32_t, 256>, 8> t{};
        for (unsigned i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (unsigned j = 0; j < 8; ++j)
                c = (c >> 1) ^ ((c & 1) ? 0x82f63b78u : 0);
            t[0][i] = c;
        }
        for (unsigned slice = 1; slice < 8; ++slice)
            for (unsigned i = 0; i < 256; ++i) {
                const auto c = t[slice - 1][i];
                t[slice][i] = t[0][c & 255] ^ (c >> 8);
            }
        return t;
    }();
    uint32_t crc = ~uint32_t{0};
    while (size >= 8) {
        const uint32_t a = u32(data) ^ crc, b = u32(data + 4);
        crc = table[7][a & 255] ^ table[6][(a >> 8) & 255] ^ table[5][(a >> 16) & 255] ^
              table[4][a >> 24] ^ table[3][b & 255] ^ table[2][(b >> 8) & 255] ^
              table[1][(b >> 16) & 255] ^ table[0][b >> 24];
        data += 8;
        size -= 8;
    }
    while (size--)
        crc = table[0][(crc ^ *data++) & 255] ^ (crc >> 8);
    return ~crc;
}
namespace {
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#if defined(__GNUC__) || defined(__clang__)
__attribute__((target("sse4.2")))
#endif
uint32_t crc32cHardware(const uint8_t *data, size_t size) {
    uint32_t crc = ~uint32_t{0};
#if defined(__x86_64__) || defined(_M_X64)
    uint64_t wide = crc;
    while (size >= 8) {
        uint64_t word;
        std::memcpy(&word, data, sizeof(word));
        wide = _mm_crc32_u64(wide, word);
        data += 8;
        size -= 8;
    }
    crc = static_cast<uint32_t>(wide);
#else
    while (size >= 4) {
        uint32_t word;
        std::memcpy(&word, data, sizeof(word));
        crc = _mm_crc32_u32(crc, word);
        data += 4;
        size -= 4;
    }
#endif
    while (size--)
        crc = _mm_crc32_u8(crc, *data++);
    return ~crc;
}
#endif

// Each width is constant within a block. Specializing it removes the inner
// byte-gather loops and lets the compiler combine adjacent field extraction.
template <unsigned Bits, size_t I> uint16_t unpackField(const uint8_t *data) {
    constexpr unsigned bit = I * Bits, shift = bit & 7;
    constexpr unsigned bytes = (Bits + shift + 7) / 8;
    const auto p = data + bit / 8;
    uint32_t word = 0;
    if constexpr (bytes >= 1)
        word = p[0];
    if constexpr (bytes >= 2)
        word |= uint32_t(p[1]) << 8;
    if constexpr (bytes >= 3)
        word |= uint32_t(p[2]) << 16;
    return static_cast<uint16_t>((word >> shift) & ((1u << Bits) - 1));
}
template <unsigned Bits, size_t... I>
void unpackFields(const uint8_t *data, uint16_t *output, std::index_sequence<I...>) {
    ((output[I] = unpackField<Bits, I>(data)), ...);
}
template <unsigned Bits, size_t Lane> uint64_t unpackLane(const uint8_t *data) {
    uint64_t value = 0;
    if constexpr (Bits >= 4)
        value |= uint64_t(u16(data + 2 * Lane));
    if constexpr (Bits >= 8)
        value |= uint64_t(u16(data + 16 + 2 * Lane)) << 16;
    if constexpr (Bits >= 12)
        value |= uint64_t(u16(data + 32 + 2 * Lane)) << 32;
    constexpr unsigned offset = 16 * (Bits / 4), shift = 16 * (Bits / 4);
    if constexpr (Bits & 2)
        value |= uint64_t(data[offset + Lane]) << shift;
    if constexpr (Bits & 1) {
        constexpr unsigned tail = offset + ((Bits & 2) ? 8 : 0);
        constexpr unsigned tailShift = shift + ((Bits & 2) ? 8 : 0);
        value |= uint64_t((data[tail + Lane / 2] >> (4 * (Lane & 1))) & 15) << tailShift;
    }
    return value;
}
template <unsigned Bits, size_t... I>
void unpackLanes(const uint8_t *data, uint16_t *output, std::index_sequence<I...>) {
    const std::array<uint64_t, 8> lanes{unpackLane<Bits, 0>(data), unpackLane<Bits, 1>(data),
                                        unpackLane<Bits, 2>(data), unpackLane<Bits, 3>(data),
                                        unpackLane<Bits, 4>(data), unpackLane<Bits, 5>(data),
                                        unpackLane<Bits, 6>(data), unpackLane<Bits, 7>(data)};
    ((output[I] = static_cast<uint16_t>((lanes[I & 7] >> ((I / 8) * Bits)) & ((1u << Bits) - 1))),
     ...);
}
template <unsigned Bits, bool LanePacking> void unpackBlock(const uint8_t *data, uint16_t *output) {
    if constexpr (LanePacking)
        unpackLanes<Bits>(data, output, std::make_index_sequence<32>{});
    else
        unpackFields<Bits>(data, output, std::make_index_sequence<32>{});
}
template <bool LanePacking> void unpackBlock(const uint8_t *data, unsigned bits, uint16_t *output) {
#define MC24_UNPACK_CASE(N)                                                                        \
    case N:                                                                                        \
        unpackBlock<N, LanePacking>(data, output);                                                 \
        break
    switch (bits) {
        MC24_UNPACK_CASE(0);
        MC24_UNPACK_CASE(1);
        MC24_UNPACK_CASE(2);
        MC24_UNPACK_CASE(3);
        MC24_UNPACK_CASE(4);
        MC24_UNPACK_CASE(5);
        MC24_UNPACK_CASE(6);
        MC24_UNPACK_CASE(7);
        MC24_UNPACK_CASE(8);
        MC24_UNPACK_CASE(9);
        MC24_UNPACK_CASE(10);
        MC24_UNPACK_CASE(11);
        MC24_UNPACK_CASE(12);
        MC24_UNPACK_CASE(13);
        MC24_UNPACK_CASE(14);
        MC24_UNPACK_CASE(15);
    }
#undef MC24_UNPACK_CASE
}
// Keep residuals separate from reconstructed neighbors so the compiler can
// schedule unpacking and Bayer reconstruction without overlapping buffers.
int signedResidual(uint16_t residual) { return int(residual >> 1) ^ -int(residual & 1); }

void reconstructSeedRow(const uint16_t *residuals, uint16_t *row, unsigned width) {
    constexpr unsigned cfaPeriod = 2;
    std::copy_n(residuals, std::min(width, cfaPeriod), row);
    for (unsigned x = cfaPeriod; x < width; ++x)
        row[x] = static_cast<uint16_t>(row[x - cfaPeriod] + signedResidual(residuals[x]));
}

int adaptivePrediction(int left, int top, int corner, int right, unsigned threshold) {
    const int neighborLow = std::min(left, top);
    const int neighborHigh = std::max(left, top);
    const int rangeLow = std::min(neighborLow, std::min(corner, right));
    const int rangeHigh = std::max(neighborHigh, std::max(corner, right));
    if (unsigned(rangeHigh - rangeLow) <= threshold)
        return (std::min(65535, left + top + corner + right) + 2) / 4;
    return std::clamp(left + top - corner, neighborLow, neighborHigh);
}

void reconstructAdaptiveRow(const uint16_t *residuals, uint16_t *row, const uint16_t *above,
                            unsigned width, unsigned threshold) {
    constexpr unsigned cfaPeriod = 2;
    const unsigned seeds = std::min(width, cfaPeriod);
    for (unsigned x = 0; x < seeds; ++x)
        row[x] = static_cast<uint16_t>(above[x] + signedResidual(residuals[x]));
    const auto restore = [&](unsigned x, unsigned right) {
        const int prediction = adaptivePrediction(row[x - cfaPeriod], above[x],
                                                  above[x - cfaPeriod], above[right], threshold);
        row[x] = static_cast<uint16_t>(prediction + signedResidual(residuals[x]));
    };
    unsigned x = cfaPeriod;
    for (; x + cfaPeriod < width; ++x)
        restore(x, x + cfaPeriod);
    for (; x < width; ++x)
        restore(x, x);
}
} // namespace
uint32_t crc32c(const uint8_t *data, size_t size) {
    static const auto implementation = [] {
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#if defined(_MSC_VER)
        int cpu[4]{};
        __cpuid(cpu, 1);
        if (cpu[2] & (1 << 20))
            return &crc32cHardware;
#elif defined(__GNUC__) || defined(__clang__)
        if (__builtin_cpu_supports("sse4.2"))
            return &crc32cHardware;
#endif
#endif
        return &crc32cPortable;
    }();
    return implementation(data, size);
}
void decodeStrip(const uint8_t *data, size_t size, uint16_t *output, unsigned width, unsigned rows,
                 unsigned coder, unsigned packing, unsigned predictor, unsigned threshold) {
    require(width && rows && width <= 16384 && rows <= 16384 && coder == 0 && packing <= 1 &&
                predictor <= 1,
            "Unsupported MCPRO strip codec");
    constexpr unsigned samplesPerBlock = 32;
    constexpr unsigned cfaPeriod = 2;
    const size_t blocksPerRow = (width + samplesPerBlock - 1) / samplesPerBlock;
    const size_t blockCount = blocksPerRow * rows;
    size_t position = (blockCount + 1) / 2;
    require(position <= size, "Truncated MCPRO strip widths");
    std::vector<uint16_t> residuals(blocksPerRow * samplesPerBlock);
    for (unsigned y = 0; y < rows; ++y) {
        auto *row = output + size_t(y) * width;
        for (size_t block = 0; block < blocksPerRow; ++block) {
            const size_t index = y * blocksPerRow + block;
            const unsigned bits = (data[index / 2] >> (4 * (index & 1))) & 15;
            const size_t packedBytes = samplesPerBlock * bits / 8;
            require(packedBytes <= size - position, "Truncated MCPRO packed block");
            const size_t x = block * samplesPerBlock;
            auto *destination = residuals.data() + x;
            if (packing == 1)
                unpackBlock<true>(data + position, bits, destination);
            else
                unpackBlock<false>(data + position, bits, destination);
            position += packedBytes;
        }
        if (predictor == 0 || y < cfaPeriod)
            reconstructSeedRow(residuals.data(), row, width);
        else
            reconstructAdaptiveRow(residuals.data(), row, row - cfaPeriod * width, width,
                                   threshold);
    }
    require(position == size, "Trailing MCPRO strip data");
}
} // namespace mc24
} // namespace motioncam
