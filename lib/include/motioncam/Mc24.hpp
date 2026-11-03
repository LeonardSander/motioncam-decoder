#ifndef MOTIONCAM_MC24_HPP
#define MOTIONCAM_MC24_HPP
#include <cstddef>
#include <cstdint>
namespace motioncam::mc24 {
// Decode one independently predicted strip into native (pre-linearization) samples.
void decodeStrip(const uint8_t *data, size_t size, uint16_t *output, unsigned width, unsigned rows,
                 unsigned coder, unsigned packing, unsigned predictor, unsigned threshold);
// Explicit portable path also permits validating fallback CRC behavior on x86.
uint32_t crc32cPortable(const uint8_t *data, size_t size);
uint32_t crc32c(const uint8_t *data, size_t size);
} // namespace motioncam::mc24
#endif
