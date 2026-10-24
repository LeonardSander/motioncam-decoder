/*
lj92.h
(c) Andrew Baldwin 2014

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.


sourced from MLV repository https://github.com/ilia3101/MLV-App/tree/master/src/mlv/liblj92
*/

#ifndef LJ92_H
#define LJ92_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum LJ92_ERRORS {
    LJ92_ERROR_NONE = 0,
    LJ92_ERROR_CORRUPT = -1,
    LJ92_ERROR_NO_MEMORY = -2,
    LJ92_ERROR_BAD_HANDLE = -3,
    LJ92_ERROR_TOO_WIDE = -4,
    LJ92_ERROR_ENCODER = -5
};

typedef struct _ljp* lj92;

/* Parse a lossless JPEG (1992) structure returning
 * - a handle that can be used to decode the data
 * - width/height/bitdepth of the data
 * Returns status code.
 * If status == LJ92_ERROR_NONE, handle must be closed with lj92_close
 */
int lj92_open(lj92* lj, // Return handle here
              uint8_t* data,int datalen, // The encoded data
              int* width,int* height,int* bitdepth,int* components); // Width, height, bitdepth and components

/* Release a decoder object */
void lj92_close(lj92 lj);

/*
 * Decode previously opened lossless JPEG (1992) into a 2D tile of memory
 * Starting at target, write writeLength 16bit values, then skip 16bit skipLength value before writing again
 * If linearize is not NULL, use table at linearize to convert data values from output value to target value
 * Data is only correct if LJ92_ERROR_NONE is returned
 */
int lj92_decode(lj92 lj,
                uint16_t* target, int writeLength, int skipLength, // The image is written to target as a tile
                uint16_t* linearize, int linearizeLength); // If not null, linearize the data using this table

/*
 * Encode an interleaved image supplied as unpacked 16-bit values within the
 * given bitdepth. readLength and skipLength are measured in samples.
 * Read from tile in the image
 * Apply delinearization if given
 * Return the encoded lossless JPEG stream
 */
int lj92_encode(uint16_t* image, int width, int height, int bitdepth, int components,
                int readLength, int skipLength,
                uint16_t* delinearize,int delinearizeLength,
                uint8_t** encoded, int* encodedLength);

/* Optional entropy writer. Return zero and set written on success; returning
 * nonzero lets liblj92 use its CPU entropy writer. When residuals is NULL,
 * the fallback recomputes differences from image samples. The output must
 * include JPEG byte stuffing, but must not include the EOI marker. */
typedef int (*lj92_entropy_writer)(const int16_t* residuals, int sampleCount,
                                   const uint16_t codes[18],
                                   const uint16_t codeBits[18],
                                   uint8_t* output, int capacity,
                                   int* written, void* context);
int lj92_encode_with_entropy_writer(uint16_t* image, int width, int height,
                                   int bitdepth, int components, int readLength,
                                   int skipLength, int predictor,
                                   const int histogram[18],
                                   const int16_t* residuals,
                                   lj92_entropy_writer writer, void* context,
                                   uint8_t** encoded, int* encodedLength);

#ifdef __cplusplus
}
#endif

#endif
