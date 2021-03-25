/****************************************************************************
 * examples/sigproc/dsp/dsp_fft.c
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdlib.h>

#include "arm_math.h"
#include "arm_const_structs.h"

#include "resource.h"
#include "sigproc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_FFT_SAMPLES_WORK_BUFFER_SIZE
#  define CONFIG_EXAMPLES_FFT_SAMPLES_WORK_BUFFER_SIZE (1024 * 8)
#endif

#define TEMP_OUT_LEN  (FFT_BINS_2048 / 2)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Internal temporary storage use during processing */

g_workBuffer[CONFIG_EXAMPLES_FFT_SAMPLES_WORK_BUFFER_SIZE];

/* Temporary output used between processing rounds. */
/* A bit wasteful, could be handled better */

float g_temp_out[TEMP_OUT_LEN];
uint16_t g_chunk_count;

/* FFT settings */

float g_bin_width;
uint32_t g_samplerate;
static enum fft_bins_t   g_bins = FFT_BINS_1024;
static enum fft_window_t g_window = FFT_WIN_RECTANGLE;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*
 * Retrieve a chunk of the internal static temporary storage
 */
static void *allocate_buffer(uint32_t fftLen)
{
  static int index = 0;
  void *workPtr;

  if ((index + fftLen) > CONFIG_EXAMPLES_FFT_SAMPLES_WORK_BUFFER_SIZE)
    {
      index = 0;
    }

  workPtr = &g_workBuffer[index];
  arm_fill_f32(0.0f, workPtr, fftLen);

  index += fftLen;

  return workPtr;
}

/*
 * Apply Hamming window to the given data
 */
static int window_hamming(float *pData, uint32_t fftLen)
{
  int ret = OK;
  int i;
  float weight;

  for (i = 0; i < fftLen / 2; i++)
    {
      weight = 0.54f - (0.46f * arm_cos_f32(2 * PI * (float)i / (fftLen - 1)));
      pData[i] *= weight;
      pData[fftLen - i + 1] *= weight;
    }

  return ret;
}

/*
 * Apply Hanning window to the given data
 */
static int window_hanning(float *pData, uint32_t fftLen)
{
  int ret = OK;
  int i;
  float weight;

  for (i = 0; i < fftLen / 2; i++)
    {
      weight = 0.54f - (1.0f - arm_cos_f32(2 * PI * (float)i / (fftLen - 1)));
      pData[i] *= weight;
      pData[fftLen - i + 1] *= weight;
    }

  return ret;
}

/*
 * Run FFT on the data given in "src" and write the output to "dst"
 */
static int fft(float *src, float *dst)
{
  arm_rfft_fast_instance_f32 S;

  switch (g_bins)
    {
      case 32:
        arm_rfft_32_fast_init_f32(&S);
        break;
      case 64:
        arm_rfft_64_fast_init_f32(&S);
        break;
      case 128:
        arm_rfft_128_fast_init_f32(&S);
        break;
      case 256:
        arm_rfft_256_fast_init_f32(&S);
        break;
      case 512:
        arm_rfft_512_fast_init_f32(&S);
        break;
      case 1024:
        arm_rfft_1024_fast_init_f32(&S);
        break;
      case 2048:
        arm_rfft_2048_fast_init_f32(&S);
        break;
#if 0
      /* Exclude to save the program size */

      case 4096:
        arm_rfft_4096_fast_init_f32(&S);
        break;
#endif
      default:
        return ERROR;
    }

  /* Apply window function */

  if (g_window == FFT_WIN_HAMMING)
    {
      window_hamming(src, g_bins);
    }
  else if (g_window == FFT_WIN_HANNING)
    {
      window_hanning(src, g_bins);
    }

  /* Calculation */

  float32_t *tmpBuf = allocate_buffer(g_bins);

  arm_rfft_fast_f32(&S, src, tmpBuf, 0);

  arm_cmplx_mag_f32(&tmpBuf[2], &dst[1], g_bins / 2 - 1);
  dst[0] = tmpBuf[0];
  dst[g_bins / 2] = tmpBuf[1];

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*
 * Initialize FFT state
 */
void dsp_fft_setup(enum fft_bins_t bins, enum fft_window_t window,
                   uint32_t samplerate)
{
  g_bins = bins;
  g_window = window;
  g_samplerate = samplerate;
  g_bin_width = (float) samplerate / (float) bins;
}

/*
 * Get FFT size (number of bins)
 */
int dsp_fft_size(void)
{
  return g_bins;
}

/*
 * Get frequency range of one FFT bin
 */
float dsp_fft_bin_width(void)
{
  return g_bin_width;
}

/*
 * Reset internal state of FFT
 */
void dsp_fft_reset(void)
{
  arm_fill_f32(0.0f, g_temp_out, TEMP_OUT_LEN);
  g_chunk_count = 0;
}

/*
 * Run FFT on data in "src" with total length "size", write
 * accumulated result to "out". Set "last" to non-zero on last chunk.
 */
int dsp_fft(float *src, float *out, size_t size, uint8_t last)
{
  int b = 0;
  int ret = OK;

  /* Run FFT on data chunk */

  ret = fft(src, g_temp_out);
  if (ret != OK)
    {
      goto exit;
    }

  /* Accumulate mean output in destination buffer */

  for (b = 0; b < g_bins / 2; b++)
    {
      out[b] = (g_chunk_count * out[b] + g_temp_out[b]) /
               (g_chunk_count + 1);
    }

  if (last)
    {
      g_chunk_count = 0;
    }
  else
    {
      g_chunk_count++;
    }

exit:
  return ret;
}

float dsp_fft_get(float *fft_out, float frequency)
{
  int bin = (int) (frequency / g_bin_width);

  return fft_out[bin];
}

