/****************************************************************************
 * examples/pdm_demo/fft/fft.h
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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

#ifndef __EXAMPLES_PDM_DEMO_SIGPROC_H
#define __EXAMPLES_PDM_DEMO_SIGPROC_H

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum fft_bins_t {
  FFT_BINS_32   = 32,
  FFT_BINS_64   = 64,
  FFT_BINS_128  = 128,
  FFT_BINS_256  = 256,
  FFT_BINS_512  = 512,
  FFT_BINS_1024 = 1024,
  FFT_BINS_2048 = 2048
};

enum fft_window_t {
  FFT_WIN_RECTANGLE = 0x00000000,
  FFT_WIN_HAMMING   = 0x00000001,
  FFT_WIN_HANNING   = 0x00000002
};

struct bearing_t
{
  float ftf;
  float bsf;
  float bpfi;
  float bpfo;
};

struct sensor_t
{
  uint16_t zero_out;
  float    sensitivity;
  float    range_min;
  float    range_max;
};

struct sigproc_result_t
{
  uint32_t samples;
  float min;
  float peak;
  float peak_to_peak;
  double rms;
  float crest;
  float bearing_ftf;
  float bearing_bsf;
  float bearing_bpfi;
  float bearing_bpfo;
  float rpm_1x;
  float rpm_2x;
  float rpm_3x;
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/**/
bool sigproc_init(size_t buffer_size, uint32_t samplerate);

void sigproc_deinit(void);

bool sigproc_set_fft(enum fft_bins_t bins, enum fft_window_t window);

bool sigproc_set_motor(uint16_t rpm,
                       float bearing_ftf, float bearing_bsf,
                       float bearing_bpfi, float bearing_bpfo);

bool sigproc_set_sensor(uint16_t zero_out, uint16_t sensitivity,
                        int16_t range_min, int16_t range_max);

bool sigproc_exec(int8_t *data, uint32_t size);

bool sigproc_sync(void);

bool sigproc_result(struct sigproc_result_t *result);

void sigproc_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __EXAMPLES_PDM_DEMO_SIGPROC_H */
