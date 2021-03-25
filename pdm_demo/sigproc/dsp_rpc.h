/****************************************************************************
 * examples/pdm_demo/sigproc/dsp_rpc.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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

#ifndef __EXAMPLES_SIGPROC_DSP_RPC_H
#define __EXAMPLES_SIGPROC_DSP_RPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include "resource.h"
#include "sigproc.h"

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

int dsp_load_library(const char *filename);

void dsp_unload_library(void);

bool dsp_init(uint32_t samplerate);

bool dsp_deinit(void);

void dsp_set_fft(enum fft_bins_t bins, enum fft_window_t window, float *out);

void dsp_set_motor(uint16_t rpm, struct bearing_t *bearing);

void dsp_set_sensor(struct sensor_t *sensor);

void dsp_exec(int16_t *data, size_t size);

int dsp_sync(uint32_t ms, void *paddr);

void dsp_result(struct sigproc_result_t *result);

bool dsp_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __EXAMPLES_SIGPROC_DSP_RPC_H */
