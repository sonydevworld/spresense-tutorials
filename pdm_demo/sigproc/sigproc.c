/****************************************************************************
 * examples/fft/fft_main.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <math.h>

#include "dsp_rpc.h"
#include "sigproc.h"
#include "pdm/log.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_FFT_DSP_PATH
#  define DSP_LIBFILE CONFIG_EXAMPLES_FFT_DSP_PATH
#else
#  define DSP_LIBFILE "/mnt/sd0/BIN/DSPPDM"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* FFT output data area */

float *g_fft_out = NULL;

/* Flag to make sure we skip DSP sync before the first exec call */

bool first_sync = true;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*
 * Initialize signal processing to use the given buffer size (in bytes)
 * and samplerat.
 */
bool sigproc_init(size_t buffer_size, uint32_t samplerate)
{
  int ret;
  struct stat buffer;

  /* Check that file exists */

  ret = stat(DSP_LIBFILE, &buffer);
  if (ret == -1)
    {
      pdm_loge("\nDSPFFT binary was not found. Make sure to copy it\n");
      printf("from your \"examples/pdm_demo/fft/dsp/\" folder to \"/BIN/\"\n");
      printf("on your SD-card.\n");

      return false;
    }

  /* Load DSP binary */

  ret = dsp_load_library(DSP_LIBFILE);
  if (ret < 0)
    {
      pdm_loge("Could not load the DSP binary\n");

      return false;
    }

  /* Initialize DSP side */

  (void)dsp_init(samplerate);

  return true;
}

/*
 * Release all DSP related resources
 */
void sigproc_deinit(void)
{
  dsp_deinit();

  dsp_unload_library();

  if (g_fft_out)
    {
      free(g_fft_out);
      g_fft_out = NULL;
    }
}

/*
 * Configure FFT settings (size/bins and window tupe) to use on the DSP
 */
bool sigproc_set_fft(enum fft_bins_t bins, enum fft_window_t window)
{
  if (g_fft_out)
    {
      free(g_fft_out);
    }

  g_fft_out = malloc(sizeof(float) * bins);
  if (!g_fft_out)
    {
      pdm_loge("Couldn't allocate FFT ouput buffer\n");
    }

  dsp_set_fft(bins, window, g_fft_out);

  return true;
}

/*
 * Set motor parameters RPM and bearing frequency characteristics.
 * Should be called before each measurement if the RPM could change.
 */
bool sigproc_set_motor(uint16_t rpm,
                       float bearing_ftf, float bearing_bsf,
                       float bearing_bpfi, float bearing_bpfo)
{
  struct bearing_t bearing = {
      bearing_ftf, bearing_bsf, bearing_bpfi, bearing_bpfo
    };

  dsp_set_motor(rpm, &bearing);

  return true;
}

/*
 * Set range and other specifications of the sensor
 */
bool sigproc_set_sensor(uint16_t zero_out, uint16_t sensitivity,
                        int16_t range_min, int16_t range_max)
{
  struct sensor_t sensor = {
      zero_out, sensitivity, range_min, range_max
    };

  dsp_set_sensor(&sensor);

  return true;
}

/*
 * Perform signal processing on given data with the given size
 */
bool sigproc_exec(int8_t *data, uint32_t size)
{
  /* Convert indata to float before sending to the DSP */

  dsp_exec((int16_t *)data, size / sizeof(int16_t));

  return true;
}


/*
 * Synchronise with the DSP, i.e. wait for a message indicating
 * completion of a previous asynchronous call
 */
bool sigproc_sync(void)
{
  if (first_sync)
    {
      first_sync = false;
      return true;
    }

  /* Message queue returns negative code on error */

  return dsp_sync(0, NULL) >= 0;
}

/*
 * Fetch the signal processing result from the DSP
 */
bool sigproc_result(struct sigproc_result_t *result)
{
  dsp_result(result);

  return true;
}

/*
 * Reset the DSP current state to start a new measurement
 */
void sigproc_reset(void)
{
  first_sync = true;

  dsp_reset();
}

