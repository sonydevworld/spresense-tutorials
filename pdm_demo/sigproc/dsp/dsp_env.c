/****************************************************************************
 * examples/pdm_demo/sigproc/dsp/dsp_env.c
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

#include "arm_math.h"
#include "dsp_env.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*
 * Get a coefficient for the basic LP filter used in envelope
 */
float dsp_env_get_lpfcoeff(uint16_t cutoff, uint32_t samplerate)
{
  float rc = (1.0 / (cutoff * 2 * 3.14));
  float dt = (1.0 / samplerate);

  return dt / (rc + dt);
}

/*
 * Very basic implementatino using a sliding RMS window
 */
int dsp_env_exec(int16_t *data, size_t size, int16_t *win, size_t win_size,
                 float lpf_coeff, int16_t *out, int8_t is_first)
{
  int32_t sum;
  q31_t result;

  for (int i = 0; i < size - win_size; i++)
    {
      /* Calculate sliding RMS over window */

      sum = 0;
      for (int w = i; w < i + win_size - 1; w++)
        {
          sum += data[w] * data[w];
        }

      sum /= win_size;
      arm_sqrt_q31(&sum,  &result);

      out[i] = (int16_t) result;

      /* Apply basic low pass Filter to smooth (if enabled, i.e. > 0.0) */

      if (lpf_coeff)
        {
          if (i > 1 || !is_first)
            {
              out[i] = out[i-1] + (lpf_coeff * (out[i] - out[i-1]));
            }
        }
    }

  return 1;
}
