/****************************************************************************
 * examples/sigproc/dsp/dsp_main.c
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
#include <float.h>
#include <stdlib.h>

#include <asmp/types.h>
#include <asmp/mpmq.h>
#include <asmp/mpshm.h>

#include "asmp.h"

#include "arm_math.h"
#include "arm_const_structs.h"

#include "dsp_env.h"
#include "dsp_fft.h"
#include "dsp_main.h"

#include "sigproc.h"
#include "resource.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ASSERT(cond) if (!(cond)) wk_abort()

/* Internal error must be other than returns of math functions. */

#define INTERNAL_ERROR -99

/* Utility macros for declare RPC entry function and construct
 * their table.
 */

#define RPCENTRY(sym) { DSP_ ## sym, rpcentry_ ## sym }
#define DECLARE_RPCENTRY(sym) static int rpcentry_ ## sym
#define RPCENTRY_TERMINATE { 0, NULL }

#define DATA_BUFFER_SIZE (1024 * 4 * 4)

/* Note: Due to not having malloc on DSP, only support 2048 bins */

#define FLOAT_BUFFER_LEN  FFT_BINS_2048

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* RPC ID/handler pair referenced in call handling */

struct rpcentry_s
{
  uint32_t hash;
  int (*func)(uint32_t *args);
};

float g_float_buffer[FLOAT_BUFFER_LEN];

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Communication message queues */

static mpmq_t g_mq;
static mpmq_t g_mq2;

/* Processing settings */

static uint32_t g_samplerate;

/* FFT output buffer */

static float32_t *g_fft_out = NULL;
//float g_fft_out[FLOAT_BUFFER_LEN];

/* Current motor settings */

static uint16_t g_motor_rpm = 0;
static struct bearing_t g_motor = {0.0f, 0.0f, 0.0f, 0.0f };

/* Sensor specification */

static struct sensor_t g_sensor = { 0, 0.0f, 0.0f, 0.0f };

/* Internal processing state */

struct dsp_state_t g_state;

/* Sample size in acceleration (g) */

static float g_sample_size_g;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*
 * Convert the given int16_t input reading to float acceleration:
 *   Acc. (g) = [reading (mV) – zero output (mV)] / sensitivity (mV/g)
 */
static float sample_to_acc(int16_t sample)
{
  return (float) ((sample * g_sample_size_g) - g_sensor.zero_out) /
                 g_sensor.sensitivity;
}

/* Clear internalstate struct */

static void dsp_reset_state(void)
{
  g_state.min = FLT_MAX;
  g_state.max = -FLT_MAX;
  g_state.peak = -FLT_MAX;
  g_state.mean = 0.0f;
  g_state.samples = 0;
  g_state.rounds = 0;

  arm_fill_f32(0.0f, g_fft_out, dsp_fft_size() / 2);
  arm_fill_f32(0.0f, g_float_buffer, FLOAT_BUFFER_LEN);
}

/*
 * Do signal processing on the given data with the given number of values.
 * The results are stored in the internal state struct.
 */
static int dsp_exec(int16_t *data, size_t size)
{
  int ret = OK;
  int i = 0;
  int s = 0;
  int chunk_size;
  float32_t val;
  float32_t rms = 0.0;

  chunk_size = dsp_fft_size();


  /* Process the data one FFT chunk (number of bins) at the time. */

  for (i = 0; i < size; i += chunk_size)
    {
      for (s = 0; s < chunk_size; s++)
        {
          /* Convert input to acceleration (float) */

          g_float_buffer[s] = sample_to_acc(data[i + s]);

          /* Update max/min values */

          if (g_float_buffer[s] > g_state.max)
            {
              g_state.max = g_float_buffer[s];
            }

          if (g_float_buffer[s] < g_state.min)
            {
              g_state.min = g_float_buffer[s];
            }
        }

      /* Update mean RMS for this round*/

      arm_rms_f32(g_float_buffer, chunk_size, &val);
      if (i == 0)
        {
          rms = val;
        }
      else
        {
          rms = ((i * rms) + val) / (i + 1);
        }


      /* Run FFT, the result is used in dsp_get_result */

      ret = dsp_fft(g_float_buffer, g_fft_out, chunk_size,
                    (i + chunk_size >= size));
    }

  /* Update totals */

  g_state.rms = ((g_state.rms * g_state.rounds) + rms) /
                (g_state.rounds + 1);

  /* update peak */

  if (g_state.max > g_state.peak)
    {
      g_state.peak = g_state.max;
    }

  g_state.samples += size;
  g_state.rounds++;

  return ret;
}

/*
 * Finalize and gather all calculation output and fill in the result
 */
static int dsp_get_result(struct sigproc_result_t *result)
{
  int ret = OK;
  float rps = (float) g_motor_rpm / 60.0f;

  result->samples = g_state.samples;
  result->min = g_state.min;
  result->peak = g_state.peak;
  result->peak_to_peak = g_state.max - g_state.min;
  result->rms = g_state.rms;
  result->crest = g_state.peak / g_state.rms;

  result->bearing_ftf = dsp_fft_get(g_fft_out, g_motor.ftf * rps);
  result->bearing_bsf = dsp_fft_get(g_fft_out, g_motor.bsf * rps);
  result->bearing_bpfi = dsp_fft_get(g_fft_out, g_motor.bpfi * rps);
  result->bearing_bpfo = dsp_fft_get(g_fft_out, g_motor.bpfo * rps);

  result->rpm_1x = dsp_fft_get(g_fft_out, rps);
  result->rpm_2x = dsp_fft_get(g_fft_out, rps * 2);
  result->rpm_3x = dsp_fft_get(g_fft_out, rps * 3);

  return ret;
}

/*
 * Performs signal processing on the given data buffer
 */
DECLARE_RPCENTRY(EXEC)(uint32_t *args)
{
  int ret = OK;
  int16_t *data = (int16_t *)args[1];
  size_t size = (size_t)args[2];

  ret = dsp_exec(data, size);

  /* Done, send message for syncing with app side */

  mpmq_send(&g_mq, DSP_RPC_RET, ret);

  return ret;
}

/*
 * Retrieve the processing result
 */
DECLARE_RPCENTRY(RESULT)(uint32_t *args)
{
  int ret = OK;

  struct sigproc_result_t *pResult = (struct sigproc_result_t*)args[1];

  ret = dsp_get_result(pResult);

  return ret;
}

/*
 * Reset the internal DSP processing state
 */
DECLARE_RPCENTRY(RESET)(uint32_t *args)
{
  int ret = OK;

  dsp_fft_reset();

  dsp_reset_state();

  return ret;
}

/*
 * Set FFT number of bins and window type, and also output
 * buffer for the FFT result since we don't allocate on DSP side.
 * Size of output buffer must be number of FFT bins / 2.
 */
DECLARE_RPCENTRY(SET_FFT)(uint32_t *args)
{
  int ret = OK;

  enum fft_bins_t bins = (enum fft_bins_t)args[1];
  enum fft_window_t window = (enum fft_window_t)args[2];

  g_fft_out = (float *)args[3];

  dsp_fft_setup(bins, window, g_samplerate);

  return ret;
}

/*
 * Set motor speed and bearing characteristics
 */
DECLARE_RPCENTRY(SET_MOTOR)(uint32_t *args)
{
  int ret = OK;

  g_motor_rpm = (uint16_t)args[1];

  struct bearing_t *bearing = (struct bearing_t*)args[2];

  g_motor.ftf = bearing->ftf;
  g_motor.bsf = bearing->bsf;
  g_motor.bpfi = bearing->bpfi;
  g_motor.bpfo = bearing->bpfo;

  return ret;
}

/*
 * Set sensor range and other specifics
 */
DECLARE_RPCENTRY(SET_SENSOR)(uint32_t *args)
{
  int ret = OK;

  struct sensor_t *sensor = (struct sensor_t*)args[1];
  g_sensor.zero_out = sensor->zero_out;
  g_sensor.range_min = sensor->range_min;
  g_sensor.range_max = sensor->range_max;
  g_sensor.sensitivity = sensor->sensitivity;

  /* Sample value in number of g (12 bit ADC) */

  g_sample_size_g = (float)(g_sensor.range_max - g_sensor.range_min) / 0xfFff;

  return ret;
}

/*
 * Setup the DSP environment according to the given parameters and
 * allocate any needed buffers
 */
DECLARE_RPCENTRY(INIT)(uint32_t *args)
{
  int ret = OK;

  g_samplerate = (uint32_t)args[1];

  return ret;
}

/*
 * Release any resources allocated on the DSP
 */
DECLARE_RPCENTRY(DEINIT)(uint32_t *args)
{
  int ret = OK;

  /* Nothing to do right now */

  return ret;
}

struct rpcentry_s g_rpcentry[] =
{
  RPCENTRY(INIT),
  RPCENTRY(DEINIT),
  RPCENTRY(SET_FFT),
  RPCENTRY(SET_MOTOR),
  RPCENTRY(SET_SENSOR),
  RPCENTRY(EXEC),
  RPCENTRY(RESET),
  RPCENTRY(RESULT),
  RPCENTRY_TERMINATE
};

/*
 * Find and execute the message handler that corresponds to the command
 * ID given in the argument.
 */
static int rpcmain(uint32_t *args)
{
  struct rpcentry_s *e;

  for (e = g_rpcentry; e->hash; e++)
    {
      if (e->hash == args[0])
        {
          return e->func(args);
        }
    }

  return INTERNAL_ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*
 * The DSP main loop basically just lstens for messages from the application
 * side and executes the corresponding RPCENTRY function when it gets one.
 */
int main(void)
{
  uint32_t msgdata;
  int ret;

  /* Initialize MP message queue,
   * On the worker side, 3rd argument is ignored.
   */

  ret = mpmq_init(&g_mq, DSP_MQID, 0);
  ASSERT(ret == 0);

  ret = mpmq_init(&g_mq2, DSP_MQID2, 0);
  ASSERT(ret == 0);

  for (;;)
    {
      uint32_t *args;

      /* Receive message from supervisor */

      ret = mpmq_receive(&g_mq, &msgdata);
      if (ret == DSP_RPC_UNLOAD)
        {
          break;
        }

      if (ret == DSP_RPC_MSG)
        {
          args = (uint32_t *)msgdata;

          /* Always send response, even on asynchronous call */

          ret = rpcmain(args);
          if (ret == INTERNAL_ERROR)
            {
              mpmq_send(&g_mq, DSP_RPC_UNDEF, args[0]);
            }
          else
            {
              mpmq_send(&g_mq, DSP_RPC_RET, ret);
            }
        }
    }

  return 0;
}
