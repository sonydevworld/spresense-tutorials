/****************************************************************************
 * examples/fft/dsp_rpc.c
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

#include <stdio.h>
#include <string.h>

#include <assert.h>

#include <asmp/mptask.h>
#include <asmp/mpmq.h>

#include "dsp_rpc.h"
#include "resource.h"
#include "pdm/log.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ARGVAL(v) (uint32_t)(v)
#define ARGPTR(p) ((uint32_t)(uintptr_t)(p))

#define ASYNC_BUF_NUM 8

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mptask_t g_dsptask = {0};
static mpmq_t   g_dspmq = {0};
static mpmq_t   g_dspmq2 = {0};
static uint32_t g_buffer[8];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*
 * Send a message to the DSP side and return when there is a response.
 * If the command is asynchronous, a second response will be sent when
 * the command has finished executing on the DSP.
 */
static int dsp_rpc(void *args)
{
  int ret;
  uint32_t data;

  data = (uint32_t)(uintptr_t)args;

  /* Send RPC message to DSP */

  ret = mpmq_send(&g_dspmq, DSP_RPC_MSG, data);
  if (ret < 0)
    {
      pdm_loge("mpmq_send() failure. %d\n", ret);
      return ret;
    }

  /* Wait for DSP function is done */

  ret = mpmq_receive(&g_dspmq, &data);
  if (ret < 0)
    {
      pdm_loge("mpmq_recieve() failure. %d\n", ret);
      return ret;
    }

  return (int)data;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*
 * Load the given DSP binary on one of the sub cores
 */
int dsp_load_library(const char *filename)
{
  int ret;

  memset(g_buffer, 0, sizeof(g_buffer));

  /* Initialize DSP Math library */

  ret = mptask_init(&g_dsptask, filename);
  if (ret != 0)
    {
      pdm_loge("mptask_init() failure. %d\n", ret);
      return ret;
    }

  ret = mptask_assign(&g_dsptask);
  if (ret != 0)
    {
      pdm_loge("mptask_asign() failure. %d\n", ret);
      return ret;
    }

  /* Initialize MP message queue with asigned CPU ID, and bind it to MP task */

  ret = mpmq_init(&g_dspmq, DSP_MQID, mptask_getcpuid(&g_dsptask));
  if (ret < 0)
    {
      pdm_loge("mpmq_init() failure. %d\n", ret);
      return ret;
    }
  ret = mptask_bindobj(&g_dsptask, &g_dspmq);
  if (ret < 0)
    {
      pdm_loge("mptask_bindobj(mq) failure. %d\n", ret);
      return ret;
    }

  ret = mpmq_init(&g_dspmq2, DSP_MQID2, mptask_getcpuid(&g_dsptask));
  if (ret < 0)
    {
      pdm_loge("mpmq_init() failure. %d\n", ret);
      return ret;
    }
  ret = mptask_bindobj(&g_dsptask, &g_dspmq2);
  if (ret < 0)
    {
      pdm_loge("mptask_bindobj(mq) failure. %d\n", ret);
      return ret;
    }

  ret = mptask_exec(&g_dsptask);
  if (ret < 0)
    {
      pdm_loge("mptask_exec() failure. %d\n", ret);
      return ret;
    }

  return 0;
}

/*
 * Unload the DSP binary on the sub core
 */
void dsp_unload_library(void)
{
  /* Send quit request to DSP */

  mpmq_send(&g_dspmq, DSP_RPC_UNLOAD, 0);

  /* Destroy DSP and successfully done. */

  (void)mptask_destroy(&g_dsptask, false, NULL);
  mpmq_destroy(&g_dspmq);
}

/*
 * Initialize the signal processing state and prepare resources.
 * Synchronous call.
 */
bool dsp_init(uint32_t samplerate)
{
  uint32_t *args = g_buffer;

  args[0] = DSP_INIT;
  args[1] = ARGPTR(samplerate);

  (void)dsp_rpc(args);

  return true;
}

/*
 * De-initialize the DSP and clear resources
 * Synchronous call.
 */
bool dsp_deinit(void)
{
  uint32_t *args = g_buffer;

  args[0] = DSP_DEINIT;

  (void)dsp_rpc(args);

  return true;
}

/*
 * Set FFT size and window type to use on the DSP
 * Synchronous call.
 */
void dsp_set_fft(enum fft_bins_t bins, enum fft_window_t window, float *out)
{
  uint32_t *args = g_buffer;

  args[0] = DSP_SET_FFT;
  args[1] = ARGPTR(bins);
  args[2] = ARGPTR(window);
  args[3] = ARGPTR(out);

  (void)dsp_rpc(args);
}

/*
 * Set motor RPM and bearing characteristics to use on the DSP
 * Synchronous call.
 */
void dsp_set_motor(uint16_t rpm, struct bearing_t *bearing)
{
  uint32_t *args = g_buffer;

  args[0] = DSP_SET_MOTOR;
  args[1] = ARGPTR(rpm);
  args[2] = ARGPTR(bearing);

  (void)dsp_rpc(args);
}

/*
 * Set sensor details to use on the DSP
 * Synchronous call.
 */
void dsp_set_sensor(struct sensor_t *sensor)
{
  uint32_t *args = g_buffer;

  args[0] = DSP_SET_SENSOR;
  args[1] = ARGPTR(sensor);

  (void)dsp_rpc(args);
}

/*
 * Run signal processing on the DSP asynchronously
 * Asynchronous call.
 */
void dsp_exec(int16_t *data, size_t size)
{
  uint32_t *args = g_buffer;

  args[0] = DSP_EXEC;
  args[1] = ARGPTR(data);
  args[2] = ARGPTR(size);

  (void)dsp_rpc(args);
}

/*
 * Fetch the result from the last number of processing runs
 * Synchronous call.
 */
void dsp_result(struct sigproc_result_t *result)
{
  uint32_t *args = g_buffer;

  args[0] = DSP_RESULT;
  args[1] = ARGPTR(result);

  (void)dsp_rpc(args);
}

/*
 * Reset DSP internal state.
 * Synchronous call.
 */
bool dsp_reset(void)
{
  uint32_t *args = g_buffer;

  args[0] = DSP_RESET;

  (void)dsp_rpc(args);

  return true;
}

/*
 * Wait for a response from an asynchronous DSP call, either with a
 * timeout for the number of milliseconds given in 'ms' or blocking
 * if 'ms' is set to 0. The pointer 'paddr' will contain the returned
 * data if given.
 */
int dsp_sync(uint32_t ms, void *paddr)
{
  int ret;
  uint32_t data;

  if (ms)
    {
      ret = mpmq_timedreceive(&g_dspmq, &data, ms);
    }
  else
    {
      ret = mpmq_receive(&g_dspmq, &data);
    }

  if (ret == DSP_RPC_RET)
    {
      if (paddr)
        {
          *(uint32_t *)paddr = data;
        }
    }

  return ret;
}

