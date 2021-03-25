/****************************************************************************
 * pdm_demo/pdm_demo_main.cxx
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/board/board.h>
#include <nuttx/config.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "audio/audio.h"
#include "pdm/api_senseye.h"
#include "pdm/config.h"
#include "pdm/datahandler.h"
#include "pdm/log.h"
#include "pdm/net.h"
#include "pdm/scheduler.h"

#include "sigproc/sigproc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

//#define SKIP_API_POST
//#define PRINT_REPORT
//#define PRINT_RESULT

#define MEASURE_LENGTH_S 5     /* Measure for 5 seconds */
#define MEASURE_PERIOD_S 300   /* Measure every 5 minutes (300 seconds) */
#define REPORT_PERIOD_S  3600  /* Send report every hour (3600 seconds) */
#define CONFIG_FILE "/mnt/sd0/pdm.config"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Names of the data fields to use in the data handler */

static const char *data_names[] = {
      "RMS", "Crest factor", "Peak to peak",
      "Bearing FTF", "Bearing BSF", "Bearing BPFI", "Bearing BPFO",
      "1xRPM", "2xRPM", "3xRPM"};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef PRINT_RESULT
/**
 * For debugging, print result struct contents on stdout
 */
static void print_result(struct sigproc_result_t *result)
{
  pdm_log("Processing result (samples=%d):\n"
         "        rms=%f, crest=%f, p2p=%f, peak=%f, min=%f\n"
         "        ftf=%f, bsf=%f, bpfi=%f, bpfo=%f\n"
         "        1xRPM=%f, 2xRPM=%f, 3xRPM=%f\n",
         result->samples, result->rms, result->crest,
         result->peak_to_peak, result->peak, result->min,
         result->bearing_ftf, result->bearing_bsf,
         result->bearing_bpfi, result->bearing_bpfo,
         result->rpm_1x, result->rpm_2x, result->rpm_3x);
}
#endif /* PRINT_RESULT */


/**
 * Audio processing callback: called during measurement when new
 * audio data is available
 */
static bool process_audio_cb(int8_t *buffer, uint32_t size, bool is_last)
{
  /* Wait for previous DSP processing to finish if there is any.
     We only process one buffer at the time */

  sigproc_sync();

  /* Forward the data to the signal processing component. Processing will
     continue running on the DSP after this call returns. */

  if (!sigproc_exec(buffer, size))
    {
      pdm_loge("Signal processing failed\n");
    }

  return true;
}


/**
 * Measurement task: called pretty often, e.g. every 10 minutes
 */
static bool measure_task()
{
  bool ret = true;

  static struct sigproc_result_t result;

  /* Init the signal processing component for a new measurement */

  sigproc_reset();

  /* Set motor speed and bearing info. Currently it seems a bit
     unnecessary to set these each time, but ultimately you'd
     probably want to read the RPM from somewhere else to get
     the actual current value */

  sigproc_set_motor(config_int("motor_rpm"),
                    config_float("bearing_ftf"),
                    config_float("bearing_bsf"),
                    config_float("bearing_bpfi"),
                    config_float("bearing_bpfo"));

  /* Capture data for a number of seconds. This will periodically
     call the audio callback function when data is available */

  audio_capture(MEASURE_LENGTH_S);

  /* Wait for ongoing signal processing to finish and get the result */

  sigproc_sync();
  sigproc_result(&result);

#ifdef PRINT_RESULT
  print_result(&result);
#endif

  /* Save the values with the current time to the data store */

  datahandler_new_record(scheduler_time());
  datahandler_add_data("RMS", result.rms);
  datahandler_add_data("Crest factor", result.crest);
  datahandler_add_data("Peak to peak", result.peak_to_peak);
  datahandler_add_data("Bearing FTF", result.bearing_ftf);
  datahandler_add_data("Bearing BSF", result.bearing_bsf);
  datahandler_add_data("Bearing BPFI", result.bearing_bpfi);
  datahandler_add_data("Bearing BPFO", result.bearing_bpfo);
  datahandler_add_data("1xRPM", result.rpm_1x);
  datahandler_add_data("2xRPM", result.rpm_2x);
  datahandler_add_data("3xRPM", result.rpm_3x);
  datahandler_save_record();

  return ret;
}

/**
 * Reporting task: called more seldom, e.g. hourly
 */
static bool report_task()
{
  bool done = false;
  bool result = true;
  char *report;
  size_t max_data;

  /* Connect to senseye.io API and initialize the communication */

  result = net_connect(config("api_host"), config("api_port"));
  if (!result)
    {
      pdm_loge("Network connection failed\n");
    }
  else
    {
      result = api_senseye_init(config("api_user"), config("api_pass"));
      if (!result)
        {
          pdm_loge("Could not connect to API\n");
        }
      else
        {
          /* Get maximum data we can post */

          max_data = api_senseye_max_post_size();

          /* Get the json report one chunk at the time */

          while (!done)
            {
              done = datahandler_report_json(max_data, &report);

#ifdef PRINT_REPORT
              pdm_log("Posting data (size=%d, id=%s):\n%s\n",
                      strlen(report),  config("sensor_id"), report);
#endif

#ifndef SKIP_API_POST
              result = api_senseye_post(config("sensor_id"), report, done);
              if (!result)
                {
                  pdm_loge("Failed to send report\n");
                }
#endif
            }
        }

      net_disconnect();
    }

  return result;
}

/**
 * Actual main function, called from wrapper functions below
 */
static int pdm_main(int argc, FAR char *argv[])
{
  bool ret;

  pdm_log("PDM demo started\n");

  /* Read configuration file */

  ret = config_init(CONFIG_FILE);
  if (!ret)
    {
      pdm_loge("Couldn't read configuration file\n");
      goto exit;
    }

  /* Initiate data store and register our data names */

  ret = datahandler_init(data_names, 10);
  if (!ret)
    {
      pdm_loge("Datahandler initialization failed\n");
      goto exit;
    }

  /* Connect to wifi */

  ret = net_init(config("wifi_ssid"), config("wifi_pass"));
  if (!ret)
    {
      pdm_loge("Network initialization failed\n");
      goto exit;
    }

  /* Initialize scheduler and create tasks */

  ret = scheduler_init();
  if (!ret)
    {
      pdm_loge("Scheduler initialization failed\n");
      goto exit;
    }
  scheduler_register("measure", measure_task, MEASURE_PERIOD_S);
  scheduler_register("report", report_task, REPORT_PERIOD_S);

  /* Initialize audio system with samplerate and set data callback */

  ret = audio_init(48000);
  if (!ret)
    {
      pdm_loge("Audio initialization failed\n");
      goto error_audio_init;
    }
  audio_set_process_cb(&process_audio_cb);

  /* Initialize signal processing component on DSP */

  ret = sigproc_init(audio_buffer_size(), audio_sample_rate());
  if (!ret)
    {
      pdm_loge("Sigproc initialization failed\n");
      goto error_fft_init;
    }

  /* Set signal processing parameters */

  sigproc_set_fft(FFT_BINS_2048,
                  FFT_WIN_HAMMING);

  sigproc_set_sensor(config_int("sensor_zero_mV"),
                     config_int("sensor_sens_mV/g"),
                     config_int("sensor_rmin_g"),
                     config_int("sensor_rmax_g"));


  /* Give control to the scheduler to run our tasks! This blocks. */

  scheduler_start();


  /* Scheduler exited, clean up... */

  sigproc_deinit();

error_fft_init:
  audio_deinit();

error_audio_init:
  scheduler_deinit();

exit:
  config_deinit();

  pdm_log("PDM demo done\n");

  return (int) ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL

/**
 * "Regular" main function
 */
extern "C" int main(int argc, FAR char *argv[])
{
  return pdm_main(argc, argv);
}

#else

/**
 * To call as start routine in nuttx sdk configuration
 */
extern "C" int pdm_demo_main(int argc, char *argv[])
{
  (void)boardctl(BOARDIOC_INIT, 0);

  /* Delay a bit to make sure the sdcard is accessible */

  usleep(2000000);

  return pdm_main(argc, argv);
}

#endif
