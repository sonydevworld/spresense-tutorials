/****************************************************************************
 * pdm_demo/pdm/datahandler.cxx
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <netutils/cJSON.h>

#include "log.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DATA_MAX_COUNT   15
#define DATA_DECIMALS    6

#define LOGFILE_DIR      "/mnt/sd0/"
#define LOGFILE_START    "pdm_data_"
#define LOGFILE_END      ".log"
#define LOGFILE_NAME_MAX 27

#define BUFFER_LEN       70

/****************************************************************************
 * Private data
 ****************************************************************************/

/* Currently set data field names and count */

static char **g_names;
static uint8_t g_name_count = 0;

/* The actual data field values */

static double g_data[DATA_MAX_COUNT];

/* Timestamp of the current data record*/

static uint32_t g_unixtime;

/* Current logfile name and index */

static char g_filename[LOGFILE_NAME_MAX];
static uint16_t g_file_index;

/* Variables used when generating the json report */

const char g_report_header[] = "{\"e\":[";
const char g_report_footer[] = "]}";
static int g_head_foot_len;

/* Json report buffer and max size */

static char   *g_report_buf = NULL;
static size_t  g_report_max;


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*
 * Generate a new logfile name.
 */
static bool get_new_logfile_name(char *filename)
{
  g_file_index++;
  snprintf(filename, LOGFILE_NAME_MAX, "%s%s%d%s",
           LOGFILE_DIR, LOGFILE_START, g_file_index, LOGFILE_END);
  return true;
}

/*
 * Prepare the logfile for collecting measurements.
 * Currently it is simply removed.
 */
static bool prepare_logfile(char *filename)
{
  int ret = 0;

  ret = unlink(filename);
  if (ret != 0 && errno != ENOENT)
    {
      pdm_logd("Couldn't delete log file (%d, %s)\n", errno, strerror(errno));
      return false;
    }
  return true;
}

/*
 * Reset all data fielda, i.e. start a new data record.
 */
static void reset_data(void)
{
  int i;

  for (i = 0; i < DATA_MAX_COUNT; i++)
    {
      g_data[i] = 0.0f;
    }
}

/*
 * Add a data value to the data field with the given name.
 */
static bool add_data(char *name, double value)
{
  int i;

  for (i = 0; i < g_name_count; i++)
    {
      if (strcmp(name, g_names[i]) == 0)
        {
          g_data[i] = value;
          return true;
        }
    }

  pdm_loge("Data value \"%s\" not found\n", name);

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*
 * Initialize the data handler and provide an array of strings
 * with names to use for the data fields in the measurements
 */
bool datahandler_init(const char **names, uint8_t name_count)
{
  if (name_count >= DATA_MAX_COUNT)
    {
      pdm_loge("Max data names (%d) exceeded\n", DATA_MAX_COUNT);
      return false;
    }

  g_names = names;
  g_name_count = name_count;

  g_file_index = 0;
  if (!get_new_logfile_name(g_filename))
    {
      pdm_loge("Could not create logfile\n");
      return false;
    }

  if (!prepare_logfile(g_filename))
    {
      pdm_loge("Could not prepare logfile\n");
      return false;
    }

  pdm_log("Datahandler: Init (%s)\n", g_filename);

  return true;
}

/*
 * Reset the contents of the internal data fields so a new
 * measurement record can be started.
 */
bool datahandler_new_record(uint32_t unixtime)
{
  if (g_name_count == 0)
    {
      pdm_loge("Data handler not initialized\n");
      return false;
    }

  g_unixtime = unixtime;

  reset_data();

  return true;
}

/*
 * Add a value to the given field name of the current data record.
 * Returns false if the given name does not exist.
 */
bool datahandler_add_data(char *name, double value)
{
  return add_data(name, value);
}

/*
 * Save the current data record to a log on the sd-card.
 */
bool datahandler_save_record(void)
{
  int i;
  FILE *log_fd;
  bool res = true;
  static char buffer[BUFFER_LEN];

  cJSON *object;
  cJSON *measurement;

  log_fd = fopen(g_filename, "a");

  if (!log_fd)
    {
      pdm_loge("Couldn't open \"%s\" for writing\n", g_filename);
      res = false;
    }
  else
    {
      /* Save all gathered measurement data */

      for (i = 0; i < g_name_count; i++)
        {
          /* Create container object */

          measurement = cJSON_CreateObject();
          if (measurement == NULL)
            {
              pdm_loge("Could not create json data measurement\n");
            }

          /* Add measurement name */

          object = cJSON_CreateString(g_names[i]);
          if (object == NULL)
            {
              pdm_loge("Could not create json data name\n");
            }
          cJSON_AddItemToObject(measurement, "n", object);

          /* Add measurement value after limiting its precision */

          snprintf(buffer, BUFFER_LEN, "%.*f", DATA_DECIMALS, g_data[i]);

          object = cJSON_CreateNumber(atof(buffer));
          if (object == NULL)
            {
              pdm_loge("Could not create json data value\n");
            }
          cJSON_AddItemToObject(measurement, "v", object);

          /* Add timestamp */

          object = cJSON_CreateNumber(g_unixtime);
          if (object == NULL)
            {
              pdm_loge("Could not create json data timestamp\n");
            }
          cJSON_AddItemToObject(measurement, "t", object);

          /* Generate JSON string and write it to the log file */

          res = (bool) cJSON_PrintPreallocated(measurement, buffer,
                          BUFFER_LEN, false);
          if (res)
            {
              fprintf(log_fd, "%s,", buffer);
            }
          else
            {
              pdm_loge("Couldn't generate json file record\n");
            }

          cJSON_Delete(measurement);
        }

      fclose(log_fd);
    }

  return res;
}

/*
 * Allocate a buffer of maximum size <max> and fill it up with
 * gathered measurements in json format. The buffer must not be
 * free'd by the caller. Returns true if all data has been
 * processed or false if the is more data in the log so that
 * more calsl should be made.
 */
bool datahandler_report_json(size_t max, char **out)
{
  static bool done = true;
  char *report;
  char *report_end;
  struct stat info;
  static FILE *fd = NULL;
  static int offset = 0;
  static size_t size = 0;

  /* Size needed in buffer for opening and closing brackets */

  g_head_foot_len = strlen(g_report_header) + strlen(g_report_footer);

  /* First file read. Note that file descriptor, offset etc. are
     static and kept between calls. They are reset after we are
     done sending a complete report and starting a new one */

  if (done)
    {
      /* Get file size */

      stat(g_filename, &info);
      size = info.st_size;
      offset = 0;

      fd = fopen(g_filename, "rb");
      if (!fd)
        {
          pdm_loge("Couldn't open json logfile\n");

          return true;
        }
    }

  /* Allocate report output buffer if needed */

  if (g_report_buf == NULL || g_report_max != max)
    {
      g_report_max = max;
      if (g_report_buf != NULL)
        {
          free(g_report_buf);
        }

      g_report_buf = (char *)malloc(max);
      if (g_report_buf == NULL)
        {
          pdm_loge("Couldn't allocate buffer for json logfile\n");
          return true;
        }
    }

  *out = g_report_buf;

  /* Add header at start of report */

  report = g_report_buf;
  report += sprintf(report, "%s", g_report_header);

  /* Adjust available space for data and find next chunk */

  max -= g_head_foot_len;

  fseek(fd, offset, SEEK_SET);

  if (max < size - offset)
    {
      /* At least one full chunk left, read max size */

      fread(report, max, 1, fd);

      report_end = report + max;

      done = false;
    }
  else
    {
      /* Only a partial chunk left, read rest of the file */

      fread(report, (size - offset), 1, fd);

      report_end = report + size - offset;

      if (fd)
        {
          fclose(fd);
        }

      done = true;

      /* Reset the logfile for next report */

      if (!prepare_logfile(g_filename))
        {
          pdm_loge("Could not prepare logfile\n");
          return false;
        }
    }

  /* Find end of last complete measurement and replace the
     trailing comma with closing brackets (footer) */

  while (report_end != report + 1)
    {
      if (*report_end == ',' && *(report_end - 1) == '}')
        {
          sprintf(report_end, "%s", g_report_footer);
          break;
        }
      report_end--;
    }

  /* Add size of used data to offset for next chunk */

  if (!done)
    {
      offset += report_end - report + 1;
    }

  return done;
}

