
/****************************************************************************
 * examples/pdm_demo/pdm/api_senseye.cxx
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
#include <string.h>
#include <netutils/cJSON.h>

#include "log.h"
#include "net.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BUFFER_SIZE 1024

#define TOKEN_MAX 1300

#define SENSORID_SIZE 38

/****************************************************************************
 * Private data
 ****************************************************************************/

static const char path_auth[] = "/oauth/token";
static const char path_post[] = "/v1/hub/sensors/%s/data";
static const char path_sensor_list[] = "/v1/hub/sensors";
static const char path_sensor_info[] = "/v1/hub/sensors/%s";
static const char path_sensor_data[] = "/v1/hub/sensors/%s/data";
static const char path_measure_list[] = "/v1/hub/sensors/%s/measures";
static const char path_measure_data[] = "/v1/hub/sensors/%s/measures/%s/data";

static char buffer[BUFFER_SIZE];
static size_t buffer_len;
static char *buffer_pos;

static char token[TOKEN_MAX];

static size_t max_post_size = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*
 * Reset the internal send/receive buffer
 */
static void reset_buffer()
{
  buffer_pos = buffer;
  buffer_len = 0;
  memset(buffer, 0, BUFFER_SIZE);
}

/*
 * Send the given data to the given server path and receive the
 * server response in the internal buffer
 */
static bool do_request_post(char *path, char *data, bool is_last)
{
  int result;
  int status = 0;

  result = net_send_data(NET_HTTP_POST, NET_HTTP_CONTENT_JSON,
                         path, data, token, is_last);
  if (!result)
    {
      pdm_loge("Failed to post sensor data (%d)\n", result);
      goto exit;
    }

  result = net_read();
  if (!result)
    {
      pdm_loge("Failed to read response (%d)\n", result);
      goto exit;
    }

  status = net_getstatus();
  pdm_log("HTTP post status = %d\n", status);

exit:
  return net_status_success(status);
}

/*
 * Retrieve data from the specified server path and return the
 * contents or NULL on error
 */
static char *do_request_get(char *path)
{
  int result;
  int status = 0;

  result = net_send(NET_HTTP_GET, NET_HTTP_CONTENT_URL, path, token);
  if (!result)
    {
      pdm_loge("Failed to post sensor data (%d)\n", result);
      goto exit;
    }

  result = net_read();
  if (!result)
    {
      pdm_loge("Failed to read response (%d)\n", result);
      goto exit;
    }

  status = net_getstatus();

  if (net_status_success(status))
    {
      return net_getbody();
    }
  pdm_loge("HTTP request failed with status %d\n", status);

exit:
  return NULL;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*
 * Establish connection to backend end retrieve bearer token
 */
bool api_senseye_init(const char *user, const char *pass)
{
  cJSON *json_body;
  cJSON *json_token;

  pdm_log("Senseye: Init\n");

  reset_buffer();

  /* Construct the URL encoded request body in buffer */

  buffer_pos += sprintf(buffer_pos, "username=");
  buffer_pos += net_percent_encode(user, buffer_pos, (buffer_pos - buffer));

  buffer_pos += sprintf(buffer_pos, "&password=");
  buffer_pos += net_percent_encode(pass, buffer_pos, (buffer_pos - buffer));

  buffer_pos += sprintf(buffer_pos, "&grant_type=password");

  /* Send the request */

  net_send_data(NET_HTTP_POST, NET_HTTP_CONTENT_URL,
                path_auth, buffer, NULL, true);

  net_read();

  /* Fetch the json response and parse it to get the access token */

  const char *body = net_getbody();

  json_body = cJSON_Parse(body);
  if (json_body == NULL)
    {
      pdm_loge("Failed to parse json response\n");
      return false;
    }

  json_token = cJSON_GetObjectItemCaseSensitive(json_body, "access_token");
  if (!cJSON_IsString(json_token))
    {
      pdm_loge("Failed to retrieve bearer token\n");
      return false;
    }

  /* Save token for future requests */

  strcpy(token, json_token->valuestring);

  /* Save estimation of max post data size */

  max_post_size = net_max_data_size() -
                  (strlen(token) - strlen(path_post) + SENSORID_SIZE);

  /* Cleanup */

  cJSON_Delete(json_body);

  return true;
}

size_t api_senseye_max_post_size(void)
{
  return max_post_size;
}

bool api_senseye_post(const char *sensor_id, char *json, bool is_last)
{
  reset_buffer();
  snprintf(buffer, BUFFER_SIZE, path_post, sensor_id);

  return do_request_post(buffer, json, is_last);
}

char *api_senseye_sensor_list(void)
{
  reset_buffer();
  snprintf(buffer, BUFFER_SIZE, path_sensor_list);

  return do_request_get(buffer);
}

char *api_senseye_sensor_info(const char *sensor_id)
{
  reset_buffer();
  snprintf(buffer, BUFFER_SIZE, path_sensor_info, sensor_id);

  return do_request_get(buffer);
}

char *api_senseye_sensor_data(const char *sensor_id)
{
  reset_buffer();
  snprintf(buffer, BUFFER_SIZE, path_sensor_data, sensor_id);

  return do_request_get(buffer);
}

char *api_senseye_measure_list(const char *sensor_id)
{
  reset_buffer();
  snprintf(buffer, BUFFER_SIZE, path_measure_list, sensor_id);

  return do_request_get(buffer);
}

char *api_senseye_measure_data(const char *sensor_id, const char *measure)
{
  reset_buffer();
  snprintf(buffer, BUFFER_SIZE, path_measure_data, sensor_id, measure);

  return do_request_get(buffer);
}

