/****************************************************************************
 * examples/pdm_demo/pdm/net.cxx
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

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <mbedtls/net.h>
#include <mbedtls/ssl.h>
#include <mbedtls/entropy.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/debug.h>

#include "log.h"
#include "net.h"
#include "config/cert.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HOSTNAME_MAX 30
#define BUFFER_MAX   4500

#define SOCKET_TRIES        20
#define SOCKET_TIMEOUT_MS   500
#define SOCKET_START_WAIT_S 6

//#define DBG_DUMP_HTTP

/****************************************************************************
 * Private Data
 ****************************************************************************/

static  mbedtls_net_context server_fd;
static  mbedtls_entropy_context entropy;
static  mbedtls_ctr_drbg_context ctr_drbg;
static  mbedtls_ssl_context ssl;
static  mbedtls_ssl_config conf;
static  mbedtls_x509_crt cacert;

static char host[HOSTNAME_MAX] = {'\0'};

static char buffer[BUFFER_MAX];

static char req_template[] =
    "%s %s HTTP/1.1\r\n"
    "Host: %s\r\n"
    "Content-Type: application/%s\r\n"
    "Content-Length: %d\r\n"
    "\r\n%s\r\n";

static char req_token_template[] =
    "%s %s HTTP/1.1\r\n"
    "Host: %s\r\n"
    "Content-Type: application/%s\r\n"
    "Content-Length: %d\r\n"
    "Authorization: Bearer %s\r\n"
    "\r\n%s\r\n";

static char req_token_close_template[] =
    "%s %s HTTP/1.1\r\n"
    "Host: %s\r\n"
    "Content-Type: application/%s\r\n"
    "Content-Length: %d\r\n"
    "Connection: close\r\n"
    "Authorization: Bearer %s\r\n"
    "\r\n%s\r\n";

static char temp_wifi[] = "gs2200m %s %s &";

static int max_data_size;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

bool connect_wifi(const char *ssid, const char *pass)
{
  int status;

  /* Start gs2200m WiFi usrsck daemon */

  snprintf(buffer, BUFFER_MAX, temp_wifi, ssid, pass);

  status = system(buffer);
  if (status == -1) {
    pdm_loge("Couldn't connect to WiFi\n");
    return false;
  }

  /* Haven't found a good way to check if daemon started, just wait a bit */

  usleep(SOCKET_START_WAIT_S * 1000000);

  pdm_log("Net: WiFi initialized\n");

  return true;
}

char *get_request_name(enum net_http_request_t request)
{
  if (request == NET_HTTP_GET)
    {
      return "GET";
    }
  return "POST";
}

char *get_content_name(enum net_http_content_t content)
{
  if (content == NET_HTTP_CONTENT_JSON)
    {
      return "json";
    }
  return "x-www-form-urlencoded";
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

bool net_init(const char *wifi_ssid, const char *wifi_pass)
{
  /* Connect using gs2200m */

  if (!connect_wifi(wifi_ssid, wifi_pass))
    {
      pdm_loge("WiFi connection failed\n");
      return false;
    }

  /* Estimate max data size for sending, token and path not included */

  max_data_size = BUFFER_MAX -
                  (strlen(req_token_template) +
                   strlen(get_request_name(NET_HTTP_POST)) +
                   strlen(get_content_name(NET_HTTP_CONTENT_URL)) +
                   HOSTNAME_MAX +
                   4 /* size of content length value */
                   );

  return true;
}

bool net_connect(const char *hostname, const char *port)
{
  int32_t ret;
  const char *pers = "pdm_demo_string";

  /* Initialize mbedtls internal state and certs */

  mbedtls_net_init(&server_fd);
  mbedtls_ssl_init(&ssl);
  mbedtls_ssl_config_init(&conf);
  mbedtls_ctr_drbg_init(&ctr_drbg);
  mbedtls_x509_crt_init(&cacert);
  mbedtls_entropy_init(&entropy);

  ret = mbedtls_ctr_drbg_seed(&ctr_drbg,
                              mbedtls_entropy_func,
                              &entropy,
                              (const unsigned char *) pers,
                              strlen(pers));
  if (ret != 0)
    {
      pdm_loge("mbedtls_ctr_drbg_seed failed (%d)\n", ret);
      return false;
    }

  ret = mbedtls_x509_crt_parse(&cacert, pdm_ca_certs, sizeof(pdm_ca_certs));
  if (ret != 0)
    {
      pdm_loge("mbedtls_x509_crt_parse failed (%d)\n", ret);
      return false;
    }

  ret = mbedtls_ssl_config_defaults(&conf,
                                    MBEDTLS_SSL_IS_CLIENT,
                                    MBEDTLS_SSL_TRANSPORT_STREAM,
                                    MBEDTLS_SSL_PRESET_DEFAULT);
  if (ret != 0)
    {
      pdm_loge("mbedtls_ssl_config_defaults failed (%d)\n", ret);
      return false;
    }

  mbedtls_ssl_conf_authmode(&conf, MBEDTLS_SSL_VERIFY_OPTIONAL);
  mbedtls_ssl_conf_ca_chain(&conf, &cacert, NULL);
  mbedtls_ssl_conf_rng(&conf, mbedtls_ctr_drbg_random, &ctr_drbg);

  ret = mbedtls_ssl_setup(&ssl, &conf);
  if (ret != 0)
    {
      pdm_loge("mbedtls_ssl_setup failed (%d)\n", ret);
      return false;
    }
  strcpy(host, hostname);

  ret = mbedtls_ssl_set_hostname(&ssl, host);
  if (ret != 0)
    {
      pdm_loge("mbedtls_ssl_set_hostname failed (%d)\n", ret);
      return false;
    }

  mbedtls_ssl_set_bio(&ssl, &server_fd, mbedtls_net_send, mbedtls_net_recv, NULL);

  ret = mbedtls_net_connect(&server_fd, host, port, MBEDTLS_NET_PROTO_TCP);
  if (ret != 0)
    {
      pdm_loge("mbedtls_net_connect failed (%d)\n", ret);
      return false;
    }

  do
    {
      ret = mbedtls_ssl_handshake(&ssl);
      if (ret == 0)
        {
          break;
        }

      if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE)
        {
          pdm_loge("mbedtls_ssl_handshake failed (-0x%x)\n", -ret);

          return false;
        }
    }
  while (1);

  pdm_log("Net: Connected to %s:%s\n", host, port);

  return true;
}

bool net_disconnect(void)
{
  mbedtls_ssl_close_notify(&ssl);

  mbedtls_net_free(&server_fd);
  mbedtls_ssl_free(&ssl);
  mbedtls_ssl_config_free(&conf);
  mbedtls_ctr_drbg_free(&ctr_drbg);
  mbedtls_entropy_free(&entropy);
  mbedtls_x509_crt_free(&cacert);

  pdm_log("Net: Disconnected\n");
  return true;
}

size_t net_max_data_size(void)
{
  return max_data_size;
}

int net_send(enum net_http_request_t request, enum net_http_content_t type,
             const char *path, char *token)
{
  return net_send_data(request, type, path, "", token, true);
}

int net_send_data(enum net_http_request_t request, enum net_http_content_t type,
                  const char *path, const char *data, char *token, bool is_last)
{
  int len;
  int ret = -1;

  if (token == NULL)
    {
      len = snprintf(buffer, BUFFER_MAX, req_template,
                     get_request_name(request),
                     path, host,
                     get_content_name(type),
                     strlen(data), data);
    }
  else if (is_last)
    {
      len = snprintf(buffer, BUFFER_MAX, req_token_close_template,
                     get_request_name(request),
                     path, host,
                     get_content_name(type),
                     strlen(data), token, data);
    }
  else
    {
      len = snprintf(buffer, BUFFER_MAX, req_token_template,
                     get_request_name(request),
                     path, host,
                     get_content_name(type),
                     strlen(data), token, data);
    }

  if (len + 1 > BUFFER_MAX)
    {
      pdm_loge("HTTP request too large for buffer\n");
      goto exit;
    }

#ifdef DBG_DUMP_HTTP
  pdm_log(">>>>>>\n%s\n>>>>>>\n", buffer);
#endif

  do
    {
      ret = mbedtls_ssl_write(&ssl, (const unsigned char *) buffer, strlen(buffer));
      if (ret == MBEDTLS_ERR_SSL_WANT_READ ||
          ret == MBEDTLS_ERR_SSL_WANT_WRITE)
        {
          continue;
        }
      else if (ret < 0)
        {
          pdm_loge("SSL write failed (%d)\n", ret);
          return ret;
        }
    }
  while (ret <= 0);

//  pdm_log("Net: Wrote %d bytes\n", ret);

exit:
  return ret;
}

int net_read(void)
{
  size_t len = 0;
  int ret;

  do
    {
      ret = mbedtls_ssl_read(&ssl, (unsigned char *)buffer, BUFFER_MAX - 1);
      if (ret == MBEDTLS_ERR_SSL_WANT_READ ||
          ret == MBEDTLS_ERR_SSL_WANT_WRITE)
        {
          continue;
        }
      else if (ret == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY)
        {
          pdm_loge("connection being closed by peer\n");
          goto error;
        }
      else if (ret < 0)
        {
          pdm_loge("mbedtls_ssl_read failed (%d)\n", ret);
          goto error;
        }

      len += ret;
      if (ret == 0 || ret < BUFFER_MAX)
        {
          goto done;
        }
    }
  while(1);

done:
  buffer[len] = '\0';

#ifdef DBG_DUMP_HTTP
  pdm_log("<<<<<<\n%s\n<<<<<<\n", buffer);
#endif

//  pdm_log("Net: Read %d bytes\n", len);
  return len;

error:
  return ret;
}

int net_getstatus(void)
{
  char *pos;
  const char *head = "HTTP/1.1 ";

  pos = strstr(buffer, head);
  if (pos != NULL)
    {
      char code[4];

      pos += strlen(head);
      code[0] = *pos;
      code[1] = *(pos + 1);
      code[2] = *(pos + 2);
      code[3] = '\0';
      return atoi(code);
    }
  return 0;
}

bool net_status_success(int status)
{
  return  (status >= 200 && status < 300);
}

const char *net_getbody(void)
{
  char *pos;

  pos = strstr(buffer, "\r\n\r\n");
  if (pos != NULL)
    {
      return (const char *) pos + 3;
    }
  pdm_log("Warning: HTTP response body not found\n");
  return NULL;
}

size_t net_percent_encode(const char *input, const char *output,
                          size_t output_size)
{
  char *out = (char *)output;

  static const size_t num_chars = 19;

  static const char normal[] = {'!', '#', '$', '%', '&', '\'', '(', ')',
                                '*', '+', ',', '/', ':', ';',  '=', '?',
                                '@', '[', ']' };

  static const char *encoded[] = {"21", "23", "24", "25", "26", "27", "28", "29",
                                  "2A", "2B", "2C", "2F", "3A", "3B", "3D", "3F",
                                  "40", "5B", "5D" };

  while (*input != '\0')
    {
      bool updated = false;

      for (int i = 0; i < num_chars; i++)
        {
          if (*input == normal[i])
            {
              *out = '%';
              *(out + 1) = encoded[i][0];
              *(out + 2) = encoded[i][1];

              out += 3;
              output_size -= 3;

              if (output_size <= 0)
                {
                  pdm_loge("Percent encoding buffer too small\n");
                  goto exit;
                }

              updated = true;
              break;
            }
        }

      if (!updated)
        {
          *out = *input;
          out++;
          output_size--;
        }

      input++;
    }

exit:
  *out = '\0';

  return (out - output);
}

