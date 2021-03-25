/****************************************************************************
 * examples/pdm_demo/pdm/net.h
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

#ifndef __EXAMPLES_PDM_DEMO_NET_H
#define __EXAMPLES_PDM_DEMO_NET_H

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

enum net_http_request_t {
  NET_HTTP_POST,
  NET_HTTP_GET
};

enum net_http_content_t {
  NET_HTTP_CONTENT_JSON,
  NET_HTTP_CONTENT_URL
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

bool net_init(const char *wifi_ssid, const char *wifi_pass);
bool net_connect(const char *host, const char *port);
bool net_disconnect(void);

size_t net_max_data_size(void);

int net_send(enum net_http_request_t request,
             enum net_http_content_t type,
             const char *path, char *token);
int net_send_data(enum net_http_request_t request,
                  enum net_http_content_t type,
                  const char *path, const char *data,
                  char *token, bool is_last);
int net_read(void);
const char *net_getbody(void);

int net_getstatus(void);
bool net_status_success(int status);

size_t net_percent_encode(const char *input, const char *output,
                          size_t output_size);

#ifdef __cplusplus
}
#endif

#endif /* __EXAMPLES_PDM_DEMO_NET_H */
