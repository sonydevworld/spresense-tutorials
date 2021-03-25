/****************************************************************************
 * examples/pdm_demo/pdm/log.h
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

#ifndef __EXAMPLES_PDM_DEMO_LOG_H
#define __EXAMPLES_PDM_DEMO_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAKE_STR(x) #x
#define INT_TO_STR(x) MAKE_STR(x)

#define LOGD_PREFIX "DEBUG [" __FILE__ ":" INT_TO_STR(__LINE__) "]: "
#define LOGW_PREFIX "WARNING [" __FILE__ ":" INT_TO_STR(__LINE__) "]: "
#define LOGE_PREFIX "ERROR [" __FILE__ ":" INT_TO_STR(__LINE__) "]: "

/****************************************************************************
 * Public Pre-processor Definitions
 ****************************************************************************/

#define pdm_log(fmt, ...)   pdm_log_func(fmt, ##__VA_ARGS__)
#define pdm_logd(fmt, ...)  pdm_log_func(LOGD_PREFIX fmt, ##__VA_ARGS__)
#define pdm_logw(fmt, ...)  pdm_log_func(LOGW_PREFIX fmt, ##__VA_ARGS__)
#define pdm_loge(fmt, ...)  pdm_log_func(LOGE_PREFIX fmt, ##__VA_ARGS__)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Not intended to be used directly */

void pdm_log_func(const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* __EXAMPLES_PDM_DEMO_LOG_H */
