/****************************************************************************
 * examples/pdm_demo/pdm/scheduler.h
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

#ifndef __EXAMPLES_PDM_DEMO_SCHEDULER_H
#define __EXAMPLES_PDM_DEMO_SCHEDULER_H

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

typedef bool (*scheduler_task_t)(void);

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/*
 * The scheduler handles tasks that shall be run peridodically separated
 * by a set period and makes sure execution is halted the correct amount
 * if time between tasks during sleep
 */
bool scheduler_init(void);

/*
 * Clear task list and release used memory
 */
void scheduler_deinit(void);

/*
 * Register a new periodic task to run when the given number of seconds
 * has passed. The tag (single bit) is used for indentification during wakeup
 */
void scheduler_register(const char *name, scheduler_task_t callback,
                            uint16_t period);

/*
 * Starts running the scheduler. This will block and run the specified
 * callback functions when their periods have passed
 */
bool scheduler_start(void);

/*
 * Stop the scheduler. Execution will continue from where start was called
 */
void scheduler_stop(void);

/*
 * Get current unix time (seconds since Jan 1 1970)
 */
uint32_t scheduler_time(void);

/*
 * Get a string containing the current date. The pointer returned points
 * to an internal buffer so please do not modify or try to free it.
 */
const char *scheduler_date_string(void);

/*
 * Get a string containing the current time. The pointer returned points
 * to an internal buffer so please do not modify or try to free it.
 */
const char *scheduler_time_string(void);

#ifdef __cplusplus
}
#endif

#endif /* __EXAMPLES_PDM_DEMO_SCHEDULER_H */

