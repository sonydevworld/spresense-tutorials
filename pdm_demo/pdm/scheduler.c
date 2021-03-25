/****************************************************************************
 * examples/pdm_demo/pdm/scheduler.c
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
#include <stdint.h>
#include <stdlib.h>
#include <sys/time.h>

#include "netutils/ntpclient.h"

#include "log.h"
#include "scheduler.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NTP_CLOCK_CHECKS    20
#define NTP_TIMEOUT_MS      500
#define NTP_REBOOT_ON_ERROR
#define NTP_DO_SYNC

#define LOG_TASK_TIME

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/* Internal task representation */

struct task_item_t;

struct task_item_t {
  const char *name;
  scheduler_task_t cb;
  int32_t period;
  int32_t left;
  struct task_item_t *next;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* String buffers for retrieving current date and time */

static char date_buffer[12];
static char time_buffer[10];

/* Internal list of registered tasks */

static struct task_item_t *tasks;

/* Internal running flag */

static bool g_running;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * Update time left of the given task by subtracting elapsed time.
 * If it ends up lower than 0 reset to full period minus rest of elapsed.
 */
static bool update_task(struct task_item_t *task, uint16_t elapsed)
{
  task->left -= elapsed;

  if (task->left <= 0)
    {
      task->left = task->period + task->left;

      return true;
    }

  return false;
}

/**
 * Find the task with the least amount of time left and sleep
 * for that amount of time
 */
static uint32_t scheduler_sleep()
{
  uint32_t elapsed;
  uint16_t shortest_sleep = UINT16_MAX;
  struct task_item_t *current = tasks;

  while (current != NULL)
    {
      if (current->left < shortest_sleep)
        {
          shortest_sleep = current->left;
        }

      current = current->next;
    }

  pdm_log("Scheduler: Sleep %d s\n", shortest_sleep);

  if (shortest_sleep > 0)
    {
      usleep(shortest_sleep * 1000000);
    }

  return shortest_sleep;
}

/*
 * Scheduler main loop. Sleep until at least one task has expired.
 * Then execute all expired tasks until we can go to sleep again.
 */
static bool scheduler_loop(void)
{
  uint32_t passed_time = 0;
  uint32_t start_time = 0;
  struct task_item_t *current;
  bool done;

  while (g_running)
    {
      /* Woke up, check time and execute while we have expired tasks */

      do
        {
          done = true;

          start_time = scheduler_time();

          current = tasks;
          while (current != NULL)
            {
              /* If a task has expired, execute its callback and
                 continue running in case more tasks have expired */

              if (update_task(current, passed_time))
                {
                  done = false;
#ifdef LOG_TASK_TIME
                  pdm_log("\t\nScheduler: Run \"%s\" at %s\n",
                          current->name, scheduler_time_string());
#else
                  pdm_log("\t\nScheduler: Run \"%s\"\n", current->name);
#endif
                  current->cb();
                }
              current = current->next;
            }

          passed_time = scheduler_time() - start_time;
        }
      while (!done);

      /* No more expired tasks, start over and sleep */

      passed_time = scheduler_sleep();
    }

  return true;
}

/**
 * Start the NTP client and try to sync system time with the server
 */
static bool sync_ntp_time(void)
{
  uint16_t checks;

  ntpc_start();

  checks = 0;
  while (checks < NTP_CLOCK_CHECKS)
    {
      if (scheduler_time() > 100000)
        {
          break;
        }

      usleep(NTP_TIMEOUT_MS * 1000);
      checks ++;
    }

  ntpc_stop();

  if (scheduler_time() < 100000)
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * Initialize the scheduler and set system time
 */
bool scheduler_init(void)
{
  /* Try to sync time with server */

  pdm_log("Scheduler: Init. Updating system time...\n");

#ifdef NTP_DO_SYNC
  if (!sync_ntp_time())
    {
#ifdef NTP_REBOOT_ON_ERROR
      pdm_loge("NTP sync failed. Rebooting...\n\t\n");

      usleep(1000000);
      if (system("reboot"))
        {
          pdm_loge("Couldn't reboot the system\n");
        }
#else
      pdm_loge("NTP sync failed.\n");

      return false;
#endif /* NTP_REBOOT_ON_ERROR */
    }
#endif /* NTP_DO_SYNC */

  pdm_log("Scheduler: Time set to %s %s\n",
          scheduler_date_string(), scheduler_time_string());

  tasks = NULL;

  return true;
}

/**
 * Release all scheduler resources
 */
void scheduler_deinit(void)
{
  struct task_item_t *current = tasks;
  struct task_item_t *next = tasks;

  while (current != NULL)
    {
      next = current->next;
      free(current);
      current = next;
    }
  tasks = NULL;

  pdm_log("Scheduler: Deinit\n");
}

/**
 * Register a new scheduler task with the provided name.
 * The given callback function will be called when the set
 * period of time (in seconds) has expired.
 */
void scheduler_register(const char *name, scheduler_task_t callback,
                        uint16_t period)
{
  struct task_item_t *task;

  /* Allocate and initialize the new task */

  task = (struct task_item_t *) malloc(sizeof(struct task_item_t));
  if (task == NULL)
    {
      pdm_loge("Failed to allocate new task\n");

      return;
    }

  task->name = name;
  task->cb = callback;
  task->period = period;
  task->left = period;
  task->next = NULL;

  /* Add the new task to the task list */

  if (tasks == NULL)
    {
      tasks = task;
    }
  else
    {
      struct task_item_t *current = tasks;

      while (current->next != NULL)
        {
          current = current->next;
        }
      current->next = task;
    }

  pdm_log("Scheduler: Registered task \"%s\" (%d s)\n", name, period);
}

/**
 * Start running the scheduler, i.e. enter the blocking scheduler
 * loop that does time keeping and runs tasks accordingly
 */
bool scheduler_start(void)
{
  g_running = true;

  return scheduler_loop();
}

/**
 * Stop running the scheduler
 */
void scheduler_stop(void)
{
  g_running = false;
}

/**
 * Retrieve the current unix time
 */
uint32_t scheduler_time(void)
{
  struct timeval tv;

  gettimeofday(&tv, NULL);

  return tv.tv_sec;
}

/**
 * Retrieve a string containing the current date
 */
const char *scheduler_date_string(void)
{
  time_t t = time(NULL);
  struct tm tm = *localtime(&t);

  snprintf(date_buffer, 12, "%d-%02d-%02d",
           tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday);

  return date_buffer;
}

/**
 * Retrieve a string containing the current time
 */
const char *scheduler_time_string(void)
{
  time_t t = time(NULL);
  struct tm tm = *localtime(&t);

  snprintf(time_buffer, 10, "%02d:%02d:%02d",
           tm.tm_hour, tm.tm_min, tm.tm_sec);

  return time_buffer;
}

