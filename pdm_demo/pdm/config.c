/****************************************************************************
 * examples/pdm_demo/pdm/config.c
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

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "log.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_MAX_LINE  60
#define CONFIG_MAX_NAME  20
#define CONFIG_MAX_VALUE 40
#define CONFIG_DELIMITER ':'

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

struct config_t {
  char name[CONFIG_MAX_NAME];
  char value[CONFIG_MAX_VALUE];
};

struct settings_t {
  uint8_t count;
  struct config_t *config;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The current settings */

static struct settings_t set_g;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*
 * Print all loaded settings (for debug purposes)
 */
static void dump(void)
{
  int i;

  pdm_log("Found %d settings:\n", set_g.count);

  for (i = 0; i < set_g.count; i++)
    {
      pdm_log("  \"%s\" = \"%s\"\n", set_g.config[i].name, set_g.config[i].value);
    }
}

/*
 * Remove leading and trailiing spaces
 */
static void trim(char *string)
{
  char *pos = string;
  char *end = string + strlen(string);

  /* Skip leading spaces */

  while (isspace(*pos))
    {
      pos++;
    }

  /* Move text to start */

  if (string != pos)
    {
      while (pos <= end)
        {
          *string = *pos;
          string++;
          pos++;
        }
    }

  /* NULL terminate and overwrite trailing spaces with '\0' */

  do
    {
      *end = '\0';
      end--;
    }
  while (isspace(*end));
}

/*
 * Get the number of lines of text in the file including
 * empty lines and comments
 */
static int count_lines(FILE *file)
{
  char c;
  int lines = 0;
  int is_start = 1;
  int is_comment = 0;

  while (!feof(file))
    {
      c = fgetc(file);
      if (is_start && c == '#')
        {
          is_comment = 1;
        }

      if (c == '\n')
        {
          if (!is_start && !is_comment)
            {
              lines++;
            }
          is_start = 1;
          is_comment = 0;
        }
      else if (!isspace(c))
        {
          is_start = 0;
        }
    }

  rewind(file);
  return lines;
}

/*
 * Get the value associated with the given name.
 * Returns an empty string if not found.
 */
const char *config_get(const char *name)
{
  int i;
  for (i = 0; i < set_g.count; i++)
    {
      if (strcmp(name, set_g.config[i].name) == 0)
        {
          return set_g.config[i].value;
        }
    }

  pdm_logw("Config \"%d\" not found\n", name);

  return "";
}

/*
 * Get the value associated with the given name.
 * Returns the given default string if not found.
 */
const char *config_get_def(const char *name, const char *def)
{
  const char *result = config_get(name);

  if (strlen(result) == 0)
    {
      return def;
    }
  return result;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*
 * Parse file and store parsed settings in static struct set_g
 */
bool config_init(const char *filename)
{
  bool ret = false;

  FILE *file = fopen(filename, "r");
  if (file)
    {
      int i = 0;
      char *pos;

      set_g.count = count_lines(file);

      /* Allocate memory for all lines and parse them one by one */

      set_g.config = (struct config_t *)
          calloc(1, set_g.count * sizeof(struct config_t));
      if (!set_g.config)
        {
          pdm_loge("Couldn't allocate string buffer\n");
          goto exit;
        }

      while (i < set_g.count)
        {
          int len;
          char *temp;
          char buf[CONFIG_MAX_LINE] = {'\0'};

          if (!fgets(buf, CONFIG_MAX_LINE, file))
            {
              pdm_loge("Couldn't read line from %s\n", filename);
              goto exit;
            }

          /* Skip empty lines and comments (beginning with '#') */

          trim(buf);
          if (strlen(buf) == 0 || buf[0] == '#')
            {
              continue;
            }

          /* Find delimiter */

          pos = strchr(buf, CONFIG_DELIMITER);
          if (pos == NULL)
            {
              pdm_loge("No delimiter in %s\n", buf);
              goto exit;
            }

          len = pos - buf;
          for (temp = pos - 1; isspace(*temp); temp--)
            {
              len--;
            }
          strncpy(set_g.config[i].name, buf, len);

          do
            {
              pos++;
            }
          while (isspace(*pos));

          strncpy(set_g.config[i].value, pos, strlen(pos));

          i++;
        }
      ret = true;
    }
  else
    {
      pdm_loge("Couldn't open %s\n", filename);
    }

exit:
  if (file)
    {
      fclose(file);
    }

  return ret;
}

/*
 * Release resources
 */
void config_deinit(void)
{
  if (set_g.config)
    {
      free(set_g.config);
    }
  set_g.config = NULL;
}

/*
 * Get the string value associated with the given name.
 * Returns an empty string if not found.
 */
const char *config(const char *name)
{
  return config_get(name);
}

/*
 * Get the string value associated with the given name.
 * Returns the given default string if not found.
 */
const char *config_def(const char *name, const char *def)
{
  const char *result = config_get(name);

  if (strlen(result) == 0)
    {
      return def;
    }
  return result;
}

/*
 * Get the integer value associated with the given name.
 * Returns 0 if not found or conversion cannot be made.
 */
int config_int(const char *name)
{
  return atoi(config_get(name));
}

/*
 * Get the float value associated with the given name.
 * Returns 0.0 if not found or conversion cannot be made.
 */
float config_float(const char *name)
{
  return atof(config_get(name));
}

/*
 * Check if the given configuration name exists
 */
bool config_exists(const char *name)
{
  return(strcmp(config_get(name), "") != 0);
}
