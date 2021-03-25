/****************************************************************************
 * pdm_demo/pdm/audio.cxx
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

#include <nuttx/config.h>
#include <stdio.h>
#include <strings.h>
#include <asmp/mpshm.h>
#include "memutils/simple_fifo/CMN_SimpleFifo.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/Message.h"
#include "audio/audio_frontend_api.h"
#include "audio/audio_capture_api.h"
#include "audio/audio_message_types.h"
#include "audio/utilities/frame_samples.h"

#include "audio/include/msgq_id.h"
#include "audio/include/mem_layout.h"
#include "audio/include/memory_layout.h"
#include "audio/include/msgq_pool.h"
#include "audio/include/pool_layout.h"
#include "audio/include/fixed_fence.h"
#include "audio/audio.h"
#include "pdm/log.h"

#include <fcntl.h>

/****************************************************************************
 * Codec parameters
 ****************************************************************************/

/* Sampling rate
 * 44.1kHz : AS_SAMPLINGRATE_44100
 * 48kHz   : AS_SAMPLINGRATE_48000
 * 192kHz  : AS_SAMPLINGRATE_192000
 */

#define DEF_SAMPLINGRATE      AS_SAMPLINGRATE_48000

/* Channel number
 * MONO (1ch)   : AS_CHANNEL_MONO
 * STEREO (2ch) : AS_CHANNEL_STEREO
 * 4ch          : AS_CHANNEL_4CH
 */

#define CHANNEL_NUMBER        AS_CHANNEL_MONO

/* Bit length
 * 16bit : AS_BITLENGTH_16
 * 24bit : AS_BITLENGTH_24
 */

#define BIT_LENGTH            AS_BITLENGTH_16

/* Size of one block to read from the FIFO
 * Set the maximum size of configurable parameters.
 */

#define SIMPLE_FIFO_READ_SIZE (1024 * 4 * 4)

/* Number of FIFO stages */

#define SIMPLE_FIFO_FRAME_NUM (30)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

using namespace MemMgrLite;

/* PCM codec search path. */

#define DSPBIN_PATH   "/mnt/sd0/BIN"

/* PCM FIFO buffer size */

#define SIMPLE_FIFO_BUF_SIZE  (SIMPLE_FIFO_READ_SIZE * SIMPLE_FIFO_FRAME_NUM)

/* For debugging */

/* #define DBG_AUDIO_DUMP */

#ifdef DBG_AUDIO_DUMP
#define DEBUG_FILENAME "/mnt/sd0/pdm_audio_dump.raw"
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/* For FIFO. */

struct fifo_info_s
{
  CMN_SimpleFifoHandle  handle;
  uint32_t              fifo_area[SIMPLE_FIFO_BUF_SIZE/sizeof(uint32_t)];
  uint8_t               write_buf[SIMPLE_FIFO_READ_SIZE];
};

/* For internal shutdown handling */

enum audio_init_stage {
  AUDIO_INIT_NONE = 0,
  AUDIO_INIT_LIBS,
  AUDIO_INIT_AUDIOSYSTEM,
  AUDIO_INIT_POWER,
  AUDIO_INIT_FIFO,
  AUDIO_INIT_FRONTEND
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Set audio processing callback */

static audio_process_cb g_process_cb = NULL;

/* For intarnal shutdown handling */

static enum audio_init_stage g_init_stage = AUDIO_INIT_NONE;

/* For FIFO. */

static fifo_info_s s_fifo;

/* For share memory. */

static mpshm_t s_shm;

/* For target codec parameters. */

static uint32_t  target_samplingrate   = DEF_SAMPLINGRATE;
static uint32_t  target_channel_number = CHANNEL_NUMBER;
static uint32_t  target_bit_lengt      = BIT_LENGTH;

#ifdef DBG_AUDIO_DUMP
int debug_fd = -1;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*
 * Initialize the FIFO for audio system communication
 */
static bool init_simple_fifo(void)
{
  if (CMN_SimpleFifoInitialize(&s_fifo.handle,
                               s_fifo.fifo_area,
                               SIMPLE_FIFO_BUF_SIZE, NULL) != 0)
    {
      pdm_loge("Fail to initialize simple FIFO.");
      return false;
    }
  CMN_SimpleFifoClear(&s_fifo.handle);

  return true;
}

/*
 * Pop a buffer from the fifo and execute callback as needed
 */
static void pop_simple_fifo(bool is_end = false)
{
  size_t occupied_simple_fifo_size =
    CMN_SimpleFifoGetOccupiedSize(&s_fifo.handle);
  uint32_t output_size = 0;

  while (occupied_simple_fifo_size > 0)
    {
      if (is_end && occupied_simple_fifo_size < SIMPLE_FIFO_READ_SIZE)
        {
          output_size = occupied_simple_fifo_size;
        }
      else
        {
          output_size = SIMPLE_FIFO_READ_SIZE;
        }

      if (CMN_SimpleFifoPoll(&s_fifo.handle,
                            (void*)s_fifo.write_buf,
                            output_size) == 0)
        {
          break;
        }

      /* Data output */

      if (g_process_cb != NULL)
      {
        g_process_cb((int8_t*)s_fifo.write_buf, output_size, is_end);
      }

#ifdef DBG_AUDIO_DUMP
      write(debug_fd, (char *)s_fifo.write_buf, output_size);
#endif

      if (is_end && occupied_simple_fifo_size < SIMPLE_FIFO_READ_SIZE)
        {
          /* At end and data size left is less than one read block */

          occupied_simple_fifo_size = CMN_SimpleFifoGetOccupiedSize(&s_fifo.handle);
        }
      else
        {
          occupied_simple_fifo_size -= output_size;
        }
    }
}

/*
 * Setup the components used in the audio system
 */
static bool create_audio_sub_system(void)
{
  bool result = false;

  /* Create Frontend. */

  AsCreateMicFrontendParams_t frontend_create_param;
  frontend_create_param.msgq_id.micfrontend = MSGQ_AUD_FRONTEND;
  frontend_create_param.msgq_id.mng         = MSGQ_AUD_MGR;
  frontend_create_param.msgq_id.dsp         = MSGQ_AUD_PREDSP;
  frontend_create_param.pool_id.input       = S0_INPUT_BUF_POOL;
  frontend_create_param.pool_id.output      = S0_NULL_POOL;
  frontend_create_param.pool_id.dsp         = S0_PRE_APU_CMD_POOL;

  AS_CreateMicFrontend(&frontend_create_param, NULL);

  /* Create Capture feature. */

  AsCreateCaptureParam_t capture_create_param;
  capture_create_param.msgq_id.dev0_req  = MSGQ_AUD_CAP;
  capture_create_param.msgq_id.dev0_sync = MSGQ_AUD_CAP_SYNC;
  capture_create_param.msgq_id.dev1_req  = 0xFF;
  capture_create_param.msgq_id.dev1_sync = 0xFF;

  result = AS_CreateCapture(&capture_create_param);
  if (!result)
    {
      pdm_loge("As_CreateCapture() failure. system memory insufficient!\n");
      return false;
    }

  return true;
}

/*
 * De-initialize the audio system by releaseing its components
 */
static void deact_audio_sub_system(void)
{
  AS_DeleteMicFrontend();
  AS_DeleteCapture();
}

static bool receive_object_reply(uint32_t id = 0)
{
  AudioObjReply reply_info;

  AS_ReceiveObjectReply(MSGQ_AUD_MGR, &reply_info);

  if (reply_info.type != AS_OBJ_REPLY_TYPE_REQ)
    {
      pdm_loge("receive_object_reply() type 0x%x failed\n",
             reply_info.type);
      return false;
    }

  if (id && reply_info.id != id)
    {
      pdm_loge("receive_object_reply() id 0x%x failed (request id 0x%x)\n",
             reply_info.id, id);
      return false;
    }

  if (reply_info.result != OK)
    {
      pdm_loge("receive_object_reply() failed (result 0x%x)\n",
             reply_info.result);
      return false;
    }

  return true;
}

/*
 * Power on the audio hardware
 */
static bool power_on(void)
{
  return (cxd56_audio_poweron() == CXD56_AUDIO_ECODE_OK);
}

/*
 * Power off the audio hardware
 */
static bool power_off(void)
{
  return (cxd56_audio_poweroff() == CXD56_AUDIO_ECODE_OK);
}

/*
 * Disable mic and put audiosystem in ready state
 */
static bool set_ready(void)
{
  AsDeactivateMicFrontendParam  deact_param;

  AS_DeactivateMicFrontend(&deact_param);

  if (!receive_object_reply(MSG_AUD_MFE_CMD_DEACT))
    {
      /* To end processing */;
    }

  /* Disable input */

  if (cxd56_audio_dis_input() != CXD56_AUDIO_ECODE_OK)
    {
      return false;
    }

  return true;
}

/*
 * Set initial mic gain to 0 for all channels
 */
static bool init_mic_gain(void)
{
  cxd56_audio_mic_gain_t  mic_gain;

  mic_gain.gain[0] = 0;
  mic_gain.gain[1] = 0;
  mic_gain.gain[2] = 0;
  mic_gain.gain[3] = 0;
  mic_gain.gain[4] = 0;
  mic_gain.gain[5] = 0;
  mic_gain.gain[6] = 0;
  mic_gain.gain[7] = 0;

  return (cxd56_audio_set_micgain(&mic_gain) == CXD56_AUDIO_ECODE_OK);
}

/*
 * Enable microphone frontend
 */
static bool set_frontend(void)
{
  /* Enable input. */

  if (cxd56_audio_en_input() != CXD56_AUDIO_ECODE_OK)
    {
      return false;
    }

  /* Set frontend parameter */

  AsActivateMicFrontend act_param;

  act_param.param.input_device = AsMicFrontendDeviceMic;
  act_param.cb                 = NULL;

  AS_ActivateMicFrontend(&act_param);

  if (!receive_object_reply(MSG_AUD_MFE_CMD_ACT))
    {
      return false;
    }

  return true;
}

/*
 * Initialize the microphone frontend
 */
static bool init_frontend(void)
{
  /* Set frontend init parameter */

  AsInitMicFrontendParam  init_param;

  init_param.channel_number           = target_channel_number;
  init_param.bit_length               = target_bit_lengt;
  init_param.samples_per_frame        = getCapSampleNumPerFrame(AS_CODECTYPE_LPCM,
                                                                target_samplingrate);
  init_param.preproc_type             = AsMicFrontendPreProcThrough;
  init_param.data_path                = AsDataPathSimpleFIFO;
  init_param.dest.simple_fifo_handler = &s_fifo.handle;

  AS_InitMicFrontend(&init_param);

  return receive_object_reply(MSG_AUD_MFE_CMD_INIT);
}

/*
 * Start recording from the microphone
 */
static bool start_capture(void)
{
  CMN_SimpleFifoClear(&s_fifo.handle);

  AsStartMicFrontendParam cmd;

  AS_StartMicFrontend(&cmd);

  return receive_object_reply(MSG_AUD_MFE_CMD_START);
}

/*
 * Stop recording
 */
static bool stop_capture(void)
{
  AsStopMicFrontendParam  cmd;

  cmd.stop_mode = 0;

  AS_StopMicFrontend(&cmd);

  if (!receive_object_reply())
    {
      return false;
    }

  /* Spout processing of remaining data. */

  pop_simple_fifo(true);

  return true;
}

/*
 * Select whether to use the high resolution or regular
 * clock mode depending on the selected sample rate
 */
static bool set_clkmode(void)
{
  cxd56_audio_clkmode_t clk_mode;

  if (target_samplingrate == AS_SAMPLINGRATE_192000)
    {
      clk_mode = CXD56_AUDIO_CLKMODE_HIRES;
    }
  else
    {
      clk_mode = CXD56_AUDIO_CLKMODE_NORMAL;
    }

  return (cxd56_audio_set_clkmode(clk_mode) == CXD56_AUDIO_ECODE_OK);
}

/*
 * Initialize support libraries for shared memory and
 * message handling used in the audio system
 */
static bool init_libraries(void)
{
  int ret;
  uint32_t addr = AUD_SRAM_ADDR;

  /* Initialize shared memory.*/

  ret = mpshm_init(&s_shm, 1, 1024 * 128 * 2);
  if (ret < 0)
    {
      pdm_loge("mpshm_init() failure. %d\n", ret);
      return false;
    }

  ret = mpshm_remap(&s_shm, (void *)addr);
  if (ret < 0)
    {
      pdm_loge("mpshm_remap() failure. %d\n", ret);
      return false;
    }

  /* Initalize MessageLib. */

  err_t err = MsgLib::initFirst(NUM_MSGQ_POOLS, MSGQ_TOP_DRM);
  if (err != ERR_OK)
    {
      pdm_loge("MsgLib::initFirst() failure. 0x%x\n", err);
      return false;
    }

  err = MsgLib::initPerCpu();
  if (err != ERR_OK)
    {
      pdm_loge("MsgLib::initPerCpu() failure. 0x%x\n", err);
      return false;
    }

  void* mml_data_area = translatePoolAddrToVa(MEMMGR_DATA_AREA_ADDR);
  err = Manager::initFirst(mml_data_area, MEMMGR_DATA_AREA_SIZE);
  if (err != ERR_OK)
    {
      pdm_loge("Manager::initFirst() failure. 0x%x\n", err);
      return false;
    }

  err = Manager::initPerCpu(mml_data_area, static_pools, pool_num, layout_no);
  if (err != ERR_OK)
    {
      pdm_loge("Manager::initPerCpu() failure. 0x%x\n", err);
      return false;
    }

  /* Create static memory pool */

  const uint8_t sec_no = AUDIO_SECTION;
  const NumLayout layout_no = MEM_LAYOUT_PCM_CAPTURE;
  void* work_va = translatePoolAddrToVa(S0_MEMMGR_WORK_AREA_ADDR);
  const PoolSectionAttr *ptr  = &MemoryPoolLayouts[AUDIO_SECTION][layout_no][0];
  err = Manager::createStaticPools(sec_no,
                                   layout_no,
                                   work_va,
                                   S0_MEMMGR_WORK_AREA_SIZE,
                                   ptr);
  if (err != ERR_OK)
    {
      pdm_loge("Manager::createStaticPools() failure. %d\n", err);
      return false;
    }

  return true;
}

/*
 * Release support libraries and related resources
 */
static bool finalize_libraries(void)
{
  /* Finalize MessageLib. */

  MsgLib::finalize();

  /* Destroy static pools. */

  MemMgrLite::Manager::destroyStaticPools(AUDIO_SECTION);

  /* Finalize memory manager. */

  MemMgrLite::Manager::finalize();

  /* Destroy shared memory. */

  int ret;
  ret = mpshm_detach(&s_shm);
  if (ret < 0)
    {
      pdm_loge("mpshm_detach() failure. %d\n", ret);
      return false;
    }

  ret = mpshm_destroy(&s_shm);
  if (ret < 0)
    {
      pdm_loge("mpshm_destroy() failure. %d\n", ret);
      return false;
    }

  return true;
}

/*
 * Record data for the given number of seconds
 */
static void capture_process(uint32_t rec_time)
{
  /* Timer Start */

  time_t start_time;
  time_t cur_time;

  time(&start_time);

  do
    {
      pop_simple_fifo();

    } while((time(&cur_time) - start_time) < rec_time);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*
 * Set callback function to be called when a full audio buffer is ready
 */
void audio_set_process_cb(audio_process_cb cb)
{
  g_process_cb = cb;
}

/*
 * Get the audio buffer size in use
 */
size_t audio_buffer_size()
{
  return SIMPLE_FIFO_READ_SIZE;
}

/*
 * Get the audio sample rate currently being used
 */
uint32_t audio_sample_rate()
{
  switch (target_samplingrate)
    {
      case AS_SAMPLINGRATE_8000:  return 8000;
      case AS_SAMPLINGRATE_11025: return 11025;
      case AS_SAMPLINGRATE_12000: return 12000;
      case AS_SAMPLINGRATE_16000: return 16000;
      case AS_SAMPLINGRATE_22050: return 22050;
      case AS_SAMPLINGRATE_24000: return 24000;
      case AS_SAMPLINGRATE_32000: return 32000;
      case AS_SAMPLINGRATE_44100: return 44100;
      case AS_SAMPLINGRATE_48000: return 48000;
      case AS_SAMPLINGRATE_64000: return 64000;
      case AS_SAMPLINGRATE_88200: return 88200;
      case AS_SAMPLINGRATE_96000: return 96000;
      case AS_SAMPLINGRATE_128000: return 128000;
      case AS_SAMPLINGRATE_176400: return 176400;
      case AS_SAMPLINGRATE_192000: return 192000;
      default:
        return 0; /* AS_SAMPLINGRATE_AUTO */
    }
}

/*
 * Setup the audio system for recording
 */
bool audio_init(uint32_t samplerate)
{
  pdm_log("Audio: Init\n");

  /* Set samplerate */

  switch (samplerate)
    {
      case 8000:  target_samplingrate = AS_SAMPLINGRATE_8000;  break;
      case 11025: target_samplingrate = AS_SAMPLINGRATE_11025; break;
      case 12000: target_samplingrate = AS_SAMPLINGRATE_12000; break;
      case 16000: target_samplingrate = AS_SAMPLINGRATE_16000; break;
      case 22050: target_samplingrate = AS_SAMPLINGRATE_22050; break;
      case 24000: target_samplingrate = AS_SAMPLINGRATE_24000; break;
      case 32000: target_samplingrate = AS_SAMPLINGRATE_32000; break;
      case 44100: target_samplingrate = AS_SAMPLINGRATE_44100; break;
      case 48000: target_samplingrate = AS_SAMPLINGRATE_48000; break;
      case 64000: target_samplingrate = AS_SAMPLINGRATE_64000; break;
      case 88200: target_samplingrate = AS_SAMPLINGRATE_88200; break;
      case 96000: target_samplingrate = AS_SAMPLINGRATE_96000; break;
      case 128000: target_samplingrate = AS_SAMPLINGRATE_128000; break;
      case 176400: target_samplingrate = AS_SAMPLINGRATE_176400; break;
      case 192000: target_samplingrate = AS_SAMPLINGRATE_192000; break;
      default:
        target_samplingrate = AS_SAMPLINGRATE_AUTO; break;
    }

  /* First, initialize the shared memory and memory utility used by AudioSubSystem. */

  g_init_stage = AUDIO_INIT_LIBS;
  if (!init_libraries())
    {
      pdm_loge("init_libraries() failure.\n");
      return false;
    }

  /* Next, Create the features used by AudioSubSystem. */

  g_init_stage = AUDIO_INIT_AUDIOSYSTEM;
  if (!create_audio_sub_system())
    {
      pdm_loge("act_audiosubsystem() failure.\n");
      return false;
    }

  /* On and after this point, AudioSubSystem must be active.
   * Register the callback function to be notified when a problem occurs.
   */

  /* Change AudioSubsystem to Ready state so that I/O parameters can be changed. */

  g_init_stage = AUDIO_INIT_POWER;
  if (!power_on())
    {
      pdm_loge("power_on() failure.\n");
      return false;
    }

  /* Initialize simple fifo. */

  g_init_stage = AUDIO_INIT_FIFO;
  if (!init_simple_fifo())
    {
      pdm_loge("init_simple_fifo() failure.\n");
      return false;
    }

  /* Set the initial gain of the microphone to be used. */

  if (!init_mic_gain())
    {
      pdm_loge("init_mic_gain() failure.\n");
      return false;
    }

  /* Set audio clock mode. */

  if (!set_clkmode())
    {
      pdm_loge("set_clkmode() failure.\n");
      return false;
    }

  /* Set frontend operation mode. */

  if (!set_frontend())
    {
      pdm_loge("set_frontend() failure.\n");
      return false;
    }

  /* Initialize frontend. */

  g_init_stage = AUDIO_INIT_FRONTEND;
  if (!init_frontend())
    {
      pdm_loge("init_frontend() failure.\n");
      return false;
    }

#ifdef DBG_AUDIO_DUMP
  debug_fd = open(DEBUG_FILENAME, O_WRONLY | O_CREAT);
#endif

  return true;
}

/*
 * De-initialize the audio system and release all related resources
 */
bool audio_deinit()
{
  pdm_log("Audio: Deinit\n");

#ifdef DBG_AUDIO_DUMP
  close(debug_fd);
#endif

  switch (g_init_stage)
    {
      case AUDIO_INIT_NONE:
        break;

      case AUDIO_INIT_FRONTEND:
        if (!set_ready())
          {
            pdm_loge("set_ready() failure.\n");
          }
        /* Fall through */

      case AUDIO_INIT_FIFO:
        if (!power_off())
          {
            pdm_loge("power_off() failure.\n");
          }
        /* Fall through */

      case AUDIO_INIT_POWER:
        deact_audio_sub_system();
        /* Fall through */

      case AUDIO_INIT_AUDIOSYSTEM:
        if (!finalize_libraries())
          {
            pdm_loge("finalize_libraries() failure.\n");
          }
        break;

      default:
        pdm_loge("invalid audio init state.\n");
        return false;
    }

  return true;
}

/*
 * Do record for the given number of seconds. Note that a audio
 * callback must have been previously set using audio_set_process_cb
 */
bool audio_capture(uint16_t duration_s)
{
  bool result = true;

  /* Start capture operation. */

  if (!start_capture())
    {
      pdm_loge("start_capture() failure.\n");
      result = false;
    }
  else
    {
      /* Running... */

      pdm_log("Audio: Capture %d s\n", duration_s);

      capture_process(duration_s);

      /* Stop capture operation. */

      if (!stop_capture())
        {
          pdm_loge("stop_capture() failure.\n");
          result = false;
        }
    }

  return result;
}
