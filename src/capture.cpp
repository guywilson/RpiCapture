#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <math.h>
#include <pthread.h>
#include <time.h>

#include <interface/mmal/mmal.h>
#include <interface/mmal/mmal_types.h>
#include <interface/mmal/mmal_logging.h>
#include <interface/mmal/mmal_buffer.h>
#include <interface/mmal/mmal_parameters_camera.h>
#include <interface/mmal/util/mmal_util.h>
#include <interface/mmal/util/mmal_util_params.h>
#include <interface/mmal/util/mmal_default_components.h>
#include <interface/mmal/util/mmal_connection.h>

extern "C" {
#include <bcm_host.h>
#include <interface/vcos/vcos.h>

#include "RaspiCommonSettings.h"
#include "RaspiCamControl.h"
#include "RaspiHelpers.h"
}

#include "rpi_error.h"
#include "currenttime.h"
#include "logger.h"

#define MMAL_CAMERA_CAPTURE_PORT 2

// Stills format information
// 0 implies variable
#define STILLS_FRAME_RATE_NUM 0
#define STILLS_FRAME_RATE_DEN 1

// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

#define MAX_USER_EXIF_TAGS      32
#define MAX_EXIF_PAYLOAD_LENGTH 128

/** Structure containing all state information for the current run
 */
typedef struct {
   RASPICOMMONSETTINGS_PARAMETERS common_settings;     /// Common settings
   int timeout;                        /// Time taken before frame is grabbed and app then shuts down. Units are milliseconds
   int quality;                        /// JPEG quality setting (1-100)
   char *linkname;                     /// filename of output file
   int frameStart;                     /// First number of frame output counter
   MMAL_PARAM_THUMBNAIL_CONFIG_T thumbnailConfig;
   MMAL_FOURCC_T encoding;             /// Encoding to use for the output file.
   const char *exifTags[MAX_USER_EXIF_TAGS]; /// Array of pointers to tags supplied from the command line
   int numExifTags;                    /// Number of supplied tags
   int enableExifTags;                 /// Enable/Disable EXIF tags in output
   int datetime;                       /// Use DateTime instead of frame#
   int timestamp;                      /// Use timestamp instead of frame#
   int restart_interval;               /// JPEG restart interval. 0 for none.

   RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters

   MMAL_COMPONENT_T *camera_component;    /// Pointer to the camera component
   MMAL_COMPONENT_T *encoder_component;   /// Pointer to the encoder component
   MMAL_COMPONENT_T *null_sink_component; /// Pointer to the null sink component
   MMAL_CONNECTION_T *encoder_connection; /// Pointer to the connection from camera to encoder

   MMAL_POOL_T *encoder_pool; /// Pointer to the pool of buffers used by encoder output port
}
RASPISTILL_STATE;

/** Struct used to pass information in encoder port userdata to callback
 */
typedef struct {
   FILE *file_handle;                   /// File handle to write buffer data to.
   VCOS_SEMAPHORE_T complete_semaphore; /// semaphore which is posted when we reach end of frame (indicates end of capture or fault)
   RASPISTILL_STATE *pstate;            /// pointer to our state in case required in callback
}
PORT_USERDATA;

/**
 * Assign a default set of parameters to the state passed in
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void default_status(RASPISTILL_STATE *state)
{
   if (!state)
   {
      vcos_assert(0);
      return;
   }

   memset(state, 0, sizeof(*state));

   raspicommonsettings_set_defaults(&state->common_settings);

   state->timeout = -1; // replaced with 5000ms later if unset
   state->quality = 85;
   state->linkname = NULL;
   state->frameStart = 0;
   state->thumbnailConfig.enable = 1;
   state->thumbnailConfig.width = 64;
   state->thumbnailConfig.height = 48;
   state->thumbnailConfig.quality = 35;
   state->camera_component = NULL;
   state->encoder_component = NULL;
   state->encoder_connection = NULL;
   state->encoder_pool = NULL;
   state->encoding = MMAL_ENCODING_JPEG;
   state->numExifTags = 0;
   state->enableExifTags = 1;
   state->datetime = 0;
   state->timestamp = 0;
   state->restart_interval = 0;

   // Set up the camera_parameters to default
   raspicamcontrol_set_defaults(&state->camera_parameters);
}

/**
 * Create the camera component, set up its ports
 *
 * @param state Pointer to state control struct. camera_component member set to the created camera_component if successful.
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_camera_component(RASPISTILL_STATE *state)
{
   MMAL_COMPONENT_T  *camera = 0;
   MMAL_ES_FORMAT_T  *format;
   MMAL_PORT_T       *still_port = NULL;
   MMAL_STATUS_T     status;

   Logger & log = Logger::getInstance();

   try {
      /* Create the component */
      status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

      if (status != MMAL_SUCCESS) {
         log.logError("Failed to create camera component");
         throw rpi_error("Failed to create camera component", __FILE__, __LINE__);
      }

      log.logDebug("MMAL: Created component");

      MMAL_PARAMETER_INT32_T camera_num =
         {{MMAL_PARAMETER_CAMERA_NUM, sizeof(camera_num)}, state->common_settings.cameraNum};

      status = mmal_port_parameter_set(camera->control, &camera_num.hdr);

      if (status != MMAL_SUCCESS) {
         log.logError("Could not select camera : error %d", status);
         throw rpi_error(rpi_error::buildMsg("Could not select camera : error %d", status), __FILE__, __LINE__);
      }

      log.logDebug("MMAL: Selected camera");

      if (!camera->output_num) {
         status = MMAL_ENOSYS;
         log.logError("Camera doesn't have output ports");
         throw rpi_error("Camera doesn't have output ports", __FILE__, __LINE__);
      }

      status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG, state->common_settings.sensor_mode);

      if (status != MMAL_SUCCESS) {
         log.logError("Could not set sensor mode : error %d", status);
         throw rpi_error(rpi_error::buildMsg("Could not set sensor mode : error %d", status), __FILE__, __LINE__);
      }

      log.logDebug("MMAL: Set sensor mode");

      still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

      if (still_port == NULL) {
         log.logError("still capture port is NULL");
         throw rpi_error("Failed to get still capture port", __FILE__, __LINE__);
      }

      // Enable the camera, and tell it its control callback function
      status = mmal_port_enable(camera->control, default_camera_control_callback);

      if (status != MMAL_SUCCESS) {
         log.logError("Unable to enable control port : error %d", status);
         throw rpi_error(rpi_error::buildMsg("Unable to enable control port : error %d", status), __FILE__, __LINE__);
      }

      log.logDebug("MMAL: Enabled the camera");

      //  set up the camera configuration
      {
         MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
         {
            { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
            .max_stills_w = state->common_settings.width,
            .max_stills_h = state->common_settings.height,
            .stills_yuv422 = 0,
            .one_shot_stills = 1,
            .max_preview_video_w = 64,
            .max_preview_video_h = 48,
            .num_preview_video_frames = 3,
            .stills_capture_circular_buffer_height = 0,
            .fast_preview_resume = 0,
            .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
         };

         mmal_port_parameter_set(camera->control, &cam_config.hdr);

         log.logDebug("MMAL: Set camera configuration");
      }

      raspicamcontrol_dump_parameters(&state->camera_parameters);
      raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);

      // Now set up the port formats

      format = still_port->format;

      if(state->camera_parameters.shutter_speed > 6000000) {
         MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
            { 5, 1000 }, {166, 1000}
         };
         mmal_port_parameter_set(still_port, &fps_range.hdr);
      }
      else if(state->camera_parameters.shutter_speed > 1000000) {
         MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
            { 167, 1000 }, {999, 1000}
         };
         mmal_port_parameter_set(still_port, &fps_range.hdr);
      }

      // Set our stills format on the stills (for encoder) port
      format->encoding = MMAL_ENCODING_OPAQUE;
      format->es->video.width = VCOS_ALIGN_UP(state->common_settings.width, 32);
      format->es->video.height = VCOS_ALIGN_UP(state->common_settings.height, 16);
      format->es->video.crop.x = 0;
      format->es->video.crop.y = 0;
      format->es->video.crop.width = state->common_settings.width;
      format->es->video.crop.height = state->common_settings.height;
      format->es->video.frame_rate.num = STILLS_FRAME_RATE_NUM;
      format->es->video.frame_rate.den = STILLS_FRAME_RATE_DEN;

      status = mmal_port_format_commit(still_port);

      if (status != MMAL_SUCCESS) {
         log.logError("camera still format couldn't be set");
         throw rpi_error("camera still format couldn't be set", __FILE__, __LINE__);
      }

      log.logDebug("MMAL: Set camera still format");

      /* Ensure there are enough buffers to avoid dropping frames */
      if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
         still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

      /* Enable component */
      status = mmal_component_enable(camera);

      if (status != MMAL_SUCCESS) {
         log.logError("camera component couldn't be enabled");
         throw rpi_error("camera component couldn't be enabled", __FILE__, __LINE__);
      }

      log.logDebug("MMAL: Enabled camera");

      state->camera_component = camera;
   }
   catch (rpi_error & e) {
      if (camera)
         mmal_component_destroy(camera);
   }

   return status;
}

/**
 * Destroy the camera component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_camera_component(RASPISTILL_STATE *state)
{
   if (state->camera_component) {
      mmal_component_destroy(state->camera_component);
      state->camera_component = NULL;
   }
}

/**
 * Create the encoder component, set up its ports
 *
 * @param state Pointer to state control struct. encoder_component member set to the created camera_component if successful.
 *
 * @return a MMAL_STATUS, MMAL_SUCCESS if all OK, something else otherwise
 */
static MMAL_STATUS_T create_encoder_component(RASPISTILL_STATE *state)
{
   MMAL_COMPONENT_T *encoder = 0;
   MMAL_PORT_T *encoder_input = NULL, *encoder_output = NULL;
   MMAL_STATUS_T status;
   MMAL_POOL_T *pool;

   Logger & log = Logger::getInstance();

   try {
      status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &encoder);

      if (status != MMAL_SUCCESS) {
         log.logError("Unable to create JPEG encoder component");
         throw rpi_error("Unable to create JPEG encoder component", __FILE__, __LINE__);
      }

      if (!encoder->input_num || !encoder->output_num) {
         status = MMAL_ENOSYS;
         log.logError("JPEG encoder doesn't have input/output ports");
         throw rpi_error("JPEG encoder doesn't have input/output ports", __FILE__, __LINE__);
      }

      encoder_input = encoder->input[0];
      encoder_output = encoder->output[0];

      // We want same format on input and output
      mmal_format_copy(encoder_output->format, encoder_input->format);

      // Specify out output format
      encoder_output->format->encoding = state->encoding;

      encoder_output->buffer_size = encoder_output->buffer_size_recommended;

      if (encoder_output->buffer_size < encoder_output->buffer_size_min)
         encoder_output->buffer_size = encoder_output->buffer_size_min;

      encoder_output->buffer_num = encoder_output->buffer_num_recommended;

      if (encoder_output->buffer_num < encoder_output->buffer_num_min)
         encoder_output->buffer_num = encoder_output->buffer_num_min;

      // Commit the port changes to the output port
      status = mmal_port_format_commit(encoder_output);

      if (status != MMAL_SUCCESS) {
         log.logError("Unable to set format on video encoder output port");
         throw rpi_error("Unable to set format on video encoder output port", __FILE__, __LINE__);
      }

      // Set the JPEG quality level
      status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_JPEG_Q_FACTOR, state->quality);

      if (status != MMAL_SUCCESS) {
         log.logError("Unable to set JPEG quality");
         throw rpi_error("Unable to set JPEG quality", __FILE__, __LINE__);
      }

      // Set the JPEG restart interval
      status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_JPEG_RESTART_INTERVAL, state->restart_interval);

      if (state->restart_interval && status != MMAL_SUCCESS) {
         log.logError("Unable to set JPEG restart interval");
         throw rpi_error("Unable to set JPEG restart interval", __FILE__, __LINE__);
      }

      // Set up any required thumbnail
      {
         MMAL_PARAMETER_THUMBNAIL_CONFIG_T param_thumb = {{MMAL_PARAMETER_THUMBNAIL_CONFIGURATION, sizeof(MMAL_PARAMETER_THUMBNAIL_CONFIG_T)}, 0, 0, 0, 0};

         if ( state->thumbnailConfig.enable &&
               state->thumbnailConfig.width > 0 && state->thumbnailConfig.height > 0 )
         {
            // Have a valid thumbnail defined
            param_thumb.enable = 1;
            param_thumb.width = state->thumbnailConfig.width;
            param_thumb.height = state->thumbnailConfig.height;
            param_thumb.quality = state->thumbnailConfig.quality;
         }
         status = mmal_port_parameter_set(encoder->control, &param_thumb.hdr);
      }

      //  Enable component
      status = mmal_component_enable(encoder);

      if (status  != MMAL_SUCCESS) {
         log.logError("Unable to enable video encoder component");
         throw rpi_error("Unable to enable video encoder component", __FILE__, __LINE__);
      }

      /* Create pool of buffer headers for the output port to consume */
      pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);

      if (!pool) {
         log.logError("Failed to create buffer header pool for encoder output port %s", encoder_output->name);
      }

      state->encoder_pool = pool;
      state->encoder_component = encoder;
   }
   catch (rpi_error & e) {
      if (encoder)
         mmal_component_destroy(encoder);
   }

   return status;
}

/**
 * Destroy the encoder component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_encoder_component(RASPISTILL_STATE *state)
{
   // Get rid of any port buffers first
   if (state->encoder_pool) {
      mmal_port_pool_destroy(state->encoder_component->output[0], state->encoder_pool);
   }

   if (state->encoder_component) {
      mmal_component_destroy(state->encoder_component);
      state->encoder_component = NULL;
   }
}

/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void encoder_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   int complete = 0;

   Logger & log = Logger::getInstance();

   // We pass our file handle and other stuff in via the userdata field.

   PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;

   if (pData) {
      int bytes_written = buffer->length;

      if (buffer->length && pData->file_handle) {
         mmal_buffer_header_mem_lock(buffer);

         bytes_written = fwrite(buffer->data, 1, buffer->length, pData->file_handle);

         mmal_buffer_header_mem_unlock(buffer);
      }

      // We need to check we wrote what we wanted - it's possible we have run out of storage.
      if (bytes_written != buffer->length) {
         log.logError("Did not write enough bytes");
         complete = 1;
      }

      // Now flag if we have completed
      if (buffer->flags & (MMAL_BUFFER_HEADER_FLAG_FRAME_END | MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED)) {
         complete = 1;
      }
   }
   else {
      log.logError("Received a encoder buffer callback with no state");
   }

   // release buffer back to the pool
   mmal_buffer_header_release(buffer);

   // and send one back to the port (if still open)
   if (port->is_enabled) {
      MMAL_STATUS_T status = MMAL_SUCCESS;
      MMAL_BUFFER_HEADER_T *new_buffer;

      new_buffer = mmal_queue_get(pData->pstate->encoder_pool->queue);

      if (new_buffer) {
         status = mmal_port_send_buffer(port, new_buffer);
      }
      
      if (!new_buffer || status != MMAL_SUCCESS) {
         log.logError("Unable to return a buffer to the encoder port");
      }
   }

   if (complete) {
      vcos_semaphore_post(&(pData->complete_semaphore));
   }
}

void capture(RASPISTILL_STATE * state)
{

}

int main(int argc, char **argv)
{
   // Our main data storage vessel..
   RASPISTILL_STATE     state;
   PORT_USERDATA        callback_data;
   VCOS_STATUS_T        vcos_status;
   int                  frame; 
   int                  num;
   int                  q;
	int				      defaultLoggingLevel = LOG_LEVEL_DEBUG | LOG_LEVEL_INFO | LOG_LEVEL_ERROR | LOG_LEVEL_FATAL;
   bool                 keep_looping = true;
   FILE *               output_file = NULL;

   MMAL_STATUS_T status = MMAL_SUCCESS;
   MMAL_PORT_T *camera_still_port = NULL;
   MMAL_PORT_T *encoder_input_port = NULL;
   MMAL_PORT_T *encoder_output_port = NULL;

   Logger & log = Logger::getInstance();

	log.initLogger(defaultLoggingLevel);

   bcm_host_init();

   log.logDebug("Initialised bcm host");

   default_status(&state);

   if (argc > 1) {
      state.common_settings.filename = strdup(&argv[1][0]);
   }
   else {
      state.common_settings.filename = "out.jpg";
   }

   log.logDebug("Got file name %s", state.linkname);

   // Setup for sensor specific parameters
   get_sensor_defaults(state.common_settings.cameraNum, state.common_settings.camera_name,
                       &state.common_settings.width, &state.common_settings.height);

   log.logDebug("Got sensor defaults");
   
   status = create_camera_component(&state);

   if (status != MMAL_SUCCESS) {
      log.logError("Failed to create camera component");
      throw rpi_error("Failed to create camera component", __FILE__, __LINE__);
   }

   log.logDebug("Created camera component");

   status = create_encoder_component(&state);

   if (status != MMAL_SUCCESS) {
      mmal_component_destroy(state.camera_component);
      state.camera_component = NULL;

      log.logError("Failed to create encoder component");
      throw rpi_error("Failed to create encoder component", __FILE__, __LINE__);
   }

   log.logDebug("Created encoder component");

   camera_still_port   = state.camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
   encoder_input_port  = state.encoder_component->input[0];
   encoder_output_port = state.encoder_component->output[0];

   // Now connect the camera to the encoder
   status = connect_ports(camera_still_port, encoder_input_port, &state.encoder_connection);

   if (status != MMAL_SUCCESS) {
      check_disable_port(encoder_output_port);

      if (state.encoder_connection) {
         mmal_connection_destroy(state.encoder_connection);
      }

      /* Disable components */
      if (state.encoder_component) {
         mmal_component_disable(state.encoder_component);
      }

      if (state.camera_component) {
         mmal_component_disable(state.camera_component);
      }

      destroy_encoder_component(&state);
      destroy_camera_component(&state);

      log.logError("Failed to connect camera to encoder");

      throw rpi_error("Failed to connect camera to encoder", __FILE__, __LINE__);
   }

   log.logDebug("Connected camera to encoder");

   // Set up our userdata - this is passed though to the callback where we need the information.
   // Null until we open our filename
   callback_data.file_handle = NULL;
   callback_data.pstate = &state;
   vcos_status = vcos_semaphore_create(&callback_data.complete_semaphore, "RpiCapture-sem", 0);

   vcos_assert(vcos_status == VCOS_SUCCESS);

   log.logDebug("Created semaphore");

   // Open the file
   if (state.common_settings.filename) {
      output_file = fopen(state.common_settings.filename, "wb");

      if (!output_file) {
         check_disable_port(encoder_output_port);

         if (state.encoder_connection) {
            mmal_connection_destroy(state.encoder_connection);
         }

         /* Disable components */
         if (state.encoder_component) {
            mmal_component_disable(state.encoder_component);
         }

         if (state.camera_component) {
            mmal_component_disable(state.camera_component);
         }

         destroy_encoder_component(&state);
         destroy_camera_component(&state);

         log.logError("Failed to open file %s", state.common_settings.filename);

         throw rpi_error(rpi_error::buildMsg("Faied to open file %s", state.common_settings.filename), __FILE__, __LINE__);
      }

      log.logDebug("Opened output file %s", state.common_settings.filename);

      callback_data.file_handle = output_file;
   }
   
   if (output_file) {
      mmal_port_parameter_set_boolean(
         state.encoder_component->output[0], MMAL_PARAMETER_EXIF_DISABLE, 1);

      log.logDebug("Disabled exif");

      // There is a possibility that shutter needs to be set each loop.
      status = mmal_port_parameter_set_uint32(
                     state.camera_component->control, 
                     MMAL_PARAMETER_SHUTTER_SPEED, 
                     state.camera_parameters.shutter_speed);

      if (status != MMAL_SUCCESS) {
         log.logError("Failed to set shutter speed");
      }

      log.logDebug("Set shutter speed");

      // Enable the encoder output port
      encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&callback_data;

      // Enable the encoder output port and tell it its callback function
      status = mmal_port_enable(encoder_output_port, encoder_buffer_callback);

      log.logDebug("Enabled encoder output port");

      // Send all the buffers to the encoder output port
      num = mmal_queue_length(state.encoder_pool->queue);

      for (q = 0;q < num;q++) {
         MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.encoder_pool->queue);

         if (!buffer) {
            log.logError("Failed to get queue");
         }

         status = mmal_port_send_buffer(encoder_output_port, buffer);

         if (status != MMAL_SUCCESS) {
            log.logError("Failed to send buffer");
         }
      }

      log.logDebug("Sent buffers to encoder output");

      status = mmal_port_parameter_set_boolean(camera_still_port, MMAL_PARAMETER_CAPTURE, 1);

      if (status != MMAL_SUCCESS) {
         log.logError("Failed to start capture");
      }
      else {
         // Wait for capture to complete
         // For some reason using vcos_semaphore_wait_timeout sometimes returns immediately with bad parameter error
         // even though it appears to be all correct, so reverting to untimed one until figure out why its erratic
         vcos_semaphore_wait(&callback_data.complete_semaphore);
      }

      log.logDebug("Initaiated capture");

      // Ensure we don't die if get callback with no open file
      callback_data.file_handle = NULL;

      // Disable encoder output port
      status = mmal_port_disable(encoder_output_port);

      log.logDebug("Disabled port");

      if (status != MMAL_SUCCESS) {
         throw rpi_error("Failed to disable port", __FILE__, __LINE__);
      }
   }

   vcos_semaphore_delete(&callback_data.complete_semaphore);

   log.logDebug("Finished!");
}
