#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <math.h>
#include <pthread.h>
#include <time.h>

#include <mmal/mmal.h>
#include <mmal/mmal_types.h>
#include <mmal/mmal_logging.h>
#include <mmal/mmal_buffer.h>
#include <mmal/mmal_parameters_camera.h>
#include <mmal/util/mmal_util.h>
#include <mmal/util/mmal_util_params.h>
#include <mmaml/util/mmal_default_components.h>
#include <mmal/util/mmal_connection.h>
#include <vcos/vcos.h>
#include <RaspiGPS.h>
#include <RaspiCommonSettings.h>
#include <RaspiCamControl.h>
#include <RaspiHelpers.h>

#include "rpi_error.h"
#include "currenttime.h"
#include "logger.h"

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

   RASPIPREVIEW_PARAMETERS preview_parameters;    /// Preview setup parameters
   RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters

   MMAL_COMPONENT_T *camera_component;    /// Pointer to the camera component
   MMAL_COMPONENT_T *encoder_component;   /// Pointer to the encoder component
   MMAL_COMPONENT_T *null_sink_component; /// Pointer to the null sink component
   MMAL_CONNECTION_T *encoder_connection; /// Pointer to the connection from camera to encoder

   MMAL_POOL_T *encoder_pool; /// Pointer to the pool of buffers used by encoder output port

   RASPITEX_STATE raspitex_state; /// GL renderer state and parameters
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

void capture(RASPISTILL_STATE * state)
{

}

int main(void)
{
   // Our main data storage vessel..
   RASPISTILL_STATE     state;
   PORT_USERDATA        callback_data;
   VCOS_STATUS_T        vcos_status;
   int                  frame; 
   int                  exit_code = EX_OK;
   int                  num;
   int                  q;
   bool                 keep_looping = true;
   FILE *               output_file = NULL;
   char *               use_filename = NULL;      // Temporary filename while image being written
   char *               final_filename = NULL;    // Name that file gets once writing complete

   MMAL_STATUS_T status = MMAL_SUCCESS;
   MMAL_PORT_T *camera_still_port = NULL;
   MMAL_PORT_T *encoder_input_port = NULL;
   MMAL_PORT_T *encoder_output_port = NULL;

   Logger & log = Logger::getInstance();

   bcm_host_init();

   default_status(&state);

   // Setup for sensor specific parameters
   get_sensor_defaults(state.common_settings.cameraNum, state.common_settings.camera_name,
                       &state.common_settings.width, &state.common_settings.height);
   
   status = create_camera_component(&state);

   if (status != MMAL_SUCCESS) {
      throw rpi_error("Failed to create camera component", __FILE__, __LINE__);
   }

   status = create_encoder_component(&state);

   if (status != MMAL_SUCCESS) {
      mmal_component_destroy(state->camera_component);
      state->camera_component = NULL;

      throw rpi_error("Failed to create encoder component", __FILE__, __LINE__);
   }

   camera_still_port   = state.camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
   encoder_input_port  = state.encoder_component->input[0];
   encoder_output_port = state.encoder_component->output[0];

   // Now connect the camera to the encoder
   status = connect_ports(camera_still_port, encoder_input_port, &state.encoder_connection);

   if (status != MMAL_SUCCESS) {
      throw rpi_error("Failed to connect camera to encoder", __FILE__, __LINE__);
   }

   // Set up our userdata - this is passed though to the callback where we need the information.
   // Null until we open our filename
   callback_data.file_handle = NULL;
   callback_data.pstate = &state;
   vcos_status = vcos_semaphore_create(&callback_data.complete_semaphore, "RpiCapture-sem", 0);

   vcos_assert(vcos_status == VCOS_SUCCESS);

   frame = state.frameStart - 1;

   while (keep_looping) {
      keep_looping = false;

      if (state.datetime) {
         time_t rawtime;
         struct tm *timeinfo;

         time(&rawtime);
         timeinfo = localtime(&rawtime);

         frame = timeinfo->tm_mon+1;
         frame *= 100;
         frame += timeinfo->tm_mday;
         frame *= 100;
         frame += timeinfo->tm_hour;
         frame *= 100;
         frame += timeinfo->tm_min;
         frame *= 100;
         frame += timeinfo->tm_sec;
      }
      else if (state.timestamp) {
         frame = (int)time(NULL);
      }

      // Open the file
      if (state.common_settings.filename) {
         vcos_assert(use_filename == NULL && final_filename == NULL);

         status = create_filenames(&final_filename, &use_filename, state.common_settings.filename, frame);

         if (status != MMAL_SUCCESS) {
            throw rpi_error("Failed to create filenames", __FILE__, __LINE__);
         }

         output_file = fopen(use_filename, "wb");

         if (!output_file) {
            throw rpi_error(rpi_error::buildMsg("Faied to open file %s", use_filename), __FILE__, __LINE__);
         }

         callback_data.file_handle = output_file;
      }
      
      if (output_file) {
         if ( state.enableExifTags ) {
            struct gps_data_t *gps_data = raspi_gps_lock();
            add_exif_tags(&state, gps_data);
            raspi_gps_unlock();
         }
         else {
            mmal_port_parameter_set_boolean(
               state.encoder_component->output[0], MMAL_PARAMETER_EXIF_DISABLE, 1);
         }

         // There is a possibility that shutter needs to be set each loop.
         if (mmal_status_to_int(mmal_port_parameter_set_uint32(state.camera_component->control, MMAL_PARAMETER_SHUTTER_SPEED, state.camera_parameters.shutter_speed)) != MMAL_SUCCESS)
            vcos_log_error("Unable to set shutter speed");

         // Enable the encoder output port
         encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&callback_data;

         // Enable the encoder output port and tell it its callback function
         status = mmal_port_enable(encoder_output_port, encoder_buffer_callback);

         // Send all the buffers to the encoder output port
         num = mmal_queue_length(state.encoder_pool->queue);

         for (q = 0;q < num;q++) {
            MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.encoder_pool->queue);

            if (!buffer) {
               throw rpi_error("Faied to get queue", __FILE__, __LINE__);
            }

            status = mmal_port_send_buffer(encoder_output_port, buffer);

            if (status != MMAL_SUCCESS) {
               throw rpi_error("Failed to send buffer", __FILE__, __LINE__);
            }
         }

         status = mmal_port_parameter_set_boolean(camera_still_port, MMAL_PARAMETER_CAPTURE, 1);

         if (status != MMAL_SUCCESS) {
            throw rpi_error("Failed to initiate capture", __FILE__, __LINE__);
         }
         else {
            // Wait for capture to complete
            // For some reason using vcos_semaphore_wait_timeout sometimes returns immediately with bad parameter error
            // even though it appears to be all correct, so reverting to untimed one until figure out why its erratic
            vcos_semaphore_wait(&callback_data.complete_semaphore);
         }

         // Ensure we don't die if get callback with no open file
         callback_data.file_handle = NULL;

         rename_file(&state, output_file, final_filename, use_filename, frame);

         // Disable encoder output port
         status = mmal_port_disable(encoder_output_port);

         if (status != MMAL_SUCCESS) {
            throw rpi_error("Failed to disable port", __FILE__, __LINE__);
         }
      }

      if (use_filename) {
         free(use_filename);
         use_filename = NULL;
      }

      if (final_filename) {
         free(final_filename);
         final_filename = NULL;
      }
   } // end while

   vcos_semaphore_delete(&callback_data.complete_semaphore);
}
