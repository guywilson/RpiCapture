#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <math.h>
#include <pthread.h>
#include <time.h>

#include <bcm_host.h>
#include <interface/mmal/mmal.h>
#include <interface/mmal/mmal_types.h>
#include <interface/mmal/mmal_logging.h>
#include <interface/mmal/mmal_buffer.h>
#include <interface/mmal/mmal_parameters_camera.h>
#include <interface/mmal/util/mmal_util.h>
#include <interface/mmal/util/mmal_util_params.h>
#include <interface/mmaml/util/mmal_default_components.h>
#include <interface/mmal/util/mmal_connection.h>
#include <interface/vcos/vcos.h>

#include "RaspiGPS.h"
#include "RaspiCommonSettings.h"
#include "RaspiCamControl.h"
#include "RaspiHelpers.h"

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

/**
 * Add a basic set of EXIF tags to the capture
 * Make, Time etc
 *
 * @param state Pointer to state control struct
 *
 */
static void add_exif_tags(RASPISTILL_STATE *state, struct gps_data_t *gpsdata)
{
   time_t rawtime;
   struct tm *timeinfo;
   char model_buf[32];
   char time_buf[32];
   char exif_buf[128];
   int i;

   snprintf(model_buf, 32, "IFD0.Model=RP_%s", state->common_settings.camera_name);
   add_exif_tag(state, model_buf);
   add_exif_tag(state, "IFD0.Make=RaspberryPi");

   time(&rawtime);
   timeinfo = localtime(&rawtime);

   snprintf(time_buf, sizeof(time_buf),
            "%04d:%02d:%02d %02d:%02d:%02d",
            timeinfo->tm_year+1900,
            timeinfo->tm_mon+1,
            timeinfo->tm_mday,
            timeinfo->tm_hour,
            timeinfo->tm_min,
            timeinfo->tm_sec);

   snprintf(exif_buf, sizeof(exif_buf), "EXIF.DateTimeDigitized=%s", time_buf);
   add_exif_tag(state, exif_buf);

   snprintf(exif_buf, sizeof(exif_buf), "EXIF.DateTimeOriginal=%s", time_buf);
   add_exif_tag(state, exif_buf);

   snprintf(exif_buf, sizeof(exif_buf), "IFD0.DateTime=%s", time_buf);
   add_exif_tag(state, exif_buf);


   // Add GPS tags
   if (state->common_settings.gps)
   {
      // clear all existing tags first
      add_exif_tag(state, "GPS.GPSDateStamp=");
      add_exif_tag(state, "GPS.GPSTimeStamp=");
      add_exif_tag(state, "GPS.GPSMeasureMode=");
      add_exif_tag(state, "GPS.GPSSatellites=");
      add_exif_tag(state, "GPS.GPSLatitude=");
      add_exif_tag(state, "GPS.GPSLatitudeRef=");
      add_exif_tag(state, "GPS.GPSLongitude=");
      add_exif_tag(state, "GPS.GPSLongitudeRef=");
      add_exif_tag(state, "GPS.GPSAltitude=");
      add_exif_tag(state, "GPS.GPSAltitudeRef=");
      add_exif_tag(state, "GPS.GPSSpeed=");
      add_exif_tag(state, "GPS.GPSSpeedRef=");
      add_exif_tag(state, "GPS.GPSTrack=");
      add_exif_tag(state, "GPS.GPSTrackRef=");

      if (gpsdata->online)
      {
         if (state->common_settings.verbose)
            fprintf(stderr, "Adding GPS EXIF\n");
         if (gpsdata->set & TIME_SET)
         {
            rawtime = gpsdata->fix.time;
            timeinfo = localtime(&rawtime);
            strftime(time_buf, sizeof(time_buf), "%Y:%m:%d", timeinfo);
            snprintf(exif_buf, sizeof(exif_buf), "GPS.GPSDateStamp=%s", time_buf);
            add_exif_tag(state, exif_buf);
            strftime(time_buf, sizeof(time_buf), "%H/1,%M/1,%S/1", timeinfo);
            snprintf(exif_buf, sizeof(exif_buf), "GPS.GPSTimeStamp=%s", time_buf);
            add_exif_tag(state, exif_buf);
         }
         if (gpsdata->fix.mode >= MODE_2D)
         {
            snprintf(exif_buf, sizeof(exif_buf), "GPS.GPSMeasureMode=%c",
                     (gpsdata->fix.mode >= MODE_3D) ? '3' : '2');
            add_exif_tag(state, exif_buf);
            if ((gpsdata->satellites_used > 0) && (gpsdata->satellites_visible > 0))
            {
               snprintf(exif_buf, sizeof(exif_buf), "GPS.GPSSatellites=Used:%d,Visible:%d",
                        gpsdata->satellites_used, gpsdata->satellites_visible);
               add_exif_tag(state, exif_buf);
            }
            else if (gpsdata->satellites_used > 0)
            {
               snprintf(exif_buf, sizeof(exif_buf), "GPS.GPSSatellites=Used:%d",
                        gpsdata->satellites_used);
               add_exif_tag(state, exif_buf);
            }
            else if (gpsdata->satellites_visible > 0)
            {
               snprintf(exif_buf, sizeof(exif_buf), "GPS.GPSSatellites=Visible:%d",
                        gpsdata->satellites_visible);
               add_exif_tag(state, exif_buf);
            }

            if (gpsdata->set & LATLON_SET)
            {
               if (isnan(gpsdata->fix.latitude) == 0)
               {
                  if (deg_to_str(fabs(gpsdata->fix.latitude), time_buf, sizeof(time_buf)) == 0)
                  {
                     snprintf(exif_buf, sizeof(exif_buf), "GPS.GPSLatitude=%s", time_buf);
                     add_exif_tag(state, exif_buf);
                     snprintf(exif_buf, sizeof(exif_buf), "GPS.GPSLatitudeRef=%c",
                              (gpsdata->fix.latitude < 0) ? 'S' : 'N');
                     add_exif_tag(state, exif_buf);
                  }
               }
               if (isnan(gpsdata->fix.longitude) == 0)
               {
                  if (deg_to_str(fabs(gpsdata->fix.longitude), time_buf, sizeof(time_buf)) == 0)
                  {
                     snprintf(exif_buf, sizeof(exif_buf), "GPS.GPSLongitude=%s", time_buf);
                     add_exif_tag(state, exif_buf);
                     snprintf(exif_buf, sizeof(exif_buf), "GPS.GPSLongitudeRef=%c",
                              (gpsdata->fix.longitude < 0) ? 'W' : 'E');
                     add_exif_tag(state, exif_buf);
                  }
               }
            }
            if ((gpsdata->set & ALTITUDE_SET) && (gpsdata->fix.mode >= MODE_3D))
            {
               if (isnan(gpsdata->fix.altitude) == 0)
               {
                  snprintf(exif_buf, sizeof(exif_buf), "GPS.GPSAltitude=%d/10",
                           (int)(gpsdata->fix.altitude*10+0.5));
                  add_exif_tag(state, exif_buf);
                  add_exif_tag(state, "GPS.GPSAltitudeRef=0");
               }
            }
            if (gpsdata->set & SPEED_SET)
            {
               if (isnan(gpsdata->fix.speed) == 0)
               {
                  snprintf(exif_buf, sizeof(exif_buf), "GPS.GPSSpeed=%d/10",
                           (int)(gpsdata->fix.speed*MPS_TO_KPH*10+0.5));
                  add_exif_tag(state, exif_buf);
                  add_exif_tag(state, "GPS.GPSSpeedRef=K");
               }
            }
            if (gpsdata->set & TRACK_SET)
            {
               if (isnan(gpsdata->fix.track) == 0)
               {
                  snprintf(exif_buf, sizeof(exif_buf), "GPS.GPSTrack=%d/100",
                           (int)(gpsdata->fix.track*100+0.5));
                  add_exif_tag(state, exif_buf);
                  add_exif_tag(state, "GPS.GPSTrackRef=T");
               }
            }
         }
      }
   }

   // Now send any user supplied tags

   for (i=0; i<state->numExifTags && i < MAX_USER_EXIF_TAGS; i++)
   {
      if (state->exifTags[i])
      {
         add_exif_tag(state, state->exifTags[i]);
      }
   }
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
	int				      defaultLoggingLevel = LOG_LEVEL_DEBUG | LOG_LEVEL_INFO | LOG_LEVEL_ERROR | LOG_LEVEL_FATAL;
   bool                 keep_looping = true;
   FILE *               output_file = NULL;
   char *               use_filename = NULL;      // Temporary filename while image being written
   char *               final_filename = NULL;    // Name that file gets once writing complete

   MMAL_STATUS_T status = MMAL_SUCCESS;
   MMAL_PORT_T *camera_still_port = NULL;
   MMAL_PORT_T *encoder_input_port = NULL;
   MMAL_PORT_T *encoder_output_port = NULL;

   Logger & log = Logger::getInstance();

	log.initLogger(defaultLoggingLevel);

   bcm_host_init();

   default_status(&state);

   // Setup for sensor specific parameters
   get_sensor_defaults(state.common_settings.cameraNum, state.common_settings.camera_name,
                       &state.common_settings.width, &state.common_settings.height);
   
   status = create_camera_component(&state);

   if (status != MMAL_SUCCESS) {
      log.logError("Failed to create camera component");
      throw rpi_error("Failed to create camera component", __FILE__, __LINE__);
   }

   status = create_encoder_component(&state);

   if (status != MMAL_SUCCESS) {
      mmal_component_destroy(state->camera_component);
      state->camera_component = NULL;

      log.logError("Failed to create encoder component");

      throw rpi_error("Failed to create encoder component", __FILE__, __LINE__);
   }

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

      if (state.common_settings.gps) {
         raspi_gps_shutdown(state.common_settings.verbose);
      }

      log.logError("Failed to connect camera to encoder");

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

            if (state.common_settings.gps) {
               raspi_gps_shutdown(state.common_settings.verbose);
            }

            log.logError("Failed to open file %s", state.common_settings.filename);

            throw rpi_error(rpi_error::buildMsg("Faied to open file %s", state.common_settings.filename), __FILE__, __LINE__);
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
         status = mmal_status_to_int(
                     mmal_port_parameter_set_uint32(
                        state.camera_component->control, 
                        MMAL_PARAMETER_SHUTTER_SPEED, 
                        state.camera_parameters.shutter_speed));

         if (status != MMAL_SUCCESS) {
            log.logError("Failed to set shutter speed");
         }

         // Enable the encoder output port
         encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&callback_data;

         // Enable the encoder output port and tell it its callback function
         status = mmal_port_enable(encoder_output_port, encoder_buffer_callback);

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
