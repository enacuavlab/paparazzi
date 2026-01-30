/*
 * Copyright (C) 2025 Jean-Baptiste FORESTIER <jean-baptiste.forestier@enac.fr>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/lidar/lidar_sf30d.c"
 * @author Jean-Baptiste FORESTIER
 * @brief driver for the Lightware SF30/D LIDAR connected over i2c bus.
 * https://support.lightware.co.za/sf30d/#/
 * Don't hesitate to contact the support : support@lightwarelidar.com
 * 
 * The laser on the sensor is continuously firing at a rate of 20kHz
 * The data output however is dependent on the update rate set. This also determines how many samples are internally sampled / compared for an output. 
 * For example, if the sensor is set to 39 readings/s output, the sensor is internally comparing 512 samples per output of the 39 readings output rate. 
 * If you set the sensor higher than 1250 readings per second, the sensor disables this algorithm and returns the raw data. 
 * 
 * We choose at first to use the main loop at 500Hz (2000 us) and set the exposure time to 1600 us. 
 * Thanks to that, we have an average parameter by the LIDAR.
 * Futur work is to increase the I2C communication rate with a thread closer to 20kHz, and do our own average processing as we do already
 * Data return : First, FirstFiltered, FirstStrength, Last
 * We calculate homefiltered thanks to a median filter on the first value which is raw
 * 
 * TODO @USER in Lightware Studio :
 * - Set the Output type (legacy) to = Full communication mode
 * - Set the Exposure time to 1600 us (625 /sec)
 * - Set the I2C address in Lidar Software to 102
 */
 
//TODO 
// - Choice between binary and ASCII output
// - Update rate in the configuration. Not working with this command.
// - ABI message ?




#include "modules/lidar/lidar_sf30d.h"
#include "modules/core/abi.h"
#include "math/pprz_algebra_float.h"
#include "modules/datalink/downlink.h"
#include "pprzlink/messages.h"
#include "generated/airframe.h"
#include "filters/median_filter.h"
#include "state.h" // State interface for rotation compensation
#include "modules/gps/gps.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "std.h"



// ** Declaration ** // 

struct LidarSF30D lidar_sf30d;
struct MedianFilterInt lidar_sf30d_filter;
char date_sys_lidar[6], time_sys_lidar[6];


/**
 * Initialization function
 */
void lidar_sf30d_init(void) {

  lidar_sf30d.log_ptu_started = false;
  lidar_sf30d.trans.status = I2CTransDone;
  lidar_sf30d.addr = LIDAR_SF30D_I2C_ADRESS;
  lidar_sf30d.update_agl = USE_LIDAR_SF30D_AGL;
  lidar_sf30d.compensate_rotation = LIDAR_SF30D_COMPENSATE_ROTATION;
  lidar_sf30d.init_status = LIDAR_CONF_UNINIT;
  lidar_sf30d.error_init = false;
  lidar_sf30d.initialized = false;

  lidar_sf30d.msg.first = 0;
  lidar_sf30d.msg.firstFiltered = 0;
  lidar_sf30d.msg.firstStrength = 0;
  lidar_sf30d.msg.last = 0;
  lidar_sf30d.msg.homeFiltered = 0;
  lidar_sf30d.msg.homeFiltered_raw = 0;
  lidar_sf30d.msg.phi = 0;
  lidar_sf30d.msg.theta = 0;
  lidar_sf30d.msg.gain = 0;
  lidar_sf30d.msg.now_ts = 0;

  lidar_sf30d.gps_data.lat = 0;
	lidar_sf30d.gps_data.lon = 0;
	lidar_sf30d.gps_data.num_sv = 0;			
	lidar_sf30d.gps_data.pdop = 0;		
	lidar_sf30d.gps_data.hmsl = 0;
	lidar_sf30d.gps_data.vground = 0;	 
  lidar_sf30d.gps_data.course = 0;	
      
  init_median_filter_i(&lidar_sf30d_filter, MEDIAN_DEFAULT_SIZE);
}

/**
 * Lidar send configuration function
 * Configuration function called once before nominal use
 */
void lidar_sf30d_send_config(void) {

  switch (lidar_sf30d.init_status) {
    case LIDAR_BASED_PROTOCOL:
      // Enable register based protocol
      lidar_sf30d.trans.buf[0]= 120;
      lidar_sf30d.trans.buf[1]= 170;
      lidar_sf30d.trans.buf[2]= 170;
      i2c_transmit(&LIDAR_SF30D_I2C_DEV, &lidar_sf30d.trans, lidar_sf30d.addr, 3);
      lidar_sf30d.init_status++;
      break;
    case LIDAR_OUTPUT_MODE:
      // Set output mode
      int distance_output_value = 255;
      lidar_sf30d.trans.buf[0] = 29;
      lidar_sf30d.trans.buf[1] = (distance_output_value >> 0) & 0xFF;
      lidar_sf30d.trans.buf[2] = (distance_output_value >> 8) & 0xFF;
      lidar_sf30d.trans.buf[3] = (distance_output_value >> 16) & 0xFF;
      lidar_sf30d.trans.buf[4] = (distance_output_value >> 24) & 0xFF;
      i2c_transmit(&LIDAR_SF30D_I2C_DEV, &lidar_sf30d.trans, lidar_sf30d.addr, 5);
      lidar_sf30d.init_status++;
      break;
 /* case LIDAR_UPDATE_RATE:
      // Set update rate
      lidar_sf30d.trans.buf[0] = 76;
      lidar_sf30d.trans.buf[1] = 5; (625 readings/sec)
      i2c_transmit(&LIDAR_SF30D_I2C_DEV, &lidar_sf30d.trans, lidar_sf30d.addr, 2);
      lidar_sf30d.init_status++;
      break;*/
    case LIDAR_CONF_DONE:
      lidar_sf30d.initialized = true;
      lidar_sf30d.trans.status = I2CTransDone;
      break;
    default:
      break;
  }
}

/**
 * Lidar configure function
 * wait before starting the configuration
 * doing to early may void the mode configuration
 */
void lidar_sf30d_start_configure(void) {

  if (lidar_sf30d.init_status == LIDAR_CONF_UNINIT && get_sys_time_float() > LIDAR_SF30D_STARTUP_DELAY) {
    lidar_sf30d.init_status++;
    if (lidar_sf30d.trans.status == I2CTransSuccess || lidar_sf30d.trans.status == I2CTransDone) {
      lidar_sf30d_send_config();
    }
  }
}

/**
 * Lidar event function
 * Check if the transaction succeded before reading the result
 */
void lidar_sf30d_event(void) {
  
  if (lidar_sf30d.initialized) {

    if (lidar_sf30d.trans.status == I2CTransFailed) {
      lidar_sf30d.trans.status = I2CTransDone;

    } else if (lidar_sf30d.trans.status == I2CTransSuccess) {
      
      // Reset data
      lidar_sf30d.msg.first = 0;
      lidar_sf30d.msg.firstFiltered = 0;
      lidar_sf30d.msg.firstStrength = 0;
      lidar_sf30d.msg.last = 0;
      lidar_sf30d.msg.homeFiltered = 0;
      lidar_sf30d.msg.homeFiltered_raw = 0;
      lidar_sf30d.msg.phi = 0;
      lidar_sf30d.msg.theta = 0;
      lidar_sf30d.msg.gain = 0;
      lidar_sf30d.msg.now_ts = 0;

      //Get system date and time if GPS + GPS 
      if(gps.fix >= GPS_FIX_3D ) {
        lidar_get_system_date_str(date_sys_lidar, sizeof(date_sys_lidar)+1, time_sys_lidar, sizeof(time_sys_lidar)+1);
        lidar_recover_gps_data();
        lidar_convert_deg_to_DDMM(lidar_sf30d.gps_data.lat, lidar_sf30d.gps_data.lat_buf, 1);
        lidar_convert_deg_to_DDMM(lidar_sf30d.gps_data.lon, lidar_sf30d.gps_data.lon_buf, 0);
        lidar_sf30d.gps_data.lat_hemi = (lidar_sf30d.gps_data.lat >= 0) ? 'N' : 'S';
        lidar_sf30d.gps_data.lon_hemi = (lidar_sf30d.gps_data.lon >= 0) ? 'E' : 'W';
      } else {
        // Reset GPS data
        strcpy(lidar_sf30d.gps_data.lat_buf, "0000");
        strcpy(lidar_sf30d.gps_data.lon_buf, "0000");
        strcpy(date_sys_lidar, "0000");
        strcpy(time_sys_lidar, "0000");
        lidar_sf30d.gps_data.lat_hemi = 'N';
        lidar_sf30d.gps_data.lon_hemi = 'E';
        lidar_sf30d.gps_data.lat = 0;
        lidar_sf30d.gps_data.lon = 0;
        lidar_sf30d.gps_data.num_sv = 0;			
        lidar_sf30d.gps_data.pdop = 0;		
        lidar_sf30d.gps_data.hmsl = 0.0;
        lidar_sf30d.gps_data.vground = 0.0;	 
        lidar_sf30d.gps_data.course = 0.0;
      }

      // Process results
      lidar_sf30d.msg.now_ts = get_sys_time_usec();
      lidar_sf30d.msg.first = lidar_sf30d.trans.buf[0] | (lidar_sf30d.trans.buf[1] << 8); 
      lidar_sf30d.msg.firstFiltered = lidar_sf30d.trans.buf[2] | (lidar_sf30d.trans.buf[3] << 8); 
      lidar_sf30d.msg.firstStrength = lidar_sf30d.trans.buf[4] | (lidar_sf30d.trans.buf[5] << 8); 
      lidar_sf30d.msg.last = lidar_sf30d.trans.buf[6] | (lidar_sf30d.trans.buf[7] << 8); 

      // filter data
      lidar_sf30d.msg.homeFiltered_raw = update_median_filter_i(&lidar_sf30d_filter,lidar_sf30d.msg.first);
      lidar_sf30d.msg.homeFiltered = ((float)lidar_sf30d.msg.homeFiltered_raw)/100.0;

      // compensate rotation if requested
      if (lidar_sf30d.compensate_rotation) {
          lidar_sf30d.msg.phi = stateGetNedToBodyEulers_f()->phi;
          lidar_sf30d.msg.theta = stateGetNedToBodyEulers_f()->theta;
          lidar_sf30d.msg.gain = (float)fabs( (double) (cosf(lidar_sf30d.msg.phi) * cosf(lidar_sf30d.msg.theta)));
          lidar_sf30d.msg.homeFiltered = lidar_sf30d.msg.homeFiltered / lidar_sf30d.msg.gain;
      }

      // Communication error management, if the first strength is 0 and the last distance is greater than 3000 cm, we consider it as an error
      if (lidar_sf30d.msg.firstStrength == 0 && lidar_sf30d.msg.last > 3000) {
        lidar_sf30d_init();
        lidar_sf30d.error_init = true;
        return;
      }

      // send ABI message (if requested)
      if (lidar_sf30d.update_agl) {
        //AbiSendMsgAGL(AGL_LIDAR_SF30D_ID, lidar_sf30d.msg.now_ts, lidar_sf30d.msg.homeFiltered);
      }

      // save to sd card
      lidar_sf30d_log_data();

      lidar_sf30d.trans.status = I2CTransDone;
    }

  } else if (lidar_sf30d.init_status != LIDAR_CONF_UNINIT) { // Configuring but not yet initialized
    if (lidar_sf30d.trans.status == I2CTransSuccess || lidar_sf30d.trans.status == I2CTransDone) {
      lidar_sf30d.trans.status = I2CTransDone;
      lidar_sf30d_send_config();
    }
    if (lidar_sf30d.trans.status == I2CTransFailed) {
      if(lidar_sf30d.init_status > LIDAR_CONF_UNINIT){
        lidar_sf30d.init_status--; // Decrement the status to retry the last step, we don't want to go below 0 (LIDAR_CONF_UNINIT)
      }
      lidar_sf30d.trans.status = I2CTransDone;
      lidar_sf30d_send_config(); // Retry config (TODO max retry)
    }
  }
}

/**
 * Lidar read function to register 
 */
void lidar_sf30d_read(void) {

  if (lidar_sf30d.initialized && lidar_sf30d.trans.status == I2CTransDone) {

      lidar_sf30d.trans.buf[0] = LIDAR_SF30D_REG_REQ_DATA; //set tx to distance register
      i2c_transceive(&LIDAR_SF30D_I2C_DEV, &lidar_sf30d.trans, lidar_sf30d.addr, 1, 8); // We send the reg and we get in return 8 byte
  }    
}

/**
 * Poll Lidar for data
 */
void lidar_sf30d_periodic(void) {

  if (lidar_sf30d.initialized) {
    lidar_sf30d_read();
  } else {
    lidar_sf30d_start_configure();
  }
}



/**
 * Downlink message for debug
 */
void lidar_sf30d_downlink(void) {

  uint8_t trans = lidar_sf30d.trans.status;
  uint8_t status = 3; // Default status: 3 (no error), random value chosen for debug
  if(lidar_sf30d.error_init == true){
    status = 1; // Error during initialization
  }
  lidar_sf30d.error_init = false; // Reset error flag after sending
  float distance = lidar_sf30d.msg.homeFiltered;

  DOWNLINK_SEND_LIDAR(DefaultChannel, DefaultDevice,
      &distance,
      &status,
      &trans);
}

/**
 * Log SD function
 */
void lidar_sf30d_log_data(void) {
	
  if(LOG_ASCII == 0) {

  if (pprzLogFile != -1) {
            
    if (lidar_sf30d.log_ptu_started == false) {
      sdLogWriteLog(pprzLogFile, "\n");
      sdLogWriteLog(pprzLogFile,"date_sys,time_sys,now_ts,lat, lat_hemi, lon, lon_hemi, num_sv, pdop, hmsl, vground, course, first(cm),firstFiltered(cm),firstStrength,homeFiltered(m),homeFiltered_raw(cm),last(cm),phi(deg),theta(deg),gain\n");
      lidar_sf30d.log_ptu_started = true;
    } else {

      sdLogWriteLog(pprzLogFile, "%s,%s,%lu,%s,%c,%s,%c,%02u,%04u,%09.3f,%05.1f,%05.1f,%d,%d,%d,%.2f,%lu,%d,%.6f,%.6f,%.6f \n",
      date_sys_lidar,
      time_sys_lidar,
      lidar_sf30d.msg.now_ts,
      lidar_sf30d.gps_data.lat_buf,
      lidar_sf30d.gps_data.lat_hemi,
      lidar_sf30d.gps_data.lon_buf,
      lidar_sf30d.gps_data.lon_hemi,
      lidar_sf30d.gps_data.num_sv,
      lidar_sf30d.gps_data.pdop,
      lidar_sf30d.gps_data.hmsl,
      lidar_sf30d.gps_data.vground,
      lidar_sf30d.gps_data.course,      
      lidar_sf30d.msg.first,
      lidar_sf30d.msg.firstFiltered,
      lidar_sf30d.msg.firstStrength,
      lidar_sf30d.msg.homeFiltered,
      lidar_sf30d.msg.homeFiltered_raw,
      lidar_sf30d.msg.last,
      lidar_sf30d.msg.phi,
      lidar_sf30d.msg.theta,
      lidar_sf30d.msg.gain
      );
    }  
  }
  }
  else if(LOG_ASCII == 1) {

      if (pprzLogFile != -1) {
      //TODO ?      
  }
  }
}


/**
 * Function to recover data available onboard from GPS
 */
void lidar_recover_gps_data(void) {	
	
      lidar_sf30d.gps_data.lat = (double)gps.lla_pos.lat / 1e7;
      lidar_sf30d.gps_data.lon = (double)gps.lla_pos.lon / 1e7;
      lidar_sf30d.gps_data.num_sv = gps.num_sv;
      lidar_sf30d.gps_data.pdop = gps.pdop;
      lidar_sf30d.gps_data.hmsl = (float)gps.hmsl / 1000.0; //in meters
      lidar_sf30d.gps_data.vground = (float)stateGetHorizontalSpeedNorm_f();
      lidar_sf30d.gps_data.course = (float)DegOfRad(gps.course/1e7);
}

/**
 * Get Date from system 
 */
void lidar_get_system_date_str(char *buf_date, size_t buf_date_size, char *buf_time, size_t buf_time_size) {


    const int64_t gps_epoch_days = 3657; //day frm 1970-01-01
    uint64_t gps_sec = (uint64_t)gps.week * 7 * 24 * 3600 + gps.tow / 1000; //Total time gps
    int64_t days = gps_epoch_days + gps_sec / 86400; //Day gps + gps epoch

    //Algo to get year month day from Week and tow
    days += 719468; // Shift day count so that day 0 corresponds to the Gregorian epoch (0000-03-01)
    int64_t era = (days >= 0 ? days : days - 146096) / 146097; // Determine the 400-year Gregorian cycle, One Gregorian era = 400 years = 146097 days.
    uint64_t doe = (uint64_t)(days - era * 146097); // Day Of Era: number of days elapsed within the current 400-year cycle.
    uint64_t yoe = (doe - doe/1460 + doe/36524 - doe/146096) / 365; // Year Of Era: compute the year number inside the cycle, correcting for leap years (every 4 years, except centuries, except 400-year multiples).
    int64_t y = (int64_t)yoe + era * 400; // Convert "year in era" to an absolute Gregorian year.
    uint64_t doy = doe - (365*yoe + yoe/4 - yoe/100); // Day Of Year: subtract all full days from completed years to get day index within the year.
    uint64_t mp = (5*doy + 2)/153; // Month part: day-of-year into a month index in a calendar where March = 0 and February = 11
    uint64_t d = doy - (153*mp+2)/5 + 1; // Compute the day-of-month
    uint64_t m = mp + (mp < 10 ? 3 : -9); // Convert shifted month index back to standard month  0-12   
    y += (m <= 2);

    // Hours / minutes / secondes
    uint32_t sec_of_day = gps_sec % 86400;
    uint8_t hour = sec_of_day / 3600;
    uint8_t min  = (sec_of_day % 3600) / 60;
    uint8_t sec  = sec_of_day % 60;

    // Format : DDMMYY
    snprintf(buf_date, buf_date_size, "%02d%02d%02d",
             (uint8_t)d,
             (uint8_t)m,
             (uint8_t)(y % 100));

    // Format : HHMMSS
    snprintf(buf_time, buf_time_size, "%02d%02d%02d",
             hour,
             min,
             sec);             
}

/**
 * Function to convert lat long in DDLL.MMMM Format
 * Lat and long don't have the same length
 */
void lidar_convert_deg_to_DDMM(double deg, char *buf, int is_lat) {
    double abs_deg = fabs(deg);
    int d = (int)abs_deg;
    double minutes = (abs_deg - d) * 60.0;

    if (is_lat)
        sprintf(buf, "%02d%07.4f", d, minutes); // 2 integers for degrees 0 to 90 without sign => %02d | 2 integers for minutes, 1 for point, 4 for minutes decimals 
    else
        sprintf(buf, "%03d%07.4f", d, minutes); // 3 integers for degrees 0 to 180 without sign => %03d | 2 integers for minutes, 1 for point, 4 for minutes decimals 
}
