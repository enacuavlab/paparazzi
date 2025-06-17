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

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "std.h"



// ** Declaration ** // 

struct LidarSF30D lidar_sf30d;
struct MedianFilterInt lidar_sf30d_filter;


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
      sdLogWriteLog(pprzLogFile,"time(us),first(cm),firstFiltered(cm),firstStrength,homeFiltered(m),homeFiltered_raw(cm),last(cm),phi(deg),theta(deg),gain\n");
      lidar_sf30d.log_ptu_started = true;
    } else {
      sdLogWriteLog(pprzLogFile, "%lu,%d,%d,%d,%.2f,%lu,%d,%.6f,%.6f,%.6f \n",
      lidar_sf30d.msg.now_ts,
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

