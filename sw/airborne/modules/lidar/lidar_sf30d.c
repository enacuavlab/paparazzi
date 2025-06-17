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
// #include "filters/median_filter.h"
#include "state.h" // State interface for rotation compensation


#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "std.h"

// ** Declaration ** // 

struct LidarSF30D lidar_sf30d;

static void sf30d_init(struct LidarSF30D* sf30d);
static void sf30d_downlink(struct LidarSF30D* sf30d);
static void lidar_sf30d_thd(void* arg);


/**
 * Initialization function
 */
void lidar_sf30d_init(void) {
  sf30d_init(&lidar_sf30d);
}

static void sf30d_init(struct LidarSF30D* sf30d) {
  sf30d->log_ptu_started = false;
  sf30d->trans.status = I2CTransDone;
  sf30d->addr = LIDAR_SF30D_I2C_ADRESS;
  sf30d->update_agl = USE_LIDAR_SF30D_AGL;
  sf30d->compensate_rotation = LIDAR_SF30D_COMPENSATE_ROTATION;
  sf30d->init_status = LIDAR_CONF_UNINIT;
  sf30d->error_init = false;
  sf30d->initialized = false;

  sf30d->msg = (const struct lidar_sf30d_msg_t){0};

  init_median_filter_i(&sf30d->lidar_sf30d_filter, MEDIAN_DEFAULT_SIZE);

  pprz_mtx_init(&sf30d->mtx);

  pprz_thread_create(&sf30d->thd_handle, 512, "sf30d", PPRZ_NORMAL_PRIO+1, lidar_sf30d_thd, sf30d);
}


static void lidar_sf30d_thd(void* arg) {
  struct LidarSF30D* sf30d = (struct LidarSF30D*)arg;

  sys_time_msleep(LIDAR_SF30D_STARTUP_DELAY*1000);

  // Enable register based protocol
  sf30d->trans.buf[0]= 120;
  sf30d->trans.buf[1]= 170;
  sf30d->trans.buf[2]= 170;

  while(i2c_blocking_transmit(&LIDAR_SF30D_I2C_DEV, &sf30d->trans, sf30d->addr, 3, 0.5) != I2CTransSuccess) {
    sys_time_msleep(50);
  }

  // Set output mode
  int distance_output_value = 255;
  sf30d->trans.buf[0] = 29;
  sf30d->trans.buf[1] = (distance_output_value >> 0) & 0xFF;
  sf30d->trans.buf[2] = (distance_output_value >> 8) & 0xFF;
  sf30d->trans.buf[3] = (distance_output_value >> 16) & 0xFF;
  sf30d->trans.buf[4] = (distance_output_value >> 24) & 0xFF;
  while(i2c_blocking_transmit(&LIDAR_SF30D_I2C_DEV, &sf30d->trans, sf30d->addr, 5, 0.5) != I2CTransSuccess) {
    sys_time_msleep(50);
  }

  while(true) {
    //set tx to distance register
    sf30d->trans.buf[0] = LIDAR_SF30D_REG_REQ_DATA;
     // We send the reg and we get in return 8 byte
    if(i2c_blocking_transceive(&LIDAR_SF30D_I2C_DEV, &sf30d->trans, sf30d->addr, 1, 8, 0.5) != I2CTransSuccess) {
      continue;
    }

    struct lidar_sf30d_msg_t msg = {0};

    // Process results
    msg.now_ts = get_sys_time_usec();
    struct lidar_sf30d_data_t* data = (struct lidar_sf30d_data_t*)sf30d->trans.buf;
    msg.sensor_data = *data;
    // filter data
    msg.homeFiltered_raw = update_median_filter_i(&sf30d->lidar_sf30d_filter,msg.sensor_data.first);
    msg.homeFiltered = ((float)msg.homeFiltered_raw)/100.0;

    // compensate rotation if requested
    if (sf30d->compensate_rotation) {
        msg.phi = stateGetNedToBodyEulers_f()->phi;
        msg.theta = stateGetNedToBodyEulers_f()->theta;
        msg.gain = (float)fabs( (double) (cosf(msg.phi) * cosf(msg.theta)));
        msg.homeFiltered = msg.homeFiltered / msg.gain;
    }

    // Communication error management, if the first strength is 0 and the last distance is greater than 3000 cm, we consider it as an error
    // if (lidar_sf30d.msg.firstStrength == 0 && lidar_sf30d.msg.last > 3000) {
    //   lidar_sf30d_init();
    //   lidar_sf30d.error_init = true;
    //   return;
    // }

    // send ABI message (if requested)
    if (sf30d->update_agl) {
      //AbiSendMsgAGL(AGL_LIDAR_SF30D_ID, lidar_sf30d.msg.now_ts, lidar_sf30d.msg.homeFiltered);
    }

    // save to sd card
    // lidar_sf30d_log_data();

    pprz_mtx_lock(&sf30d->mtx);
    sf30d->msg = msg;
    pprz_mtx_unlock(&sf30d->mtx);

    sys_time_msleep(2);

  }

}


/**
 * Downlink message for debug
 */
void lidar_sf30d_downlink(void) {
  sf30d_downlink(&lidar_sf30d);
}

static void sf30d_downlink(struct LidarSF30D* sf30d) {
  pprz_mtx_lock(&sf30d->mtx);
  uint8_t trans = sf30d->trans.status;
  uint8_t status = 3; // Default status: 3 (no error), random value chosen for debug
  if(sf30d->error_init == true){
    status = 1; // Error during initialization
  }
  sf30d->error_init = false; // Reset error flag after sending
  float distance = sf30d->msg.homeFiltered;
  pprz_mtx_unlock(&sf30d->mtx);

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
      lidar_sf30d.msg.sensor_data.first,
      lidar_sf30d.msg.sensor_data.firstFiltered,
      lidar_sf30d.msg.sensor_data.firstStrength,
      lidar_sf30d.msg.homeFiltered,
      lidar_sf30d.msg.homeFiltered_raw,
      lidar_sf30d.msg.sensor_data.last,
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

