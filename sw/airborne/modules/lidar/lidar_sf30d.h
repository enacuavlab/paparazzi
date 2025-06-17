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

#ifndef LIDAR_SF30D_H
#define LIDAR_SF30D_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "modules/core/threads.h"
#include "filters/median_filter.h"

#define LIDAR_SF30D_ID 18
#define UPDATE_RATE 8
#define LIDAR_SF30D_REG_REQ_DATA 44 // Register to request data
#define LIDAR_SF30D_STARTUP_DELAY 2.5 // Delay to wait before reading data after startup
#define LIDAR_SF30D_I2C_ADRESS  (0x66 << 1) // LIDAR SF30/D I2C address (7-bit address shifted to 8-bit)
#define LOG_ASCII 0 // 0 == ASCII and 1 == Binary


struct lidar_sf30d_data_t {
  int16_t first;
  int16_t firstFiltered;
  int16_t firstStrength;
  int16_t last;
  int16_t lastFiltered;
  int16_t lastStrength;
  int16_t backgroundNoise;
  int16_t temperature;
} __attribute__((packed));


struct lidar_sf30d_msg_t {
  struct lidar_sf30d_data_t sensor_data;

  float homeFiltered;
  uint32_t homeFiltered_raw;
  float phi;
  float theta;
  float gain;
  uint32_t now_ts;
};

/** config status states */
enum LidarSF30DConfStatus {
  LIDAR_CONF_UNINIT,
  LIDAR_BASED_PROTOCOL,
  LIDAR_OUTPUT_MODE,
  LIDAR_CONF_DONE
};

struct LidarSF30D
{
  struct i2c_transaction trans;
  uint8_t addr;
  enum LidarSF30DConfStatus init_status; 
  bool update_agl;
  bool compensate_rotation;
  bool log_ptu_started;
  bool initialized;
  bool error_init; // Flag to indicate if there was an error during initialization
  struct lidar_sf30d_msg_t msg;

  pprz_mutex_t mtx;
  pprz_thread_t thd_handle;
  struct MedianFilterInt lidar_sf30d_filter;
};


void lidar_sf30d_send_config(void);
void lidar_sf30d_start_configure(void);
void lidar_sf30d_read(void);
void lidar_sf30d_log_data(void);

extern void lidar_sf30d_init(void);
extern void lidar_sf30d_event(void);
extern void lidar_sf30d_periodic(void);
extern void lidar_sf30d_downlink(void);



#endif

