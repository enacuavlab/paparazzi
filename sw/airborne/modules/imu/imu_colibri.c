/*
 * Copyright (C) 2023 Florian Sansou <florian.sansou@enac.fr>
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

/** @file "modules/imu/imu_colibri.c"
 * @author Florian Sansou <florian.sansou@enac.fr>
 * IMU driver for Colibri
- MPU60X0 tawaki board
- MPU9250 wing frame
 */

#include "modules/imu/imu_colibri.h"
#include "modules/imu/imu.h"
#include "modules/core/abi.h"
#include "mcu_periph/spi.h"
#include "mcu_periph/i2c.h"
#include "peripherals/mpu9250_i2c.h"
#include "peripherals/mpu60x0_spi.h"
//#include "peripherals/mpu60x0_i2c.h"


static struct Mpu60x0_Spi mpu60x0;
//static struct Mpu60x0_I2c mpu60x0_wg;
static struct Mpu9250_I2c mpu9250;


void imu_colibri_init(void)
{
  struct Int32RMat rmat;
  struct Int32Eulers eulers;

  /* 
  MPU60x0 tawaki
  */
  mpu60x0_spi_init(&mpu60x0, &(IMU_MPU60X0_SPI_DEV), IMU_MPU60X0_SPI_SLAVE_IDX);

  // Rotation
  eulers.phi = ANGLE_BFP_OF_REAL(0);
  eulers.theta = ANGLE_BFP_OF_REAL(0);
  eulers.psi = ANGLE_BFP_OF_REAL(RadOfDeg(0));
  int32_rmat_of_eulers(&rmat, &eulers);

  // change the default configuration
  mpu60x0.config.smplrt_div = 3;
  mpu60x0.config.dlpf_cfg = MPU60X0_DLPF_256HZ;
  mpu60x0.config.dlpf_cfg_acc = MPU60X0_DLPF_ACC_218HZ; // only for ICM sensors
  mpu60x0.config.gyro_range = MPU60X0_GYRO_RANGE_1000;
  mpu60x0.config.accel_range = MPU60X0_ACCEL_RANGE_8G;

  // Set the default scaling
  imu_set_defaults_gyro(IMU_MPU60X0_ID, &rmat, NULL, MPU60X0_GYRO_SENS_FRAC[MPU60X0_GYRO_RANGE_1000]);
  imu_set_defaults_accel(IMU_MPU60X0_ID, &rmat, NULL, MPU60X0_ACCEL_SENS_FRAC[MPU60X0_ACCEL_RANGE_8G]);


  /*
  MPU60x0 wing
  */
  /*
  mpu60x0_i2c_init(&mpu60x0_wg, &(IMU_MPU60X0_WG_I2C_DEV), MPU60X0_ADDR);

  // Rotation
  eulers.phi = ANGLE_BFP_OF_REAL(0);
  eulers.theta = ANGLE_BFP_OF_REAL(180);
  eulers.psi = ANGLE_BFP_OF_REAL(RadOfDeg(0));
  int32_rmat_of_eulers(&rmat, &eulers);

  // change the default configuration
  mpu60x0.config.smplrt_div = 3;
  mpu60x0.config.dlpf_cfg = MPU60X0_DLPF_256HZ;
  mpu60x0.config.dlpf_cfg_acc = MPU60X0_DLPF_ACC_218HZ; // only for ICM sensors
  mpu60x0.config.gyro_range = MPU60X0_GYRO_RANGE_1000;
  mpu60x0.config.accel_range = MPU60X0_ACCEL_RANGE_8G;

  // Set the default scaling
  imu_set_defaults_gyro(IMU_MPU60X0_WG_ID, &rmat, NULL, MPU60X0_GYRO_SENS_FRAC[MPU60X0_GYRO_RANGE_1000]);
  imu_set_defaults_accel(IMU_MPU60X0_WG_ID, &rmat, NULL, MPU60X0_ACCEL_SENS_FRAC[MPU60X0_ACCEL_RANGE_8G]);
    */
  /* MPU9225 */  
  
  mpu9250_i2c_init(&mpu9250, &(IMU_MPU9250_I2C_DEV), MPU9250_ADDR);

  // Rotation
  eulers.phi = ANGLE_BFP_OF_REAL(RadOfDeg(0));
  eulers.theta = ANGLE_BFP_OF_REAL(RadOfDeg(0));
  eulers.psi = ANGLE_BFP_OF_REAL(RadOfDeg(90));
  int32_rmat_of_eulers(&rmat, &eulers);

  // change the default configuration
  mpu9250.config.smplrt_div = 3;
  mpu9250.config.dlpf_gyro_cfg = MPU9250_DLPF_GYRO_250HZ;
  mpu9250.config.dlpf_accel_cfg = MPU9250_DLPF_ACCEL_184HZ; // only for ICM sensors
  mpu9250.config.gyro_range = MPU9250_GYRO_RANGE_1000;
  mpu9250.config.accel_range = MPU9250_ACCEL_RANGE_8G;

  // Set the default scaling
  imu_set_defaults_gyro(IMU_MPU9250_ID, &rmat, NULL, MPU9250_GYRO_SENS_FRAC[MPU9250_GYRO_RANGE_1000]);
  imu_set_defaults_accel(IMU_MPU9250_ID, &rmat, NULL, MPU9250_ACCEL_SENS_FRAC[MPU9250_ACCEL_RANGE_8G]);


}

void imu_colibri_periodic(void)
{
  mpu60x0_spi_periodic(&mpu60x0);
  //mpu60x0_i2c_periodic(&mpu60x0_wg);
  mpu9250_i2c_periodic(&mpu9250);
  
}


void imu_colibri_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  // If the MPU60X0 tawaki I2C transaction has succeeded: convert the data
  mpu60x0_spi_event(&mpu60x0);
  if (mpu60x0.data_available) {
    // set channel order
    struct Int32Vect3 accel = {
      (int32_t)(mpu60x0.data_accel.value[1]),
      -(int32_t)(mpu60x0.data_accel.value[0]),
      (int32_t)(mpu60x0.data_accel.value[2])
    };
    struct Int32Rates rates = {
      (int32_t)(mpu60x0.data_rates.value[1]),
      -(int32_t)(mpu60x0.data_rates.value[0]),
      (int32_t)(mpu60x0.data_rates.value[2])
    };

    mpu60x0.data_available = false;

    AbiSendMsgIMU_GYRO_RAW(IMU_MPU60X0_ID, now_ts, &rates, 1, IMU_MPU_SPI_PERIODIC_FREQ, mpu60x0.temp);
    AbiSendMsgIMU_ACCEL_RAW(IMU_MPU60X0_ID, now_ts, &accel, 1, IMU_MPU_SPI_PERIODIC_FREQ, mpu60x0.temp);
   
  }
/*
  mpu60x0_i2c_event(&mpu60x0_wg);
  if (mpu60x0_wg.data_available) {
    // set channel order
    struct Int32Vect3 accel = {
      (int32_t)(mpu60x0_wg.data_accel.value[1]),
      (int32_t)(mpu60x0_wg.data_accel.value[0]),
      -(int32_t)(mpu60x0_wg.data_accel.value[2])
    };
    struct Int32Rates rates = {
      (int32_t)(mpu60x0_wg.data_rates.value[1]),
      (int32_t)(mpu60x0_wg.data_rates.value[0]),
      -(int32_t)(mpu60x0_wg.data_rates.value[2])
    };

    mpu60x0_wg.data_available = false;

    AbiSendMsgIMU_GYRO_RAW(IMU_MPU60X0_WG_ID, now_ts, &rates, 1, IMU_MPU_I2C_PERIODIC_FREQ, mpu60x0_wg.temp);
    AbiSendMsgIMU_ACCEL_RAW(IMU_MPU60X0_WG_ID, now_ts, &accel, 1, IMU_MPU_I2C_PERIODIC_FREQ, mpu60x0_wg.temp);
   
  }
*/
  
  // If the MPU9250 I2C transaction has succeeded: convert the data
  mpu9250_i2c_event(&mpu9250);
  if (mpu9250.data_available) {
    // set channel order
    struct Int32Vect3 accel = {
      (int32_t)(mpu9250.data_accel.value[0]),
      (int32_t)(mpu9250.data_accel.value[1]),
      (int32_t)(mpu9250.data_accel.value[2])
    };
    struct Int32Rates rates = {
      (int32_t)(mpu9250.data_rates.value[0]),
      (int32_t)(mpu9250.data_rates.value[1]),
      (int32_t)(mpu9250.data_rates.value[2])
    };

    mpu60x0.data_available = false;

    AbiSendMsgIMU_GYRO_RAW(IMU_MPU9250_ID, now_ts, &rates, 1, IMU_MPU_I2C_PERIODIC_FREQ, NAN);
    AbiSendMsgIMU_ACCEL_RAW(IMU_MPU9250_ID, now_ts, &accel, 1, IMU_MPU_I2C_PERIODIC_FREQ, NAN);
   
  }
  
}


