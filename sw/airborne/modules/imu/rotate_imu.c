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

/** @file "modules/imu/rotate_imu.c"
 * @author Florian Sansou <florian.sansou@enac.fr>
 */

#include "modules/imu/rotate_imu.h"
#include "modules/sensors/encoder_amt22.h"
#include "filters/low_pass_filter.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_simple_matrix.h"
#include "modules/core/abi.h"

#include "modules/datalink/downlink.h"

/* Enable by default */
#ifndef ROTATE_IMU_ENABLED
#define ROTATE_IMU_ENABLED TRUE
#endif

/* meters */
#ifndef ROTATE_IMU_POS_CENTER_IN_IMU_X
#define ROTATE_IMU_POS_CENTER_IN_IMU_X -0.03517f
#endif
PRINT_CONFIG_VAR(ROTATE_IMU_POS_CENTER_IN_IMU_X)

/* meters */
#ifndef ROTATE_IMU_POS_CENTER_IN_IMU_Z
#define ROTATE_IMU_POS_CENTER_IN_IMU_Z 0.15438f
#endif
PRINT_CONFIG_VAR(ROTATE_IMU_POS_CENTER_IN_IMU_Z)

/**
 * ABI bindings
 *
 * by default bind to all IMU raw data and send rotate data
 * receivers (AHRS, INS) should bind to this module
 */
/** IMU (gyro, accel) */
#ifndef ROT_IMU_BIND_ID
#define ROT_IMU_BIND_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(ROT_IMU_BIND_ID)

static abi_event gyro_ev;
static abi_event accel_ev;
static abi_event mag_ev; // only passthrough

struct RotateImu rotate_imu;

struct FloatVect3 vect_fuselage_rate;

struct FloatVect3 accel_f;
struct FloatVect3 accel_rot_f;
float angle_filter;

float angular_accel[3] = {0., 0., 0.};
Butterworth2LowPass meas_lowpass_filters[3];

static void gyro_cb(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro)
{
  if (sender_id == IMU_ROT_ID) {
    return; // don't process own data
  }

  if (rotate_imu.enabled) {
    
    struct FloatRates wing_angular_speed_f ={0, rotate_imu.angular_speed, 0};
    struct Int32Rates wing_angular_speed_i;
    RATES_BFP_OF_REAL(wing_angular_speed_i, wing_angular_speed_f);
    RATES_ADD(*gyro, wing_angular_speed_i)
    
    // send data
    AbiSendMsgIMU_GYRO(IMU_ROT_ID, stamp, gyro);
  } else {
    AbiSendMsgIMU_GYRO(IMU_ROT_ID, stamp, gyro);
  }
}

static void accel_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel)
{
  if (sender_id == IMU_ROT_ID) {
    return; // don't process own data
  }

  if (rotate_imu.enabled) {
    
    ACCELS_FLOAT_OF_BFP(accel_f, *accel);

    struct FloatRates *body_rates = stateGetBodyRates_f();

    vect_fuselage_rate.x = body_rates->p;
    vect_fuselage_rate.y = body_rates->q - rotate_imu.angular_speed;
    vect_fuselage_rate.z = body_rates->r;

   float rate_vect[3] = {vect_fuselage_rate.x, vect_fuselage_rate.y, vect_fuselage_rate.z};
   int8_t i;
   for (i = 0; i < 3; i++) {
     update_butterworth_2_low_pass(&meas_lowpass_filters[i], rate_vect[i]);
  
     //Calculate the angular acceleration via finite difference
     angular_accel[i] = (meas_lowpass_filters[i].o[0]
                                - meas_lowpass_filters[i].o[1]) * PERIODIC_FREQUENCY;
   }

    struct FloatVect3 _tmp1, _tmp2, ang_accel_f;

    ang_accel_f.x = angular_accel[0];
    ang_accel_f.y = angular_accel[1];
    ang_accel_f.z = angular_accel[2];

    VECT3_CROSS_PRODUCT(_tmp1, vect_fuselage_rate, rotate_imu.centre_rot_2_imu);
    VECT3_CROSS_PRODUCT(_tmp2, vect_fuselage_rate, _tmp1);

    struct FloatVect3 _tmp3;
    VECT3_CROSS_PRODUCT(_tmp3, ang_accel_f, rotate_imu.centre_rot_2_imu);

    VECT3_ADD(_tmp3, _tmp2);
    VECT3_ADD(accel_f, _tmp3);
    
    // compute rotation
    
    float_rmat_vmult(&accel_rot_f, &rotate_imu.Rot_mat_f, &accel_f);

  
    // send data
    struct Int32Vect3 accel_rot_i;
    ACCELS_BFP_OF_REAL(accel_rot_i, accel_rot_f);
    AbiSendMsgIMU_ACCEL(IMU_ROT_ID, stamp, &accel_rot_i);

  } else {
    AbiSendMsgIMU_ACCEL(IMU_ROT_ID, stamp, accel);
  }
}

static void mag_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp, struct Int32Vect3 *mag)
{
  if (sender_id == IMU_ROT_ID) {
    return; // don't process own data
  }
   if (rotate_imu.enabled) {

    struct FloatVect3 mag_f;
    //Convert in float
    MAGS_FLOAT_OF_BFP(mag_f, *mag);
    //Need to rotate magnetometer ?
    struct FloatVect3 mag_rot;
    float_rmat_vmult(&mag_rot, &rotate_imu.Rot_mat_f, &mag_f);

    // send data
    struct Int32Vect3 mag_rot_i;
    //Convert to int
    MAGS_BFP_OF_REAL(mag_rot_i, mag_rot);
    AbiSendMsgIMU_MAG(IMU_ROT_ID, stamp, &mag_rot_i);
   }
     else {
    AbiSendMsgIMU_ACCEL(IMU_ROT_ID, stamp, mag);
  }
  
}


void rotate_imu_init(void)
{
  rotate_imu.enabled = ROTATE_IMU_ENABLED;
  rotate_imu.angular_speed = 0.;
  rotate_imu.angular_accel = 0;
  rotate_imu.centre_rot_2_imu.x = ROTATE_IMU_POS_CENTER_IN_IMU_X;
  rotate_imu.centre_rot_2_imu.y = 0;
  rotate_imu.centre_rot_2_imu.z = ROTATE_IMU_POS_CENTER_IN_IMU_Z;
  

  FLOAT_MAT33_DIAG(rotate_imu.Rot_mat_f, 1., 1., 1.);

  float tau = 1.0 / (2.0 * M_PI * 20.0);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  // Filtering of the gyroscope
  int8_t i;
  for (i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&meas_lowpass_filters[i], tau, sample_time, 0.0);
  }

  AbiBindMsgIMU_GYRO(ROT_IMU_BIND_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL(ROT_IMU_BIND_ID, &accel_ev, accel_cb);
  AbiBindMsgIMU_MAG(ROT_IMU_BIND_ID, &mag_ev, mag_cb);
}


/**
 * settings handlers
 */

extern void rotate_imu_update_dcm_matrix(void){

  angle_filter = -encoder_amt22.H_g_filter.hatx[0]; 
  rotate_imu.angular_speed = -encoder_amt22.H_g_filter.hatx[1]; 
  rotate_imu.angular_accel = -encoder_amt22.H_g_filter.hatx[2];
  struct FloatEulers euler_f = { RadOfDeg(0.), angle_filter, RadOfDeg(0.)};
  float_rmat_of_eulers_321(&rotate_imu.Rot_mat_f, &euler_f);
}

extern void rotate_imu_reset(float enabled){
  rotate_imu.enabled = enabled;
  rotate_imu.angular_speed = 0;
  rotate_imu.angular_accel = 0;
  FLOAT_MAT33_DIAG(rotate_imu.Rot_mat_f, 1., 1., 1.);
}



extern void rotate_imu_report(void){
  // debug
  float f[7] = {angle_filter, accel_f.x, accel_f.y, accel_f.z, accel_rot_f.x, accel_rot_f.y, accel_rot_f.z};
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 7, f);
}