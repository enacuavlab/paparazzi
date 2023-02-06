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
#include "filters/high_gain_filter.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_simple_matrix.h"
#include "modules/core/abi.h"

#include "modules/datalink/downlink.h"

/** Enable by default */
#ifndef ROTATE_IMU_ENABLED
#define ROTATE_IMU_ENABLED TRUE
#endif


// these parameters are used in the filtering of the angular acceleration
// define them in the airframe file if different values are required
#ifndef ROTATE_IMU_FILT_CUTOFF
#define ROTATE_IMU_FILT_CUTOFF 8.0
#endif

// the yaw sometimes requires more filtering
#ifndef ROTATE_IMU_FILT_CUTOFF_RDOT
#define ROTATE_IMU_FILT_CUTOFF_RDOT ROTATE_IMU_FILT_CUTOFF
#endif

#ifndef ROTATE_IMU_POS_CENTER_IN_IMU_X
#define ROTATE_IMU_POS_CENTER_IN_IMU_X 0.1
#endif

#ifndef ROTATE_IMU_POS_CENTER_IN_IMU_Y
#define ROTATE_IMU_POS_CENTER_IN_IMU_Y 0.0
#endif

#ifndef ROTATE_IMU_POS_CENTER_IN_IMU_Z
#define ROTATE_IMU_POS_CENTER_IN_IMU_Z -0.1
#endif


/**
 * ABI bindings
 *
 * by default bind to all IMU raw data and send filtered data
 * receivers (AHRS, INS) should bind to this prefilter module
 */
/** IMU (gyro, accel) */
#ifndef IMU_ROT_IMU_BIND_ID
#define IMU_ROT_IMU_BIND_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(IMU_ROT_IMU_BIND_ID)

static abi_event gyro_ev;
static abi_event accel_ev;
static abi_event mag_ev; // only passthrough

struct RotateImu rotate_imu;

 static inline void filter_pqr(Butterworth2LowPass *filter, struct FloatRates *new_values)
 {
   update_butterworth_2_low_pass(&filter[0], new_values->p);
   update_butterworth_2_low_pass(&filter[1], new_values->q);
   update_butterworth_2_low_pass(&filter[2], new_values->r);
 }
  
 static inline void finite_difference_from_filter(float *output, Butterworth2LowPass *filter)
 {
   for (int8_t i = 0; i < 3; i++) {
     output[i] = (filter[i].o[0] - filter[i].o[1]) * PERIODIC_FREQUENCY;
   }
 }

static void gyro_cb(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro)
{
  if (sender_id == IMU_ROT_ID) {
    return; // don't process own data
  }

  if (rotate_imu.enabled) {
    struct FloatRates gyro_f;
    RATES_FLOAT_OF_BFP(gyro_f, *gyro);
    // compute rotation
    gyro_f.q += rotate_imu.angular_speed; 
    // send data
    struct Int32Rates gyro_i;
    RATES_BFP_OF_REAL(gyro_i, gyro_f);
    AbiSendMsgIMU_GYRO(IMU_ROT_ID, stamp, &gyro_i);
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
    struct FloatVect3 accel_f;
    ACCELS_FLOAT_OF_BFP(accel_f, *accel);


    struct FloatVect3 body_rate_f_vect;
   
    // Only pass really low frequencies so you don't adapt to noise
    struct FloatRates *body_rates = stateGetBodyRates_f();

    filter_pqr(rotate_imu.rate, body_rates);
    
    // Calculate the first (and second) derivatives of the rates 
    finite_difference_from_filter(rotate_imu.rate_d, rotate_imu.rate);
    //finite_difference(est->rate_dd, est->rate_d, rate_d_prev);

    body_rate_f_vect.x = body_rates->p;
    body_rate_f_vect.y = body_rates->q;
    body_rate_f_vect.z = body_rates->r;

    struct FloatVect3 _tmp;
    struct FloatVect3 accel_r;
    struct FloatVect3 accel_theta;
    

    VECT3_CROSS_PRODUCT(_tmp, body_rate_f_vect, body_rate_f_vect);
    VECT3_CROSS_PRODUCT(accel_r, _tmp, rotate_imu.centre_rot_2_imu)

    struct FloatVect3 rate_d_f;
    rate_d_f.x = rotate_imu.rate_d[0];
    rate_d_f.y = rotate_imu.rate_d[1];
    rate_d_f.z = rotate_imu.rate_d[2];
    VECT3_CROSS_PRODUCT(accel_theta, rate_d_f, rotate_imu.centre_rot_2_imu);

    // Rotation de la base de frenet au repere imu
    struct FloatVect3 _tmp1;
    struct FloatVect3 _tmp2;
    float_rmat_vmult(&_tmp1, &rotate_imu.Rot_frenet2imu_f, &accel_r);
    float_rmat_vmult(&_tmp2, &rotate_imu.Rot_frenet2imu_f, &accel_theta);

    VECT3_SUB(accel_f, _tmp1);
    VECT3_SUB(accel_f, _tmp2);

    // compute rotation
    struct FloatVect3 accel_rot_f;
    float_rmat_vmult(&accel_rot_f, &rotate_imu.Rot_mat_f, &accel_f);
    //float f[3] = {accel_rot_f.x, accel_rot_f.y, accel_rot_f.z};
    //DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 3, f);

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
  rotate_imu.centre_rot_2_imu.x = ROTATE_IMU_POS_CENTER_IN_IMU_X;
  rotate_imu.centre_rot_2_imu.y = ROTATE_IMU_POS_CENTER_IN_IMU_Y;
  rotate_imu.centre_rot_2_imu.z = ROTATE_IMU_POS_CENTER_IN_IMU_Z;

  FLOAT_MAT33_DIAG(rotate_imu.Rot_mat_f, 1., 1., 1.);

  struct FloatEulers eul_frenet2imu;
  EULERS_ASSIGN(eul_frenet2imu, RadOfDeg(0.), atan(rotate_imu.centre_rot_2_imu.z/rotate_imu.centre_rot_2_imu.x), RadOfDeg(0.));

  float_rmat_of_eulers_321(&rotate_imu.Rot_frenet2imu_f, &eul_frenet2imu);

  // tau = 1/(2*pi*Fc)
   float tau = 1.0 / (2.0 * M_PI * ROTATE_IMU_FILT_CUTOFF);
   float tau_rdot = 1.0 / (2.0 * M_PI * ROTATE_IMU_FILT_CUTOFF_RDOT);
   float tau_axis[3] = {tau, tau, tau_rdot};
   float sample_time = 1.0 / PERIODIC_FREQUENCY;
   // Filtering of gyroscope and actuators
   for (int8_t i = 0; i < 3; i++) {
     init_butterworth_2_low_pass(&rotate_imu.rate[i], tau_axis[i], sample_time, 0.0);
   }

  AbiBindMsgIMU_GYRO(IMU_ROT_IMU_BIND_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL(IMU_ROT_IMU_BIND_ID, &accel_ev, accel_cb);
  AbiBindMsgIMU_MAG(IMU_ROT_IMU_BIND_ID, &mag_ev, mag_cb);
}


/**
 * settings handlers
 */

extern void rotate_imu_update_dcm_matrix(void){

  float angle_filter = H_g_filter_rot.hatx[0]; 
  rotate_imu.angular_speed = H_g_filter_rot.hatx[1]; 
  struct FloatEulers euler_f = { RadOfDeg(0.), angle_filter, RadOfDeg(0.)};
  float_rmat_of_eulers_321(&rotate_imu.Rot_mat_f, &euler_f);

  //float f[2] = {H_g_filter_rot.hatx[0], H_g_filter_rot.hatx[1]};
  //DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 2, f);
}

extern void rotate_imu_reset(float enabled){
  rotate_imu.enabled = enabled;
  rotate_imu.angular_speed = 0;
  FLOAT_MAT33_DIAG(rotate_imu.Rot_mat_f, 1., 1., 1.);
}



