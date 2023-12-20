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

/** MAG */
#ifndef ROT_MAG_BIND_ID
#define ROT_MAG_BIND_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(ROT_MAG_BIND_ID)

static abi_event gyro_ev;
static abi_event accel_ev;
static abi_event mag_ev; 

struct RotateImu rotate_imu;

struct FloatVect3 vect_fuselage_rate;

struct FloatVect3 accel_imu_f;
struct FloatRates gyro_imu_f;
struct FloatVect3 mag_imu_f;
struct FloatVect3 accel_rot_f;
struct FloatVect3 old_accel_rot_f;
uint32_t accel_last_stamp;
struct FloatRates gyro_rot_f;
struct FloatRates old_gyro_rot_f;
uint32_t gyro_last_stamp;
struct FloatVect3 mag_rot_f;
struct FloatVect3 rates_vect;
float angle_filter;


float angular_accel[3] = {0., 0., 0.};
Butterworth2LowPass meas_lowpass_filters[3];


/* All ABI callbacks */
static void gyro_cb(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro);
static void accel_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel);
static void mag_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, struct Int32Vect3 *mag);
// static void gyro_int_cb(uint8_t sender_id, uint32_t stamp, struct FloatRates *delta_gyro, uint16_t dt);
// static void accel_int_cb(uint8_t sender_id, uint32_t stamp, struct FloatVect3 *delta_accel, uint16_t dt);


#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#include "modules/ctrl/body_stabilisation.h"
static void send_payload_float(struct transport_tx *trans, struct link_device *dev)
{
  float f[15] = {-encoder_amt22.amt22.angle_rad, -encoder_amt22.H_g_filter.hatx[0], -encoder_amt22.H_g_filter.hatx[1],
                 motor_cmd, elevator_cmd, actuators_pprz[6], actuators_pprz[7], DegOfRad(euler_fus.theta), angle_wing2fus, body_stab.discrete_state,
                 gyro_rot_f.p, gyro_rot_f.q, gyro_rot_f.r, encoder_amt22.amt22.position, rates_vect.y};
  pprz_msg_send_PAYLOAD_FLOAT(trans, dev, AC_ID, 15, f);
}
#endif


void rotate_imu_init(void)
{

  rotate_imu.enabled = ROTATE_IMU_ENABLED;
  rotate_imu.angular_speed = 0.;
  rotate_imu.angular_accel = 0;
  rotate_imu.centre_rot_2_imu.x = ROTATE_IMU_POS_CENTER_IN_IMU_X;
  rotate_imu.centre_rot_2_imu.y = 0;
  rotate_imu.centre_rot_2_imu.z = ROTATE_IMU_POS_CENTER_IN_IMU_Z;
  
  float_quat_identity(&rotate_imu.quat_encoder);

  accel_last_stamp = 0;
  gyro_last_stamp = 0;
 

  float tau = 1.0 / (2.0 * M_PI * 20.0);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;

  // Filtering of the acceleration to perform a finite difference
  int8_t i;
  for (i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&meas_lowpass_filters[i], tau, sample_time, 0.0);
  }

   /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgIMU_GYRO(ROT_IMU_BIND_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL(ROT_IMU_BIND_ID, &accel_ev, accel_cb);
  AbiBindMsgIMU_MAG(ROT_MAG_BIND_ID, &mag_ev, mag_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PAYLOAD_FLOAT, send_payload_float);
#endif
}


/**
 * settings handlers
 */

extern void rotate_imu_update_quat(void){

  angle_filter = encoder_amt22.H_g_filter.hatx[0]; 
  rotate_imu.angular_speed = encoder_amt22.H_g_filter.hatx[1]; 
  rotate_imu.angular_accel = encoder_amt22.H_g_filter.hatx[2];
  QUAT_ASSIGN(rotate_imu.quat_encoder, cos(angle_filter/2),0,sin(angle_filter/2),0);
}

extern void rotate_imu_reset(float enabled){
  rotate_imu.enabled = enabled;
  rotate_imu.angular_speed = 0;
  rotate_imu.angular_accel = 0;
  float_quat_identity(&rotate_imu.quat_encoder);
}



extern void rotate_imu_report(void){
  // debug
  //float f[5] = {DegOfRad(angle_filter), DegOfRad(rotate_imu.angular_speed), DegOfRad(stateGetNedToBodyEulers_f()->phi), DegOfRad(stateGetNedToBodyEulers_f()->theta), DegOfRad(stateGetNedToBodyEulers_f()->psi)};
  //float f[6] = {DegOfRad(angle_filter), DegOfRad(rotate_imu.angular_speed), stateGetNedToBodyQuat_f()->qi, stateGetNedToBodyQuat_f()->qx, stateGetNedToBodyQuat_f()->qy, stateGetNedToBodyQuat_f()->qz};
  float f[8] = {DegOfRad(angle_filter), DegOfRad(rotate_imu.angular_speed), accel_imu_f.x, accel_imu_f.y, accel_imu_f.z, accel_rot_f.x, accel_rot_f.y, accel_rot_f.z};
  //float f[8] = {DegOfRad(angle_filter), DegOfRad(rotate_imu.angular_speed), gyro_imu_f.p, gyro_imu_f.q, gyro_imu_f.r, gyro_rot_f.p, gyro_rot_f.q, gyro_rot_f.r};
  //float f[8] = {DegOfRad(angle_filter), DegOfRad(rotate_imu.angular_speed), mag_imu_f.x, mag_imu_f.y, mag_imu_f.z, mag_rot_f.x, mag_rot_f.y, mag_rot_f.z};
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 8, f);
}

static void gyro_cb(uint8_t sender_id, uint32_t stamp __attribute__((unused)), struct Int32Rates *gyro)
{
  if (sender_id == IMU_ROT_ID) {
    return; // don't process own data
  }
  uint32_t now_ts = get_sys_time_usec();
  if (rotate_imu.enabled) {

    struct FloatRates rates_f;//, rates_pendule_f;
    RATES_FLOAT_OF_BFP(rates_f, *gyro);
    
    //Only for report
    RATES_COPY(gyro_imu_f, rates_f)

    rates_f.q += rotate_imu.angular_speed;
    
    rates_vect.x = rates_f.p;
    rates_vect.y = rates_f.q;
    rates_vect.z = rates_f.r;
    
     struct FloatVect3 rates_rot_vect;
    float_quat_vmult(&rates_rot_vect, &rotate_imu.quat_encoder, &rates_vect);

    gyro_rot_f.p = rates_rot_vect.x;
    gyro_rot_f.q = rates_rot_vect.y;
    gyro_rot_f.r = rates_rot_vect.z;

  
    #if IMU_INTEGRATION
      float rate = IMU_MPU_SPI_PERIODIC_FREQ;
      // Only integrate if we have gotten a previous measurement and didn't overflow the timer
      if(!isnan(rate) && gyro_last_stamp > 0 && stamp > gyro_last_stamp) {
        struct FloatRates integrated;

        // Trapezoidal integration
        integrated.p = (old_gyro_rot_f.p + gyro_rot_f.p) * 0.5f;
        integrated.q = (old_gyro_rot_f.q + gyro_rot_f.q) * 0.5f;
        integrated.r = (old_gyro_rot_f.r + gyro_rot_f.r) * 0.5f;

        // Divide by the time of the collected samples
        integrated.p = integrated.p * (1.f / rate);
        integrated.q = integrated.q * (1.f / rate);
        integrated.r = integrated.r * (1.f / rate);
      
      uint16_t dt = (1e6 / rate); //FIXME
      AbiSendMsgIMU_GYRO_INT(IMU_ROT_ID, now_ts, &integrated, dt); //integrated for ekf2 
      RATES_COPY(old_gyro_rot_f, gyro_rot_f);
      }
      gyro_last_stamp = stamp;
    #endif
    
    struct Int32Rates gyro_rot_i;
    RATES_BFP_OF_REAL(gyro_rot_i, gyro_rot_f);
   
    // send data
    AbiSendMsgIMU_GYRO(IMU_ROT_ID, now_ts, &gyro_rot_i);

  } else {
    AbiSendMsgIMU_GYRO(IMU_ROT_ID, now_ts, gyro);
  }
}

static void accel_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel)
{
  if (sender_id == IMU_ROT_ID) {
    return; // don't process own data
  }
  uint32_t now_ts = get_sys_time_usec();
  if (rotate_imu.enabled) {
    
    struct FloatVect3 accel_f;// , accel_pendule_f;
    ACCELS_FLOAT_OF_BFP(accel_f, *accel);

    //Only for report
    VECT3_COPY(accel_imu_f, accel_f);

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
    
    float_quat_vmult(&accel_rot_f, &rotate_imu.quat_encoder, &accel_f);
    

    #if IMU_INTEGRATION
      float rate = IMU_MPU_SPI_PERIODIC_FREQ;
      // Only integrate if we have gotten a previous measurement and didn't overflow the timer
      if(!isnan(rate) && accel_last_stamp > 0 && stamp > accel_last_stamp) {
        struct FloatVect3 integrated;

        // Trapezoidal integration
        integrated.x = (old_accel_rot_f.x + accel_rot_f.x) * 0.5f;
        integrated.y = (old_accel_rot_f.y + accel_rot_f.y) * 0.5f;
        integrated.z = (old_accel_rot_f.z + accel_rot_f.z) * 0.5f;

        // Divide by the time of the collected samples
        integrated.x = integrated.x * (1.f / rate);
        integrated.y = integrated.y * (1.f / rate);
        integrated.z = integrated.z * (1.f / rate);
      
      uint16_t dt = (1e6 / rate); //FIXME
      AbiSendMsgIMU_ACCEL_INT(IMU_ROT_ID, now_ts, &integrated, dt); //integrated for ekf2 
      VECT3_COPY(old_accel_rot_f, accel_rot_f);
      }
      accel_last_stamp = stamp;
    #endif

    // send data
    struct Int32Vect3 accel_rot_i;
    ACCELS_BFP_OF_REAL(accel_rot_i, accel_rot_f);
    AbiSendMsgIMU_ACCEL(IMU_ROT_ID, now_ts, &accel_rot_i);

  } else {
    AbiSendMsgIMU_ACCEL(IMU_ROT_ID, now_ts, accel);
  }
}

static void mag_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)), struct Int32Vect3 *mag)
{
  if (sender_id == MAG_ROT_ID) {
    return; // don't process own data
  }
  uint32_t now_ts = get_sys_time_usec();
  if (rotate_imu.enabled) {
    
    //Convert in float
    MAGS_FLOAT_OF_BFP(mag_imu_f, *mag);
    //Need to rotate magnetometer ?
    
    float_quat_vmult(&mag_rot_f, &rotate_imu.quat_encoder, &mag_imu_f);

    // send data
    struct Int32Vect3 mag_rot_i;
    //Convert to int
    MAGS_BFP_OF_REAL(mag_rot_i, mag_rot_f);
    AbiSendMsgIMU_MAG(MAG_ROT_ID, now_ts, &mag_rot_i);
  } else {
    AbiSendMsgIMU_MAG(MAG_ROT_ID, now_ts, mag);
  }
  
}
