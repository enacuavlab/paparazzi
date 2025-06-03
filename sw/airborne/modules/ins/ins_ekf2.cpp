/*
 * Copyright (C) 2022 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/ins/ins_ekf2.cpp
 *
 * INS based in the EKF2 of PX4
 *
 */


#include "modules/ins/ins_ekf2.h"
#include "modules/nav/waypoints.h"
#include "modules/core/abi.h"
#include "stabilization/stabilization_attitude.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "EKF/ekf.h"
#include "math/pprz_isa.h"
#include "math/pprz_geodetic_wgs84.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"

/** For SITL and NPS we need special includes */
#if defined SITL && USE_NPS
#include "nps_autopilot.h"
#include <stdio.h>
#endif

#if defined(CONFIG_EKF2_GNSS)
static abi_event gps_ev;
static void gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s);
#ifndef INS_EKF2_GPS_ID
#define INS_EKF2_GPS_ID GPS_MULTI_ID
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_ID)
#endif

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
static abi_event optical_flow_ev;
static void optical_flow_cb(uint8_t sender_id, uint32_t stamp, int32_t flow_x, int32_t flow_y, int32_t flow_der_x, int32_t flow_der_y, float quality, float size_divergence);
#ifndef INS_EKF2_OF_ID
#define INS_EKF2_OF_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_OF_ID)
#endif

#ifndef INS_EKF2_GYRO_ID
#define INS_EKF2_GYRO_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_GYRO_ID)
static abi_event gyro_int_ev;
static void gyro_int_cb(uint8_t sender_id, uint32_t stamp, struct FloatRates *delta_gyro, uint16_t dt);

#ifndef INS_EKF2_ACCEL_ID
#define INS_EKF2_ACCEL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_ACCEL_ID)
static abi_event accel_int_ev;
static void accel_int_cb(uint8_t sender_id, uint32_t stamp, struct FloatVect3 *delta_accel, uint16_t dt);


static Ekf ekf;

static parameters *ekf_params;                    ///< The EKF parameters

struct ekf2_t ekf2;                               ///< Local EKF2 status structure

void ins_ekf2_init(void)
{
  ekf_params = ekf.getParamHandle();

  AbiBindMsgIMU_GYRO_INT(INS_EKF2_GYRO_ID, &gyro_int_ev, gyro_int_cb);
  AbiBindMsgIMU_ACCEL_INT(INS_EKF2_ACCEL_ID, &accel_int_ev, accel_int_cb);


#if defined(CONFIG_EKF2_GNSS)
  AbiBindMsgGPS(INS_EKF2_GPS_ID, &gps_ev, gps_cb);
#endif
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
  AbiBindMsgOPTICAL_FLOW(INS_EKF2_OF_ID, &optical_flow_ev, optical_flow_cb);
#endif
}

static void ins_ekf2_publish_attitude(uint32_t stamp)
{
  imuSample imu_sample = {};
  imu_sample.time_us = stamp;
  imu_sample.delta_ang_dt = ekf2.gyro_dt * 1.e-6f;
  imu_sample.delta_ang = Vector3f{ekf2.delta_gyro.p, ekf2.delta_gyro.q, ekf2.delta_gyro.r};
  imu_sample.delta_vel_dt = ekf2.accel_dt * 1.e-6f;
  imu_sample.delta_vel = Vector3f{ekf2.delta_accel.x, ekf2.delta_accel.y, ekf2.delta_accel.z};
  ekf.setIMUData(imu_sample);

  if (ekf.attitude_valid()) {
    // Calculate the quaternion
    struct FloatQuat ltp_to_body_quat;

    //const Quatf att_q{ekf.calculate_quaternion()};
    const Quatf att_q{ekf.getQuaternion()};

    ltp_to_body_quat.qi = att_q(0);
    ltp_to_body_quat.qx = att_q(1);
    ltp_to_body_quat.qy = att_q(2);
    ltp_to_body_quat.qz = att_q(3);

    // Publish it to the state
    stateSetNedToBodyQuat_f(MODULE_INS_EKF2_ID, &ltp_to_body_quat);

    /* Check the quaternion reset state */
    float delta_q_reset[4];
    uint8_t quat_reset_counter;
    ekf.get_quat_reset(delta_q_reset, &quat_reset_counter);

#ifndef NO_RESET_UPDATE_SETPOINT_HEADING
    // FIXME is this hard reset of control setpoint really needed ? is it the right place ?
    if (ekf2.quat_reset_counter < quat_reset_counter) {
      float psi = matrix::Eulerf(matrix::Quatf(delta_q_reset)).psi();
      guidance_h.sp.heading += psi;
      guidance_h.rc_sp.heading += psi;
      nav.heading += psi;
      //guidance_h_read_rc(autopilot_in_flight());
      stabilization_attitude_enter();
      ekf2.quat_reset_counter = quat_reset_counter;
    }
#endif

    /* Get in-run gyro bias */
    struct FloatRates body_rates;
    Vector3f gyro_bias{ekf.getGyroBias()};
    body_rates.p = (ekf2.delta_gyro.p / (ekf2.gyro_dt * 1.e-6f)) - gyro_bias(0);
    body_rates.q = (ekf2.delta_gyro.q / (ekf2.gyro_dt * 1.e-6f)) - gyro_bias(1);
    body_rates.r = (ekf2.delta_gyro.r / (ekf2.gyro_dt * 1.e-6f)) - gyro_bias(2);

    // Publish it to the state
    stateSetBodyRates_f(MODULE_INS_EKF2_ID, &body_rates);

    /* Get the in-run acceleration bias */
    struct Int32Vect3 accel;
    Vector3f accel_bias{ekf.getAccelBias()};
    accel.x = ACCEL_BFP_OF_REAL((ekf2.delta_accel.x / (ekf2.accel_dt * 1e-6f)) - accel_bias(0));
    accel.y = ACCEL_BFP_OF_REAL((ekf2.delta_accel.y / (ekf2.accel_dt * 1e-6f)) - accel_bias(1));
    accel.z = ACCEL_BFP_OF_REAL((ekf2.delta_accel.z / (ekf2.accel_dt * 1e-6f)) - accel_bias(2));

    // Publish it to the state
    stateSetAccelBody_i(MODULE_INS_EKF2_ID, &accel);
  }
  ekf2.gyro_valid = false;
  ekf2.accel_valid = false;
  ekf2.got_imu_data = true;
}


static void gyro_int_cb(uint8_t __attribute__((unused)) sender_id,
                    uint32_t stamp, struct FloatRates *delta_gyro, uint16_t dt)
{
  // Copy and save the gyro data
  RATES_COPY(ekf2.delta_gyro, *delta_gyro);
  ekf2.gyro_dt = dt;
  ekf2.gyro_valid = true;

  /* When Gyro and accelerometer are valid enter it into the EKF */
  if (ekf2.gyro_valid && ekf2.accel_valid) {
    ins_ekf2_publish_attitude(stamp);
  }
}

static void accel_int_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp, struct FloatVect3 *delta_accel, uint16_t dt)
{
  // Copy and save the gyro data
  VECT3_COPY(ekf2.delta_accel, *delta_accel);
  ekf2.accel_dt = dt;
  ekf2.accel_valid = true;

  /* When Gyro and accelerometer are valid enter it into the EKF */
  if (ekf2.gyro_valid && ekf2.accel_valid) {
    ins_ekf2_publish_attitude(stamp);
  }
}


#if defined(CONFIG_EKF2_GNSS)
static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp,
                   struct GpsState *gps_s)
{
  gnssSample gps_msg = {};
//  gps_msg.time_usec = stamp;
  struct LlaCoor_i lla_pos = lla_int_from_gps(gps_s);
  gps_msg.lat = lla_pos.lat;
  gps_msg.lon = lla_pos.lon;
  gps_msg.alt = gps_s->hmsl; // EKF2 works with HMSL
#if INS_EKF2_GPS_COURSE_YAW
  gps_msg.yaw = wrap_pi((float)gps_s->course / 1e7);
  gps_msg.yaw_offset = 0;
#elif defined(INS_EKF2_GPS_YAW_OFFSET)
  if(ekf2.rel_heading_valid) {
    gps_msg.yaw = wrap_pi(ekf2.rel_heading - RadOfDeg(INS_EKF2_GPS_YAW_OFFSET));
    ekf2.rel_heading_valid = false;
  } else {
    gps_msg.yaw = NAN;
  }

  // Offset also needs to be substracted from the heading (this is for roll/pitch angle limits)
  gps_msg.yaw_offset = RadOfDeg(INS_EKF2_GPS_YAW_OFFSET);
#else
  gps_msg.yaw = NAN;
  gps_msg.yaw_offset = NAN;
#endif
  gps_msg.fix_type = gps_s->fix;
//  gps_msg.eph = gps_s->hacc / 100.0;
//  gps_msg.epv = gps_s->vacc / 100.0;
  gps_msg.sacc = gps_s->sacc / 100.0;
//  gps_msg.vel_m_s = gps_s->gspeed / 100.0;
  struct NedCoor_f ned_vel = ned_vel_float_from_gps(gps_s);
//  gps_msg.vel_ned(0) = ned_vel.x;
//  gps_msg.vel_ned(1) = ned_vel.y;
//  gps_msg.vel_ned(2) = ned_vel.z;
//  gps_msg.vel_ned_valid = bit_is_set(gps_s->valid_fields, GPS_VALID_VEL_NED_BIT);
  gps_msg.nsats = gps_s->num_sv;
  gps_msg.pdop = gps_s->pdop;

  ekf.setGpsData(gps_msg);
}
#endif

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
static void optical_flow_cb(uint8_t sender_id __attribute__((unused)),
                            uint32_t stamp,
                            int32_t flow_x,
                            int32_t flow_y,
                            int32_t flow_der_x __attribute__((unused)),
                            int32_t flow_der_y __attribute__((unused)),
                            float quality,
                            float size_divergence __attribute__((unused)))
{
  flowSample sample;
  sample.time_us = stamp;

  // Wait for two measurements in order to integrate
  if (ekf2.flow_stamp <= 0) {
    ekf2.flow_stamp = stamp;
    return;
  }

  // Calculate the timestamp
//  sample.dt = (stamp - ekf2.flow_stamp);
  ekf2.flow_stamp = stamp;

  /* Build integrated flow and gyro messages for filter
  NOTE: pure rotations should result in same flow_x and
  gyro_roll and same flow_y and gyro_pitch */
  Vector2f flowdata;
  flowdata(0) = RadOfDeg(flow_y) ;//* (1e-6 *
                                    //sample.dt);                       // INTEGRATED FLOW AROUND Y AXIS (RIGHT -X, LEFT +X)
  flowdata(1) = - RadOfDeg(flow_x); // * (1e-6 *
                                    //sample.dt);                     // INTEGRATED FLOW AROUND X AXIS (FORWARD +Y, BACKWARD -Y)

  sample.quality = quality;                     // quality indicator between 0 and 255
//  sample.flow_xy_rad =
 //   flowdata;                   // measured delta angle of the image about the X and Y body axes (rad), RH rotaton is positive
 // sample.gyro_xyz = Vector3f{NAN, NAN, NAN};    // measured delta angle of the inertial frame about the body axes obtained from rate gyro measurements (rad), RH rotation is positive

  // Update the optical flow data based on the callback
  //ekf.setOpticalFlowData(sample);
  ekf.setOpticalFlowData(sample);
}
#endif


#if defined(CONFIG_EKF2_EXTERNAL_VISION)
void ins_ekf2_parse_EXTERNAL_POSE(uint8_t *buf) {
  if (DL_EXTERNAL_POSE_ac_id(buf) != AC_ID) { return; } // not for this aircraft

  extVisionSample sample_ev;

  sample_ev.time_us = get_sys_time_usec(); //FIXME
  sample_ev.pos(0) = DL_EXTERNAL_POSE_enu_y(buf);
  sample_ev.pos(1) = DL_EXTERNAL_POSE_enu_x(buf);
  sample_ev.pos(2) = -DL_EXTERNAL_POSE_enu_z(buf);
  sample_ev.vel(0) = DL_EXTERNAL_POSE_enu_yd(buf);
  sample_ev.vel(1) = DL_EXTERNAL_POSE_enu_xd(buf);      
  sample_ev.vel(2) = -DL_EXTERNAL_POSE_enu_zd(buf);
  sample_ev.quat(0) = DL_EXTERNAL_POSE_body_qi(buf);
  sample_ev.quat(1) = DL_EXTERNAL_POSE_body_qy(buf);
  sample_ev.quat(2) = DL_EXTERNAL_POSE_body_qx(buf);
  sample_ev.quat(3) = -DL_EXTERNAL_POSE_body_qz(buf);
  
#ifdef INS_EXT_VISION_ROTATION
  // Rotate the quaternion
  struct FloatQuat body_q = {sample_ev.quat(0), sample_ev.quat(1), sample_ev.quat(2), sample_ev.quat(3)};
  struct FloatQuat rot_q;
  float_quat_comp(&rot_q, &body_q, &ins_ext_vision_rot);
  sample_ev.quat(0) = rot_q.qi;
  sample_ev.quat(1) = rot_q.qx;
  sample_ev.quat(2) = rot_q.qy;
  sample_ev.quat(3) = rot_q.qz;
#endif

//  sample_ev.posVar.setAll(INS_EKF2_EVP_NOISE);
//  sample_ev.velCov = matrix::eye<float, 3>() * INS_EKF2_EVV_NOISE;
//  sample_ev.angVar = INS_EKF2_EVA_NOISE;
//  sample_ev.vel_frame = velocity_frame_t::LOCAL_FRAME_FRD;

  ekf.setExtVisionData(sample_ev);
}
#else
void ins_ekf2_parse_EXTERNAL_POSE(uint8_t *buf) {}
#endif

void ins_ekf2_update(void){};
void ins_ekf2_change_param(int32_t unk){};
void ins_ekf2_remove_gps(int32_t mode){};
