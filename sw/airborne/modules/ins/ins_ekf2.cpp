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

/** INS reference from flight plan, true by default */
#ifndef USE_INS_NAV_INIT
#define USE_INS_NAV_INIT TRUE
#endif


/* IMU X offset from CoG position in meters */
#ifndef INS_EKF2_IMU_POS_X
#define INS_EKF2_IMU_POS_X 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_IMU_POS_X)

/* IMU Y offset from CoG position in meters */
#ifndef INS_EKF2_IMU_POS_Y
#define INS_EKF2_IMU_POS_Y 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_IMU_POS_Y)

/* IMU Z offset from CoG position in meters */
#ifndef INS_EKF2_IMU_POS_Z
#define INS_EKF2_IMU_POS_Z 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_IMU_POS_Z)


#if defined(CONFIG_EKF2_BAROMETER)
static abi_event baro_ev;
static void baro_cb(uint8_t sender_id, uint32_t stamp, float pressure);
#ifndef INS_EKF2_BARO_ID
#if USE_BARO_BOARD
#define INS_EKF2_BARO_ID BARO_BOARD_SENDER_ID
#else
#define INS_EKF2_BARO_ID ABI_BROADCAST
#endif
#endif
PRINT_CONFIG_VAR(INS_EKF2_BARO_ID)
#ifndef INS_EKF2_BARO_NOISE
#define INS_EKF2_BARO_NOISE 3.5f
#endif
PRINT_CONFIG_VAR(INS_EKF2_BARO_NOISE)
#endif

#if defined(CONFIG_EKF2_MAGNETOMETER)
static abi_event mag_ev;
static void mag_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *mag);
#ifndef INS_EKF2_MAG_ID
#define INS_EKF2_MAG_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_MAG_ID)
#endif

#if defined(CONFIG_EKF2_GNSS)
static abi_event gps_ev;
static void gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s);
#ifndef INS_EKF2_GPS_ID
#define INS_EKF2_GPS_ID GPS_MULTI_ID
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_ID)

/* GPS X offset from CoG position in meters */
#ifndef INS_EKF2_GPS_POS_X
#define INS_EKF2_GPS_POS_X 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_POS_X)

/* GPS Y offset from CoG position in meters */
#ifndef INS_EKF2_GPS_POS_Y
#define INS_EKF2_GPS_POS_Y 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_POS_Y)

/* GPS Z offset from CoG position in meters */
#ifndef INS_EKF2_GPS_POS_Z
#define INS_EKF2_GPS_POS_Z 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_POS_Z)

/* GPS measurement noise for horizontal velocity (m/s) */
#ifndef INS_EKF2_GPS_V_NOISE
#define INS_EKF2_GPS_V_NOISE 0.3f
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_V_NOISE)

/* GPS measurement position noise (m) */
#ifndef INS_EKF2_GPS_P_NOISE
#define INS_EKF2_GPS_P_NOISE 0.5f
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_P_NOISE)
#endif


# if defined(CONFIG_EKF2_RANGE_FINDER)
static abi_event agl_ev;
static void agl_cb(uint8_t sender_id, uint32_t stamp, float distance);
#ifndef INS_EKF2_AGL_ID
#define INS_EKF2_AGL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_AGL_ID)
#endif


#if defined(CONFIG_EKF2_OPTICAL_FLOW)
static abi_event optical_flow_ev;
static void optical_flow_cb(uint8_t sender_id, uint32_t stamp, int32_t flow_x, int32_t flow_y, int32_t flow_der_x, int32_t flow_der_y, float quality, float size_divergence);
#ifndef INS_EKF2_OF_ID
#define INS_EKF2_OF_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_OF_ID)

/* Flow sensor noise in rad/sec */
#ifndef INS_EKF2_FLOW_NOISE
#define INS_EKF2_FLOW_NOISE 0.03
#endif
PRINT_CONFIG_VAR(INS_EKF2_FLOW_NOISE)

/* Flow sensor noise at qmin in rad/sec */
#ifndef INS_EKF2_FLOW_NOISE_QMIN
#define INS_EKF2_FLOW_NOISE_QMIN 0.15
#endif
PRINT_CONFIG_VAR(INS_EKF2_FLOW_NOISE_QMIN)
#endif


#if defined(CONFIG_EKF2_EXTERNAL_VISION)
/* External vision position noise (m) */
#ifndef INS_EKF2_EVP_NOISE
#define INS_EKF2_EVP_NOISE 0.02f
#endif
PRINT_CONFIG_VAR(INS_EKF2_EVP_NOISE)
/* External vision velocity noise (m/s) */
#ifndef INS_EKF2_EVV_NOISE
#define INS_EKF2_EVV_NOISE 0.1f
#endif
PRINT_CONFIG_VAR(INS_EKF2_EVV_NOISE)
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

#ifndef INS_EKF2_TEMPERATURE_ID
#define INS_EKF2_TEMPERATURE_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_TEMPERATURE_ID)
static abi_event temperature_ev;
static void temperature_cb(uint8_t sender_id, float temp);


static Ekf ekf;

static parameters *ekf_params;                    ///< The EKF parameters

struct ekf2_t ekf2;                               ///< Local EKF2 status structure


#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_ins(struct transport_tx *trans, struct link_device *dev)
{
  struct NedCoor_i pos, speed, accel;

  // Get it from the EKF
  const Vector3f pos_f{ekf.getPosition()};
  const Vector3f speed_f{ekf.getVelocity()};
  const Vector3f accel_f{ekf.getVelocityDerivative()};

  // Convert to integer
  pos.x = POS_BFP_OF_REAL(pos_f(0));
  pos.y = POS_BFP_OF_REAL(pos_f(1));
  pos.z = POS_BFP_OF_REAL(pos_f(2));
  speed.x = SPEED_BFP_OF_REAL(speed_f(0));
  speed.y = SPEED_BFP_OF_REAL(speed_f(1));
  speed.z = SPEED_BFP_OF_REAL(speed_f(2));
  accel.x = ACCEL_BFP_OF_REAL(accel_f(0));
  accel.y = ACCEL_BFP_OF_REAL(accel_f(1));
  accel.z = ACCEL_BFP_OF_REAL(accel_f(2));

  // Send the message
  pprz_msg_send_INS(trans, dev, AC_ID,
                    &pos.x, &pos.y, &pos.z,
                    &speed.x, &speed.y, &speed.z,
                    &accel.x, &accel.y, &accel.z);
}
#endif

void ins_ekf2_init(void)
{
  ekf_params = ekf.getParamHandle();

  ekf_params->switch_on_accel_bias = 0.2;
  ekf_params->acc_bias_learn_acc_lim = 25.0;
  ekf_params->acc_bias_learn_gyr_lim = 3.0;
  ekf_params->acc_bias_lim = 0.4;
  ekf_params->acc_bias_learn_tc = 0.4;
  ekf_params->accel_bias_p_noise = 0.003;
  ekf_params->accel_noise = 0.35;
  ekf_params->initial_tilt_err = 0.1;
  ekf_params->baro_ctrl = 1;
  ekf_params->baro_innov_gate = 5.0;
  ekf_params->mag_declination_source = 3;

  ekf_params->switch_on_gyro_bias = 0.1;
  ekf_params->gyro_bias_p_noise = 0.001; 

  ekf_params->gnd_effect_deadzone = 4.0;
  ekf_params->gnd_effect_max_hgt = 0.5;

  ekf_params->EKFGSF_tas_default = 15.0;
  ekf_params->gyro_bias_lim = 0.15;
  ekf_params->gyro_bias_p_noise = 0.001;
  ekf_params->gyro_noise = 0.015;

  ekf_params->imu_ctrl = 7;
  ekf_params->terrain_gradient = 0.5;
  ekf_params->terrain_p_noise = 5.0;

  ekf_params->height_sensor_ref = 1;
  ekf_params->pos_noaid_noise = 10.0;
  ekf_params->valid_timeout_max = 5000000;

  ekf_params->imu_pos_body = {
    INS_EKF2_IMU_POS_X,
    INS_EKF2_IMU_POS_Y,
    INS_EKF2_IMU_POS_Z
  };

  ekf_params->filter_update_interval_us = 10000;

  ekf_params->rng_ctrl = 0;
  ekf_params->flow_ctrl = 0;

#if defined(CONFIG_EKF2_GNSS)
  ekf_params->gps_pos_body = {
    INS_EKF2_GPS_POS_X,
    INS_EKF2_GPS_POS_Y,
    INS_EKF2_GPS_POS_Z
  };

  ekf_params->gps_check_mask = 1023;
  ekf_params->gnss_ctrl = 7;
  ekf_params->gps_delay_ms = 110;
  ekf_params->gps_pos_body(0) = 0.0;
  ekf_params->gps_pos_body(1) = 0.0;
  ekf_params->gps_pos_body(2) = 0.0;
  ekf_params->gps_pos_innov_gate = 5.0;
  ekf_params->gps_vel_innov_gate = 5.0;
  ekf_params->gps_vel_noise = INS_EKF2_GPS_V_NOISE;
  ekf_params->gps_pos_noise = INS_EKF2_GPS_P_NOISE;

  ekf_params->req_hacc = 3.0;
  ekf_params->req_vacc = 5.0;
  ekf_params->req_hdrift = 0.1;
  ekf_params->req_nsats = 6;
  ekf_params->req_pdop = 2.5;
  ekf_params->req_sacc = 0.5;
  ekf_params->req_vdrift = 0.2;

  AbiBindMsgGPS(INS_EKF2_GPS_ID, &gps_ev, gps_cb);
#endif 

  ekf2.ltp_stamp = 0;
  ekf2.flow_stamp = 0;
  ekf2.gyro_valid = false;
  ekf2.accel_valid = false;
  ekf2.got_imu_data = false;
  ekf2.quat_reset_counter = 0;
  ekf2.temp = 20.0f; // Default temperature of 20 degrees celcius
  ekf2.qnh = 1013.25f; // Default atmosphere


  /* Initialize the origin from flight plan */
#if USE_INS_NAV_INIT
  if(ekf.setEkfGlobalOrigin(NAV_LAT0*1e-7, NAV_LON0*1e-7, (NAV_ALT0)*1e-3)) // EKF2 works HMSL
  {
    struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
    llh_nav0.lat = NAV_LAT0;
    llh_nav0.lon = NAV_LON0;
    /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
    llh_nav0.alt = NAV_ALT0 + NAV_MSL0; // in millimeters above WGS84 reference ellipsoid
    ltp_def_from_lla_i(&ekf2.ltp_def, &llh_nav0);
    ekf2.ltp_def.hmsl = NAV_ALT0;
    stateSetLocalOrigin_i(MODULE_INS_EKF2_ID, &ekf2.ltp_def);
    /* update local ENU coordinates of global waypoints */
    waypoints_localize_all();
    ekf2.ltp_stamp = 1;
  }
#endif

  AbiBindMsgIMU_GYRO_INT(INS_EKF2_GYRO_ID, &gyro_int_ev, gyro_int_cb);
  AbiBindMsgIMU_ACCEL_INT(INS_EKF2_ACCEL_ID, &accel_int_ev, accel_int_cb);
  AbiBindMsgTEMPERATURE(INS_EKF2_TEMPERATURE_ID, &temperature_ev, temperature_cb);

#if defined(CONFIG_EKF2_BAROMETER)
  ekf_params->baro_noise = INS_EKF2_BARO_NOISE;

  AbiBindMsgBARO_ABS(INS_EKF2_BARO_ID, &baro_ev, baro_cb);
#endif

#if defined(CONFIG_EKF2_MAGNETOMETER)
  ekf_params->mag_acc_gate = 0.5;
  ekf_params->magb_p_noise = 0.0001;
  ekf_params->mag_check = 1;
  ekf_params->mag_check_inclination_tolerance_deg = 20.0;
  ekf_params->mag_check_strength_tolerance_gs = 0.2;
  ekf_params->mag_declination_deg = 0;
  ekf_params->mag_delay_ms = 0;
  ekf_params->mage_p_noise = 0.001;
  ekf_params->mag_innov_gate = 3.0;

  ekf_params->heading_innov_gate = 2.6;
  ekf_params->mag_heading_noise = 0.3;
  ekf_params->mag_fusion_type = 0;

  AbiBindMsgIMU_MAG(INS_EKF2_MAG_ID, &mag_ev, mag_cb);
#endif
       
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
  ekf_params->flow_ctrl = 1;
  ekf_params->flow_delay_ms = 20;
  ekf_params->flow_innov_gate = 3.0;
  ekf_params->flow_noise = INS_EKF2_FLOW_NOISE;
  ekf_params->flow_noise_qual_min = INS_EKF2_FLOW_NOISE_QMIN;
  ekf_params->flow_pos_body = {0,0,0};
  ekf_params->flow_qual_min = 1;
  ekf_params->flow_qual_min_gnd = 0;

  AbiBindMsgOPTICAL_FLOW(INS_EKF2_OF_ID, &optical_flow_ev, optical_flow_cb);
#endif

# if defined(CONFIG_EKF2_RANGE_FINDER)
  ekf_params->rng_ctrl = 1;
  ekf_params->max_hagl_for_range_aid = 5.0;
  ekf_params->range_aid_innov_gate = 1.0;
  ekf_params->max_vel_for_range_aid = 1.0;
  ekf_params->rng_gnd_clearance = 0.1;
  ekf_params->range_delay_ms = 5;
  ekf_params->range_innov_gate = 5.0;
  ekf_params->range_kin_consistency_gate = 1.0;
  ekf_params->range_noise = 0.1;
  ekf_params->rng_sens_pitch = 0.0;
  ekf_params->rng_pos_body = {0.0, 0.0, 0.0};
  ekf_params->range_valid_quality_s = 1.0;
  ekf_params->range_noise_scaler = 0.05;

  AbiBindMsgAGL(INS_EKF2_AGL_ID, &agl_ev, agl_cb);
#endif

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
 ekf_params->ev_att_noise = 0.1;
 ekf_params->ev_pos_innov_gate = 5.0;
 ekf_params->ev_vel_innov_gate = 3.0;
 ekf_params->ev_delay_ms = 0;
 ekf_params->ev_pos_body(0) = 0;
 ekf_params->ev_pos_body(1) = 0;
 ekf_params->ev_pos_body(2) = 0;
 ekf_params->ev_quality_minimum = 0;
 ekf_params->ev_pos_noise = INS_EKF2_EVP_NOISE;
 ekf_params->ev_vel_noise = INS_EKF2_EVV_NOISE;
#endif

#if PERIODIC_TELEMETRY
 register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS, send_ins);
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

/* Save the latest temperature measurement for air density calculations */
static void temperature_cb(uint8_t __attribute__((unused)) sender_id, float temp)
{
  ekf2.temp = temp;
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


#if defined(CONFIG_EKF2_RANGE_FINDER)
/* Update INS based on AGL information */
static void agl_cb(uint8_t __attribute__((unused)) sender_id, uint32_t stamp, float distance)
{
  rangeSample sample;
  sample.time_us = stamp;
  sample.rng = distance;
  sample.quality = -1;
  ekf.setRangeData(sample);
}
#endif // CONFIG_EKF2_RANGE_FINDER)

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


#if defined(CONFIG_EKF2_BAROMETER)
/* Update INS based on Baro information */
static void baro_cb(uint8_t __attribute__((unused)) sender_id, uint32_t stamp, float pressure)
{
  baroSample sample;
  sample.time_us = stamp;
  // Calculate the air density
  float rho = pprz_isa_density_of_pressure(pressure, ekf2.temp);
  ekf.set_air_density(rho);
  // Calculate the height above mean sea level based on pressure
  sample.hgt = pprz_isa_height_of_pressure_full(pressure, ekf2.qnh * 100.0f);
  ekf.setBaroData(sample);
}
#endif // CONFIG_EKF2_BAROMETER


#if defined(CONFIG_EKF2_MAGNETOMETER)
/* Update INS based on Magnetometer information */
static void mag_cb(uint8_t __attribute__((unused)) sender_id,
                   uint32_t stamp,
                   struct Int32Vect3 *mag)
{
  struct FloatVect3 mag_gauss;
  magSample sample;
  sample.time_us = stamp;
  // Convert Magnetometer information to float and to radius 0.2f
  MAGS_FLOAT_OF_BFP(mag_gauss, *mag);
  mag_gauss.x *= 0.4f;
  mag_gauss.y *= 0.4f;
  mag_gauss.z *= 0.4f;
  // Publish information to the EKF
  sample.mag(0) = mag_gauss.x;
  sample.mag(1) = mag_gauss.y;
  sample.mag(2) = mag_gauss.z;
  ekf.setMagData(sample);
  ekf2.got_imu_data = true;
}
#endif // CONFIG_EKF2_MAGNETOMETER
       
       
void ins_ekf2_update(void)
{
  /* Set EKF settings */
  ekf.set_in_air_status(autopilot_in_flight());

  /* Update the EKF */
  if (ekf2.got_imu_data) {
    // Update the EKF but ignore the response and also copy the faster intermediate filter
    ekf.update();
    filter_control_status_u control_status = ekf.control_status();

    // Only publish position after successful alignment
    if (control_status.flags.tilt_align) {
      /* Get the position */
      const Vector3f pos_f{ekf.getPosition()};
      struct NedCoor_f pos;
      pos.x = pos_f(0);
      pos.y = pos_f(1);
      pos.z = pos_f(2);

      // Publish to the state
      stateSetPositionNed_f(MODULE_INS_EKF2_ID, &pos);

      /* Get the velocity in NED frame */
      const Vector3f vel_f{ekf.getVelocity()};
      struct NedCoor_f speed;
      speed.x = vel_f(0);
      speed.y = vel_f(1);
      speed.z = vel_f(2);

      // Publish to state
      stateSetSpeedNed_f(MODULE_INS_EKF2_ID, &speed);

      /* Get the accelerations in NED frame */
      const Vector3f vel_deriv_f{ekf.getVelocityDerivative()};
      struct NedCoor_f accel;
      accel.x = vel_deriv_f(0);
      accel.y = vel_deriv_f(1);
      accel.z = vel_deriv_f(2);

      // Publish to state
      stateSetAccelNed_f(MODULE_INS_EKF2_ID, &accel);

      /* Get local origin */
      // Position of local NED origin in GPS / WGS84 frame
      double ekf_origin_lat, ekf_origin_lon;
      float ref_alt;
      struct LlaCoor_i lla_ref;
      uint64_t origin_time;

      // Only update the origin when the state estimator has updated the origin
      bool ekf_origin_valid = ekf.getEkfGlobalOrigin(origin_time, ekf_origin_lat, ekf_origin_lon, ref_alt);
      if (ekf_origin_valid && (origin_time > ekf2.ltp_stamp)) {
        lla_ref.lat = ekf_origin_lat * 1e7; // WGS-84 lat
        lla_ref.lon = ekf_origin_lon * 1e7; // WGS-84 lon
        lla_ref.alt = ref_alt * 1e3 + wgs84_ellipsoid_to_geoid_i(lla_ref.lat, lla_ref.lon); // in millimeters above WGS84 reference ellipsoid (ref_alt is in HMSL)
        ltp_def_from_lla_i(&ekf2.ltp_def, &lla_ref);
        ekf2.ltp_def.hmsl = ref_alt * 1e3;
        stateSetLocalOrigin_i(MODULE_INS_EKF2_ID, &ekf2.ltp_def);

        /* update local ENU coordinates of global waypoints */
        waypoints_localize_all();

        ekf2.ltp_stamp = origin_time;
      }
    }
  }

#if defined SITL && USE_NPS
  if (nps_bypass_ins) {
    sim_overwrite_ins();
  }
#endif

  ekf2.got_imu_data = false;
}

void ins_ekf2_change_param(int32_t unk){};
void ins_ekf2_remove_gps(int32_t mode){};
