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

/** default barometer to use in INS */
#ifndef INS_EKF2_BARO_ID
#if USE_BARO_BOARD
#define INS_EKF2_BARO_ID BARO_BOARD_SENDER_ID
#else
#define INS_EKF2_BARO_ID ABI_BROADCAST
#endif
#endif
PRINT_CONFIG_VAR(INS_EKF2_BARO_ID)

/** default temperature sensor to use in INS */
#ifndef INS_EKF2_TEMPERATURE_ID
#define INS_EKF2_TEMPERATURE_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_TEMPERATURE_ID)

/** default AGL sensor to use in INS */
#ifndef INS_EKF2_AGL_ID
#define INS_EKF2_AGL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_AGL_ID)

/* default Gyro to use in INS */
#ifndef INS_EKF2_GYRO_ID
#define INS_EKF2_GYRO_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_GYRO_ID)

/* default Accelerometer to use in INS */
#ifndef INS_EKF2_ACCEL_ID
#define INS_EKF2_ACCEL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_ACCEL_ID)

/* default Magnetometer to use in INS */
#ifndef INS_EKF2_MAG_ID
#define INS_EKF2_MAG_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_MAG_ID)

/* default GPS to use in INS */
#ifndef INS_EKF2_GPS_ID
#define INS_EKF2_GPS_ID GPS_MULTI_ID
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_ID)

/* default RELPOS to use for heading in INS */
#ifndef INS_EKF2_RELPOS_ID
#define INS_EKF2_RELPOS_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_RELPOS_ID)

/* default Optical Flow to use in INS */
#ifndef INS_EKF2_OF_ID
#define INS_EKF2_OF_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_OF_ID)

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

/* Flow sensor X offset from CoG position in meters */
#ifndef INS_EKF2_FLOW_POS_X
#define INS_EKF2_FLOW_POS_X 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_FLOW_POS_X)

/* Flow sensor Y offset from CoG position in meters */
#ifndef INS_EKF2_FLOW_POS_Y
#define INS_EKF2_FLOW_POS_Y 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_FLOW_POS_Y)

/* Flow sensor Z offset from CoG position in meters */
#ifndef INS_EKF2_FLOW_POS_Z
#define INS_EKF2_FLOW_POS_Z 0
#endif
PRINT_CONFIG_VAR(INS_EKF2_FLOW_POS_Z)

/* Barometric measurement noise for altitude (m) */
#ifndef INS_EKF2_BARO_NOISE
#define INS_EKF2_BARO_NOISE 3.5f
#endif
PRINT_CONFIG_VAR(INS_EKF2_BARO_NOISE)

/* All registered ABI events */
static abi_event baro_ev;
static abi_event temperature_ev;
static abi_event agl_ev;
static abi_event gyro_int_ev;
static abi_event accel_int_ev;
static abi_event mag_ev;
static abi_event gps_ev;
static abi_event relpos_ev;
static abi_event optical_flow_ev;
static abi_event reset_ev;

/* All ABI callbacks */

#if defined(CONFIG_EKF2_BAROMETER)
static void baro_cb(uint8_t sender_id, uint32_t stamp, float pressure);
#endif /// CONFIG_EKF2_BAROMETER
static void temperature_cb(uint8_t sender_id, float temp);
static void agl_cb(uint8_t sender_id, uint32_t stamp, float distance);
static void gyro_int_cb(uint8_t sender_id, uint32_t stamp, struct FloatRates *delta_gyro, uint16_t dt);
static void accel_int_cb(uint8_t sender_id, uint32_t stamp, struct FloatVect3 *delta_accel, uint16_t dt);
static void mag_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *mag);
static void gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s);
static void relpos_cb(uint8_t sender_id, uint32_t stamp, struct RelPosNED *relpos);
static void optical_flow_cb(uint8_t sender_id, uint32_t stamp, int32_t flow_x, int32_t flow_y, int32_t flow_der_x, int32_t flow_der_y, float quality, float size_divergence);
static void reset_cb(uint8_t sender_id, uint8_t flag);


static Ekf ekf;

static parameters *ekf_params;                    ///< The EKF parameters

struct ekf2_t ekf2;                               ///< Local EKF2 status structure
//static struct extVisionSample sample_ev;          ///< External vision sample

void ins_ekf2_init(void)
{

  ekf_params = ekf.getParamHandle();

  ekf_params->accel_bias_p_noise = 3.0e-3f;

#if defined(CONFIG_EKF2_BAROMETER)
  ekf_params->baro_noise = INS_EKF2_BARO_NOISE;
#endif // CONFIG_EKF2_BAROMETER

  ekf_params->imu_pos_body = {
    INS_EKF2_IMU_POS_X,
    INS_EKF2_IMU_POS_Y,
    INS_EKF2_IMU_POS_Z
  };

#if defined(CONFIG_EKF2_GNSS)
  /* Set the GPS position relative from the CoG in xyz (m) */
  ekf_params->gps_pos_body = {
    INS_EKF2_GPS_POS_X,
    INS_EKF2_GPS_POS_Y,
    INS_EKF2_GPS_POS_Z
  };
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
  /* Set flow sensor offset from CoG position in xyz (m) */
  ekf_params->flow_pos_body = {
    INS_EKF2_FLOW_POS_X,
    INS_EKF2_FLOW_POS_Y,
    INS_EKF2_FLOW_POS_Z
  };
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_RANGE_FINDER)
  ekf_params->max_hagl_for_range_aid = INS_EKF2_SONAR_MAX_RANGE;
#endif //  CONFIG_EKF2_RANGE_FINDER

  /* Initialize struct */
  ekf2.ltp_stamp = 0;
  ekf2.flow_stamp = 0;
  ekf2.gyro_valid = false;
  ekf2.accel_valid = false;
  ekf2.got_imu_data = false;
  ekf2.quat_reset_counter = 0;
  ekf2.temp = 20.0f; // Default temperature of 20 degrees celcius
  ekf2.qnh = 1013.25f; // Default atmosphere 

  // Don't send external vision data by default
//  sample_ev.time_us = 0;

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

  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgBARO_ABS(INS_EKF2_BARO_ID, &baro_ev, baro_cb);
  AbiBindMsgTEMPERATURE(INS_EKF2_TEMPERATURE_ID, &temperature_ev, temperature_cb);
  AbiBindMsgAGL(INS_EKF2_AGL_ID, &agl_ev, agl_cb);
  AbiBindMsgIMU_GYRO_INT(INS_EKF2_GYRO_ID, &gyro_int_ev, gyro_int_cb);
  AbiBindMsgIMU_ACCEL_INT(INS_EKF2_ACCEL_ID, &accel_int_ev, accel_int_cb);
  AbiBindMsgIMU_MAG(INS_EKF2_MAG_ID, &mag_ev, mag_cb);
  AbiBindMsgGPS(INS_EKF2_GPS_ID, &gps_ev, gps_cb);
  AbiBindMsgRELPOS(INS_EKF2_RELPOS_ID, &relpos_ev, relpos_cb);
  AbiBindMsgOPTICAL_FLOW(INS_EKF2_OF_ID, &optical_flow_ev, optical_flow_cb);
  AbiBindMsgINS_RESET(ABI_BROADCAST, &reset_ev, reset_cb);
}

static void reset_ref(void)
{
#if defined(CONFIG_EKF2_GNSS)
#if USE_GPS
  if (GpsFixValid()) {
    struct LlaCoor_i lla_pos = lla_int_from_gps(&gps);
    if (ekf.setEkfGlobalOrigin(lla_pos.lat*1e-7, lla_pos.lon*1e-7, gps.hmsl*1e-3)) {
      ltp_def_from_lla_i(&ekf2.ltp_def, &lla_pos);
      ekf2.ltp_def.hmsl = gps.hmsl;
      stateSetLocalOrigin_i(MODULE_INS_EKF2_ID, &ekf2.ltp_def);
    }
  }
#endif // USE_GPS
#endif // CONFIG_EKF2_GNSS
}

static void reset_vertical_ref(void)
{
#if defined(CONFIG_EKF2_GNSS)
#if USE_GPS
  if (GpsFixValid()) {
    struct LlaCoor_i lla_pos = lla_int_from_gps(&gps);
    struct LlaCoor_i lla = {
      .lat = stateGetLlaOrigin_i().lat,
      .lon = stateGetLlaOrigin_i().lon,
      .alt = lla_pos.alt
    };
    if (ekf.setEkfGlobalOrigin(lla.lat*1e-7, lla.lon*1e-7, gps.hmsl*1e-3)) {
      ltp_def_from_lla_i(&ekf2.ltp_def, &lla);
      ekf2.ltp_def.hmsl = gps.hmsl;
      stateSetLocalOrigin_i(MODULE_INS_EKF2_ID, &ekf2.ltp_def);
    }
  }
#endif // USE_GPS
#endif // CONFIG_EKF2_GNSS
}

static void reset_cb(uint8_t sender_id UNUSED, uint8_t flag)
{
  switch (flag) {
    case INS_RESET_REF:
      reset_ref();
      break;
    case INS_RESET_VERTICAL_REF:
      reset_vertical_ref();
      break;
    default:
      // unsupported cases
      break;
  }
}

/** Publish the attitude and get the new state
 *  Directly called after a succeslfull gyro+accel reading
 */
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
    const Quatf att_q = ekf.getQuaternion();

    struct FloatQuat ltp_to_body_quat;
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
//  ekf.setBaroData(sample); TODO
}
#endif // CONFIG_EKF2_BAROMETER

/* Save the latest temperature measurement for air density calculations */
static void temperature_cb(uint8_t __attribute__((unused)) sender_id, float temp)
{
  ekf2.temp = temp;
}


#if defined(CONFIG_EKF2_RANGE_FINDER)
/* Update INS based on AGL information */
static void agl_cb(uint8_t __attribute__((unused)) sender_id, uint32_t stamp, float distance)
{
  rangeSample sample;
  sample.time_us = stamp;
  sample.rng = distance;
  sample.quality = -1;

//  ekf.setRangeData(sample); TODO
}
#endif // CONFIG_EKF2_RANGE_FINDER)
     
/* Update INS based on Gyro information */
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

/* Update INS based on Accelerometer information */
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

//  ekf.setMagData(sample); TODO
  ekf2.got_imu_data = true;
}
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS)
/* Update INS based on GPS information */
static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp,
                   struct GpsState *gps_s)
{
  gnssSample gps_msg = {};
  gps_msg.time_us = stamp;
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
 // gps_msg.vel_m_s = gps_s->gspeed / 100.0;
  struct NedCoor_f ned_vel = ned_vel_float_from_gps(gps_s);
//  gps_msg.vel_ned(0) = ned_vel.x;
//  gps_msg.vel_ned(1) = ned_vel.y;
//  gps_msg.vel_ned(2) = ned_vel.z;
//  gps_msg.vel_ned_valid = bit_is_set(gps_s->valid_fields, GPS_VALID_VEL_NED_BIT);
  gps_msg.nsats = gps_s->num_sv;
  gps_msg.pdop = gps_s->pdop;

  //ekf.setGpsData(gps_msg); TODO
}
#endif // CONFIG_EKF2_GNSS

/* Update the local relative position information */
static void relpos_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp __attribute__((unused)), struct RelPosNED *relpos)
{
  // Verify if we received a valid heading
  if(
#ifdef INS_EKF2_RELHEADING_REF_ID
    relpos->reference_id != INS_EKF2_RELHEADING_REF_ID ||
#endif
#ifdef INS_EKF2_RELHEADING_DISTANCE
    fabs(relpos->distance - INS_EKF2_RELHEADING_DISTANCE) > INS_EKF2_RELHEADING_ERR ||
#endif
   !__builtin_isfinite(relpos->heading)
  ) {
    return;
  }

  ekf2.rel_heading = relpos->heading;
  ekf2.rel_heading_valid = true;
}


#if defined(CONFIG_EKF2_OPTICAL_FLOW)
/* Update INS based on Optical Flow information */
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
//  flowdata(0) = RadOfDeg(flow_y) * (1e-6 *
//                                    sample.dt);                       // INTEGRATED FLOW AROUND Y AXIS (RIGHT -X, LEFT +X)
 // flowdata(1) = - RadOfDeg(flow_x) * (1e-6 *
//                                      sample.dt);                     // INTEGRATED FLOW AROUND X AXIS (FORWARD +Y, BACKWARD -Y)

  sample.quality = quality;                     // quality indicator between 0 and 255
//  sample.flow_xy_rad =
 //   flowdata;                   // measured delta angle of the image about the X and Y body axes (rad), RH rotaton is positive
 // sample.gyro_xyz = Vector3f{NAN, NAN, NAN};    // measured delta angle of the inertial frame about the body axes obtained from rate gyro measurements (rad), RH rotation is positive

  // Update the optical flow data based on the callback
//  ekf.setOpticalFlowData(sample); TODO
}
#endif // CONFIG_EKF2_OPTICAL_FLOW

void ins_ekf2_update(void){};
void ins_ekf2_change_param(int32_t unk){};
void ins_ekf2_remove_gps(int32_t mode){};
void ins_ekf2_parse_EXTERNAL_POSE(uint8_t *buf){};
void ins_ekf2_parse_EXTERNAL_POSE_SMALL(uint8_t *buf){};
