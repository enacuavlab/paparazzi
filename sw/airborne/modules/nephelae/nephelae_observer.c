/*
 * Copyright (C) Cabarbaye/Verdu
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/nephelae/nephelae_observer.c"
 * @author Cabarbaye/Verdu
 * Observer of state vector based on airplane caracteristics
 */

#include "modules/nephelae/nephelae_observer.h"

#include <stdio.h>
#include <math.h>
#include "modules/core/abi.h"
#include "std.h"
#include "generated/airframe.h"
#include "modules/datalink/telemetry.h"

/** ABI binding for IMU acceleration
*/
#ifndef IMU_ACC_NEPH_ID
#define IMU_ACC_NEPH_ID ABI_BROADCAST
#endif
static abi_event acc_ev;

/** ABI binding for IMU gyrometer
 */
#ifndef IMU_GYRO_NEPH_ID
#define IMU_GYRO_NEPH_ID ABI_BROADCAST
#endif
static abi_event gyro_ev;

struct ObsVect neph_obs_state;

static struct ObsVect neph_obs_state_mem;

static struct FloatRates neph_gyro_f;
uint32_t neph_stamp_gyro = 0;

static struct FloatVect3 neph_accel_f;
uint32_t neph_stamp_accel = 0;

float delta_f_motor = 0.f;
float delta_q_motor = 0.f;

float omega_obs;
float omega_zero;

float  dot_X_x;
float  dot_X_y;
float  dot_X_z;
float  dot_X_p;
float  dot_X_q;
float  dot_X_r;
float  dot_X_phi;
float  dot_X_theta;

uint32_t last_comput_stamp = 0;

bool processing = true;

float neph_CO1[8] = NEPH_CO1;
float neph_CO2[8] = NEPH_CO2;
float neph_CO3[8] = NEPH_CO3;
float neph_CO4[8] = NEPH_CO4;
float neph_CO5[8] = NEPH_CO5;
float neph_CO6[8] = NEPH_CO6;
float neph_DO1[2] = NEPH_DO1;
float neph_DO2[2] = NEPH_DO2;
float neph_DO3[2] = NEPH_DO3;
float neph_DO4[2] = NEPH_DO4;
float neph_DO5[2] = NEPH_DO5;
float neph_DO6[2] = NEPH_DO6;
float neph_A1[8] = NEPH_A1;
float neph_A2[8] = NEPH_A2;
float neph_A3[8] = NEPH_A3;
float neph_A4[8] = NEPH_A4;
float neph_A5[8] = NEPH_A5;
float neph_A6[8] = NEPH_A6;
float neph_A7[8] = NEPH_A7;
float neph_A8[8] = NEPH_A8;
float neph_B1[2] = NEPH_B1;
float neph_B2[2] = NEPH_B2;
float neph_B3[2] = NEPH_B3;
float neph_B4[2] = NEPH_B4;
float neph_B5[2] = NEPH_B5;
float neph_B6[2] = NEPH_B6;
float neph_B7[2] = NEPH_B7;
float neph_B8[2] = NEPH_B8;
float neph_L1[6] = NEPH_L1;
float neph_L2[6] = NEPH_L2;
float neph_L3[6] = NEPH_L3;
float neph_L4[6] = NEPH_L4;
float neph_L5[6] = NEPH_L5;
float neph_L6[6] = NEPH_L6;
float neph_L7[6] = NEPH_L7;
float neph_L8[6] = NEPH_L8;

static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                    uint32_t stamp __attribute__((unused)),
                    struct Int32Rates *gyro){

  RATES_FLOAT_OF_BFP(neph_gyro_f, *gyro); // (p, q ,r)
  neph_stamp_gyro = stamp;
  neph_obs_state.gyro_time = neph_stamp_gyro;
}

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel){

	ACCELS_FLOAT_OF_BFP(neph_accel_f, *accel); // (x ,y ,z)

  neph_stamp_accel = stamp;
  neph_obs_state.acc_time = neph_stamp_accel;

  if (processing == true){
    integration();
  }
}

void omega_process(void){
  float A = NEPH_R * NEPH_KV * NEPH_KQW;
  float B = (1 / NEPH_KV) + ( NEPH_R * NEPH_KV * NEPH_KQX * (neph_obs_state.airspeed.x + NEPH_V0));
  float C = (NEPH_R * NEPH_KV * NEPH_KQU) * powf((neph_obs_state.airspeed.x + NEPH_V0), 2) + (NEPH_R * NEPH_I0) - neph_ctrl_state.v_motor;
  float delta_obs = powf(B,2) - (4*A*C);
  omega_obs = 0.f;

  if (delta_obs > 0.0){
    omega_obs = (-B + sqrt(delta_obs))/(2*A);
  }
  else{
    omega_obs = -B /(2*A);
  }
}

void omega_zero_process(void){
  float A = NEPH_KFW;
  float B = NEPH_KFX * NEPH_V0;
  float C = -trim_thrust * NEPH_M;
  float delta_obs = powf(B,2) - (4*A*C);
  omega_zero = 0.f;

  if (delta_obs > 0.0){
    omega_zero = (-B + sqrt(delta_obs))/(2*A);
  }
  else{
    omega_zero = -B /(2*A);
  }
}

void motor_process(void){
  delta_f_motor = (((NEPH_KFW * powf(omega_obs,2)) + (NEPH_KFX * omega_obs * (neph_obs_state.airspeed.x + NEPH_V0))) / NEPH_M) - (trim_thrust);
  delta_q_motor = (((NEPH_KQW * powf(omega_obs,2)) + (NEPH_KQX * omega_obs * (neph_obs_state.airspeed.x + NEPH_V0)) +
                  (NEPH_KQU * powf((neph_obs_state.airspeed.x + NEPH_V0),2))) - 
                  ((NEPH_KQW * powf(omega_zero,2)) + (NEPH_KQX * omega_zero * NEPH_V0) +
                  (NEPH_KQU * powf((NEPH_V0),2)))) / NEPH_IXX;
}

void observer_derivative(void){

  float  Y_ddot_x = neph_CO1[0] * neph_obs_state.airspeed.x;
  Y_ddot_x += neph_CO1[1] * neph_obs_state.airspeed.y;
  Y_ddot_x += neph_CO1[2] * neph_obs_state.airspeed.z;
  Y_ddot_x += neph_CO1[3] * neph_obs_state.rotspeed.p;
  Y_ddot_x += neph_CO1[4] * neph_obs_state.rotspeed.q;
  Y_ddot_x += neph_CO1[5] * neph_obs_state.rotspeed.r;
  Y_ddot_x += neph_CO1[6] * neph_obs_state.attitude.phi;
  Y_ddot_x += neph_CO1[7] * neph_obs_state.attitude.theta;

  Y_ddot_x += neph_DO1[0] * neph_ctrl_state.aileron;
  Y_ddot_x += neph_DO1[1] * neph_ctrl_state.flap;
  Y_ddot_x += delta_f_motor;

  float  Y_ddot_y = neph_CO2[0] * neph_obs_state.airspeed.x;
  Y_ddot_y += neph_CO2[1] * neph_obs_state.airspeed.y;
  Y_ddot_y += neph_CO2[2] * neph_obs_state.airspeed.z;
  Y_ddot_y += neph_CO2[3] * neph_obs_state.rotspeed.p;
  Y_ddot_y += neph_CO2[4] * neph_obs_state.rotspeed.q;
  Y_ddot_y += neph_CO2[5] * neph_obs_state.rotspeed.r;
  Y_ddot_y += neph_CO2[6] * neph_obs_state.attitude.phi;
  Y_ddot_y += neph_CO2[7] * neph_obs_state.attitude.theta;

  Y_ddot_y += neph_DO2[0] * neph_ctrl_state.aileron;
  Y_ddot_y += neph_DO2[1] * neph_ctrl_state.flap;

  float  Y_ddot_z = neph_CO3[0] * neph_obs_state.airspeed.x;
  Y_ddot_z += neph_CO3[1] * neph_obs_state.airspeed.y;
  Y_ddot_z += neph_CO3[2] * neph_obs_state.airspeed.z;
  Y_ddot_z += neph_CO3[3] * neph_obs_state.rotspeed.p;
  Y_ddot_z += neph_CO3[4] * neph_obs_state.rotspeed.q;
  Y_ddot_z += neph_CO3[5] * neph_obs_state.rotspeed.r;
  Y_ddot_z += neph_CO3[6] * neph_obs_state.attitude.phi;
  Y_ddot_z += neph_CO3[7] * neph_obs_state.attitude.theta;

  Y_ddot_z += neph_DO3[0] * neph_ctrl_state.aileron;
  Y_ddot_z += neph_DO3[1] * neph_ctrl_state.flap;

  float  Y_p = neph_CO4[0] * neph_obs_state.airspeed.x;
  Y_p += neph_CO4[1] * neph_obs_state.airspeed.y;
  Y_p += neph_CO4[2] * neph_obs_state.airspeed.z;
  Y_p += neph_CO4[3] * neph_obs_state.rotspeed.p;
  Y_p += neph_CO4[4] * neph_obs_state.rotspeed.q;
  Y_p += neph_CO4[5] * neph_obs_state.rotspeed.r;
  Y_p += neph_CO4[6] * neph_obs_state.attitude.phi;
  Y_p += neph_CO4[7] * neph_obs_state.attitude.theta;

  Y_p += neph_DO4[0] * neph_ctrl_state.aileron;
  Y_p += neph_DO4[1] * neph_ctrl_state.flap;

  float  Y_q = neph_CO5[0] * neph_obs_state.airspeed.x;
  Y_q += neph_CO5[1] * neph_obs_state.airspeed.y;
  Y_q += neph_CO5[2] * neph_obs_state.airspeed.z;
  Y_q += neph_CO5[3] * neph_obs_state.rotspeed.p;
  Y_q += neph_CO5[4] * neph_obs_state.rotspeed.q;
  Y_q += neph_CO5[5] * neph_obs_state.rotspeed.r;
  Y_q += neph_CO5[6] * neph_obs_state.attitude.phi;
  Y_q += neph_CO5[7] * neph_obs_state.attitude.theta;

  Y_q += neph_DO5[0] * neph_ctrl_state.aileron;
  Y_q += neph_DO5[1] * neph_ctrl_state.flap;

  float  Y_r = neph_CO6[0] * neph_obs_state.airspeed.x;
  Y_r += neph_CO6[1] * neph_obs_state.airspeed.y;
  Y_r += neph_CO6[2] * neph_obs_state.airspeed.z;
  Y_r += neph_CO6[3] * neph_obs_state.rotspeed.p;
  Y_r += neph_CO6[4] * neph_obs_state.rotspeed.q;
  Y_r += neph_CO6[5] * neph_obs_state.rotspeed.r;
  Y_r += neph_CO6[6] * neph_obs_state.attitude.phi;
  Y_r += neph_CO6[7] * neph_obs_state.attitude.theta;

  Y_r += neph_DO6[0] * neph_ctrl_state.aileron;
  Y_r += neph_DO6[1] * neph_ctrl_state.flap;

  ///////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////

  dot_X_x = neph_A1[0] * neph_obs_state.airspeed.x;
  dot_X_x += neph_A1[1] * neph_obs_state.airspeed.y;
  dot_X_x += neph_A1[2] * neph_obs_state.airspeed.z;
  dot_X_x += neph_A1[3] * neph_obs_state.rotspeed.p;
  dot_X_x += neph_A1[4] * neph_obs_state.rotspeed.q;
  dot_X_x += neph_A1[5] * neph_obs_state.rotspeed.r;
  dot_X_x += neph_A1[6] * neph_obs_state.attitude.phi;
  dot_X_x += neph_A1[7] * neph_obs_state.attitude.theta;

  dot_X_x += neph_B1[0] * neph_ctrl_state.aileron;
  dot_X_x += neph_B1[1] * neph_ctrl_state.flap;
  dot_X_x += delta_f_motor;

  dot_X_x += neph_L1[0] * (neph_accel_f.x - Y_ddot_x);
  dot_X_x += neph_L1[1] * (neph_accel_f.y - Y_ddot_y);
  dot_X_x += neph_L1[2] * (neph_accel_f.z - Y_ddot_z);
  dot_X_x += neph_L1[3] * (neph_gyro_f.p - Y_p);
  dot_X_x += neph_L1[4] * (neph_gyro_f.q - Y_q);
  dot_X_x += neph_L1[5] * (neph_gyro_f.r - Y_r);

  dot_X_y = neph_A2[0] * neph_obs_state.airspeed.x;
  dot_X_y += neph_A2[1] * neph_obs_state.airspeed.y;
  dot_X_y += neph_A2[2] * neph_obs_state.airspeed.z;
  dot_X_y += neph_A2[3] * neph_obs_state.rotspeed.p;
  dot_X_y += neph_A2[4] * neph_obs_state.rotspeed.q;
  dot_X_y += neph_A2[5] * neph_obs_state.rotspeed.r;
  dot_X_y += neph_A2[6] * neph_obs_state.attitude.phi;
  dot_X_y += neph_A2[7] * neph_obs_state.attitude.theta;

  dot_X_y += neph_B2[0] * neph_ctrl_state.aileron;
  dot_X_y += neph_B2[1] * neph_ctrl_state.flap;

  dot_X_y += neph_L2[0] * (neph_accel_f.x - Y_ddot_x);
  dot_X_y += neph_L2[1] * (neph_accel_f.y - Y_ddot_y);
  dot_X_y += neph_L2[2] * (neph_accel_f.z - Y_ddot_z);
  dot_X_y += neph_L2[3] * (neph_gyro_f.p - Y_p);
  dot_X_y += neph_L2[4] * (neph_gyro_f.q - Y_q);
  dot_X_y += neph_L2[5] * (neph_gyro_f.r - Y_r);

  dot_X_z = neph_A3[0] * neph_obs_state.airspeed.x;
  dot_X_z += neph_A3[1] * neph_obs_state.airspeed.y;
  dot_X_z += neph_A3[2] * neph_obs_state.airspeed.z;
  dot_X_z += neph_A3[3] * neph_obs_state.rotspeed.p;
  dot_X_z += neph_A3[4] * neph_obs_state.rotspeed.q;
  dot_X_z += neph_A3[5] * neph_obs_state.rotspeed.r;
  dot_X_z += neph_A3[6] * neph_obs_state.attitude.phi;
  dot_X_z += neph_A3[7] * neph_obs_state.attitude.theta;

  dot_X_z += neph_B3[0] * neph_ctrl_state.aileron;
  dot_X_z += neph_B3[1] * neph_ctrl_state.flap;

  dot_X_z += neph_L3[0] * (neph_accel_f.x - Y_ddot_x);
  dot_X_z += neph_L3[1] * (neph_accel_f.y - Y_ddot_y);
  dot_X_z += neph_L3[2] * (neph_accel_f.z - Y_ddot_z);
  dot_X_z += neph_L3[3] * (neph_gyro_f.p - Y_p);
  dot_X_z += neph_L3[4] * (neph_gyro_f.q - Y_q);
  dot_X_z += neph_L3[5] * (neph_gyro_f.r - Y_r);

  dot_X_p = neph_A4[0] * neph_obs_state.airspeed.x;
  dot_X_p += neph_A4[1] * neph_obs_state.airspeed.y;
  dot_X_p += neph_A4[2] * neph_obs_state.airspeed.z;
  dot_X_p += neph_A4[3] * neph_obs_state.rotspeed.p;
  dot_X_p += neph_A4[4] * neph_obs_state.rotspeed.q;
  dot_X_p += neph_A4[5] * neph_obs_state.rotspeed.r;
  dot_X_p += neph_A4[6] * neph_obs_state.attitude.phi;
  dot_X_p += neph_A4[7] * neph_obs_state.attitude.theta;

  dot_X_p += neph_B4[0] * neph_ctrl_state.aileron;
  dot_X_p += neph_B4[1] * neph_ctrl_state.flap;
  dot_X_p += NEPH_ROTMOTOR * delta_q_motor;

  dot_X_p += neph_L4[0] * (neph_accel_f.x - Y_ddot_x);
  dot_X_p += neph_L4[1] * (neph_accel_f.y - Y_ddot_y);
  dot_X_p += neph_L4[2] * (neph_accel_f.z - Y_ddot_z);
  dot_X_p += neph_L4[3] * (neph_gyro_f.p - Y_p);
  dot_X_p += neph_L4[4] * (neph_gyro_f.q - Y_q);
  dot_X_p += neph_L4[5] * (neph_gyro_f.r - Y_r);

  dot_X_q = neph_A5[0] * neph_obs_state.airspeed.x;
  dot_X_q += neph_A5[1] * neph_obs_state.airspeed.y;
  dot_X_q += neph_A5[2] * neph_obs_state.airspeed.z;
  dot_X_q += neph_A5[3] * neph_obs_state.rotspeed.p;
  dot_X_q += neph_A5[4] * neph_obs_state.rotspeed.q;
  dot_X_q += neph_A5[5] * neph_obs_state.rotspeed.r;
  dot_X_q += neph_A5[6] * neph_obs_state.attitude.phi;
  dot_X_q += neph_A5[7] * neph_obs_state.attitude.theta;

  dot_X_q += neph_B5[0] * neph_ctrl_state.aileron;
  dot_X_q += neph_B5[1] * neph_ctrl_state.flap;

  dot_X_q += neph_L5[0] * (neph_accel_f.x - Y_ddot_x);
  dot_X_q += neph_L5[1] * (neph_accel_f.y - Y_ddot_y);
  dot_X_q += neph_L5[2] * (neph_accel_f.z - Y_ddot_z);
  dot_X_q += neph_L5[3] * (neph_gyro_f.p - Y_p);
  dot_X_q += neph_L5[4] * (neph_gyro_f.q - Y_q);
  dot_X_q += neph_L5[5] * (neph_gyro_f.r - Y_r);

  dot_X_r = neph_A6[0] * neph_obs_state.airspeed.x;
  dot_X_r += neph_A6[1] * neph_obs_state.airspeed.y;
  dot_X_r += neph_A6[2] * neph_obs_state.airspeed.z;
  dot_X_r += neph_A6[3] * neph_obs_state.rotspeed.p;
  dot_X_r += neph_A6[4] * neph_obs_state.rotspeed.q;
  dot_X_r += neph_A6[5] * neph_obs_state.rotspeed.r;
  dot_X_r += neph_A6[6] * neph_obs_state.attitude.phi;
  dot_X_r += neph_A6[7] * neph_obs_state.attitude.theta;

  dot_X_r += neph_B6[0] * neph_ctrl_state.aileron;
  dot_X_r += neph_B6[1] * neph_ctrl_state.flap;

  dot_X_r += neph_L6[0] * (neph_accel_f.x - Y_ddot_x);
  dot_X_r += neph_L6[1] * (neph_accel_f.y - Y_ddot_y);
  dot_X_r += neph_L6[2] * (neph_accel_f.z - Y_ddot_z);
  dot_X_r += neph_L6[3] * (neph_gyro_f.p - Y_p);
  dot_X_r += neph_L6[4] * (neph_gyro_f.q - Y_q);
  dot_X_r += neph_L6[5] * (neph_gyro_f.r - Y_r);

  dot_X_phi = neph_A7[0] * neph_obs_state.airspeed.x;
  dot_X_phi += neph_A7[1] * neph_obs_state.airspeed.y;
  dot_X_phi += neph_A7[2] * neph_obs_state.airspeed.z;
  dot_X_phi += neph_A7[3] * neph_obs_state.rotspeed.p;
  dot_X_phi += neph_A7[4] * neph_obs_state.rotspeed.q;
  dot_X_phi += neph_A7[5] * neph_obs_state.rotspeed.r;
  dot_X_phi += neph_A7[6] * neph_obs_state.attitude.phi;
  dot_X_phi += neph_A7[7] * neph_obs_state.attitude.theta;

  dot_X_phi += neph_B7[0] * neph_ctrl_state.aileron;
  dot_X_phi += neph_B7[1] * neph_ctrl_state.flap;

  dot_X_phi += neph_L7[0] * (neph_accel_f.x - Y_ddot_x);
  dot_X_phi += neph_L7[1] * (neph_accel_f.y - Y_ddot_y);
  dot_X_phi += neph_L7[2] * (neph_accel_f.z - Y_ddot_z);
  dot_X_phi += neph_L7[3] * (neph_gyro_f.p - Y_p);
  dot_X_phi += neph_L7[4] * (neph_gyro_f.q - Y_q);
  dot_X_phi += neph_L7[5] * (neph_gyro_f.r - Y_r);

  dot_X_theta = neph_A8[0] * neph_obs_state.airspeed.x;
  dot_X_theta += neph_A8[1] * neph_obs_state.airspeed.y;
  dot_X_theta += neph_A8[2] * neph_obs_state.airspeed.z;
  dot_X_theta += neph_A8[3] * neph_obs_state.rotspeed.p;
  dot_X_theta += neph_A8[4] * neph_obs_state.rotspeed.q;
  dot_X_theta += neph_A8[5] * neph_obs_state.rotspeed.r;
  dot_X_theta += neph_A8[6] * neph_obs_state.attitude.phi;
  dot_X_theta += neph_A8[7] * neph_obs_state.attitude.theta;

  dot_X_theta += neph_B8[0] * neph_ctrl_state.aileron;
  dot_X_theta += neph_B8[1] * neph_ctrl_state.flap;

  dot_X_theta += neph_L8[0] * (neph_accel_f.x - Y_ddot_x);
  dot_X_theta += neph_L8[1] * (neph_accel_f.y - Y_ddot_y);
  dot_X_theta += neph_L8[2] * (neph_accel_f.z - Y_ddot_z);
  dot_X_theta += neph_L8[3] * (neph_gyro_f.p - Y_p);
  dot_X_theta += neph_L8[4] * (neph_gyro_f.q - Y_q);
  dot_X_theta += neph_L8[5] * (neph_gyro_f.r - Y_r);
}

void integration(void){
  processing = false;
  omega_process();
  omega_zero_process();
  motor_process();
  observer_derivative();

  uint32_t delta_stamp = neph_stamp_accel - last_comput_stamp;

  neph_obs_state.airspeed.x += dot_X_x * (float)delta_stamp/1000000.f;
  if(neph_obs_state.airspeed.x + NEPH_V0 < 0){
      neph_obs_state.airspeed.x = -NEPH_V0;
  }
  neph_obs_state.airspeed.y += dot_X_y * (float)delta_stamp/1000000.f;
  neph_obs_state.airspeed.z += dot_X_z * (float)delta_stamp/1000000.f;
  neph_obs_state.rotspeed.p += dot_X_p * (float)delta_stamp/1000000.f;
  neph_obs_state.rotspeed.q += dot_X_q * (float)delta_stamp/1000000.f;
  neph_obs_state.rotspeed.r += dot_X_r * (float)delta_stamp/1000000.f;
  neph_obs_state.attitude.phi += dot_X_phi * (float)delta_stamp/1000000.f;
  neph_obs_state.attitude.theta += dot_X_theta * (float)delta_stamp/1000000.f;

  last_comput_stamp = neph_stamp_accel;
  
  processing = true;
}

/** IMU Acc and Gyro Observer for state feedback control. Called at startup.
 *  Bind ABI messages
 */
void nephelae_observer_init(void) {

  neph_obs_state.airspeed.x = 0.f;
  neph_obs_state.airspeed.y = 0.f;
  neph_obs_state.airspeed.z = 0.f;

  neph_obs_state.rotspeed.p = 0.f;
  neph_obs_state.rotspeed.q = 0.f;
  neph_obs_state.rotspeed.r = 0.f;

  neph_obs_state.attitude.phi = 0.f;
  neph_obs_state.attitude.theta = 0.f;
  neph_obs_state.attitude.psi = 0.f;

  //neph_obs_state.current_time = 0;
  neph_obs_state.gyro_time = 0;
  neph_obs_state.debug = 0.f;
  neph_obs_state.acc_time = 0;

	AbiBindMsgIMU_ACCEL_INT32(IMU_ACC_NEPH_ID, &acc_ev, accel_cb);
	AbiBindMsgIMU_GYRO_INT32(IMU_GYRO_NEPH_ID, &gyro_ev, gyro_cb);
}