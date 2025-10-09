/*
 * Copyright (C) Gautier Hattenberger, Mohamad Hachem
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "firmwares/rotorcraft/guidance/guidance_indi.h"

#include "math/pprz_algebra_float.h"

//
//  // Kpxy
//  Ap =     0.9985;//0.9986;//0.997;//0.9992;
//  Bp =     0.04362;//-0.0009647;//-0.003251;//-0.002109;
//  Cp =     0.009739;//-0.4403;//-0.5215;//-0.01485;
//  Dp =     0.5602;//0.5603;//0.2667;// 0.6843;
//  // Kdxy
//  Ad =      1;//1;//1;//1;
//  Bd =     -0.01418;//-0.003343;//-0.01631;//0.0009014;
//  Cd =     -0.2814;//-1.194;//-0.4905;//2.883;
//  Dd =      2.397;//2.397;//3.605;//3.609;
//  // Kpz
//  Apz =  0.9987;//0.9816;//0.9953;
//  Bpz = -0.0003935;//0.007528;//-0.00106 ;
//  Cpz = -1.052;//-5.243;//-3.451;
//  Dpz =  1.084;//3.58;//-0.0406;
//  // Kdz
//  Adz =  1;//0.9937;// 0.9999;
//  Bdz =  0.003449;//-0.0008827;//-0.004018;
//  Cdz =  1.548;//-3.132;//-7.692;
//  Ddz =  3.357;//1.82;//6.773;

// proportional control part (horizontal position)
static float Ap = GUIDANCE_INDI_HINF_Ap
static float Bp = GUIDANCE_INDI_HINF_Bp
static float Cp = GUIDANCE_INDI_HINF_Cp
static float Dp = GUIDANCE_INDI_HINF_Dp
// derivative control part (horizontal speed)
static float Ad = GUIDANCE_INDI_HINF_Ad
static float Bd = GUIDANCE_INDI_HINF_Bd
static float Cd = GUIDANCE_INDI_HINF_Cd
static float Dd = GUIDANCE_INDI_HINF_Dd
// proportional control part (vertical position)
static float Apz = GUIDANCE_INDI_HINF_Apz
static float Bpz = GUIDANCE_INDI_HINF_Bpz
static float Cpz = GUIDANCE_INDI_HINF_Cpz
static float Dpz = GUIDANCE_INDI_HINF_Dpz
// derivative control part (vertical speed)
static float Adz = GUIDANCE_INDI_HINF_Adz
static float Bdz = GUIDANCE_INDI_HINF_Bdz
static float Cdz = GUIDANCE_INDI_HINF_Cdz
static float Ddz = GUIDANCE_INDI_HINF_Ddz

/** Acceleration controller based Hinfinity
 */
struct FloatVect3 guidance_indi_controller(bool in_flight, struct HorizontalGuidance *gh, struct VerticalGuidance *gv, enum GuidanceIndi_HMode h_mode, enum GuidanceIndi_VMode v_mode)
{
  struct FloatVect3 pos_err = { 0 };
  struct FloatVect3 speed_err = {0};

  struct FloatVect3 accel_sp = { 0 };
  struct FloatVect3 speed_fb;
  
  // Matrices for Kp
  float Ap,Bp,Cp,Dp,xk1px,xk1py;  // xkpx and xkpy should be defined as global variable (should be intialized as zeros)
  float Ad,Bd,Cd,Dd,xk1dx,xk1dy;  // xkdx and xkdy should be defined as global variables ( should be intialized as zeros) 
  float Apz,Bpz,Cpz,Dpz,xk1pz;
  float Adz,Bdz,Cdz,Ddz,xk1dz;

  static float xkpx = 0.0f;
  static float xkpy = 0.0f;
  static float xkpz = 0.0f;
  static float xkdx = 0.0f;
  static float xkdy = 0.0f;
  static float xkdz = 0.0f;
    if (in_flight){
      // pos_err
      pos_err.x = POS_FLOAT_OF_BFP(gh->ref.pos.x) - stateGetPositionNed_f()->x;
      pos_err.y = POS_FLOAT_OF_BFP(gh->ref.pos.y) - stateGetPositionNed_f()->y;
      pos_err.z = POS_FLOAT_OF_BFP(gv->z_ref) - stateGetPositionNed_f()->z;
      // vx_sp
      //printf("pos_err_x = %f and pos_err_y = %f \n",pos_err.x,pos_err.y);
      xk1px = Apz * xkpx + Bpz * pos_err.x;
      speed_sp.x = Cpz*xkpx + Dpz * pos_err.x;
      xkpx = xk1px;
      // vy_sp
      xk1py = Apz * xkpy + Bpz * pos_err.y;
      speed_sp.y = Cpz * xkpy + Dpz * pos_err.y;
      xkpy = xk1py;
      // Compute Acc SP
      speed_err.x = speed_sp.x - stateGetSpeedNed_f()->x;
      speed_err.y = speed_sp.y - stateGetSpeedNed_f()->y;
      // acc_spx
      xk1dx = Ad * xkdx + Bd * speed_err.x;
      speed_fb.x = Cd * xkdx + Dd * speed_err.x;
      xkdx = xk1dx;
      // acc_spy
      xk1dy = Ad * xkdy + Bd * speed_err.y;
      speed_fb.y = Cd * xkdy + Dd * speed_err.y;
      xkdy = xk1dy;
      //printf("xkdy = %f \n",xkdy);
      // z
      // if (autopilot_get_motors_on()) {
      // vz_sp
      xk1pz = Apz * xkpz + Bpz * pos_err.z;
      speed_sp.z = Cpz * xkpz + Dpz * pos_err.z;
      xkpz = xk1pz;
      // acc_zsp
      speed_err.z = speed_sp.z - stateGetSpeedNed_f()->z;
      xk1dz = Adz * xkdz + Bdz * speed_err.z;
      speed_fb.z = Cdz * xkdz + Ddz * speed_err.z;
      xkdz = xk1dz;
      //printf("xkdz = %f \n",xkdz);

    }
  accel_sp.x = speed_fb.x;    //; + ACCEL_FLOAT_OF_BFP(gh->ref.accel.x);
  accel_sp.y = speed_fb.y;  //; + ACCEL_FLOAT_OF_BFP(gh->ref.accel.y);
  accel_sp.z = speed_fb.z ; //; + ACCEL_FLOAT_OF_BFP(gv->zdd_ref);


}


/** Angular acceleration controller based on Hinfinity
 *
 * Takes the current rates filtered state and setpoint and compute the desired acceleration.
 */
struct FloatRates stabilization_indi_rate_controller(struct FloatRates rates, struct FloatRates sp)
{
  struct FloatRates rate_error;
  RATES_DIFF(rate_error, sp, rates);

  struct FloatRates accel_ref;
  if (autopilot_in_flight()) {
    accel_ref.p = Cd * rate_state.p + Dd * rate_error.p;
    rate_state.p = Ad * rate_state.p + Bd * rate_error.p;

    accel_ref.q = Cd * rate_state.q + Dd * rate_error.q;
    rate_state.q = Ad * rate_state.q + Bd * rate_error.q;

    accel_ref.r = rate_error.r * indi_gains.rate.r;
  } else {
    FLOAT_RATES_ZERO(rate_state);
    FLOAT_RATES_ZERO(accel_ref);
  }

  return accel_ref
}

/** Angular rate controller based on Hinfinity
 *
 * Takes the current attitude filtered state and setpoint and compute the desired rates.
 * Can be redefined elsewhere to use an other control scheme.
 */
struct FloatRates WEAK stabilization_indi_attitude_controller(struct FloatQuat att, struct FloatQuat att_sp, struct FloatRates rates_ff)
{
  /* attitude error */
  struct FloatQuat att_err;
  float_quat_inv_comp_norm_shortest(&att_err, &att, &sp);

  struct FloatVect3 att_fb;
#if TILT_TWIST_CTRL
  struct FloatQuat tilt;
  struct FloatQuat twist;
  float_quat_tilt_twist(&tilt, &twist, &att_err);
  att_fb.x = tilt.qx;
  att_fb.y = tilt.qy;
  att_fb.z = twist.qz;
#else
  att_fb.x = att_err.qx;
  att_fb.y = att_err.qy;
  att_fb.z = att_err.qz;
#endif

  struct FloatRates rate_sp;
  if (autopilot_in_flight()) {
    rate_sp.p  = Cp * att_state.p + Dp * att_fb.x;
    att_state.p = Ap * att_state.p + Bp * att_fb.x;

    rate_sp.q = Cp * att_state.q + Dp * att_fb.y;
    att_state.q = Ap * att_state.q + Bp * att_fb.y;

    rate_sp.r = indi_gains.att.r * att_fb.z / indi_gains.rate.r;

    // add feed-forward term
    RATES_ADD(rate_sp, rates_ff);
  } else {
    FLOAT_RATES_ZERO(att_state);
    FLOAT_RATES_ZERO(rate_sp);
  }

  return rate_sp;
}

