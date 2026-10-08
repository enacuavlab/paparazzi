/*
 * Copyright (C) 2025 Gautier Hattenberger <gautier.hattenberger@enac.fr> Jean-Baptiste Forestier <jean-baptiste.forestier@enac.fr>
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

/** @file "modules/ctrl/eff_scheduling_quadplane.c"
 * Interpolation of control effectivenss matrix of a quadplane
 */

#include "modules/ctrl/eff_scheduling_quadplane.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "modules/radio_control/radio_control.h"
#include "state.h"
#include "autopilot.h"
#include "filters/median_filter.h"

#include "modules/actuators/actuators.h"


#include "modules/datalink/downlink.h"



#if INDI_NUM_ACT != 7
#error "eff_scheduling_quadplane requires INDI_NUM_ACT == 7"
#endif

#if INDI_OUTPUTS != 5
#error "eff_scheduling_quadplane requires INDI_OUTPUTS == 5"
#endif

// Airframe file parameter checks
#ifndef QUADPLANE_EFF_IXX
#error "QUADPLANE_EFF_IXX [float] not defined in airframe file"
#endif
#ifndef QUADPLANE_EFF_IYY
#error "QUADPLANE_EFF_IYY [float] not defined in airframe file"
#endif
#ifndef QUADPLANE_EFF_IZZ
#error "QUADPLANE_EFF_IZZ [float] not defined in airframe file"
#endif
#ifndef GUIDANCE_INDI_MASS
#error "GUIDANCE_INDI_MASS [float] not defined in airframe file"
#endif
#ifndef QUADPLANE_EFF_RX
#error "QUADPLANE_EFF_RX [4x1 float] not defined in airframe file"
#endif
#ifndef QUADPLANE_EFF_RY
#error "QUADPLANE_EFF_RY [4x1 float] not defined in airframe file"
#endif
#ifndef QUADPLANE_EFF_RZ
#error "QUADPLANE_EFF_RZ [4x1 float] not defined in airframe file"
#endif
/* T(u) = a + b*u + c*u² for the vertical motors. */
#ifndef QUADPLANE_EFF_ROTOR_K_T_PPRZ
#error "QUADPLANE_EFF_K_T_PPRZ [2x1 float] not defined in airframe file — {b [N/pprz], c [N/pprz^2]}"
#endif
#ifndef QUADPLANE_EFF_KAPPA
#error "QUADPLANE_EFF_KAPPA [float] not defined in airframe file"
#endif
#ifndef QUADPLANE_EFF_SPIN_DIR
#error "QUADPLANE_EFF_SPIN_DIR [4x1 float] (+/-1) not defined in airframe file"
#endif
#ifndef QUADPLANE_EFF_K_LIFT
#error "QUADPLANE_EFF_K_LIFT [float] not defined in airframe file"
#endif
#ifndef QUADPLANE_EFF_V_WING
#define QUADPLANE_EFF_V_WING 10.0f
#endif
/* Pusher */
#ifndef QUADPLANE_EFF_PUSHER_K_T_PPRZ
#error "QUADPLANE_EFF_PUSHER_K_T_PPRZ not defined"
#endif
#ifndef QUADPLANE_EFF_PUSHER_RY
#error "QUADPLANE_EFF_PUSHER_RY float not defined"
#endif
#ifndef QUADPLANE_EFF_PUSHER_RZ
#error "QUADPLANE_EFF_PUSHER_RZ float not defined"
#endif
#ifndef QUADPLANE_EFF_PUSHER_SPIN_DIR
#error "QUADPLANE_EFF_PUSHER_SPIN_DIR float (+/-1) not defined in airframe file"
#endif
#ifndef QUADPLANE_EFF_K_ELEVON_DEFLECT
#error "QUADPLANE_EFF_K_ELEVON_DEFLECT [float] not defined in airframe file — [rad/pprz]"
#endif
#ifndef QUADPLANE_EFF_K_ELEVON_ROLL
#error "QUADPLANE_EFF_K_ELEVON_ROLL [float] not defined in airframe file"
#endif
#ifndef QUADPLANE_EFF_K_ELEVON_PITCH
#error "QUADPLANE_EFF_K_ELEVON_PITCH [float] not defined in airframe file"
#endif
#ifndef QUADPLANE_EFF_K_ELEVON_PROPWASH
#define QUADPLANE_EFF_K_ELEVON_PROPWASH 0.0f
#endif
#ifndef QUADPLANE_EFF_K_ELEVON_V_FULL
#define QUADPLANE_EFF_K_ELEVON_V_FULL 0.0f
#endif
// Elevon command slew limit [pprz/s]
#ifndef QUADPLANE_EFF_ELEVON_RATE
#define QUADPLANE_EFF_ELEVON_RATE 9600.0f
#endif
/* Differential-drag yaw: dMz/d(delta_cmd) = k * V^2 * delta. */
#ifndef QUADPLANE_EFF_K_ELEVON_DIFF_DRAG_YAW
#define QUADPLANE_EFF_K_ELEVON_DIFF_DRAG_YAW 0.0f
#endif
// Motor Hover Value [pprz]
#ifndef QUADPLANE_EFF_MOTOR_HOVER
#error "QUADPLANE_EFF_MOTOR_HOVER [float] not defined in airframe file"
#endif
#ifndef QUADPLANE_EFF_MOTOR_IDLE
#error "QUADPLANE_EFF_MOTOR_IDLE [float] not defined in airframe file"
#endif





struct quadplane_eff_sched_param_t quadplane_eff_sched_p = {
  .Ixx = QUADPLANE_EFF_IXX,
  .Iyy = QUADPLANE_EFF_IYY,
  .Izz = QUADPLANE_EFF_IZZ,
  .mass = GUIDANCE_INDI_MASS,

  .r_x = QUADPLANE_EFF_RX,
  .r_y = QUADPLANE_EFF_RY,
  .r_z = QUADPLANE_EFF_RZ,
  .spin_dir = QUADPLANE_EFF_SPIN_DIR,

  .rotor_k_t_pprz = QUADPLANE_EFF_ROTOR_K_T_PPRZ,
  .kappa = QUADPLANE_EFF_KAPPA,

  .pusher_k_t_pprz = QUADPLANE_EFF_PUSHER_K_T_PPRZ,
  .pusher_y = QUADPLANE_EFF_PUSHER_RY,
  .pusher_z = QUADPLANE_EFF_PUSHER_RZ,
  .pusher_spin_dir = QUADPLANE_EFF_PUSHER_SPIN_DIR,

  .elevon_deflect = QUADPLANE_EFF_K_ELEVON_DEFLECT,
  .elevon_roll = QUADPLANE_EFF_K_ELEVON_ROLL,
  .elevon_pitch = QUADPLANE_EFF_K_ELEVON_PITCH,
  .elevon_propwash = QUADPLANE_EFF_K_ELEVON_PROPWASH,
  .elevon_diff_drag_yaw = QUADPLANE_EFF_K_ELEVON_DIFF_DRAG_YAW,
  .elevon_v_full = QUADPLANE_EFF_K_ELEVON_V_FULL,

  .k_lift = QUADPLANE_EFF_K_LIFT,
  .v_wing = QUADPLANE_EFF_V_WING,
};

static float quadplane_eff_liftd;
float const grav = 9.81f;  // Gravitational Acceleration [m/s^2]
float quadplane_eff_elevon_rate = QUADPLANE_EFF_ELEVON_RATE;                 // Elevon command slew limit [pprz/s]
float quadplane_eff_periodic_freq = EFF_SCHEDULING_QUADPLANE_PERIODIC_FREQ;   // Module Frequency [Hz]


static inline void quadplane_update_cmd_cache(void);
static inline void quadplane_update_airspeed_cache(void);
static inline void quadplane_update_thrust_model(void);
static inline void quadplane_update_motor_effectiveness(void);
static inline void quadplane_update_elevon_effectiveness(void);


void eff_scheduling_quadplane_init(void)
{
  if (quadplane_eff_sched_p.rotor_k_t_pprz[1] <= 0.f ||
      quadplane_eff_sched_p.pusher_k_t_pprz[1] <= 0.f) {
    while (true) {
      /* Invalid thrust calibration: do not arm INDI. */
      /*The change in thrust due to change in pprz is given by: dT/du = b + 2*c*u,
      * if rotor_k_t_pprz[0] is negative an increase in pprz results in a decrease in thrust, breaking the thrust model*/
    }
  }

  /*Initialize commands for 4 hoovering motors */
  for (int i = 0; i < 4; i++)
  {
    quadplane_eff_sched_v.rotor_cmd[i] = QUADPLANE_EFF_MOTOR_HOVER;
    quadplane_eff_sched_v.rotor_cmd_T[i] = quadplane_eff_sched_p.mass*grav/4.f;
    quadplane_eff_sched_v.rotor_dT_dpprz[i] = quadplane_eff_sched_p.rotor_k_t_pprz[0];
  }

  /*Initialize pusher commands */
  quadplane_eff_sched_v.pusher_cmd = 0;
  quadplane_eff_sched_v.pusher_cmd_T = 0;
  quadplane_eff_sched_v.pusher_dT_dpprz = quadplane_eff_sched_p.pusher_k_t_pprz[0];

  /*Initialize airspeed */
  quadplane_eff_sched_v.airspeed = 0.f;
  quadplane_eff_sched_v.airspeed_sq = 0.f;

  /*Initialize elevon commands */
  quadplane_eff_sched_v.cmd_elevon_r = 0.f;
  quadplane_eff_sched_v.cmd_elevon_l = 0.f;
  
  quadplane_eff_liftd = 0.f;
}


void eff_scheduling_quadplane_periodic(void)
{
  quadplane_update_cmd_cache();
  quadplane_update_airspeed_cache();
  quadplane_update_thrust_model();

  quadplane_update_motor_effectiveness();
  quadplane_update_elevon_effectiveness();
}



static inline void quadplane_update_cmd_cache(void)
{
  quadplane_eff_sched_v.rotor_cmd[0] = actuator_state_filt_vect[QUADPLANE_ACT_MOTOR_FR];
  quadplane_eff_sched_v.rotor_cmd[1] = actuator_state_filt_vect[QUADPLANE_ACT_MOTOR_BR];
  quadplane_eff_sched_v.rotor_cmd[2] = actuator_state_filt_vect[QUADPLANE_ACT_MOTOR_BL];
  quadplane_eff_sched_v.rotor_cmd[3] = actuator_state_filt_vect[QUADPLANE_ACT_MOTOR_FL]; 
  quadplane_eff_sched_v.cmd_elevon_r = actuator_state_filt_vect[QUADPLANE_ACT_ELEVON_R];
  quadplane_eff_sched_v.cmd_elevon_l = actuator_state_filt_vect[QUADPLANE_ACT_ELEVON_L];
  quadplane_eff_sched_v.pusher_cmd   = actuator_state_filt_vect[QUADPLANE_ACT_PUSHER]; 
}

// Airspeed = forward (body x) EKF speed, standing in for the airspeed sensor
static inline void quadplane_update_airspeed_cache(void)
{
  const struct FloatRMat *ned_to_body = stateGetNedToBodyRMat_f();
  const struct NedCoor_f *vel = stateGetSpeedNed_f();

  float u = RMAT_ELMT(*ned_to_body, 0, 0) * vel->x
          + RMAT_ELMT(*ned_to_body, 0, 1) * vel->y
          + RMAT_ELMT(*ned_to_body, 0, 2) * vel->z;

  // Written so that a NaN velocity also lands on 0: this feeds the inner-loop B matrix
  if (!(u > 0.f)) { u = 0.f; }
  if (u > 30.f)   { u = 30.f; }
  quadplane_eff_sched_v.airspeed = u;
  quadplane_eff_sched_v.airspeed_sq = u * u;
}

/* Quadratic thrust model:
 *   T(u)      = b*u + c*u^2
 *   dT/du(u)  = b + 2*c*u
 * k_T_pprz = [b, c].
 */
static inline void quadplane_update_thrust_model(void)
{
  const float b = quadplane_eff_sched_p.rotor_k_t_pprz[0];
  const float c = quadplane_eff_sched_p.rotor_k_t_pprz[1];

  const float d = quadplane_eff_sched_p.pusher_k_t_pprz[0];
  const float e = quadplane_eff_sched_p.pusher_k_t_pprz[1];

  /* For 4 motors in hoovering */
  for (int i = 0; i < 4; i++) {
    float u = quadplane_eff_sched_v.rotor_cmd[i];
    Bound(u, 0.f, MAX_PPRZ);
    quadplane_eff_sched_v.rotor_cmd_T[i]        = b * u + c * u * u;
    quadplane_eff_sched_v.rotor_dT_dpprz[i] = b + 2.f * c * u;
    Bound(quadplane_eff_sched_v.rotor_dT_dpprz[i], 0.1f * b, 5.0f * b);
  }

  /* For pusher motor */
  float u = quadplane_eff_sched_v.pusher_cmd;
  Bound(u, 0.f, MAX_PPRZ);
  quadplane_eff_sched_v.pusher_cmd_T = d * u + e * u * u;
  quadplane_eff_sched_v.pusher_dT_dpprz = d + 2.f * e * u;
  Bound(quadplane_eff_sched_v.pusher_dT_dpprz, 0.1f * d, 5.0f * d);
}

/*
 * Control effectiveness wrt. motor throttle (dT = dT/du)
 * Global model with tilt, our case a = 0
 * dMx/du = -y * dT * cos(a) + spin * κ * dT * sin(a)
 * dMy/du = x * dT * cos(a) + z * dT * sin(a)
 * dMz/du = -y * dT * sin(a) - spin * κ * dT * cos(a)
 * dFx/du = dT * sin(a)
 * dFz/du = -dT * cos(a)
 */
static inline void quadplane_update_motor_effectiveness(void)
{
  /* 4 Hoovering motor Moment and Force calculations to fill G1G2 */
  for (int i = 0; i < 4; i++) {
    const float x  = quadplane_eff_sched_p.r_x[i];
    const float y  = quadplane_eff_sched_p.r_y[i];
    const float z  = quadplane_eff_sched_p.r_z[i];
    const float spin = quadplane_eff_sched_p.spin_dir[i];
    const float kappa = quadplane_eff_sched_p.kappa;
    const float rotor_dT = quadplane_eff_sched_v.rotor_dT_dpprz[i];


    /* Angular-acceleration effectiveness wrt. motor commands */
    float dMx = -y * rotor_dT;
    float dMy = x * rotor_dT;
    float dMz = - spin * kappa * rotor_dT;
    /* Linear-acceleration effectiveness wrt. motor commands*/
    float dAz = -rotor_dT;

    g1g2[QUADPLANE_VC_MX][i] = dMx / quadplane_eff_sched_p.Ixx;
    g1g2[QUADPLANE_VC_MY][i] = dMy / quadplane_eff_sched_p.Iyy;
    g1g2[QUADPLANE_VC_MZ][i] = dMz / quadplane_eff_sched_p.Izz;
    g1g2[QUADPLANE_VC_AX][i] = 0.f;
    g1g2[QUADPLANE_VC_AZ][i] = dAz / quadplane_eff_sched_p.mass;
  }

  /* Pusher motor Moment and Force calculations to fill G1G2 */
  const float y_pusher = quadplane_eff_sched_p.pusher_y;
  const float z_pusher = quadplane_eff_sched_p.pusher_z;
  const float spin_pusher = quadplane_eff_sched_p.pusher_spin_dir;
  const float kappa = quadplane_eff_sched_p.kappa;
  const float pusher_dT = quadplane_eff_sched_v.pusher_dT_dpprz;

  float dMx = spin_pusher * kappa * pusher_dT;
  float dMy = z_pusher * pusher_dT;
  float dMz = -y_pusher * pusher_dT;
  float dAx = pusher_dT;

  g1g2[QUADPLANE_VC_MX][QUADPLANE_ACT_PUSHER] = dMx / quadplane_eff_sched_p.Ixx;
  g1g2[QUADPLANE_VC_MY][QUADPLANE_ACT_PUSHER] = dMy / quadplane_eff_sched_p.Iyy;
  g1g2[QUADPLANE_VC_MZ][QUADPLANE_ACT_PUSHER] = dMz / quadplane_eff_sched_p.Izz;
  g1g2[QUADPLANE_VC_AX][QUADPLANE_ACT_PUSHER] = dAx / quadplane_eff_sched_p.mass;
  g1g2[QUADPLANE_VC_AZ][QUADPLANE_ACT_PUSHER] = 0.f;

}


/*
 * Control Effectiveness wrt. elevons
 *
 * Convention: +pprz = elevon up (trailing edge up)
 * (+)QUADPLANE_ACT_ELEVON_R (+)QUADPLANE_ACT_ELEVON_L  -> pitch up (+My)
 * (+)QUADPLANE_ACT_ELEVON_R (-)QUADPLANE_ACT_ELEVON_L  -> roll right (+Mx).
 *
 * Scheduled on the forward speed (see quadplane_update_airspeed_cache): the columns are
 * zero at rest and grow with V^2, so the allocator brings the elevons in on its own.
 */
static inline void quadplane_update_elevon_effectiveness(void)
{
  // Forward thrust component: Tx only Pusher 
  float Tx = quadplane_eff_sched_v.pusher_cmd_T;
  const float V = quadplane_eff_sched_v.airspeed;
  const float V2 = quadplane_eff_sched_v.airspeed_sq;
  const float dDelta = quadplane_eff_sched_p.elevon_deflect; // [rad per pprz]
  

  // dM/dDelta [N.m/rad] 
  float dMx_dDelta = quadplane_eff_sched_p.elevon_roll * V2;
  float dMy_dDelta = quadplane_eff_sched_p.elevon_pitch * V2
                    + quadplane_eff_sched_p.elevon_propwash * Tx * V;

  float dMx = dMx_dDelta * dDelta / quadplane_eff_sched_p.Ixx;   // [rad/s^2 per pprz]
  float dMy = dMy_dDelta * dDelta / quadplane_eff_sched_p.Iyy;   // [rad/s^2 per pprz]

  // Differential-drag yaw: dMz/d(delta_cmd) = k * V^2 * delta
  const float yaw_gain = quadplane_eff_sched_p.elevon_diff_drag_yaw * V2 * dDelta * dDelta
                       / quadplane_eff_sched_p.Izz;
  float dMz_r =  yaw_gain * quadplane_eff_sched_v.cmd_elevon_r;
  float dMz_l = -yaw_gain * quadplane_eff_sched_v.cmd_elevon_l;

  Bound(dMx, 0.f, 0.1f);
  Bound(dMy, 0.f, 0.1f);
  Bound(dMz_r, -0.1f, 0.1f);
  Bound(dMz_l, -0.1f, 0.1f);

  // Elevon effectiveness matrix
  // Right Elevon (pitch up -> +delta, roll right -> +delta)
  g1g2[QUADPLANE_VC_MX][QUADPLANE_ACT_ELEVON_R] = dMx;
  g1g2[QUADPLANE_VC_MY][QUADPLANE_ACT_ELEVON_R] = dMy;
  // Left Elevon (pitch up -> +delta, roll right -> -delta)
  g1g2[QUADPLANE_VC_MX][QUADPLANE_ACT_ELEVON_L] = -dMx;
  g1g2[QUADPLANE_VC_MY][QUADPLANE_ACT_ELEVON_L] = dMy;
  // Differential-drag yaw effect
  g1g2[QUADPLANE_VC_MZ][QUADPLANE_ACT_ELEVON_R] = dMz_r;  
  g1g2[QUADPLANE_VC_MZ][QUADPLANE_ACT_ELEVON_L] = dMz_l; 
  // No elevon effect on vertical or forward acceleration
  g1g2[QUADPLANE_VC_AX][QUADPLANE_ACT_ELEVON_R] = 0.f;
  g1g2[QUADPLANE_VC_AZ][QUADPLANE_ACT_ELEVON_R] = 0.f;
  g1g2[QUADPLANE_VC_AX][QUADPLANE_ACT_ELEVON_L] = 0.f;
  g1g2[QUADPLANE_VC_AZ][QUADPLANE_ACT_ELEVON_L] = 0.f;

}

/**
 * Implement if necessary
 */
void eff_scheduling_quadplane_report(void)
{

}


/**
 * Configure the weighted least-squares (WLS) control allocator for the
 * quadplane INDI stabilizer.
 */
void stabilization_indi_set_wls_settings(void)
{
#ifdef STABILIZATION_INDI_WLS_PRIORITIES
  const float Wv_pref[INDI_OUTPUTS] = STABILIZATION_INDI_WLS_PRIORITIES;
  for (uint8_t i = 0; i < INDI_OUTPUTS; i++) {
    wls_stab_p.Wv[i] = Wv_pref[i];
  }
#endif

  for (uint8_t i = 0; i < INDI_NUM_ACT; i++) {
    wls_stab_p.u_min[i] = act_is_servo[i] ? -MAX_PPRZ : QUADPLANE_EFF_MOTOR_IDLE;
    wls_stab_p.u_max[i]  =  MAX_PPRZ;
    wls_stab_p.u_pref[i] =  act_pref[i];
  }
/* Same same, need to define correctly act_is_servo
  // 4 hoovering motors 
  for (uint8_t i = 0; i < 4; i++) {
    wls_stab_p.u_min[i] = QUADPLANE_EFF_MOTOR_IDLE;
    wls_stab_p.u_max[i] = MAX_PPRZ;
    wls_stab_p.u_pref[i] = act_pref[i];
  }

  // Elevons : servos 
  wls_stab_p.u_min[QUADPLANE_ACT_ELEVON_R] = -MAX_PPRZ;
  wls_stab_p.u_max[QUADPLANE_ACT_ELEVON_R] =  MAX_PPRZ;
  wls_stab_p.u_pref[QUADPLANE_ACT_ELEVON_R] = act_pref[QUADPLANE_ACT_ELEVON_R];

  wls_stab_p.u_min[QUADPLANE_ACT_ELEVON_L] = -MAX_PPRZ;
  wls_stab_p.u_max[QUADPLANE_ACT_ELEVON_L] =  MAX_PPRZ;
  wls_stab_p.u_pref[QUADPLANE_ACT_ELEVON_L] = act_pref[QUADPLANE_ACT_ELEVON_L];

  // Pusher : motor 
  wls_stab_p.u_min[QUADPLANE_ACT_PUSHER] = QUADPLANE_EFF_MOTOR_IDLE;
  wls_stab_p.u_max[QUADPLANE_ACT_PUSHER] = MAX_PPRZ;
  wls_stab_p.u_pref[QUADPLANE_ACT_PUSHER] = act_pref[QUADPLANE_ACT_PUSHER];
*/

  // Elevons: the travel opens with dynamic pressure, (V / elevon_v_full)^2 of the range.
  // if elevon_v_full = 0 => full travel at any speed
  float open = 1.f;
  if (quadplane_eff_sched_p.elevon_v_full > 0.f) {
    const float q = quadplane_eff_sched_v.airspeed / quadplane_eff_sched_p.elevon_v_full;
    if (q < 1.f) { open = q * q; }
  }
  // Each step moves at most elevon_rate / freq around the last command, inside that travel
  const float lim = open * MAX_PPRZ;
  const float step = quadplane_eff_elevon_rate / quadplane_eff_periodic_freq;
  for (int e = QUADPLANE_ACT_ELEVON_R; e <= QUADPLANE_ACT_ELEVON_L; e++) {
    float u = actuators_pprz[e];
    Bound(u, -lim, lim);
    wls_stab_p.u_min[e]  = Max(u - step, -lim);
    wls_stab_p.u_max[e]  = Min(u + step, lim);
    wls_stab_p.u_pref[e] = open * actuator_state_filt_vect[e];
  }
}