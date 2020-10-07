/*
 * Copyright (C) Murat BRONZ
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
/**
 * @file "modules/autonomous_soaring/autonomous_soaring.c"
 * @author Murat BRONZ
 * Soaring related stuff
 */

#include "modules/autonomous_soaring/autonomous_soaring.h"
#include "subsystems/datalink/telemetry.h"

#if FLIGHTRECORDER_SDLOG
#include "modules/loggers/pprzlog_tp.h"
#include "modules/loggers/sdlog_chibios.h"
#endif

// #include "modules/datalink/extra_pprz_dl.h"
// #include "subsystems/datalink/telemetry.h"
#include <string.h>
#include "generated/airframe.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/downlink.h"

// State interface
#include "state.h"

// Time and uint32_t 
#include "mcu_periph/sys_time.h"

#include "subsystems/imu.h"

// local variables
static uint32_t last_periodic_time;     // last periodic time
static float wx_old;
static float wz_old;
static int sampling;

#ifndef SMARTPROBE_AOA_OFFSET
#define SMARTPROBE_AOA_OFFSET 0.0
#endif

#ifndef SMARTPROBE_AOA_COEFF
#define SMARTPROBE_AOA_COEFF 0.85
#endif

#ifndef SMARTPROBE_DT_SAMPLE
#define SMARTPROBE_DT_SAMPLE 1
#endif

struct Soaring_states soaring_states;
struct Smartprobe smartprobe;
struct Soaring_coeffs soaring_coeffs;


static void soaring_downlink(struct transport_tx *trans, struct link_device *dev){
  pprz_msg_send_SOARING_TELEMETRY(trans, dev, AC_ID,
   &smartprobe.va,
   &smartprobe.aoa,
   &smartprobe.beta,
   &smartprobe.q,
   &smartprobe.p,

   &soaring_states.wx,
   &soaring_states.wz,
   &soaring_states.d_wx,
   &soaring_states.d_wz,
   &soaring_states.p_w);
}

void soaring_init(void) {
  memset(&soaring_states, 0, sizeof(struct Soaring_states));
  memset(&smartprobe, 0, sizeof(struct Smartprobe));

  wx_old = 0.0;
  wz_old = 0.0;
  last_periodic_time = 0;
  soaring_coeffs.sample_nr = SMARTPROBE_DT_SAMPLE;
  soaring_coeffs.aoa_coeff = SMARTPROBE_AOA_COEFF;
  soaring_coeffs.aoa_offset = SMARTPROBE_AOA_OFFSET;
  sampling = 0;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SOARING_TELEMETRY, soaring_downlink);
#endif
}

void soaring_parse_AEROPROBE(uint8_t *buf)
{
  smartprobe.va   =  DL_AEROPROBE_velocity(buf) / 100.;
  smartprobe.aoa  =  (DL_AEROPROBE_a_attack(buf) * 0.000174527 * soaring_coeffs.aoa_coeff) - (soaring_coeffs.aoa_offset*0.0174);
  smartprobe.beta =  DL_AEROPROBE_a_sideslip(buf) * 0.000174527;
  smartprobe.q    =  DL_AEROPROBE_dynamic_p(buf);
  smartprobe.p    =  DL_AEROPROBE_static_p(buf);
}

void soaring_status_report(void)
{
  soaring_downlink(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
}
// put a parameter for the rho and temperature or static pressure of the day.

static inline void soaring_run_step(void){
  float va = soaring_states.va;
  soaring_states.gama = soaring_states.theta - soaring_states.aoa;
  soaring_states.wx =  va*cosf(soaring_states.gama) - soaring_states.Vx;
  // soaring_states.wz = -1*(va*sinf(soaring_states.gama) - soaring_states.Vz);
  soaring_states.wz = soaring_states.Vz + va*sinf(soaring_states.gama);

  // Calculate the derivatives
  soaring_states.d_wx = (soaring_states.wx - wx_old) / soaring_states.dt;
  soaring_states.d_wz = (soaring_states.wz - wz_old) / soaring_states.dt;
  wx_old = soaring_states.wx;
  wz_old = soaring_states.wz;
  
  float gama_sign = 1.;
  if (soaring_states.gama < 0){
    gama_sign = -1.0;
  } else {
    gama_sign = 1.0;
  }

  soaring_states.p_w = -soaring_states.d_wx*(-va*cosf(soaring_states.gama)*gama_sign);
  // -d_wx*(-vel * np.sign(gamma) * cos(gamma) )

  // if (va > 5.0f){
  //   soaring_states.theta_cmd = gust_gains.p_wx*soaring_states.wx + gust_gains.d_wx*d_wx + gust_gains.p_wz*soaring_states.wz + gust_gains.d_wz*d_wz; // stateGetBodyRates_f()->q // just to try for now...
  // } else {
  //   soaring_states.theta_cmd = 0.0;
  // }
  // Keep the last pitch command
  // theta_cmd_old = soaring_states.theta_cmd;
}

void soaring_periodic(void) {
// Get state variables...
  soaring_states.va    = smartprobe.va;  // stateGetAirspeed_f();
  // soaring_states.va    = stateGetAirspeed_f();           // m/s
  soaring_states.aoa   = smartprobe.aoa; // stateGetAngleOfAttack_f();      // rad.
  soaring_states.theta = stateGetNedToBodyEulers_f()->theta; // pitch angle in rad.
  soaring_states.Vx    = stateGetHorizontalSpeedNorm_f(); // Ground speedNorm, used as in-plane forward direction groundspeed
  soaring_states.Vz    = stateGetSpeedNed_f()->z;         // Vertical speed  m/s
  
  if (last_periodic_time == 0) {
    soaring_states.dt = SOARING_PERIODIC_PERIOD;
    last_periodic_time = get_sys_time_msec();
  } else {
    soaring_states.dt = (get_sys_time_msec() - last_periodic_time); //   / 1000.f;
    last_periodic_time = get_sys_time_msec();
  }
  // if (gust_states.aoa < gust_gains.max_aoa){
  if (sampling > soaring_coeffs.sample_nr){
    soaring_run_step();
    sampling = 0; 
  }
  sampling += 1; // FIXME should we stop sampling during stall ?
  
}

// void soaring_datalink_callback() {}

void autonomous_soaring_Set_Sampling(int _v)
{
  soaring_coeffs.sample_nr = _v;
}
