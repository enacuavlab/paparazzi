/*
 * Copyright (C) Florian Sansou <florian.sansou@enac.fr>
 *  *
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



#include "math/pprz_algebra_float.h"
#include "state.h"

#include "modules/radio_control/radio_control.h"
#include "modules/actuators/actuators.h"
#include "modules/core/abi.h"
#include "filters/low_pass_filter.h"
#include "math/pprz_simple_matrix.h"
#include <stdio.h>
#include "pprzlink/messages.h"
#include "modules/datalink/datalink.h"
#include "modules/datalink/downlink.h"
#include "modules/loggers/flight_recorder.h"
#include "filters/low_pass_filter.h"

#include "generated/flight_plan.h"
#include "generated/airframe.h"

#define MAX(a,b) ((a) > (b) ? (a) : (b))

#ifndef SMEUR_TO_BARTH_PHI
#define SMEUR_TO_BARTH_PHI 0.
#endif

#ifndef SMEUR_TO_BARTH_THETA
#define SMEUR_TO_BARTH_THETA 90.
#endif

#ifndef SMEUR_TO_BARTH_PSI
#define SMEUR_TO_BARTH_PSI 0.
#endif

#define POS_INC 0.005

#define COEFF_DABB 0

#define FILTRE 1

#if COEFF_DABB == 1
  #include "coef_dabb_good.h"
#else
  //#include "coef.h"
  //  #include "coef_systune.h"
  // #include "coef_systune_struct.h"
  // #include "coef_systune_struct_armand.h"
  #include "coef_systune_struct_dyn_mot_multi.h"
  // #include "coef_systune_struct_dyn_mot2.h"
  // #include "coef_systune_decouplage.h"
  //#include "coef_dabb_good_feed_neg_3.h"
  // #include "coef_dabb_good_feed_neg.h"
#endif




/*#include "modules/loggers/sdlog_chibios.h"
#include "mcu_periph/sys_time.h"
*/

float x_e[CTRL_HOVER_WIND_INPUT][1] = {{0}, {0}};
float state_vector_redu[CTRL_HOVER_WIND_INPUT][1];

float eps[CTRL_HOVER_WIND_INPUT][1];
float dot_x_e_dt[CTRL_HOVER_WIND_NUM_INTEGRATOR_STATE][1]; 
float u_integrator[CTRL_HOVER_WIND_NUM_ACT][1];
float u_prop[CTRL_HOVER_WIND_NUM_ACT][1];
float u_filter[CTRL_HOVER_WIND_NUM_ACT][1];
float u_sub[CTRL_HOVER_WIND_NUM_ACT][1];
float u[CTRL_HOVER_WIND_NUM_ACT][1];
float u_scale[CTRL_HOVER_WIND_NUM_ACT][1];

// rotation from Smeur frame to Barth frame 
static struct FloatQuat quat_smeur_2_barth;

struct FloatQuat quat_att_barth_frame;

struct NedCoor_f pos_target;
float rc_x;
float rc_y;
float rc_z;

float tf_state1[2] = {0, 0};
float tf_state2[2] = {0, 0};
float tf_state3[2] = {0, 0};
float tf_state4[2] = {0, 0};

Butterworth2LowPass rates_lowpass_filters[3];


#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_payload_float(struct transport_tx *trans, struct link_device *dev)
{
  float f[35] = {pos_target.x, pos_target.y, pos_target.z,
                 eps[0][0], eps[1][0], eps[2][0], eps[3][0], eps[4][0], eps[5][0], eps[6][0], eps[7][0], eps[8][0], eps[9][0], eps[10][0],
                 u_scale[0][0], u_scale[1][0], u_scale[2][0], u_scale[3][0],
                 u_prop[0][0], u_prop[1][0], u_prop[2][0], u_prop[3][0],
                 u_filter[0][0],u_filter[1][0],u_filter[2][0],u_filter[3][0],
                 u_integrator[0][0], u_integrator[1][0],
                 u[0][0], u[1][0], u[2][0], u[3][0],
                 rc_x, rc_y, rc_z,
                }; // temps, frequence_actuelle, sinusoidale, increment
  pprz_msg_send_PAYLOAD_FLOAT(trans, dev, AC_ID, 35, f);
}
#endif


void stabilization_hover_wind_init(void){

  //log_hoverwind_start();
  struct FloatEulers eul2smeurbarth = {
    RadOfDeg(SMEUR_TO_BARTH_PHI),
    RadOfDeg(SMEUR_TO_BARTH_THETA),
    RadOfDeg(SMEUR_TO_BARTH_PSI)
  };
  float_quat_of_eulers(&quat_smeur_2_barth, &eul2smeurbarth);

  x_e[0][0] = 0;
  x_e[1][0] = 0;
  tf_state1[0] = 0;
  tf_state1[1] = 0;
  tf_state2[0] = 0;
  tf_state2[1] = 0;
  tf_state3[0] = 0;
  tf_state3[1] = 0;
  tf_state4[0] = 0;
  tf_state4[1] = 0;
  pos_target = *stateGetPositionNed_f(); 

  //hack for takeoff
  pos_target.z = -1.3070;

  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PAYLOAD_FLOAT, send_payload_float);
  #endif

  rc_x = 0;
  rc_y = 0;
  rc_z = 0;

  float tau = 1.0 / (2.0 * M_PI * 20.0);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  
  // Filtering of the gyroscope
  int8_t i;
  for (i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&rates_lowpass_filters[i], tau, sample_time, 0.0);
  }
  // init_butterworth_2_low_pass(&rates_lowpass_filters[0], tau, sample_time, 0.0);
}

void stabilization_hover_wind_run(bool in_flight){

  rc_x = -(radio_control.values[RADIO_PITCH]/9600.0); //[-1, 1]
  rc_y = (radio_control.values[RADIO_ROLL]/9600.0); //[-1, 1]
  rc_z = (radio_control.values[RADIO_THROTTLE]/9600.0); 

 
  if(fabs(rc_x) >= 0.5){
    pos_target.x += rc_x*POS_INC;
  }
  
  if(fabs(rc_y) >= 0.5){
     pos_target.y += rc_y*POS_INC;
  }
 
  if(rc_z == 1){
     pos_target.x = 3.5;
     pos_target.y = 0.464;
     pos_target.z = -1.339;
  }

  
   
  

  struct FloatVect3 barth_rate, smeur_rate;
  smeur_rate.x = stateGetBodyRates_f()->p;
  smeur_rate.y = stateGetBodyRates_f()->q;
  smeur_rate.z = stateGetBodyRates_f()->r;
    
  float_quat_comp(&quat_att_barth_frame, stateGetNedToBodyQuat_f(), &quat_smeur_2_barth);
  float_quat_vmult(&barth_rate, &quat_smeur_2_barth, &smeur_rate);

  float rate_vect[3] = {barth_rate.x, barth_rate.y, barth_rate.z};

  int8_t i;
  for (i = 0; i < 3; i++) {
    update_butterworth_2_low_pass(&rates_lowpass_filters[i], rate_vect[i]);
  }

  // update_butterworth_2_low_pass(&rates_lowpass_filters[0], barth_rate.y);

  if(COEFF_DABB == 1 ){
    eps[0][0] = stateGetPositionNed_f()->x - pos_target.x; 
    eps[1][0] = stateGetPositionNed_f()->y - pos_target.y;
    eps[2][0] = stateGetPositionNed_f()->z - pos_target.z;
    eps[3][0] = stateGetSpeedNed_f()->x;
    eps[4][0] = stateGetSpeedNed_f()->y;
    eps[5][0] = stateGetSpeedNed_f()->z;
    eps[6][0] = quat_att_barth_frame.qx;
    eps[7][0] = quat_att_barth_frame.qz;
    eps[8][0] = rates_lowpass_filters[0].o[0];
    eps[9][0] = rates_lowpass_filters[1].o[0];
    eps[10][0] = rates_lowpass_filters[2].o[0];
    // eps[8][0] = barth_rate.x;
    // eps[9][0] = rates_lowpass_filters[0].o[0];
    // eps[10][0] = barth_rate.z;
  }
  else{
    eps[0][0] = pos_target.x - stateGetPositionNed_f()->x; 
    eps[1][0] = pos_target.y - stateGetPositionNed_f()->y;
    eps[2][0] = pos_target.z - stateGetPositionNed_f()->z;
    eps[3][0] = -stateGetSpeedNed_f()->x;
    eps[4][0] = -stateGetSpeedNed_f()->y;
    eps[5][0] = -stateGetSpeedNed_f()->z;
    eps[6][0] = -quat_att_barth_frame.qx;
    eps[7][0] = -quat_att_barth_frame.qz;
    eps[8][0] = -rates_lowpass_filters[0].o[0];
    eps[9][0] = -rates_lowpass_filters[1].o[0];
    eps[10][0] = -rates_lowpass_filters[2].o[0];
    // eps[8][0] = -barth_rate.x;
    // eps[9][0] = -rates_lowpass_filters[0].o[0];
    // eps[10][0] = -barth_rate.z;
  }

  MAT_MUL_c(CTRL_HOVER_WIND_NUM_INTEGRATOR_STATE, CTRL_HOVER_WIND_INPUT, 1, dot_x_e_dt, H, eps, 1./PERIODIC_FREQUENCY);
  x_e[0][0] +=  dot_x_e_dt[0][0];
  x_e[1][0] +=  dot_x_e_dt[1][0];
  u_integrator[0][0] = x_e[0][0];
  u_integrator[1][0] = x_e[1][0];

  MAT_MUL(CTRL_HOVER_WIND_NUM_ACT, CTRL_HOVER_WIND_INPUT, 1, u_prop, K, eps);

  if(FILTRE){
    float tf1_tmp;
    float tf2_tmp;
    float tf3_tmp;
    float tf4_tmp;

    tf1_tmp = u_prop[0][0] - den[1]*tf_state1[0] - den[2]* tf_state1[1];
    u_filter[0][0] = num[0]*tf1_tmp + num[1]*tf_state1[0] + num[2]*tf_state1[1];
    tf_state1[1] = tf_state1[0];
    tf_state1[0] = tf1_tmp;

    tf2_tmp = u_prop[1][0] - den[1]*tf_state2[0] - den[2]* tf_state2[1];
    u_filter[1][0] = num[0]*tf2_tmp + num[1]*tf_state2[0] + num[2]*tf_state2[1];
    tf_state2[1] = tf_state2[0];
    tf_state2[0] = tf2_tmp;

    tf3_tmp = u_prop[2][0] - den[1]*tf_state3[0] - den[2]* tf_state3[1];
    u_filter[2][0] = num[0]*tf3_tmp + num[1]*tf_state3[0] + num[2]*tf_state3[1];
    tf_state3[1] = tf_state3[0];
    tf_state3[0] = tf3_tmp;

    tf4_tmp = u_prop[3][0] - den[1]*tf_state4[0] - den[2]* tf_state4[1];
    u_filter[3][0] = num[0]*tf4_tmp + num[1]*tf_state4[0] + num[2]*tf_state4[1];
    tf_state4[1] = tf_state4[0];
    tf_state4[0] = tf4_tmp;
  }
  

  float integrator_repart[CTRL_HOVER_WIND_NUM_ACT][1] = {{u_integrator[0][0]}, {u_integrator[0][0]}, {u_integrator[1][0]}, {u_integrator[1][0]} };

  if(COEFF_DABB == 1){
    if(FILTRE){
      MAT_SUB(CTRL_HOVER_WIND_NUM_ACT, 1, u_sub, integrator_repart, u_filter);
    }
    else{
      MAT_SUB(CTRL_HOVER_WIND_NUM_ACT, 1, u_sub, integrator_repart, u_prop);
    }
  }
  else{
    if(FILTRE){
      MAT_SUM(CTRL_HOVER_WIND_NUM_ACT, 1, u_sub, integrator_repart, u_filter);
    }
    else{
      MAT_SUM(CTRL_HOVER_WIND_NUM_ACT, 1, u_sub, integrator_repart, u_prop);
    }
  }
 
  //
  MAT_SUM(CTRL_HOVER_WIND_NUM_ACT, 1, u, ueq, u_sub);


 // LEFT_MOTOR 
  if (u[0][0]>0){
    u_scale[0][0] = (sqrtf(MAX(u[0][0],0)/kf) / mot_max_speed)*MAX_PPRZ; 
  }
  else{
    u_scale[0][0] = 0; 
  }

  // RIGHT_MOTOR 
  if (u[1][0]>0){
    u_scale[1][0] = (sqrtf(MAX(u[1][0],0)/kf) / mot_max_speed)*MAX_PPRZ; 
  }
  else{
    u_scale[1][0] = 0; 
  }

 //ELEVON_LEFT  
  u_scale[2][0] = (u[2][0]*6/M_PI)*MAX_PPRZ;

  // ELEVON_RIGHT  
  u_scale[3][0] = (u[3][0]*6/M_PI)*MAX_PPRZ;


  //Cmd negative pitch up 
  // actuators_pprz[0] -> ELEVON_LEFT, actuators_pprz[1] -> ELEVON_RIGHT,  actuators_pprz[2] -> RIGHT_MOTOR,  actuators_pprz[3] -> LEFT_MOTOR
  actuators_pprz[0]=TRIM_PPRZ(u_scale[2][0]); 
  actuators_pprz[1]=TRIM_PPRZ(-u_scale[3][0]);  

  // actuators_pprz[0]=TRIM_PPRZ(0); 
  // actuators_pprz[1]=TRIM_PPRZ(0); 


  actuators_pprz[2]=TRIM_UPPRZ(u_scale[1][0]);
  actuators_pprz[3]=TRIM_UPPRZ(u_scale[0][0]); 

  // temps = increment * 0.002f;
  // frequence_actuelle = frequence_debut + (frequence_fin - frequence_debut) * (temps / duree);
  // sinusoidale = 9600.0f/4.0f*(1.2f + sinf(2.0f * M_PI * frequence_actuelle * temps));
  // if(temps <= duree){
  //   actuators_pprz[2]=TRIM_UPPRZ(sinusoidale);
  //   actuators_pprz[3]=TRIM_UPPRZ(sinusoidale); 
  // }
  // else{
  //   actuators_pprz[2]=TRIM_UPPRZ(0);
  //   actuators_pprz[3]=TRIM_UPPRZ(0); 
  // }


  // increment = increment + 1;
  
  
  if (in_flight) {
    stabilization_cmd[COMMAND_THRUST] = (actuators_pprz[2]+actuators_pprz[3]); // for in_flight detection
    //printf("in_flight\n");
  } else {
    stabilization_cmd[COMMAND_THRUST] = 1000;
    //printf("Not in_flight\n");
  };
  
}

void parse_wind_info_msg(uint8_t *buf){
  uint8_t ac_id_send = pprzlink_get_DL_WIND_INFO_ac_id(buf);
  if(ac_id_send != AC_ID) { return; }
  float airspeed = pprzlink_get_DL_WIND_INFO_airspeed(buf);
  uint8_t flags = pprzlink_get_DL_WIND_INFO_flags(buf);

  float empty = -1; 

  #if FLIGHTRECORDER_SDLOG
      // log to SD card
      pprz_msg_send_WIND_INFO_RET(&pprzlog_tp.trans_tx, &(flightrecorder_sdlog).device, AC_ID, &flags, &empty, &empty, &empty, &airspeed);
  #endif

}

