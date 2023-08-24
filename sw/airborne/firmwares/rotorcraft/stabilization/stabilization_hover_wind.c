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
#include "generated/airframe.h"
#include "modules/radio_control/radio_control.h"
#include "modules/actuators/actuators.h"
#include "modules/core/abi.h"
#include "filters/low_pass_filter.h"
#include "wls/wls_alloc.h"
#include "math/pprz_simple_matrix.h"
#include <stdio.h>
#include "modules/sensors/serial_act_t4.h"
#include "modules/core/abi.h"
#include "pprzlink/messages.h"
#include "modules/datalink/datalink.h"
#include "modules/datalink/downlink.h"
#include "modules/loggers/flight_recorder.h"

#include "generated/flight_plan.h"

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
  #include "coef_systune_struct_armand.h"
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

// serial_act_t4 variables:
struct serial_act_t4_out myserial_act_t4_out_local;
float serial_act_t4_extra_data_out_local[255] __attribute__((aligned));
static abi_event SERIAL_ACT_T4_IN;

static abi_event SERIAL_ACT_T4_IN;
float serial_act_t4_extra_data_in_local[255] __attribute__((aligned));
struct serial_act_t4_in myserial_act_t4_in_local;


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
                 rc_x, rc_y, rc_z};
  pprz_msg_send_PAYLOAD_FLOAT(trans, dev, AC_ID, 35, f);
}
#endif

/**
 * ABI routine called by the serial_act_t4 ABI event
 */
static void serial_act_t4_abi_in(uint8_t sender_id __attribute__((unused)), struct serial_act_t4_in * myserial_act_t4_in_ptr, float * serial_act_t4_extra_data_in_ptr){
    memcpy(&myserial_act_t4_in_local,myserial_act_t4_in_ptr,sizeof(struct serial_act_t4_in));
    memcpy(&serial_act_t4_extra_data_in_local,serial_act_t4_extra_data_in_ptr,255 * sizeof(float));
}

#if PERIODIC_TELEMETRY
  static void esc_msg_send(struct transport_tx *trans, struct link_device *dev) {
    float current1, voltage1, rpm1, current2, voltage2, rpm2;
    if(autopilot.motors_on) { 
      
      current1 = (float)myserial_act_t4_in_local.motor_1_current_int * 0.01f;
      voltage1 = (float)myserial_act_t4_in_local.motor_1_voltage_int * 0.01f;
      rpm1 = (float)myserial_act_t4_in_local.motor_1_rpm_int;
     
      current2 = (float)myserial_act_t4_in_local.motor_2_current_int * 0.01f;
      voltage2 = (float)myserial_act_t4_in_local.motor_2_voltage_int * 0.01f;
      rpm2 = (float)myserial_act_t4_in_local.motor_2_rpm_int;
    }
    else{
      current1 = 0;
      voltage1 = 0;
      rpm1 = 0;
            
      current2 = 0;
      voltage2 = 0;
      rpm2 = 0;
    }
    float bat_voltage = electrical.vsupply;
    float power1 = current1 * bat_voltage;
    float power2 = current2 * bat_voltage;
    float energy1 = -1;
    float energy2 = -1;
     uint8_t i = 1; 
    pprz_msg_send_ESC(trans, dev, AC_ID,
          &current1,
          &bat_voltage,
          &power1,
          &rpm1,
          &voltage1,
          &energy1,
          &i);

      i += 1;
    pprz_msg_send_ESC(trans, dev, AC_ID,
      &current2,
      &bat_voltage,
      &power2,
      &rpm2,
      &voltage2,
      &energy2,
      &i);
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

  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PAYLOAD_FLOAT, send_payload_float);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ESC, esc_msg_send);
  #endif

  //Init abi bind msg to Teensy 4.0:
  AbiBindMsgSERIAL_ACT_T4_IN(ABI_BROADCAST, &SERIAL_ACT_T4_IN, serial_act_t4_abi_in);
  rc_x = 0;
  rc_y = 0;
  rc_z = 0;
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
     pos_target.x = 2.34;
     pos_target.y = 1.69;
     pos_target.z = -1.28;
  }
   
  

  struct FloatVect3 barth_rate, smeur_rate;
  smeur_rate.x = stateGetBodyRates_f()->p;
  smeur_rate.y = stateGetBodyRates_f()->q;
  smeur_rate.z = stateGetBodyRates_f()->r;
    
  float_quat_comp(&quat_att_barth_frame, stateGetNedToBodyQuat_f(), &quat_smeur_2_barth);
  float_quat_vmult(&barth_rate, &quat_smeur_2_barth, &smeur_rate);


   if(COEFF_DABB == 1 ){
    eps[0][0] = stateGetPositionNed_f()->x - pos_target.x; 
    eps[1][0] = stateGetPositionNed_f()->y - pos_target.y;
    eps[2][0] = stateGetPositionNed_f()->z - pos_target.z;
    eps[3][0] = stateGetSpeedNed_f()->x;
    eps[4][0] = stateGetSpeedNed_f()->y;
    eps[5][0] = stateGetSpeedNed_f()->z;
    eps[6][0] = quat_att_barth_frame.qx;
    eps[7][0] = quat_att_barth_frame.qz;
    eps[8][0] = barth_rate.x;
    eps[9][0] = barth_rate.y;
    eps[10][0] = barth_rate.z;
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
    eps[8][0] = -barth_rate.x;
    eps[9][0] = -barth_rate.y;
    eps[10][0] = -barth_rate.z;
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


  actuators_pprz[2]=TRIM_UPPRZ(u_scale[1][0]);
  actuators_pprz[3]=TRIM_UPPRZ(u_scale[0][0]); 

  
  
  if (in_flight) {
    stabilization_cmd[COMMAND_THRUST] = (actuators_pprz[2]+actuators_pprz[3]); // for in_flight detection
    //printf("in_flight\n");
  } else {
    stabilization_cmd[COMMAND_THRUST] = 1000;
    //printf("Not in_flight\n");
  };
  

}

void stabilization_fill_cmd(void){

  if(!autopilot.motors_on) { //kill only the motors, put the servos straight
    //Arm motor:
    myserial_act_t4_out_local.motor_arm_int = 0;
  }
  else{
    //Arm motor:
    myserial_act_t4_out_local.motor_arm_int = 1;
  }

  if (myserial_act_t4_out_local.motor_arm_int == 1){
    myserial_act_t4_out_local.motor_1_dshot_cmd_int = 2000*actuators_pprz[3]/MAX_PPRZ; //Left motor
    myserial_act_t4_out_local.motor_2_dshot_cmd_int = 2000*actuators_pprz[2]/MAX_PPRZ; //Right motor
  }
  else{
    myserial_act_t4_out_local.motor_1_dshot_cmd_int =  (int16_t) (0); 
    myserial_act_t4_out_local.motor_2_dshot_cmd_int =  (int16_t) (0); 

  }
  
  
  AbiSendMsgSERIAL_ACT_T4_OUT(ABI_SERIAL_ACT_T4_OUT_ID, &myserial_act_t4_out_local, &serial_act_t4_extra_data_out_local[0]);
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

