/*
 * Copyright (C) 2025 Gautier Hattenberger <gautier.hattenberger.fr>
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

// TODO :
// Next flight test
// Find the flight speed
// Remove Yaw control
// Remettre controle elevon fonction de la vitesse



// Remove flag_forward
// NUM_ACT name
// Retour eff_schedu_thruster_X int16 ?

//Guidage :
// Merge eff_schedu_thruster_X function with periodic
// Guidage command transition ratio 
// Orientation face au vent : guidage




// Airspeed at which pusher motor is on/off
#ifndef EFF_SCHEDULING_QUADPLANE_PUSHER_AIRSPEED
#define EFF_SCHEDULING_QUADPLANE_PUSHER_AIRSPEED 2.0f
#endif

// Airspeed at which vertical motors are on/off
#ifndef EFF_SCHEDULING_QUADPLANE_LOW_AIRSPEED
#define EFF_SCHEDULING_QUADPLANE_LOW_AIRSPEED 17.0f
#endif

// Pusher motor rate for transition
#ifndef EFF_SCHEDULING_QUADPLANE_PUSHER_MOTOR_RATE
#define EFF_SCHEDULING_QUADPLANE_PUSHER_MOTOR_RATE 6000
#endif



#ifdef STABILIZATION_INDI_G1
static float g1g2_hover[STABILIZATION_INDI_OUTPUTS][STABILIZATION_INDI_NUM_ACT] = STABILIZATION_INDI_G1;
#else
static float g1g2_hover[STABILIZATION_INDI_OUTPUTS][STABILIZATION_INDI_NUM_ACT] = {
  STABILIZATION_INDI_G1_ROLL,
  STABILIZATION_INDI_G1_PITCH,
  STABILIZATION_INDI_G1_YAW,
  STABILIZATION_INDI_G1_THRUST,
  STABILIZATION_INDI_G1_THRUST_X
};
#endif


#ifndef STABILIZATION_INDI_G2
#error "You must define STABILIZATION_INDI_G2 matrix for the module eff_scheduling_quadplane"
#else
static float g2[STABILIZATION_INDI_NUM_ACT] = STABILIZATION_INDI_G2;
#endif

#define MODE_FULL_RC
int flag_forward = 0; // TODO : To be erase
struct MedianFilterInt airspeed_quadplane_sched_fltr;




void eff_scheduling_quadplane_init(void)
{
  for (int8_t i = 0; i < STABILIZATION_INDI_OUTPUTS; i++) {
    for (int8_t j = 0; j < STABILIZATION_INDI_NUM_ACT; j++) {
      if(i != 2) {  
        g1g2[i][j] = g1g2_hover[i][j] / INDI_G_SCALING;
      } else {
        //We add the G2 vector to the G1 Matrix here on the YAW line
        g1g2_hover[i][j] = g1g2_hover[i][j] + g2[j];
        g1g2[i][j] = g1g2_hover[i][j] / INDI_G_SCALING;
      }  
    }
  }

    init_median_filter_i(&airspeed_quadplane_sched_fltr, MEDIAN_DEFAULT_SIZE);
}

void eff_scheduling_quadplane_periodic(void)
{
  // Get airspeed with a median filter
  float airspeed_raw = stateGetAirspeed_f();
  float airspeed = update_median_filter_i(&airspeed_quadplane_sched_fltr,airspeed_raw);


#ifdef MODE_FULL_RC 
	if ((autopilot_get_mode() == AP_MODE_FORWARD && airspeed > EFF_SCHEDULING_QUADPLANE_LOW_AIRSPEED) || (flag_forward == 1 && autopilot_get_mode() != AP_MODE_ATTITUDE_DIRECT)) { // TODO : To be erase	
#else
	if (autopilot_get_mode() == AP_MODE_FORWARD && airspeed > EFF_SCHEDULING_QUADPLANE_LOW_AIRSPEED) { // The correct one for after
#endif	  

	// TODO : To be erase
	flag_forward = 1;    
    
    // turn off vertical motors
    for (int8_t i = 0; i < 4; i++) {
      for (int8_t j = 0; j < 4; j++) {
        if(i != 2) {    // TODO We keep the control on YAW for now
			g1g2[i][j] = 0;
		}
      }
    }

    // elevon function of airspeed
    float offset_airspeed = airspeed - EFF_SCHEDULING_QUADPLANE_LOW_AIRSPEED; //offset for start eff at zero!
    if(offset_airspeed < 0) {offset_airspeed =0;}
    Bound(offset_airspeed, 0.0f, 30.0f);
    float airspeed2 = offset_airspeed * offset_airspeed;

    //float roll_eff = EFF_SCHEDULING_QUADPLANE_ROLL * airspeed2;
    float roll_eff = 3.0f; //EFF_SCHEDULING_QUADPLANE_ROLL * airspeed2;
    g1g2[0][4] = roll_eff / INDI_G_SCALING; // elevon_right
    g1g2[0][5] = roll_eff / INDI_G_SCALING; // elevon_left

    //float pitch_eff = EFF_SCHEDULING_QUADPLANE_PITCH * airspeed2;
    float pitch_eff = 3.0f; 
    g1g2[1][4] =  pitch_eff / INDI_G_SCALING; // elevon_right
    g1g2[1][5] = -pitch_eff / INDI_G_SCALING; // elevon_left


    //g1g2[4][6] = EFF_SCHEDULING_QUADPLANE_THRUST_X / INDI_G_SCALING; // pusher motor
  }
#ifdef MODE_FULL_RC   
	//else if ((airspeed <= EFF_SCHEDULING_QUADPLANE_LOW_AIRSPEED && flag_forward == 0) || autopilot_get_mode() == AP_MODE_ATTITUDE_DIRECT){ // TODO : To be erase	  
	else if (autopilot_get_mode() == AP_MODE_ATTITUDE_DIRECT){ // TODO : To be erase	  
#else
	else if (airspeed <= EFF_SCHEDULING_QUADPLANE_LOW_AIRSPEED || autopilot_get_mode() == AP_MODE_ATTITUDE_DIRECT){ // The correct one for after
#endif	
	  
	// TODO : To be erase
	flag_forward = 0;   	  
	  
    // turn on vertical motorsWLS_PRIORITIES
    for (int8_t i = 0; i < 4; i++) {
      for (int8_t j = 0; j < 4; j++) {
        g1g2[i][j] = g1g2_hover[i][j] / INDI_G_SCALING;
      }
    }

    //Come back to motor control
    g1g2[0][4] = 0; // elevon_right
    g1g2[0][5] = 0; // elevon_left

    g1g2[1][4] = 0; // elevon_right
    g1g2[1][5] = 0; // elevon_left
    
    
    
        
    

    //if (airspeed < EFF_SCHEDULING_QUADPLANE_PUSHER_AIRSPEED) {
    //  g1g2[4][6] = 0; // pusher motor
    //} else {
    //  g1g2[4][6] = EFF_SCHEDULING_QUADPLANE_THRUST_X / INDI_G_SCALING; // pusher motor
    //}
  }
}


int eff_scheduling_thruster_X(void)
{			
	int thrust_x = 0;
	
	if(autopilot_get_mode() == AP_MODE_ATTITUDE_DIRECT){
		thrust_x = 0;
	}
	
	else if(autopilot_get_mode() == AP_MODE_FORWARD) {
		
    // Airspeed with a median filter
    float airspeed_raw = stateGetAirspeed_f();
    float airspeed = update_median_filter_i(&airspeed_quadplane_sched_fltr,airspeed_raw);
    
		//This condition to be piloting the X throttle and the attitude in the same time
#ifdef MODE_FULL_RC   
		if(stabilization.transition_ratio < 1.0 || (airspeed <= EFF_SCHEDULING_QUADPLANE_LOW_AIRSPEED && flag_forward == 0)){ // TODO : To be erase	 
#else
		if(stabilization.transition_ratio < 1.0 || airspeed <= EFF_SCHEDULING_QUADPLANE_LOW_AIRSPEED){ // The correct one for after
#endif	
				
			// We use the transition_ratio to make a smooth start of the pusher from 0 to EFF_SCHEDULING_QUADPLANE_PUSHER_MOTOR_RATE %
			thrust_x = (int) (EFF_SCHEDULING_QUADPLANE_PUSHER_MOTOR_RATE * stabilization.transition_ratio);
		} else {
			thrust_x = (int)radio_control_get(RADIO_THROTTLE);
		}
		
	}
	
	else if(autopilot_get_mode() == AP_MODE_NAV) {
		thrust_x = 0; //TODO LATER
	}
	
	return thrust_x;
}

extern void eff_scheduling_quadplane_report(void)
{
  float f[6] = {
    g1g2[1][4], g1g2[1][5],
    g1g2[2][4], g1g2[2][5],
    g1g2[1][0], g1g2[2][0]
  };
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 6, f);
}

