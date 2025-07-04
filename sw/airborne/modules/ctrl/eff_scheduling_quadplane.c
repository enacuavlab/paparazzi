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

#include "modules/actuators/actuators.h"


#include "modules/datalink/downlink.h"

// Vecteur command airframe : OK
// Comment commander moteur pusher et setter X thrust : OK
// autopilot_get_mode (see autopilot.h) : OK
// Effective scheduling plioter : transition_ratio (stabilization.c) (param TRANSITION_TIME) : OK


// Retour eff_schedu_thruster_X int16 ?
// Merge eff_schedu_thruster_X function with periodic
// Guidage command transition ratio 
// Orientation face au vent : guidage
// TODO Filtering airspeed


// Airspeed at which pusher motor is on/off
#ifndef EFF_SCHEDULING_QUADPLANE_PUSHER_AIRSPEED
#define EFF_SCHEDULING_QUADPLANE_PUSHER_AIRSPEED 2.0f
#endif

// Airspeed at which vertical motors are on/off
#ifndef EFF_SCHEDULING_QUADPLANE_LOW_AIRSPEED
#define EFF_SCHEDULING_QUADPLANE_LOW_AIRSPEED 10.0f
#endif

// Pusher motor rate for transition
#ifndef EFF_SCHEDULING_QUADPLANE_PUSHER_MOTOR_RATE
#define EFF_SCHEDULING_QUADPLANE_PUSHER_MOTOR_RATE 3000
#endif



#ifdef STABILIZATION_INDI_G1
static float g1g2_hover[INDI_OUTPUTS][INDI_NUM_ACT] = STABILIZATION_INDI_G1;
#else
static float g1g2_hover[INDI_OUTPUTS][INDI_NUM_ACT] = {
  STABILIZATION_INDI_G1_ROLL,
  STABILIZATION_INDI_G1_PITCH,
  STABILIZATION_INDI_G1_YAW,
  STABILIZATION_INDI_G1_THRUST,
  STABILIZATION_INDI_G1_THRUST_X
};
#endif

void eff_scheduling_quadplane_init(void)
{
  for (int8_t i = 0; i < INDI_OUTPUTS; i++) {
    for (int8_t j = 0; j < INDI_NUM_ACT; j++) {
        g1g2[i][j] = g1g2_hover[i][j] / INDI_G_SCALING;
    }
  }
}

void eff_scheduling_quadplane_periodic(void)
{
  // calculate squared airspeed
  float airspeed = stateGetAirspeed_f();

  if (autopilot_get_mode() == AP_MODE_FORWARD && airspeed > EFF_SCHEDULING_QUADPLANE_LOW_AIRSPEED) { //Only forward mode  && autopilot_get_mode() == AP_MODE_FORWARD


    // turn off vertical motors
    for (int8_t i = 0; i < 4; i++) {
      for (int8_t j = 0; j < 4; j++) {
        g1g2[i][j] = 0;
      }
    }

    // elevon function of airspeed
    float offset_airspeed = airspeed - EFF_SCHEDULING_QUADPLANE_LOW_AIRSPEED; //offset for start eff at zero!
    Bound(offset_airspeed, 0.0f, 30.0f);
    float airspeed2 = offset_airspeed * offset_airspeed;

    float roll_eff = EFF_SCHEDULING_QUADPLANE_ROLL * airspeed2;
    g1g2[0][4] = roll_eff / INDI_G_SCALING; // elevon_right
    g1g2[0][5] = roll_eff / INDI_G_SCALING; // elevon_left

    float pitch_eff = EFF_SCHEDULING_QUADPLANE_PITCH * airspeed2;
    g1g2[1][4] =  pitch_eff / INDI_G_SCALING; // elevon_right
    g1g2[1][5] = -pitch_eff / INDI_G_SCALING; // elevon_left


    //g1g2[4][6] = EFF_SCHEDULING_QUADPLANE_THRUST_X / INDI_G_SCALING; // pusher motor
  }
  else if (airspeed <= EFF_SCHEDULING_QUADPLANE_LOW_AIRSPEED || autopilot_get_mode() == AP_MODE_ATTITUDE_DIRECT){
    // turn on vertical motors
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
		
		float airspeed = stateGetAirspeed_f();
		//This condition to be piloting the X throttle and the attitude in the same time
		if(stabilization.transition_ratio < 1.0 || airspeed <= EFF_SCHEDULING_QUADPLANE_LOW_AIRSPEED){
			thrust_x = EFF_SCHEDULING_QUADPLANE_PUSHER_MOTOR_RATE;
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

