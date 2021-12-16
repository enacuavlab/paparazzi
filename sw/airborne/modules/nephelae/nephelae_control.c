/*
 * Copyright (C) Carbarbaye/Verdu
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/nephelae/nephelae_control.c"
 * @author Carbarbaye/Verdu
 * Control based on the nephelae_observer and so the airplane caracteristics
 */

#include "modules/nephelae/nephelae_control.h"

#include <stdio.h>
#include <math.h>
#include "modules/core/abi.h"
#include "std.h"
#include "autopilot.h"
#include "math/pprz_algebra_int.h"
#include "modules/datalink/telemetry.h"

/** ABI binding for GPS altitude
*/
#ifndef GPS_ID
#define GPS_ID ABI_BROADCAST
#endif
static abi_event gps_ev;


#define PI 3.14159265358979323846

struct CtrlVect neph_ctrl_state;

uint8_t Obs;
uint8_t Ctrl;

uint32_t c_time = 0;

float x_u = 0.f;
float x_v = 0.f;
float x_w = 0.f;
float x_p = 0.f;
float x_q = 0.f;
float x_r = 0.f;
float x_phi = 0.f;
float x_theta = 0.f;
float delta_u_elevon = 0.f;
float delta_u_aileron = 0.f;
float delta_u_thottle = 0.f;

float trim_elevator;
float trim_aileron;
float trim_thrust;
float speed_rate;

float gravity = 9.81;

int32_t alt_gps = 0;

float neph_temp;
float neph_press;
float neph_ro;

float neph_m = NEPH_M;

float neph_h_ctl_roll_setpoint;
pprz_t neph_h_ctl_aileron_setpoint;

float  neph_h_ctl_pitch_setpoint;
pprz_t neph_h_ctl_elevator_setpoint;

pprz_t neph_v_ctl_throttle_setpoint;
pprz_t neph_v_ctl_power_setpoint;

float neph_gamma;

float flap_tmp = 0.f;
float aileron_tmp = 0.f;
float thrust_tmp = 0.f;

float tension_tmp = 0.f;
float intensity_tmp = 0.f;
float omega_tmp;

float ailevon_left_tmp = 0.f;
float ailevon_right_tmp = 0.f;

float aileron_demod = 0.f;
float flap_demod = 0.f;

int16_t command_aileron = 0;
int16_t command_flap = 0;
int16_t command_tension = 0;

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                    uint32_t stamp __attribute__((unused)), struct GpsState *gps_s){

	alt_gps = (float)gps_s->hmsl;
	neph_temp = 288.25 - (6.5/1000.0) * (alt_gps/1000.0);
	neph_press = 101325.0*powf((neph_temp/288.0),5.255);
	neph_ro = neph_press/(287.05*neph_temp); // Calcul de ro en fonction de l'altitude GPS
}

static void send_neph_debug(struct transport_tx *trans, struct link_device *dev)
{
	pprz_msg_send_NEPHELAE_TEST_DEBUG(trans, dev, AC_ID,
										&neph_obs_state.gyro_time, &neph_obs_state.acc_time, &neph_obs_state.debug, &neph_obs_state.airspeed.x, &neph_obs_state.airspeed.y, &neph_obs_state.airspeed.z,
										&neph_obs_state.rotspeed.p, &neph_obs_state.rotspeed.q, &neph_obs_state.rotspeed.r,
										&neph_obs_state.attitude.phi, &neph_obs_state.attitude.theta,
										&neph_ctrl_state.aileron, &neph_ctrl_state.flap, &neph_ctrl_state.v_motor);
}

void gamma_process(void){
	float gamma_tmp;
	neph_gamma = neph_h_ctl_pitch_setpoint;

	float ratio = (neph_ro*powf(neph_obs_state.airspeed.x,2.0)*NEPH_SCLMAX*cos(neph_obs_state.attitude.phi))/(2.0*NEPH_M*gravity);

	if (ratio < 0.0){
		ratio = 1.0;
	}

	if(ratio < 1.0){
		gamma_tmp = -acos(ratio);
		if (gamma_tmp < neph_gamma){
			neph_gamma = gamma_tmp;
		}
	}
	
}

void control_law(void){
	flap_tmp = 0.f;
	aileron_tmp = 0.f;
	thrust_tmp = 0.f;

	float neph_kc1[8] = NEPH_KC1;
	float neph_g1[3] = NEPH_G1;
	float neph_kc2[8] = NEPH_KC2;
	float neph_g2[3] = NEPH_G2;
	float neph_kc3[8] = NEPH_KC3;
	float neph_g3[3] = NEPH_G3;

	flap_tmp += -neph_kc1[0] * neph_obs_state.airspeed.x;
	flap_tmp += -neph_kc1[1] * neph_obs_state.airspeed.y;
	flap_tmp += -neph_kc1[2] * neph_obs_state.airspeed.z;
	flap_tmp += -neph_kc1[3] * neph_obs_state.rotspeed.p;
	flap_tmp += -neph_kc1[4] * neph_obs_state.rotspeed.q;
	flap_tmp += -neph_kc1[5] * neph_obs_state.rotspeed.r;
	flap_tmp += -neph_kc1[6] * neph_obs_state.attitude.phi;
	flap_tmp += -neph_kc1[7] * neph_obs_state.attitude.theta;

	flap_tmp += neph_g1[0] * neph_v_ctl_throttle_setpoint;
	flap_tmp += neph_g1[1] * neph_h_ctl_roll_setpoint;
	flap_tmp += neph_g1[2] * neph_gamma;

	aileron_tmp += -neph_kc2[0] * neph_obs_state.airspeed.x;
	aileron_tmp += -neph_kc2[1] * neph_obs_state.airspeed.y;
	aileron_tmp += -neph_kc2[2] * neph_obs_state.airspeed.z;
	aileron_tmp += -neph_kc2[3] * neph_obs_state.rotspeed.p;
	aileron_tmp += -neph_kc2[4] * neph_obs_state.rotspeed.q;
	aileron_tmp += -neph_kc2[5] * neph_obs_state.rotspeed.r;
	aileron_tmp += -neph_kc2[6] * neph_obs_state.attitude.phi;
	aileron_tmp += -neph_kc2[7] * neph_obs_state.attitude.theta;

	aileron_tmp += neph_g2[0] * neph_v_ctl_throttle_setpoint;
	aileron_tmp += neph_g2[1] * neph_h_ctl_roll_setpoint;
	aileron_tmp += neph_g2[2] * neph_gamma;

	thrust_tmp += -neph_kc3[0] * neph_obs_state.airspeed.x;
	thrust_tmp += -neph_kc3[1] * neph_obs_state.airspeed.y;
	thrust_tmp += -neph_kc3[2] * neph_obs_state.airspeed.z;
	thrust_tmp += -neph_kc3[3] * neph_obs_state.rotspeed.p;
	thrust_tmp += -neph_kc3[4] * neph_obs_state.rotspeed.q;
	thrust_tmp += -neph_kc3[5] * neph_obs_state.rotspeed.r;
	thrust_tmp += -neph_kc3[6] * neph_obs_state.attitude.phi;
	thrust_tmp += -neph_kc3[7] * neph_obs_state.attitude.theta;

	thrust_tmp += neph_g3[0] * neph_v_ctl_throttle_setpoint;
	thrust_tmp += neph_g3[1] * neph_h_ctl_roll_setpoint;
	thrust_tmp += neph_g3[2] * neph_gamma; 
}

void control_trim(void){
	flap_tmp += trim_elevator;
	aileron_tmp += trim_aileron;
	thrust_tmp += trim_thrust;
}

void control_voltage(void){
	float A = NEPH_KFW;
	float B = (NEPH_KFX * (neph_obs_state.airspeed.x + NEPH_V0));
	float C = -(thrust_tmp * NEPH_M);
	float delta_tmp = powf(B,2) - (4*A*C);
	omega_tmp = 0.f;

	if (delta_tmp > 0.0){
		omega_tmp = (-B + sqrt(delta_tmp))/(2*A);
	}
	else{
		omega_tmp = -B /(2*A);
	}
}

void control_saturation_motor(void){
	intensity_tmp = (NEPH_KV)*((NEPH_KQW*powf(omega_tmp,2)) + 
					(NEPH_KQU*powf((neph_obs_state.airspeed.x+NEPH_V0),2)) + (NEPH_KQX*omega_tmp*(neph_obs_state.airspeed.x+NEPH_V0)));

	if( intensity_tmp > NEPH_IMAXMOTOR){
		intensity_tmp = NEPH_IMAXMOTOR;
	}

	tension_tmp = (omega_tmp/NEPH_KV) + (NEPH_R* intensity_tmp) + (NEPH_R*NEPH_I0);

	if(tension_tmp > vsupply){
		tension_tmp = (float)vsupply;
	}
	if(tension_tmp < 0){
		tension_tmp = 0.f;
	}
}

void control_saturation_actuator(void){
	float neph_ail_limit[2] = NEPH_LIMITAILERON;

	ailevon_left_tmp = flap_tmp + CTRL_SIGNL * aileron_tmp;
	ailevon_right_tmp = flap_tmp + CTRL_SIGNR * aileron_tmp;

	if(ailevon_left_tmp > neph_ail_limit[1]){
		ailevon_left_tmp = neph_ail_limit[1];
	}
	if(ailevon_left_tmp < neph_ail_limit[0]){
		ailevon_left_tmp = neph_ail_limit[0];
	}
	if(ailevon_right_tmp > neph_ail_limit[1]){
		ailevon_right_tmp = neph_ail_limit[1];
	}
	if(ailevon_right_tmp < neph_ail_limit[0]){
		ailevon_right_tmp = neph_ail_limit[0];
	}
}

void control_demodulation_actuator(void){
	aileron_demod = (ailevon_left_tmp - ailevon_right_tmp) / (CTRL_SIGNL - CTRL_SIGNR);
	flap_demod = ailevon_left_tmp - CTRL_SIGNL * aileron_tmp;
}

void control_actuator_convertion(void){
	float neph_ail_limit[2] = NEPH_LIMITAILERON;

	float a = (MAX_PPRZ - MIN_PPRZ) / (neph_ail_limit[1] - neph_ail_limit[0]);
	neph_h_ctl_aileron_setpoint = (pprz_t)(a * (aileron_demod - neph_ail_limit[0]));
	neph_h_ctl_elevator_setpoint = (pprz_t)(a * (flap_demod - neph_ail_limit[0]));

	neph_v_ctl_power_setpoint = (pprz_t)(tension_tmp / (float) vsupply);
}

void neph_h_ctl_attitude_loop(void){
	gamma_process();
	control_law();
	control_trim();
	control_voltage();
	control_saturation_motor();
	control_saturation_actuator();
	control_demodulation_actuator();
	control_actuator_convertion();

	neph_ctrl_state.aileron = aileron_demod;
	neph_ctrl_state.flap = flap_demod;
	neph_ctrl_state.v_motor = tension_tmp;
}

void nephelae_control_init(void) {

	neph_h_ctl_roll_setpoint = 0.f;
	neph_h_ctl_aileron_setpoint = 0;
	neph_h_ctl_pitch_setpoint = 0.f;
	neph_h_ctl_elevator_setpoint = 0;
	neph_v_ctl_throttle_setpoint = 0.f;
	neph_v_ctl_power_setpoint = 0;

	neph_ctrl_state.aileron = 0.f;
	neph_ctrl_state.flap = 0.f;
	neph_ctrl_state.v_motor = 0.f;

	trim_elevator = 0.f;
	trim_aileron = 0.f;
	trim_thrust = 0.f;

	AbiBindMsgGPS(GPS_ID, &gps_ev, gps_cb);

	#if PERIODIC_TELEMETRY
  	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_NEPHELAE_TEST_DEBUG, send_neph_debug);
	#endif

}


