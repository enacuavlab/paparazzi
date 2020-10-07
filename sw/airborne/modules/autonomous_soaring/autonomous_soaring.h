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
 * @file "modules/autonomous_soaring/autonomous_soaring.h"
 * @author Murat BRONZ
 * Soaring related stuff
 */

#include "std.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_geodetic_int.h"

#include "mcu_periph/sys_time.h"

#ifndef AUTONOMOUS_SOARING_H
#define AUTONOMOUS_SOARING_H

struct Smartprobe { 
	float va;
	float aoa;
	float beta;
	float q;
	float p;
	// int16_t va;
	// int16_t aoa;
	// int16_t beta;
	// int32_t q;
	// int32_t p;
};

struct Soaring_states {
  float Vx;          ///< GPS, Horizontal ground speed norm
  float Vz;          ///< GPS, Vertical speed
  float wx;          ///< in-plane horizontal wind component
  float wz;          ///< vertical wind component
  float va;          ///< airspeed
  float gama;        ///< flight path angle
  float aoa;         ///< angle of attack
  float theta;       ///< theta pitch angle
  //float theta_cmd;   ///< desired optimum pitch angle calculated from wind gradients
  uint32_t dt;       ///< time step
  float d_wx;
  float d_wz;
  float p_w;
};

struct Soaring_coeffs{
	int sample_nr;
	float aoa_offset;
	float aoa_coeff;
};

// example:
// extern float fault_right;
// extern void fault_Set_Right(float _v);


extern struct Soaring_states soaring_states;
extern struct Soaring_coeffs soaring_coeffs;
extern struct Smartprobe smartprobe;

// extern void autonomous_soaring_Set_Sampling(int _v)

extern void soaring_init(void);

extern void soaring_parse_AEROPROBE(uint8_t *buf);

extern void soaring_status_report(void);

extern void soaring_periodic(void);

// extern void soaring_datalink_callback();

#endif

