/*
 * Copyright (C) 2025 Ramon Revilla Bouso <ramonrevilla21@gmail.com>
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

/** @file "modules/ctrl/eff_scheduling_heewing.h"
 * @author Ramon Revilla Bouso <>
 * The control effectiveness scheduler for the Heewing T1 Ranger VTOL
 */

#ifndef CTRL_EFF_SCHED_HEEWING_H
#define CTRL_EFF_SCHED_HEEWING_H

#include "std.h"
 
#define ESH_EFF_MAT_COLS_NB 7
#define ESH_EFF_MAT_ROWS_NB 5

#define ESH_P 0 // X body axis (angular acceleration)
#define ESH_Q 1 // Y body axis (angular acceleration)
#define ESH_R 2 // Z body axis (angular acceleration)
#define ESH_W 3 // Z body axis (linear acceleration) 
#define ESH_U 4 // X body axis (linear acceleration)

#define ESH_CMD_MOTORR 0 // Motor Right
#define ESH_CMD_MOTORL 1 // Motor Left
#define ESH_CMD_MOTORB 2 // Motor Back
#define ESH_CMD_MOTORMT 3 // Motor Mean Tilt
#define ESH_CMD_MOTORTD 4 // Motor Tilt Diff
#define ESH_CMD_AILERONS 5 // Aileron
#define ESH_CMD_ELEVATOR 6 // Elevator

#define ESH_CMD_ROLL 0 // Roll 
#define ESH_CMD_PITCH 1 // Pitch 
#define ESH_CMD_FZ 2 // Lift
#define ESH_CMD_FX 3 // Thrust 


extern float G2[ESH_EFF_MAT_COLS_NB]                               ;
extern float G1[ESH_EFF_MAT_ROWS_NB][ESH_EFF_MAT_COLS_NB]          ; 
extern float ESH_EFF_MAT[ESH_EFF_MAT_ROWS_NB][ESH_EFF_MAT_COLS_NB] ;

struct T1_attitude{
    float phi;
    float theta;
    float psi;
    float sphi;
    float cphi;
    float stheta;
    float ctheta;
    float spsi;
    float cpsi;
  };

  struct T1_tilt{
    float rad;             // Tilt angle in radians: from ABI message
    float deg;             // Tilt angle in degrees: (clone in degrees)
    float cosr;                 // cosine of motor tilt angle
    float sinr;                 // sine of motor tilt angle
  };

  struct T1_motor{
    float T; // Thrust [N]
    float dX; // Distance from the center of mass to the motor in the X axis
    float dY; // Distance from the center of mass to the motor in the Y axis
  };

struct T1_Model{
    float wls_min_mt;

    float I_XX;
    float I_YY;
    float I_ZZ;
    struct T1_attitude att;
    struct T1_tilt tiltl;
    struct T1_tilt tiltr;
    struct T1_motor mR;
    struct T1_motor mL;
    struct T1_motor mB;    

    float aero_coeff;
    float chord;
    float span;

    float as;  // airspeed [m/s] 
    float as2; // airspeed squared [m/s²]
};

extern void eff_scheduling_heewing_init(void);
extern void eff_scheduling_heewing_periodic(void);

extern struct T1_Model T1;
extern float esh_fake_airspeed;
 
#endif  // CTRL_EFF_SCHED_HEEWING_H