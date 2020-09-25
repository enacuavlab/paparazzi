/*
 * Copyright (C) 2018 Fabien Bonneval <fabien.bonneval@gmail.com>
 *                    Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file firmwares/rover/guidance/rover_guidance_holonomic.c
 *  Basic guidance for rover.
 *  Implement standard PID control loop to track a navigation target.
 *  Guidance "modes" are using the autopilot generation with the "guidance"
 *  state machine.
 */

#define AUTOPILOT_CORE_GUIDANCE_C

#include "firmwares/rover/guidance/rover_guidance_holonomic.h"
#include "generated/airframe.h"
#include "generated/autopilot_core_guidance.h"
#include "state.h"
#include "mcu_periph/sys_time.h"

struct RoverHoloGuidance rover_holo_guidance;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
// TODO rover guidance messages
#endif

void rover_holo_guidance_init(void)
{
  FLOAT_VECT2_ZERO(rover_holo_guidance.sp.pos);
  FLOAT_VECT2_ZERO(rover_holo_guidance.sp.speed);
  rover_holo_guidance.sp.heading = 0.0f;
  rover_holo_guidance.speed_pid.p = ROVER_HOLO_GUIDANCE_SPEED_PGAIN;
  rover_holo_guidance.speed_pid.i = ROVER_HOLO_GUIDANCE_SPEED_IGAIN;
  rover_holo_guidance.speed_pid.d = ROVER_HOLO_GUIDANCE_SPEED_DGAIN;
  rover_holo_guidance.speed_pid.err = 0.f;
  rover_holo_guidance.speed_pid.d_err = 0.f;
  rover_holo_guidance.speed_pid.sum_err = 0.f;
  rover_holo_guidance.turn_pid.p = ROVER_HOLO_GUIDANCE_TURN_PGAIN;
  rover_holo_guidance.turn_pid.i = ROVER_HOLO_GUIDANCE_TURN_IGAIN;
  rover_holo_guidance.turn_pid.d = ROVER_HOLO_GUIDANCE_TURN_DGAIN;
  rover_holo_guidance.turn_pid.err = 0.f;
  rover_holo_guidance.turn_pid.d_err = 0.f;
  rover_holo_guidance.turn_pid.sum_err = 0.f;
  
  rover_holo_guidance.max_speed = ROVER_HOLO_GUIDANCE_MAX_SPEED;
  rover_holo_guidance.accel = ROVER_HOLO_GUIDANCE_ACCEL;
  

#if PERIODIC_TELEMETRY
  // TODO register messages
#endif

  // from code generation
  autopilot_core_guidance_init();
}

void rover_holo_guidance_periodic(void)
{
  // from code generation
  autopilot_core_guidance_periodic_task();
}

static float compute_pid(struct RoverHoloGuidancePID *pid)
{
  return pid->p * pid->err + pid->d * pid->d_err + pid->i * pid->sum_err;
}

//#define MAX_POS_ERR       10.f // max position error
//#define MAX_SPEED_ERR     10.f // max speed error
//#define MAX_INTEGRAL_CMD  (MAX_PPRZ / 10.f) // 10% of max command
#define PROXIMITY_DIST    0.2f // proximity distance



void rover_holo_guidance_run(float *heading_sp)
{
  // compute position error
  struct FloatVect2 pos_err;
  VECT2_DIFF(pos_err, rover_holo_guidance.sp.pos, *stateGetPositionNed_f());
  float norm_pos_err = float_vect2_norm(&pos_err);

  float norm_speed = stateGetHorizontalSpeedNorm_f();
  
  float d_decel = (norm_speed*norm_speed) / (2.f* rover_holo_guidance.accel);   // distance to stop from current speed.
  
  float norm_speed_sp = float_vect2_norm(&rover_holo_guidance.sp.speed);
  
  if (norm_pos_err < PROXIMITY_DIST) {
      // reached the target, stop here !
      norm_speed_sp = 0.f;
  } else if(norm_pos_err < 2 * d_decel) {     // why "2*" ?
      
    // time to decel
    norm_speed_sp -= rover_holo_guidance.accel / PERIODIC_FREQUENCY;
    
    if (norm_speed_sp < 0.f) { norm_speed_sp = 0.f;}

      
  } else {
      // We have to accelerate (or maintain speed)
      norm_speed_sp += rover_holo_guidance.accel / PERIODIC_FREQUENCY;
      BoundAbs(norm_speed_sp, rover_holo_guidance.max_speed);
  }
  
  
  rover_holo_guidance.sp.speed.x = norm_speed_sp * pos_err.x / norm_pos_err;
  rover_holo_guidance.sp.speed.y = norm_speed_sp * pos_err.y / norm_pos_err;
  
  float cpsi = cosf(stateGetNedToBodyEulers_f()->psi);
  float spsi = sinf(stateGetNedToBodyEulers_f()->psi);

    struct FloatVect2 speed_sp_body = {
      .x  =  cpsi * rover_holo_guidance.sp.speed.x + spsi * rover_holo_guidance.sp.speed.y,
      .y  = -spsi * rover_holo_guidance.sp.speed.x + cpsi * rover_holo_guidance.sp.speed.y
    };
  
  
  rover_holo_guidance.cmd.motor_speed_x = TRIM_PPRZ(MAX_PPRZ * speed_sp_body.x);   // * 1.0
  rover_holo_guidance.cmd.motor_speed_y = TRIM_PPRZ(MAX_PPRZ * speed_sp_body.y);   // * 1.0
  
  
  
  
//  rover_holo_guidance.speed_pid.err = float_vect2_norm(&pos_err);
//  BoundAbs(rover_holo_guidance.speed_pid.err, MAX_POS_ERR);

//  // speed update when far enough
//  if (rover_holo_guidance.speed_pid.err > PROXIMITY_DIST) {
//    // compute speed error
//    rover_holo_guidance.speed_pid.d_err = - stateGetHorizontalSpeedNorm_f();
//    BoundAbs(rover_holo_guidance.speed_pid.d_err, MAX_SPEED_ERR);
//    // integral
//    rover_holo_guidance.speed_pid.sum_err = 0.f; // nothing for now
//    // run PID
//    float speed = MAX_PPRZ * compute_pid(&rover_holo_guidance.speed_pid);
//    speed = TRIM_PPRZ(speed);
//
//    float cpsi = cosf(stateGetNedToBodyEulers_f()->psi);
//    float spsi = sinf(stateGetNedToBodyEulers_f()->psi);
//    struct FloatVect2 err_body = {
//      .x  =  cpsi * pos_err.x + spsi * pos_err.y,
//      .y  = -spsi * pos_err.x + cpsi * pos_err.y
//    };
//    // command in rover frame = speed * normalized direction vector
//    // TODO normalize to min/max speed with pprz_t scale
//    rover_holo_guidance.cmd.motor_speed_x = speed * err_body.x / rover_holo_guidance.speed_pid.err;
//    rover_holo_guidance.cmd.motor_speed_y = speed * err_body.y / rover_holo_guidance.speed_pid.err;
//  } else {
//    rover_holo_guidance.cmd.motor_speed_x = 0.f;
//    rover_holo_guidance.cmd.motor_speed_y = 0.f;
//  }
  
  
  
  
  
  
  

  rover_holo_guidance.sp.heading = *heading_sp;
  // angular error
  rover_holo_guidance.turn_pid.err = rover_holo_guidance.sp.heading - stateGetNedToBodyEulers_f()->psi;
  NormRadAngle(rover_holo_guidance.turn_pid.err);
  // turn rate error
  rover_holo_guidance.turn_pid.d_err = - stateGetBodyRates_f()->r;
  // integral
  rover_holo_guidance.turn_pid.sum_err = 0.f; // nothing for now
  // run PID
  rover_holo_guidance.cmd.motor_turn = MAX_PPRZ * compute_pid(&rover_holo_guidance.turn_pid);
  rover_holo_guidance.cmd.motor_turn = TRIM_PPRZ(rover_holo_guidance.cmd.motor_turn);
  
  //last_time = time;
}

void rover_holo_guidance_enter(void)
{
  ClearBit(rover_holo_guidance.sp.mask, 5);
  ClearBit(rover_holo_guidance.sp.mask, 7);

  rover_holo_guidance.sp.heading = stateGetNedToBodyEulers_f()->psi;
   
  //FLOAT_VECT2_ZERO(rover_holo_guidance.sp.speed);
  
  rover_holo_guidance.sp.speed.x = stateGetSpeedNed_f()->x;
  rover_holo_guidance.sp.speed.y = stateGetSpeedNed_f()->y;

  // TODO reset integral part ?
}

void rover_guidance_holonomic_set_speed_igain(float igain)
{
  rover_holo_guidance.speed_pid.i = igain;
  rover_holo_guidance.speed_pid.sum_err = 0.f;
}

void rover_guidance_holonomic_set_turn_igain(float igain)
{
  rover_holo_guidance.turn_pid.i = igain;
  rover_holo_guidance.turn_pid.sum_err = 0.f;
}


