/*
 * Copyright (C) Florian Sansou <fl.sansou@enac.fr>
 * ENAC uav lab
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file stabilization_udwadia.cpp
 * @brief 
 *
 */

#include "firmwares/rotorcraft/stabilization/stabilization_udwadia.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"

#include "math/pprz_algebra_float.h"
#include "state.h"
#include "generated/airframe.h"
#include "modules/radio_control/radio_control.h"
#include "modules/actuators/actuators.h"
#include "modules/core/abi.h"
#include <stdio.h>





// variables needed for control



struct Int32Eulers stab_att_sp_euler;
struct Int32Quat   stab_att_sp_quat;
struct FloatRates  stab_att_ff_rates;

// Register actuator feedback if we rely on RPM information
#if STABILIZATION_INDI_RPM_FEEDBACK
#ifndef STABILIZATION_INDI_ACT_FEEDBACK_ID
#define STABILIZATION_INDI_ACT_FEEDBACK_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(STABILIZATION_INDI_ACT_FEEDBACK_ID)

abi_event act_feedback_ev;
static void act_feedback_cb(uint8_t sender_id, struct act_feedback_t *feedback, uint8_t num_act);
PRINT_CONFIG_MSG("STABILIZATION_INDI_RPM_FEEDBACK")
#endif




struct FloatVect3 body_accel_f;



#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_ahrs_ref_quat(struct transport_tx *trans, struct link_device *dev)
{
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  pprz_msg_send_AHRS_REF_QUAT(trans, dev, AC_ID,
                              &stab_att_sp_quat.qi,
                              &stab_att_sp_quat.qx,
                              &stab_att_sp_quat.qy,
                              &stab_att_sp_quat.qz,
                              &(quat->qi),
                              &(quat->qx),
                              &(quat->qy),
                              &(quat->qz));
}


#endif

/**
 * Function that initializes 
 */
void stabilization_udwadia_init(void)
{
 

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_REF_QUAT, send_ahrs_ref_quat);
#endif
}

/**

 */
void stabilization_udwadia_enter(void)
{
  /* reset psi setpoint to current psi angle */
  stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();

}


/**
 * @param in_flight boolean that states if the UAV is in flight or not
 * @param sp rate setpoint
 * @param cmd output command array
 *
 * Function that calculates the commands
 */
void stabilization_udwadia_run(bool in_flight, struct StabilizationSetpoint *sp, int32_t *cmd)
{
  
}



// This function reads rc commands
void stabilization_udwadia_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn)
{
  struct FloatQuat q_sp;
#if USE_EARTH_BOUND_RC_SETPOINT
  stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#else
  stabilization_attitude_read_rc_setpoint_quat_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#endif

  QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);
}

/**
 * Function that tries to get actuator feedback.
 *
 * If this is not available it will use a first order filter to approximate the actuator state.
 * It is also possible to model rate limits (unit: PPRZ/loop cycle)
 */
void get_actuator_state(void)
{

  float_vect_copy(actuator_state, act_obs, INDI_NUM_ACT);

}
