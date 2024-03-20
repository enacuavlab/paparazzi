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

#include <Eigen/Dense> // https://eigen.tuxfamily.org/dox/GettingStarted.html



#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

// variables needed for control


#ifndef STABILIZATION_UDWADIA_IX
#define STABILIZATION_UDWADIA_IX 0.04
#endif

#ifndef STABILIZATION_UDWADIA_IY
#define STABILIZATION_UDWADIA_IY 0.04
#endif

#ifndef STABILIZATION_UDWADIA_IZ
#define STABILIZATION_UDWADIA_IZ 0.08
#endif

#ifndef STABILIZATION_UDWADIA_D1
#define STABILIZATION_UDWADIA_D1 6.3
#endif

#ifndef STABILIZATION_UDWADIA_D2
#define STABILIZATION_UDWADIA_D2 20
#endif

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



Eigen::Vector3f Fc;
Eigen::Vector4f yaw_quat;


Eigen::Matrix4f J;
J << 1, 0, 0, 0,
     0, STABILIZATION_UDWADIA_IX, 0, 0,
     0, 0, STABILIZATION_UDWADIA_IY, 0,
     0, 0, 0, STABILIZATION_UDWADIA_IZ;


void qtoe(Eigen::Matrix4f &h, Eigen::Vector4f q);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

// static void send_ahrs_ref_quat(struct transport_tx *trans, struct link_device *dev)
// {
//   struct Int32Quat *quat = stateGetNedToBodyQuat_i();
//   pprz_msg_send_AHRS_REF_QUAT(trans, dev, AC_ID,
//                               &stab_att_sp_quat.qi,
//                               &stab_att_sp_quat.qx,
//                               &stab_att_sp_quat.qy,
//                               &stab_att_sp_quat.qz,
//                               &(quat->qi),
//                               &(quat->qx),
//                               &(quat->qy),
//                               &(quat->qz));
// }


#endif

/**
 * Function that initializes 
 */
void stabilization_udwadia_init(void)
{
 

#if PERIODIC_TELEMETRY
  // register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_REF_QUAT, send_ahrs_ref_quat);
#endif


}

/**

 */
void stabilization_udwadia_enter(void)
{
 
}


/**
 * @param in_flight boolean that states if the UAV is in flight or not
 * @param sp rate setpoint
 * @param cmd output command array
 *
 * Function that calculates the commands
 */
void stabilization_udwadia_run(bool in_flight)
{
  
  struct FloatRates *body_rates = stateGetBodyRates_f();
  Eigen::Vector3f body_rates_vect;    
  body_rates_vect << body_rates.p, body_rates.q, body_rates.r;

  struct FloatQuat *att_quat = stateGetNedToBodyQuat_f();
  Eigen::Vector4f att_quat_vect;
  att_quat_vect << att_quat.qi, att_quat.qx, att_quat.qy, att_quat.qz;
  Eigen::Matrix<float, 1, 4> att_quat_vect_T = att_quat_vect.transpose();

  Eigen::Matrix4f e_att, e_att_T, e_dot_att, e_dot_att_T;
  Eigen::Matrix<float, 3, 4> h_att;

  qtoe(&e_att, att_quat_vect);
  e_att_T = e_att.transpose();
  h_att = e_att.bottomRows(3);
  Eigen::Matrix<float, 4, 3> h_att_T = h_att.transpose();
  Eigen::Vector4f q_dot = h_att_T * body_rates_vect;
  Eigen::Matrix<float, 1, 4> q_dot_T = q_dot.transpose();
  qtoe(&e_dot_att, q_dot);
  e_dot_att_T = e_dot_att.transpose();

  Eigen::Matrix4f M = e_att_T * J * e_att;
  
  Eigen::Vector4f Q = -2 * e_dot_att_T * J * e_dot_att * q_dot;

  Eigen::Matrix<float, 5, 4> A;
  Eigen::Matrix<float, 5, 1> b;
  A << att_quat_vect_T,
       2*(-Fc(2)*att_quat_vect(1)-Fc(1)*att_quat_vect(0)), 2*(-Fc(2)*att_quat_vect(0)+Fc(1)*att_quat_vect(1)), 2*(Fc(2)*att_quat_vect(3)+Fc(1)*att_quat_vect(2)), 2*(-Fc(2)*att_quat_vect(2)-Fc(1)*att_quat_vect(3)),
       2*(-Fc(2)*att_quat_vect(2)+Fc(0)*att_quat_vect(0)), 2*(-Fc(2)*att_quat_vect(3)-Fc(0)*att_quat_vect(1)), 2*(-Fc(2)*att_quat_vect(0)-Fc(0)*att_quat_vect(2)), 2*(-Fc(2)*att_quat_vect(1)+Fc(0)*att_quat_vect(3)),
       2*(Fc(1)*att_quat_vect(2)+Fc(0)*att_quat_vect(1)), 2*(Fc(1)*att_quat_vect(3)+Fc(0)*att_quat_vect(0)), 2*(Fc(1)*att_quat_vect(0)-Fc(0)*att_quat_vect(3)), 2*(Fc(1)*att_quat_vect(1)-Fc(0)*att_quat_vect(2)),
       yaw_quat(3), 0, 0, -yaw_quat(0);

  b << -STABILIZATION_UDWADIA_D1*att_quat_vect_T*q_dot-STABILIZATION_UDWADIA_D2*(att_quat_vect_T*att_quat_vect-1)-q_dot_T*q_dot;
       4*Fc(2)*(q_dot(2)*q_dot(3)-q_dot(0)*q_dot(1))+2*Fc(1)*(pow(q_dot(0),2)-pow(q_dot(2),2)-pow(q_dot(3),2)+pow(q_dot(4),2)) - STABILIZATION_UDWADIA_D1 *(2*Fc(2)*(att_quat_vect(2)*q_dot(3) + att_quat_vect(3)*q_dot(2) - att_quat_vect(0)*q_dot(1)- att_quat_vect(1)*q_dot(0))-2*Fc(1)*(att_quat_vect(0)*q_dot(0)-att_quat_vect(1)*q_dot(1)-att_quat_vect(2)*q_dot(2)+att_quat_vect(3)*q_dot(3))) - STABILIZATION_UDWADIA_D2*(2*Fc(2)*(att_quat_vect(2)*att_quat_vect(3)-att_quat_vect(0)*att_quat_vect(1)) - Fc(1)*(pow(att_quat_vect(0),2)-pow(att_quat_vect(1),2)-pow(att_quat_vect(2),2)+pow(att_quat_vect(3),2))),
       -4*Fc(2)*(q_dot(1)*q_dot(3)+q_dot(0)*q_dot(2))+2*Fc(0)*(pow(q_dot(0),2)-pow(q_dot(1),2)-pow(q_dot(2),2)+pow(q_dot(3),2)) - STABILIZATION_UDWADIA_D1 *(-2*Fc(2)*(att_quat_vect(1)*q_dot(3) + att_quat_vect(3)*q_dot(1) + att_quat_vect(0)*q_dot(2)+ att_quat_vect(2)*q_dot(0))+2*Fc(0)*(att_quat_vect(0)*q_dot(0)-att_quat_vect(1)*q_dot(1)-att_quat_vect(2)*q_dot(2)+att_quat_vect(3)*q_dot(3))) - STABILIZATION_UDWADIA_D2*(-2*Fc(2)*(att_quat_vect(1)*att_quat_vect(3)+att_quat_vect(0)*att_quat_vect(2)) + Fc(0)*(pow(att_quat_vect(0),2)-pow(att_quat_vect(1),2)-pow(att_quat_vect(2),2)+pow(att_quat_vect(3),2))),
       -4*Fc(1)*(q_dot(1)*q_dot(3)+q_dot(0)*q_dot(2))-4*Fc(0)*(q_dot(2)*q_dot(3)-q_dot(0)*q_dot(1)) - STABILIZATION_UDWADIA_D1 *(2*Fc(1)*(att_quat_vect(1)*q_dot(3) + att_quat_vect(3)*q_dot(1) + att_quat_vect(0)*q_dot(2)+ att_quat_vect(2)*q_dot(0))-2*Fc(0)*(att_quat_vect(2)*q_dot(3) + att_quat_vect(3)*q_dot(2) - att_quat_vect(0)*q_dot(1)- att_quat_vect(1)*q_dot(0))) - STABILIZATION_UDWADIA_D2*(2*Fc(1)*(att_quat_vect(1)*att_quat_vect(3)+att_quat_vect(0)*att_quat_vect(2)) - 2*Fc(0)*(att_quat_vect(2)*att_quat_vect(3)-att_quat_vect(0)*att_quat_vect(1))),
       -STABILIZATION_UDWADIA_D1 *(q_dot(0)*yaw_quat(3)-q_dot(3)*yaw_quat(0)) - STABILIZATION_UDWADIA_D2*(att_quat_vect(0)*yaw_quat(3)-att_quat_vect(3)*yaw_quat(0));

  F = 

}



// This function reads rc commands
void stabilization_udwadia_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn)
{
  struct FloatQuat q_sp;
  struct FloatEulers eul_sp;
  struct FloatVect3 force_vect = {0,0,-1};
  struct FloatVect3 tmp_vect;
  stabilization_attitude_read_rc_setpoint_quat_f(&q_sp, in_flight, in_carefree, coordinated_turn);
  stabilization_attitude_read_rc_setpoint_eulers_f(&eul_sp, in_flight, in_carefree, coordinated_turn);

  float_quat_vmult(&tmp_vect, &q_sp, &force_vect);
  Fc << tmp_vect.x, tmp_vect.y, tmp_vect.z;

  yaw_quat << cos(eul_sp.psi/2), 0, 0, sin(eul_sp.psi/2);
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


void qtoe(Eigen::Matrix4f &h, Eigen::Vector4f q){
  h <<  2*q(0),  2*q(1),  2*q(2),  2*q(3),
       -2*q(1),  2*q(0),  2*q(3), -2*q(2),
       -2*q(2), -2*q(3),  2*q(0),  2*q(1),
       -2*q(3),  2*q(2), -2*q(1),  2*q(0);
}

