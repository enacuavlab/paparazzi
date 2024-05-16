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

#ifdef __cplusplus
extern "C" {
#endif

#include "firmwares/rotorcraft/stabilization/stabilization_udwadia.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"

#include "math/pprz_algebra_float.h"
#include "state.h"
#include "generated/airframe.h"
#include "modules/radio_control/radio_control.h"
#include "modules/actuators/actuators.h"
#include "filters/low_pass_filter.h"
#include "modules/core/abi.h"
#include <stdio.h>


#ifdef __cplusplus
}
#endif

#include <Eigen/Dense> // https://eigen.tuxfamily.org/dox/GettingStarted.html
// variables needed for control

//int32_t stabilization_cmd[COMMANDS_NB];

//struct FloatEulers stab_att_sp_euler;


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

#ifndef STABILIZATION_UDWADIA_L
#define STABILIZATION_UDWADIA_L 0.1
#endif

#ifndef STABILIZATION_UDWADIA_C
#define STABILIZATION_UDWADIA_C 0.05
#endif

#ifndef STABILIZATION_UDWADIA_MASS
#define STABILIZATION_UDWADIA_MASS 0.4
#endif

const float kf = 0.000000017912000;

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


Eigen::Vector3f Fc, body_rates_vect;
Eigen::Vector4f yaw_quat;

Eigen::Matrix4f J, B, B_pinv;

Eigen::Vector4f F, F_bound, var, p1, att_quat_vect;

Butterworth2LowPass rates_lowpass_filters[3];


void qtoe(Eigen::Matrix4f *h, Eigen::Vector4f q);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_payload_float(struct transport_tx *trans, struct link_device *dev)
{
  float f[22] = {F(0), F(1), F(2), F(3),
                Fc(0), Fc(1), Fc(2),
                var(0), var(1), var(2), var(3),
                p1(0), p1(1), p1(2), p1(3),
                att_quat_vect(0), att_quat_vect(1), att_quat_vect(2), att_quat_vect(3),
                body_rates_vect(0), body_rates_vect(1), body_rates_vect(2)}; 
  pprz_msg_send_PAYLOAD_FLOAT(trans, dev, AC_ID, 22, f);
}
#endif

/**
 * Function that initializes 
 */
void stabilization_udwadia_init(void)
{

  J << 1, 0, 0, 0,
       0, STABILIZATION_UDWADIA_IX, 0, 0,
       0, 0, STABILIZATION_UDWADIA_IY, 0,
       0, 0, 0, STABILIZATION_UDWADIA_IZ;

  B <<  1, 1, 1, 1,
      -STABILIZATION_UDWADIA_L,  STABILIZATION_UDWADIA_L,  STABILIZATION_UDWADIA_L, -STABILIZATION_UDWADIA_L,
       STABILIZATION_UDWADIA_L, -STABILIZATION_UDWADIA_L,  STABILIZATION_UDWADIA_L, -STABILIZATION_UDWADIA_L,
       STABILIZATION_UDWADIA_C,  STABILIZATION_UDWADIA_C, -STABILIZATION_UDWADIA_C, -STABILIZATION_UDWADIA_C;

  yaw_quat << 1, 0, 0, 0;
 
  // B_pinv = B.completeOrthogonalDecomposition().pseudoInverse();
  B_pinv = (B.transpose()*B).ldlt().solve (B.transpose());

  // tau = 1/(2*pi*Fc)
  float tau = 1.0 / (2.0 * M_PI * 20.0);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  // Filtering of the gyroscope
  int8_t i;
  for (i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&rates_lowpass_filters[i], tau, sample_time, 0.0);
  }


  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PAYLOAD_FLOAT, send_payload_float);
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
void stabilization_udwadia_run(bool __attribute__((unused)) in_flight)
{
  

   /* Propagate the filter on the gyroscopes */
  struct FloatRates *body_rates = stateGetBodyRates_f();
  float rate_vect[3] = {body_rates->p, body_rates->q, body_rates->r};
  int8_t i;
  for (i = 0; i < 3; i++) {
    update_butterworth_2_low_pass(&rates_lowpass_filters[i], rate_vect[i]);
  }
  
  body_rates_vect << rates_lowpass_filters[0].o[0], rates_lowpass_filters[1].o[0], rates_lowpass_filters[2].o[0];

  struct FloatQuat *att_quat = stateGetNedToBodyQuat_f();
  
  att_quat_vect << att_quat->qi, att_quat->qx, att_quat->qy, att_quat->qz;
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
  Eigen::Matrix4f M_inv = M.inverse();
  
  Eigen::Vector4f Q = -2 * e_dot_att_T * J * e_dot_att * q_dot;

  Eigen::Matrix<float, 5, 4> A;
  Eigen::Matrix<float, 5, 1> b;
  // A << att_quat_vect_T,
  //      2*(-Fc(2)*att_quat_vect(1)-Fc(1)*att_quat_vect(0)), 2*(-Fc(2)*att_quat_vect(0)+Fc(1)*att_quat_vect(1)), 2*(Fc(2)*att_quat_vect(3)+Fc(1)*att_quat_vect(2)), 2*(-Fc(2)*att_quat_vect(2)-Fc(1)*att_quat_vect(3)),
  //      2*(-Fc(2)*att_quat_vect(2)+Fc(0)*att_quat_vect(0)), 2*(-Fc(2)*att_quat_vect(3)-Fc(0)*att_quat_vect(1)), 2*(-Fc(2)*att_quat_vect(0)-Fc(0)*att_quat_vect(2)), 2*(-Fc(2)*att_quat_vect(1)+Fc(0)*att_quat_vect(3)),
  //      2*(Fc(1)*att_quat_vect(2)+Fc(0)*att_quat_vect(1)), 2*(Fc(1)*att_quat_vect(3)+Fc(0)*att_quat_vect(0)), 2*(Fc(1)*att_quat_vect(0)-Fc(0)*att_quat_vect(3)), 2*(Fc(1)*att_quat_vect(1)-Fc(0)*att_quat_vect(2)),
  //      yaw_quat(3), 0, 0, -yaw_quat(0);

  // b << -STABILIZATION_UDWADIA_D1*att_quat_vect_T*q_dot-STABILIZATION_UDWADIA_D2*(att_quat_vect_T*att_quat_vect-1)-q_dot_T*q_dot,
  //      4*Fc(2)*(q_dot(2)*q_dot(3)-q_dot(0)*q_dot(1))+2*Fc(1)*(pow(q_dot(0),2)-pow(q_dot(2),2)-pow(q_dot(3),2)+pow(q_dot(3),2)) - STABILIZATION_UDWADIA_D1 *(2*Fc(2)*(att_quat_vect(2)*q_dot(3) + att_quat_vect(3)*q_dot(2) - att_quat_vect(0)*q_dot(1)- att_quat_vect(1)*q_dot(0))-2*Fc(1)*(att_quat_vect(0)*q_dot(0)-att_quat_vect(1)*q_dot(1)-att_quat_vect(2)*q_dot(2)+att_quat_vect(3)*q_dot(3))) - STABILIZATION_UDWADIA_D2*(2*Fc(2)*(att_quat_vect(2)*att_quat_vect(3)-att_quat_vect(0)*att_quat_vect(1)) - Fc(1)*(pow(att_quat_vect(0),2)-pow(att_quat_vect(1),2)-pow(att_quat_vect(2),2)+pow(att_quat_vect(3),2))),
  //      -4*Fc(2)*(q_dot(1)*q_dot(3)+q_dot(0)*q_dot(2))+2*Fc(0)*(pow(q_dot(0),2)-pow(q_dot(1),2)-pow(q_dot(2),2)+pow(q_dot(3),2)) - STABILIZATION_UDWADIA_D1 *(-2*Fc(2)*(att_quat_vect(1)*q_dot(3) + att_quat_vect(3)*q_dot(1) + att_quat_vect(0)*q_dot(2)+ att_quat_vect(2)*q_dot(0))+2*Fc(0)*(att_quat_vect(0)*q_dot(0)-att_quat_vect(1)*q_dot(1)-att_quat_vect(2)*q_dot(2)+att_quat_vect(3)*q_dot(3))) - STABILIZATION_UDWADIA_D2*(-2*Fc(2)*(att_quat_vect(1)*att_quat_vect(3)+att_quat_vect(0)*att_quat_vect(2)) + Fc(0)*(pow(att_quat_vect(0),2)-pow(att_quat_vect(1),2)-pow(att_quat_vect(2),2)+pow(att_quat_vect(3),2))),
  //      -4*Fc(1)*(q_dot(1)*q_dot(3)+q_dot(0)*q_dot(2))-4*Fc(0)*(q_dot(2)*q_dot(3)-q_dot(0)*q_dot(1)) - STABILIZATION_UDWADIA_D1 *(2*Fc(1)*(att_quat_vect(1)*q_dot(3) + att_quat_vect(3)*q_dot(1) + att_quat_vect(0)*q_dot(2)+ att_quat_vect(2)*q_dot(0))-2*Fc(0)*(att_quat_vect(2)*q_dot(3) + att_quat_vect(3)*q_dot(2) - att_quat_vect(0)*q_dot(1)- att_quat_vect(1)*q_dot(0))) - STABILIZATION_UDWADIA_D2*(2*Fc(1)*(att_quat_vect(1)*att_quat_vect(3)+att_quat_vect(0)*att_quat_vect(2)) - 2*Fc(0)*(att_quat_vect(2)*att_quat_vect(3)-att_quat_vect(0)*att_quat_vect(1))),
  //      -STABILIZATION_UDWADIA_D1 *(q_dot(0)*yaw_quat(3)-q_dot(3)*yaw_quat(0)) - STABILIZATION_UDWADIA_D2*(att_quat_vect(0)*yaw_quat(3)-att_quat_vect(3)*yaw_quat(0));
  
  A << att_quat_vect_T,
       2*(-Fc(2)*att_quat_vect(1)-Fc(1)*att_quat_vect(0)), 2*(-Fc(2)*att_quat_vect(0)+Fc(1)*att_quat_vect(1)), 2*(Fc(2)*att_quat_vect(3)+Fc(1)*att_quat_vect(2)), 2*(-Fc(2)*att_quat_vect(2)-Fc(1)*att_quat_vect(3)),
       2*(-Fc(2)*att_quat_vect(2)+Fc(0)*att_quat_vect(0)), 2*(-Fc(2)*att_quat_vect(3)-Fc(0)*att_quat_vect(1)), 2*(-Fc(2)*att_quat_vect(0)-Fc(0)*att_quat_vect(2)), 2*(-Fc(2)*att_quat_vect(1)+Fc(0)*att_quat_vect(3)),
       2*(Fc(1)*att_quat_vect(2)+Fc(0)*att_quat_vect(1)), 2*(Fc(1)*att_quat_vect(3)+Fc(0)*att_quat_vect(0)), 2*(Fc(1)*att_quat_vect(0)-Fc(0)*att_quat_vect(3)), 2*(Fc(1)*att_quat_vect(1)-Fc(0)*att_quat_vect(2)),
       -att_quat_vect(3), att_quat_vect(2), -att_quat_vect(1), att_quat_vect(0);

  b << -STABILIZATION_UDWADIA_D1*att_quat_vect_T*q_dot-STABILIZATION_UDWADIA_D2*(att_quat_vect_T*att_quat_vect-1)-q_dot_T*q_dot,
       4*Fc(2)*(q_dot(2)*q_dot(3)-q_dot(0)*q_dot(1))+2*Fc(1)*(pow(q_dot(0),2)-pow(q_dot(2),2)-pow(q_dot(3),2)+pow(q_dot(3),2)) - STABILIZATION_UDWADIA_D1 *(2*Fc(2)*(att_quat_vect(2)*q_dot(3) + att_quat_vect(3)*q_dot(2) - att_quat_vect(0)*q_dot(1)- att_quat_vect(1)*q_dot(0))-2*Fc(1)*(att_quat_vect(0)*q_dot(0)-att_quat_vect(1)*q_dot(1)-att_quat_vect(2)*q_dot(2)+att_quat_vect(3)*q_dot(3))) - STABILIZATION_UDWADIA_D2*(2*Fc(2)*(att_quat_vect(2)*att_quat_vect(3)-att_quat_vect(0)*att_quat_vect(1)) - Fc(1)*(pow(att_quat_vect(0),2)-pow(att_quat_vect(1),2)-pow(att_quat_vect(2),2)+pow(att_quat_vect(3),2))),
       -4*Fc(2)*(q_dot(1)*q_dot(3)+q_dot(0)*q_dot(2))+2*Fc(0)*(pow(q_dot(0),2)-pow(q_dot(1),2)-pow(q_dot(2),2)+pow(q_dot(3),2)) - STABILIZATION_UDWADIA_D1 *(-2*Fc(2)*(att_quat_vect(1)*q_dot(3) + att_quat_vect(3)*q_dot(1) + att_quat_vect(0)*q_dot(2)+ att_quat_vect(2)*q_dot(0))+2*Fc(0)*(att_quat_vect(0)*q_dot(0)-att_quat_vect(1)*q_dot(1)-att_quat_vect(2)*q_dot(2)+att_quat_vect(3)*q_dot(3))) - STABILIZATION_UDWADIA_D2*(-2*Fc(2)*(att_quat_vect(1)*att_quat_vect(3)+att_quat_vect(0)*att_quat_vect(2)) + Fc(0)*(pow(att_quat_vect(0),2)-pow(att_quat_vect(1),2)-pow(att_quat_vect(2),2)+pow(att_quat_vect(3),2))),
       -4*Fc(1)*(q_dot(1)*q_dot(3)+q_dot(0)*q_dot(2))-4*Fc(0)*(q_dot(2)*q_dot(3)-q_dot(0)*q_dot(1)) - STABILIZATION_UDWADIA_D1 *(2*Fc(1)*(att_quat_vect(1)*q_dot(3) + att_quat_vect(3)*q_dot(1) + att_quat_vect(0)*q_dot(2)+ att_quat_vect(2)*q_dot(0))-2*Fc(0)*(att_quat_vect(2)*q_dot(3) + att_quat_vect(3)*q_dot(2) - att_quat_vect(0)*q_dot(1)- att_quat_vect(1)*q_dot(0))) - STABILIZATION_UDWADIA_D2*(2*Fc(1)*(att_quat_vect(1)*att_quat_vect(3)+att_quat_vect(0)*att_quat_vect(2)) - 2*Fc(0)*(att_quat_vect(2)*att_quat_vect(3)-att_quat_vect(0)*att_quat_vect(1))),
       -STABILIZATION_UDWADIA_D1 *(att_quat_vect(2)*q_dot(1)-att_quat_vect(3)*q_dot(0)+att_quat_vect(0)*q_dot(3)-att_quat_vect(1)*q_dot(2)) ;

  Eigen::Matrix<float, 5, 4> temp = A*M_inv;
  Eigen::Matrix<float, 4, 5> temp_pinv = (temp.transpose()*temp).ldlt().solve (temp.transpose()); // https://www.mrtrix.org/developer-documentation/least__squares_8h_source.html#l00039
  // Eigen::Matrix<float, 4, 5> temp_pinv = temp.completeOrthogonalDecomposition().pseudoInverse();
  p1 = temp_pinv * (b+temp*Q);

  
  var << sqrtf(Fc(0)*Fc(0) + Fc(1)*Fc(1) + Fc(2)*Fc(2)),
          e_att.bottomRightCorner(3,4)*p1;
  F = (B_pinv*var);


   // Bound the inputs to the actuators
  for (int i = 0; i < 4; i++) {
    if (autopilot_get_motors_on()) {
      F_bound(i) = sqrtf(F(i)/kf)*MAX_PPRZ/30000; 
      Bound(F_bound(i), 0, MAX_PPRZ);
    } else {
      F_bound(i) = -MAX_PPRZ;
    }
    actuators_pprz[i] = (int16_t) F_bound(i);
  }

}



// This function reads rc commands
void stabilization_udwadia_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn)
{
  struct FloatQuat q_sp;
  struct FloatEulers eul_sp;
  float throttle = (float) guidance_v.rc_delta_t;
  struct FloatVect3 force_vect = {0,0,5*STABILIZATION_UDWADIA_MASS*throttle/MAX_PPRZ}; //-20*STABILIZATION_UDWADIA_MASS*throttle/MAX_PPRZ
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
// void get_actuator_state(void)
// {

//   float_vect_copy(actuator_state, act_obs, INDI_NUM_ACT);

// }


void qtoe(Eigen::Matrix4f *h, Eigen::Vector4f q){
  *h <<  2*q(0),  2*q(1),  2*q(2),  2*q(3),
       -2*q(1),  2*q(0),  2*q(3), -2*q(2),
       -2*q(2), -2*q(3),  2*q(0),  2*q(1),
       -2*q(3),  2*q(2), -2*q(1),  2*q(0);
}

