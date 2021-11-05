/*
 * Copyright (C) 2021 Hector Garcia de Marina <hgarciad@ucm.es>
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

/**
 * @file modules/guidance/gvf_parametric_surf/gvf_parametric_surf.cpp
 *
 * Guiding vector field algorithm for 2D and 3D parametric trajectories with two parameters.
 */

#include <iostream>
#include <Eigen/Dense>

#include "gvf_parametric_surf.h"
#include "gvf_parametric_surf_low_level_control.h"
#include "./trajectories/gvf_parametric_surf_3d_torus.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "autopilot.h"

// Control
uint32_t gvf_parametric_surf_t0 = 0; // We need it for calculting the time lapse delta_T

gvf_parametric_surf_con gvf_parametric_surf_control;
gvf_parametric_surf_coord gvf_parametric_surf_coordination;
gvf_parametric_surf_coord_tab gvf_parametric_surf_coordination_tables;

uint32_t gvf_parametric_surf_last_transmision = 0;

// Trajectory
gvf_parametric_surf_tra gvf_parametric_surf_trajectory;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
static void send_gvf_parametric_surf(struct transport_tx *trans, struct link_device *dev)
{
  // Do not know whether is a good idea to do this check here or to include
  // this plen in gvf_trajectory
  int plen;
  int elen;

  switch (gvf_parametric_trajectory.type) {
    case TORUS_3D:
      plen = 5;
      elen = 3;
      break;
    default:
      plen = 1;
      elen = 3;
  }

  uint8_t traj_type = (uint8_t)gvf_parametric_surf_trajectory.type;

  uint32_t now = get_sys_time_msec();
  uint32_t delta_T = now - gvf_parametric_surf_t0;

  float w1b = gvf_parametric_surf_control.w1 * gvf_parametric_surf_control.beta1;
  float w2b = gvf_parametric_surf_control.w2 * gvf_parametric_surf_control.beta2;

  if (delta_T < 200) {
    pprz_msg_send_GVF_PARAMETRIC_SURF(trans, dev, AC_ID, &traj_type, &gvf_parametric_surf_control.s1,
            &gvf_parametric_surf_control.s2, &w1b, &w2b, plen,
            gvf_parametric_surf_trajectory.p_parametric, elen, gvf_parametric_surf_trajectory.phi_errors);
  }
}

static void send_gvf_parametric_surf_coordination(struct transport_tx *trans, struct link_device *dev)
{
  if(gvf_parametric_surf_coordination.coordination)
    pprz_msg_send_GVF_PAR_SURF_COORD(trans, dev, AC_ID, 7*GVF_PARAMETRIC_SURF_COORDINATION_MAX_NEIGHBORS, &(gvf_parametric_surf_coordination_tables.tableNei[0][0]), GVF_PARAMETRIC_SURF_COORDINATION_MAX_NEIGHBORS, gvf_parametric_surf_coordination_tables.error_deltaw1, GVF_PARAMETRIC_SURF_COORDINATION_MAX_NEIGHBORS,gvf_parametric_surf_coordination_tables.error_deltaw2);
}

#endif // PERIODIC TELEMETRY

#ifdef __cplusplus
}
#endif

void gvf_parametric_surf_init(void)
{
  gvf_parametric_surf_control.w1 = 0;
  gvf_parametric_surf_control.w2 = 0;
  gvf_parametric_surf_control.delta_T = 0;
  gvf_parametric_surf_control.s1 = 1;
  gvf_parametric_surf_control.s2 = 1;
  gvf_parametric_surf_control.k_roll = GVF_PARAMETRIC_SURF_CONTROL_KROLL;
  gvf_parametric_surf_control.k_climb = GVF_PARAMETRIC_SURF_CONTROL_KCLIMB;
  gvf_parametric_surf_control.k_psi = GVF_PARAMETRIC_SURF_CONTROL_KPSI;
  gvf_parametric_surf_control.L = GVF_PARAMETRIC_SURF_CONTROL_L;
  gvf_parametric_surf_control.beta1 = GVF_PARAMETRIC_SURF_CONTROL_BETA1;
  gvf_parametric_surf_control.beta2 = GVF_PARAMETRIC_SURF_CONTROL_BETA2;
  gvf_parametric_surf_control.w1_dot = 0;
  gvf_parametric_surf_control.w2_dot = 0;

  gvf_parametric_surf_coordination.coordination = GVF_PARAMETRIC_SURF_COORDINATION;
  gvf_parametric_surf_coordination.kc1 = GVF_PARAMETRIC_SURF_COORDINATION_KC1;
  gvf_parametric_surf_coordination.kc2 = GVF_PARAMETRIC_SURF_COORDINATION_KC2;
  gvf_parametric_surf_coordination.timeout = GVF_PARAMETRIC_SURF_COORDINATION_TIMEOUT;
  gvf_parametric_surf_coordination.broadtime = GVF_PARAMETRIC_SURF_COORDINATION_BROADTIME;

  for (int i = 0; i < GVF_PARAMETRIC_SURF_COORDINATION_MAX_NEIGHBORS; i++) {
    gvf_parametric_surf_coordination_tables.tableNei[i][0] = -1;
    gvf_parametric_surf_coordination_tables.tableNei[i][1] = 0;
    gvf_parametric_surf_coordination_tables.tableNei[i][2] = 0;
    gvf_parametric_surf_coordination_tables.tableNei[i][3] = 0;
    gvf_parametric_surf_coordination_tables.tableNei[i][4] = 0;
    gvf_parametric_surf_coordination_tables.tableNei[i][5] = 0;
    gvf_parametric_surf_coordination_tables.tableNei[i][6] = 0;
    gvf_parametric_surf_coordination_tables.tableNei[i][7] = 0;
    gvf_parametric_surf_coordination_tables.error_deltaw1[i] = 0;
    gvf_parametric_surf_coordination_tables.error_deltaw2[i] = 0;
  }

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GVF_PAR_SURF_COORD, send_gvf_parametric_surf_coordination);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GVF_PARAMETRIC_SURF, send_gvf_parametric_surf);
#endif

}

void gvf_parametric_surf_set_direction_s1(int8_t s1)
{
  gvf_parametric_surf_control.s1 = s1;
}

void gvf_parametric_surf_set_direction_s2(int8_t s2)
{
  gvf_parametric_surf_control.s2 = s2;
}

// 2D trajectories with two parameters no implemented yet

// 3D trajectories
void gvf_parametric_surf_control_3D(float kx, float ky, float kz, float f1, float f2, float f3, float f1dw1, float f2dw1, float f3dw1, float f1ddw1, float f2ddw1, float f3ddw1, float f1dw2, float f2dw2, float f3dw2,float f1ddw2, float f2ddw2, float f3ddw2)
{
  uint32_t now = get_sys_time_msec();
  gvf_parametric_surf_control.delta_T = now - gvf_parametric_surf_t0;
  gvf_parametric_surf_t0 = now;

  if (gvf_parametric_surf_control.delta_T > 300) { // We need at least two iterations for Delta_T
    gvf_parametric_surf_control.w1 = 0; // Reset w since we assume the algorithm starts
    gvf_parametric_surf_control.w2 = 0; // Reset w since we assume the algorithm starts
    return;
  }

  float L = gvf_parametric_surf_control.L;
  float beta1 = gvf_parametric_surf_control.beta1 * gvf_parametric_surf_control.s1;
  float beta2 = gvf_parametric_surf_control.beta2 * gvf_parametric_surf_control.s2;

  Eigen::Matrix<float, 5, 5> J;
  Eigen::Matrix<float, 5, 1> X; // Chi

  // Error signals phi_x phi_y and phi_z
  struct EnuCoor_f *pos_enu = stateGetPositionEnu_f();
  float x = pos_enu->x;
  float y = pos_enu->y;
  float z = pos_enu->z;

  float phi1 = L * (x - f1);
  float phi2 = L * (y - f2);
  float phi3 = L * (z - f3);

  gvf_parametric_surf_trajectory.phi_errors[0] = phi1 / L; // Error signals in meters for the telemetry
  gvf_parametric_surf_trajectory.phi_errors[1] = phi2 / L;
  gvf_parametric_surf_trajectory.phi_errors[2] = phi3 / L;

  // Chi
  X(0) = -L * L * (beta1*f1dw1 - beta2*f1dw2)- kx * phi1;
  X(1) = -L * L * (beta1*f2dw1 - beta2*f2dw2)- ky * phi2;
  X(2) = -L * L * (beta1*f3dw1 - beta2*f3dw2)- kz * phi3;
  X(3) = -L * L + beta1 * (kx * phi1 * f1dw1 + ky * phi2 * f2dw1 + kz * phi3 * f3dw1);
  X(4) =  L * L + beta2 * (kx * phi1 * f1dw2 + ky * phi2 * f2dw2 + kz * phi3 * f3dw2);
  X *= L;

  // Coordination if needed for multi vehicles
  float consensus_term_w1 = 0;
  float consensus_term_w2 = 0;

  if(gvf_parametric_coordination.coordination){
      for (int i = 0; i < GVF_PARAMETRIC_SURF_COORDINATION_MAX_NEIGHBORS; i++) {
          if ((int32_t)(gvf_parametric_surf_coordination_tables.tableNei[i][0]) != -1) {
              uint32_t timeout = now - (uint32_t)(gvf_parametric_surf_coordination_tables.last_comm[i]);
              if (timeout > gvf_parametric_surf_coordination.timeout) {
                  gvf_parametric_surf_coordination_tables.tableNei[i][7] = (float)gvf_parametric_surf_coordination.timeout;
              } else {
                  gvf_parametric_surf_coordination_tables.tableNei[i][7] = (float)timeout;

                  float w1i = gvf_parametric_surf_control.w1;
                  float w1j = gvf_parametric_surf_coordination_tables.tableNei[i][1];
                  float desired_dw1 = gvf_parametric_surf_coordination_tables.tableNei[i][3];

                  float error_w1 = -beta1*(w1i-w1j) + desired_dw1;

                  consensus_term_w1 += error_w1;

                  gvf_parametric_surf_coordination_tables.error_deltaw1[i] = error_w1;

                  // DECIDIR POSICIONES DE LA TABLA PARA W2
                  float w2i = gvf_parametric_surf_control.w2;
                  float w2j = gvf_parametric_surf_coordination_tables.tableNei[i][4];
                  float desired_dw2 = gvf_parametric_surf_coordination_tables.tableNei[i][6];

                  float error_w2 = -beta2*(w2i-w2j) + desired_dw2;

                  consensus_term_w2 += error_w2;

                  gvf_parametric_surf_coordination_tables.error_deltaw2[i] = error_w2;
              }
          }
      }
  }

  X(3) += gvf_parametric_surf_coordination.kc1*consensus_term_w1;
  X(4) += gvf_parametric_surf_coordination.kc2*consensus_term_w2;

  // Jacobian
  J.setZero();
  J(0, 0) = -kx * L;
  J(1, 1) = -ky * L;
  J(2, 2) = -kz * L;
  //J(3, 0) = kx * f1d * beta * L;
  //J(3, 1) = ky * f2d * beta * L;
  //J(3, 2) = kz * f3d * beta * L;
  //J(0, 3) = -(beta * L) * (beta * L * f1dd - kx * f1d);
  //J(1, 3) = -(beta * L) * (beta * L * f2dd - ky * f2d);
  //J(2, 3) = -(beta * L) * (beta * L * f3dd - kz * f3d);
  //J(3, 3) =  beta * beta * (kx * (phi1 * f1dd - L * f1d * f1d) + ky * (phi2 * f2dd - L * f2d * f2d)
                            //+ kz * (phi3 * f3dd - L * f3d * f3d));
  J *= L;

  // Guidance algorithm
  float ground_speed = stateGetHorizontalSpeedNorm_f();
  float w1_dot = (ground_speed * X(3)) / sqrtf(X(0) * X(0) + X(1) * X(1));
  float w2_dot = (ground_speed * X(4)) / sqrtf(X(0) * X(0) + X(1) * X(1));

  Eigen::Matrix<float, 5, 1> xi_dot;
  struct EnuCoor_f *vel_enu = stateGetSpeedEnu_f();
  float course = stateGetHorizontalSpeedDir_f();

  xi_dot << vel_enu->x, vel_enu->y, vel_enu->z, w1_dot, w2_dot;

  Eigen::Matrix2f E;
  Eigen::Matrix<float, 2, 5> F;
  Eigen::Matrix<float, 2, 5> Fp;
  Eigen::Matrix<float, 5, 5> G;
  Eigen::Matrix<float, 5, 5> Gp;
  Eigen::Matrix<float, 5, 5> I;
  Eigen::Vector2f h;
  Eigen::Matrix<float, 1, 2> ht;

  h << sinf(course), cosf(course);
  ht = h.transpose();
  I.setIdentity();
  F << 1.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 1.0, 0.0, 0.0, 0.0;
  E << 0.0, -1.0,
       1.0, 0.0;
  G = F.transpose() * F;
  Fp = E * F;
  Gp = F.transpose() * E * F;

  Eigen::Matrix<float, 1, 5> Xt = X.transpose();
  Eigen::Matrix<float, 1, 5> Xh = X / X.norm();
  Eigen::Matrix<float, 1, 5> Xht = Xh.transpose();

  float aux = ht * Fp * X;
  Eigen::Matrix<float, 5, 1> aux2 =  J * xi_dot;

  // Coordination if needed for multi vehicles
  float consensus_term_w1dot = 0;
  float consensus_term_w2dot = 0;

  if(gvf_parametric_surf_coordination.coordination){
    for (int i = 0; i < GVF_PARAMETRIC_SURF_COORDINATION_MAX_NEIGHBORS; i++) {
        if ((int32_t)(gvf_parametric_surf_coordination_tables.tableNei[i][0]) != -1) {
            uint32_t timeout = now - (uint32_t)(gvf_parametric_surf_coordination_tables.last_comm[i]);
            if (timeout > gvf_parametric_surf_coordination.timeout) {
                gvf_parametric_surf_coordination_tables.tableNei[i][7] = (float)gvf_parametric_surf_coordination.timeout;
            } else {
                gvf_parametric_surf_coordination_tables.tableNei[i][7] = (float)timeout;

                float w1i_dot = gvf_parametric_surf_control.w1_dot;
                float w1j_dot = gvf_parametric_surf_coordination_tables.tableNei[i][2];

                consensus_term_w1dot += -beta1*(w1i_dot - w1j_dot);

                float w2i_dot = gvf_parametric_surf_control.w2_dot;
                float w2j_dot = gvf_parametric_surf_coordination_tables.tableNei[i][5];

                consensus_term_w2dot += -beta2*(w2i_dot - w2j_dot);
            }
        }
    }
  }

  aux2(3) += gvf_parametric_surf_coordination.kc1*consensus_term_w1dot;
  aux2(4) += gvf_parametric_surf_coordination.kc2*consensus_term_w2dot;

  float heading_rate = -1 / (Xt * G * X) * Xt * Gp * (I - Xh * Xht) * aux2 - (gvf_parametric_surf_control.k_psi * aux /
                       sqrtf(Xt * G * X));
  float climbing_rate = (ground_speed * X(2)) / sqrtf(X(0) * X(0) + X(1) * X(1));

  // Virtual coordinate update, even if the vehicle is not in autonomous mode, the parameters w1 and w2 will get "closer" to
  // the vehicle. So it is not only okei but advisable to update it.
  gvf_parametric_surf_control.w1 += w1_dot * gvf_parametric_surf_control.delta_T * 1e-3;
  gvf_parametric_surf_control.w1_dot = w1_dot;

  gvf_parametric_surf_control.w2 += w2_dot * gvf_parametric_surf_control.delta_T * 1e-3;
  gvf_parametric_surf_control.w2_dot = w2_dot;

  gvf_parametric_surf_low_level_control_3D(heading_rate, climbing_rate);

  if ((gvf_parametric_surf_coordination.coordination) && (now - gvf_parametric_surf_last_transmision > gvf_parametric_surf_coordination.broadtime) && (autopilot_get_mode() == AP_MODE_AUTO2)) {
    gvf_parametric_surf_coordination_send_w_to_nei();
    gvf_parametric_surf_last_transmision = now;
  }
}

/** 2D TRAJECTORIES **/

/** 3D TRAJECTORIES **/
// 3D TORUS

bool gvf_parametric_surf_3D_torus_XY(float xo, float yo, float rh, float rv, float zo)
{
  gvf_parametric_surf_trajectory.type = TORUS_3D;
  gvf_parametric_surf_trajectory.p_parametric[0] = xo;
  gvf_parametric_surf_trajectory.p_parametric[1] = yo;
  gvf_parametric_surf_trajectory.p_parametric[1] = zo;
  gvf_parametric_surf_trajectory.p_parametric[3] = rh;
  gvf_parametric_surf_trajectory.p_parametric[4] = rv;

  float f1, f2, f3, f1dw1, f2dw1, f3dw1, f1ddw1, f2ddw1, f3ddw1, f1dw2, f2dw2, f3dw2, f1ddw2, f2ddw2, f3ddw2;

  gvf_parametric_surf_3d_torus_info(&f1, &f2, &f3, &f1dw1, &f2dw1, &f3dw1, &f1ddw1, &f2ddw1, &f3ddw1,
                                    &f1dw2, &f2dw2, &f3dw2, &f1ddw2, &f2ddw2, &f3ddw2);
  gvf_parametric_surf_control_3D(gvf_parametric_surf_3d_torus_par.kx, gvf_parametric_surf_3d_torus_par.ky,
                            gvf_parametric_surf_3d_torus_par.kz, f1, f2, f3, f1dw1, f2dw1, f3dw1, f1ddw1, f2ddw1, f3ddw1,
                            f1dw2, f2dw2, f3dw2, f1ddw2, f2ddw2, f3ddw2);

  return true;
}

bool gvf_parametric_surf_3D_torus_wp(uint8_t wp, float rh, float rv, float alt)
{
  gvf_parametric_surf_3D_torus_XY(waypoints[wp].x, waypoints[wp].y, rh, rv, alt);
  return true;
}

void gvf_parametric_surf_coordination_send_w_to_nei(void)
{
  struct pprzlink_msg msg;

  for (int i = 0; i < GVF_PARAMETRIC_SURF_COORDINATION_MAX_NEIGHBORS; i++)
    if ((int32_t)(gvf_parametric_surf_coordination_tables.tableNei[i][0]) != -1) {
      msg.trans = &(DefaultChannel).trans_tx;
      msg.dev = &(DefaultDevice).device;
      msg.sender_id = AC_ID;
      msg.receiver_id = (uint8_t)(gvf_parametric_coordination_tables.tableNei[i][0]);
      msg.component_id = 0;
      pprzlink_msg_send_GVF_PARAMETRIC_SURF_W(&msg, &(gvf_parametric_surf_control.w1), &(gvf_parametric_surf_control.w1_dot),
             &(gvf_parametric_surf_control.w2), &(gvf_parametric_surf_control.w2_dot) );
    }
}

void gvf_parametric_surf_coordination_parseRegTable(uint8_t *buf)
{
  uint8_t ac_id = DL_GVF_PARAMETRIC_SURF_REG_TABLE_ac_id(buf);
  if (ac_id == AC_ID) {
    uint8_t nei_id = (uint8_t)(DL_GVF_PARAMETRIC_SURF_REG_TABLE_nei_id(buf));
    float desired_deltaw1 = DL_GVF_PARAMETRIC_SURF_REG_TABLE_desired_deltaw1(buf);
    float desired_deltaw2 = DL_GVF_PARAMETRIC_SURF_REG_TABLE_desired_deltaw2(buf);

    if (nei_id == 0) {
      for (int i = 0; i < GVF_PARAMETRIC_SURF_COORDINATION_MAX_NEIGHBORS; i++) {
        gvf_parametric_surf_coordination_tables.tableNei[i][0] = -1;
      }
    } else {
      for (int i = 0; i < GVF_PARAMETRIC_SURF_COORDINATION_MAX_NEIGHBORS; i++)
        if ((int8_t)(gvf_parametric_surf_coordination_tables.tableNei[i][0]) == (int8_t)nei_id) {
          gvf_parametric_surf_coordination_tables.tableNei[i][0] = nei_id;
          gvf_parametric_surf_coordination_tables.tableNei[i][3] = desired_deltaw1;
          gvf_parametric_surf_coordination_tables.tableNei[i][6] = desired_deltaw2;
          return;
        }

      for (int i = 0; i < GVF_PARAMETRIC_SURF_COORDINATION_MAX_NEIGHBORS; i++)
        if ((int8_t)(gvf_parametric_coordination_tables.tableNei[i][0]) == -1) {
          gvf_parametric_surf_coordination_tables.tableNei[i][0] = nei_id;
          gvf_parametric_surf_coordination_tables.tableNei[i][3] = desired_deltaw1;
          gvf_parametric_surf_coordination_tables.tableNei[i][6] = desired_deltaw2;
          return;
        }
    }
  }
}

void gvf_parametric_surf_coordination_parseWTable(uint8_t *buf)
{
  int16_t sender_id = (int16_t)(SenderIdOfPprzMsg(buf));
  for (int i = 0; i < GVF_PARAMETRIC_SURF_COORDINATION_MAX_NEIGHBORS; i++)
    if ((int16_t)(gvf_parametric_surf_coordination_tables.tableNei[i][0]) == sender_id) {
      gvf_parametric_surf_coordination_tables.last_comm[i] = get_sys_time_msec();
      gvf_parametric_surf_coordination_tables.tableNei[i][1] = DL_GVF_PARAMETRIC_SURF_W_w1(buf);
      gvf_parametric_coordination_tables.tableNei[i][2] = DL_GVF_PARAMETRIC_SURF_W_w1_dot(buf);
      gvf_parametric_surf_coordination_tables.tableNei[i][5] = DL_GVF_PARAMETRIC_SURF_W_w2(buf);
      gvf_parametric_coordination_tables.tableNei[i][6] = DL_GVF_PARAMETRIC_SURF_W_w2_dot(buf);
      break;
    }
}
