/*
 * Copyright (C) 2020 Hector Garcia de Marina <hgarciad@ucm.es>
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
 * @file modules/guidance/gvf_parametric/gvf_parametric.cpp
 *
 * Guiding vector field algorithm for 2D and 3D complex trajectories.
 */

#include <iostream>
// https://eigen.tuxfamily.org/dox/GettingStarted.html
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct gvf_parametric_affine_transform
{
  Eigen::Quaternion<float> rot = Eigen::Quaternion<float>::Identity();
  Eigen::Translation3f transalation = Eigen::Translation3f(0., 0., 0.);
  Eigen::Transform<float, 3, Eigen::TransformTraits::Isometry> t = transalation * rot;
} gvf_parametric_affine_tr;

#define SIGN(x) (((x) > 0) - ((x) < 0))

#ifdef __cplusplus
extern "C"
{
#endif

#include "gvf_parametric.h"
#include "autopilot.h"

  // Control
  uint32_t gvf_parametric_t0 = 0; // We need it for calculting the time lapse delta_T

  gvf_parametric_con gvf_parametric_control;
  gvf_parametric_coord gvf_parametric_coordination;
  gvf_parametric_coord_tab gvf_parametric_coordination_tables;

  uint32_t last_transmision = 0;

  // Trajectory
  gvf_parametric_tra gvf_parametric_trajectory;

  // Parameters array lenght
  int gvf_parametric_plen = 1;
  int gvf_parametric_plen_wps = 0;

  // Error signals array lenght
  int gvf_parametric_elen = 3;

  // Bézier
  bezier_t gvf_bezier_2D[GVF_PARAMETRIC_2D_BEZIER_N_SEG];
  uint32_t gvf_parametric_splines_ctr = 0; // We need it for Bézier curves splines Telemetry

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
  static void send_gvf_parametric(struct transport_tx *trans, struct link_device *dev)
  {
    uint8_t traj_type = (uint8_t)gvf_parametric_trajectory.type;

    uint32_t now = get_sys_time_msec();
    uint32_t delta_T = now - gvf_parametric_t0;

    // float wb = gvf_parametric_control.w * gvf_parametric_control.beta * gvf_parametric_control.s;
    if (delta_T < 200)
    {
      gvf_parametric_splines_ctr = (gvf_parametric_splines_ctr + 1) % 3;
      float quaternion[4] = {gvf_parametric_affine_tr.rot.x(),
                             gvf_parametric_affine_tr.rot.y(),
                             gvf_parametric_affine_tr.rot.z(),
                             gvf_parametric_affine_tr.rot.w()};

      // std::cout << "----------\nRotation: " << std::endl << gvf_parametric_affine_tr.t.rotation() << std::endl;
      // std::cout << "Buffered Rotation: " << std::endl << gvf_parametric_affine_tr.rot.matrix() << std::endl << "-----\n";
      // std::cout << "Translation: " << std::endl << gvf_parametric_affine_tr.t.translation() << std::endl;
      // std::cout << "Buffered Translation: " << std::endl << gvf_parametric_affine_tr.transalation.x() << std::endl << gvf_parametric_affine_tr.transalation.y() << std::endl << gvf_parametric_affine_tr.transalation.z() << std::endl;

      float translation[3] = {gvf_parametric_affine_tr.transalation.x(),
                              gvf_parametric_affine_tr.transalation.y(),
                              gvf_parametric_affine_tr.transalation.z()};

      float gvf_parametric_config[11] = {
          gvf_parametric_control.w,
          (float)now,
          gvf_parametric_control.s * gvf_parametric_control.beta,
          gvf_parametric_control.k_roll,
          gvf_parametric_control.k_climb,
          gvf_parametric_control.k_psi,
          gvf_parametric_control.L,
          gvf_parametric_control.w_dot,
          gvf_parametric_control.kx,
          gvf_parametric_control.ky,
          gvf_parametric_control.kz,
      };

      pprz_msg_send_GVF_PARAMETRIC(trans, dev, AC_ID, &traj_type, gvf_parametric_config, gvf_parametric_plen,
                                   gvf_parametric_trajectory.p_parametric, gvf_parametric_elen, gvf_parametric_trajectory.phi_errors,
                                   quaternion, translation);
    }
  }

  static void send_gvf_parametric_coordination(struct transport_tx *trans, struct link_device *dev)
  {

    if (gvf_parametric_coordination.coordination)
    {

      uint32_t now = get_sys_time_msec();
      float pprz_nei_table[GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS][5];
      for (int i = 0; i < GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS; i++)
      {
        pprz_nei_table[i][0] = gvf_parametric_coordination_tables.tableNei[i].nei_id;
        pprz_nei_table[i][1] = gvf_parametric_coordination_tables.tableNei[i].w;
        pprz_nei_table[i][2] = gvf_parametric_coordination_tables.tableNei[i].w_dot;
        pprz_nei_table[i][3] = gvf_parametric_coordination_tables.tableNei[i].desired_dw;
        pprz_nei_table[i][4] = gvf_parametric_coordination_tables.tableNei[i].delta_t;
      }
      pprz_msg_send_GVF_PAR_COORD(trans, dev, AC_ID,
                                  5 * GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS, &(pprz_nei_table[0][0]),
                                  GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS, gvf_parametric_coordination_tables.error_deltaw,
                                  &now);
    }
  }

  static void manual_send_gvf_parametric_coordination()
  {
    send_gvf_parametric_coordination(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
  }

#if GVF_OCAML_GCS
  static void send_circle_parametric(struct transport_tx *trans, struct link_device *dev)
  {
    uint32_t now = get_sys_time_msec();
    uint32_t delta_T = now - gvf_parametric_t0;

    if (delta_T < 200)
      if (gvf_parametric_trajectory.type == ELLIPSE_3D)
      {
        pprz_msg_send_CIRCLE(trans, dev, AC_ID, &gvf_parametric_trajectory.p_parametric[0],
                             &gvf_parametric_trajectory.p_parametric[1], &gvf_parametric_trajectory.p_parametric[2]);
      }
  }
#endif // GVF_OCAML_GCS

#endif // PERIODIC TELEMETRY

#ifdef __cplusplus
}
#endif

void gvf_parametric_init(void)
{
  gvf_parametric_control.w = 0;
  gvf_parametric_control.delta_T = 0;
  gvf_parametric_control.s = 1;
  gvf_parametric_control.k_roll = GVF_PARAMETRIC_CONTROL_KROLL;
  gvf_parametric_control.k_climb = GVF_PARAMETRIC_CONTROL_KCLIMB;
  gvf_parametric_control.k_psi = GVF_PARAMETRIC_CONTROL_KPSI;
  gvf_parametric_control.L = GVF_PARAMETRIC_CONTROL_L;
  gvf_parametric_control.beta = GVF_PARAMETRIC_CONTROL_BETA;
  gvf_parametric_control.w_dot = 0;
  gvf_parametric_control.kx = 1.;
  gvf_parametric_control.ky = 1.;
  gvf_parametric_control.kz = 1.;
  gvf_parametric_control.step_adaptation = GVF_PARAMETRIC_STEP_ADAPTATION;

  gvf_parametric_coordination.coordination = GVF_PARAMETRIC_COORDINATION_COORDINATION;
  gvf_parametric_coordination.kc = GVF_PARAMETRIC_COORDINATION_KC;
  gvf_parametric_coordination.ktol = GVF_PARAMETRIC_COORDINATION_KTOL;
  gvf_parametric_coordination.timeout = GVF_PARAMETRIC_COORDINATION_TIMEOUT;
  gvf_parametric_coordination.broadtime = GVF_PARAMETRIC_COORDINATION_BROADTIME;
  gvf_parametric_coordination.speed_ctl = GVF_PARAMETRIC_COORDINATION_SPEED_CTL;

  for (int i = 0; i < GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS; i++)
  {
    gvf_parametric_coordination_tables.tableNei[i].nei_id = -1;
    gvf_parametric_coordination_tables.tableNei[i].w = 0;
    gvf_parametric_coordination_tables.tableNei[i].w_dot = 0;
    gvf_parametric_coordination_tables.tableNei[i].desired_dw = 0;
    gvf_parametric_coordination_tables.tableNei[i].delta_t = 0;
    gvf_parametric_coordination_tables.error_deltaw[i] = 0;
  }

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GVF_PAR_COORD, send_gvf_parametric_coordination);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GVF_PARAMETRIC, send_gvf_parametric);
#if GVF_OCAML_GCS
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CIRCLE, send_circle_parametric);
#endif // GVF_OCAML_GCS
#endif // PERIODIC_TELEMETRY
}

// Simple functions modifying tranforms (may be useful for mission mode)

static void gvf_parametric_update_tr()
{
  gvf_parametric_affine_tr.t = gvf_parametric_affine_tr.transalation * gvf_parametric_affine_tr.rot;
}

void gvf_parametric_set_direction(int8_t s)
{
  gvf_parametric_control.s = s;
}

void gvf_parametric_set_w_gain(float b)
{
  gvf_parametric_control.beta = b;
}

void gvf_parametric_set_offset(float x, float y, float z)
{
  gvf_parametric_affine_tr.transalation = Eigen::Translation3f(x, y, z);
  gvf_parametric_affine_tr.t = gvf_parametric_affine_tr.transalation * gvf_parametric_affine_tr.rot;
}

void gvf_parametric_set_offset_wp(uint8_t wp)
{
  gvf_parametric_set_offset(WaypointX(wp), WaypointY(wp), WaypointAlt(wp));
}

void gvf_parametric_set_offset_wpa(uint8_t wp, float alt)
{
  gvf_parametric_set_offset(WaypointX(wp), WaypointY(wp), alt);
}

void gvf_paremetric_set_euler_rot(float rz, float ry, float rzbis)
{
  gvf_parametric_affine_tr.rot = Eigen::AngleAxisf(rzbis * M_PI / 180, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(ry * M_PI / 180, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(rz * M_PI / 180, Eigen::Vector3f::UnitZ());

  gvf_parametric_update_tr();
}

void gvf_paremetric_set_cardan_rot(float rx, float ry, float rz)
{
  gvf_parametric_affine_tr.rot = Eigen::AngleAxisf(rz * M_PI / 180, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(ry * M_PI / 180, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(rx * M_PI / 180, Eigen::Vector3f::UnitX());

  gvf_parametric_update_tr();
}

void gvf_paremetric_set_quaternion_rot(float x, float y, float z, float w)
{
  gvf_parametric_affine_tr.rot = Eigen::Quaternion<float>(w, x, y, z).normalized();

  gvf_parametric_update_tr();
}

void gvf_parametric_set_wps_rot(uint8_t wp1, uint8_t wp2)
{
  Eigen::Vector3f U = Eigen::Vector3f(WaypointX(wp2) - WaypointX(wp1),
                                      WaypointY(wp2) - WaypointY(wp1),
                                      WaypointAlt(wp2) - WaypointAlt(wp1));

  Eigen::Vector3f Base = Eigen::Vector3f(1, 0, 0);
  gvf_parametric_affine_tr.rot = Eigen::Quaternion<float>::FromTwoVectors(Base, U).normalized();

  gvf_parametric_update_tr();
}

void gvf_parametric_set_wp_rot(uint8_t wp)
{
  Eigen::Vector3f U = Eigen::Vector3f(WaypointX(wp),
                                      WaypointY(wp),
                                      WaypointAlt(wp));

  Eigen::Vector3f Base = Eigen::Vector3f(1, 0, 0);
  gvf_parametric_affine_tr.rot = Eigen::Quaternion<float>::FromTwoVectors(Base, U).normalized();

  gvf_parametric_update_tr();
}

void gvf_parametric_set_affine_tr(float x, float y, float z, float rx, float ry, float rz)
{
  gvf_parametric_set_offset(x, y, z);
  gvf_paremetric_set_cardan_rot(rx, ry, rz);
}

void gvf_parametric_set_affine_q_tr(float x, float y, float z, float qx, float qy, float qz, float qw)
{
  gvf_parametric_set_offset(x, y, z);
  gvf_paremetric_set_quaternion_rot(qx, qy, qz, qw);
}

void gvf_parametric_set_affine_tr_wp(uint8_t wp, float rx, float ry, float rz)
{
  gvf_parametric_set_offset_wp(wp);
  gvf_paremetric_set_cardan_rot(rx, ry, rz);
}

void gvf_parametric_set_affine_tr_wpa(uint8_t wp, float alt, float rx, float ry, float rz)
{
  gvf_parametric_set_offset_wpa(wp, alt);
  gvf_paremetric_set_cardan_rot(rx, ry, rz);
}

void gvf_parametric_set_affine_tr_wps(uint8_t wp1, uint8_t wp2)
{
  gvf_parametric_set_offset(WaypointX(wp1), WaypointY(wp1), WaypointAlt(wp1));
  gvf_parametric_set_wps_rot(wp1, wp2);
}

// Core 2D control algorithm

void gvf_parametric_control_2d(float kx, float ky, float f1, float f2, float f1d, float f2d, float f1dd, float f2dd)
{

  uint32_t now = get_sys_time_msec();
  gvf_parametric_control.delta_T = now - gvf_parametric_t0;
  gvf_parametric_t0 = now;

  if (gvf_parametric_control.delta_T > 300)
  {                               // We need at least two iterations for Delta_T
    gvf_parametric_control.w = 0; // Reset w since we assume the algorithm starts
    return;
  }

  gvf_parametric_control.kx = kx;
  gvf_parametric_control.ky = ky;
  gvf_parametric_control.kz = 0.;

  // Carrot position
#ifdef FIXEDWING_FIRMWARE
  desired_x = f1;
  desired_y = f2;
#endif

  float L = gvf_parametric_control.L;
  float beta = gvf_parametric_control.beta * gvf_parametric_control.s;

  Eigen::Vector3f X;
  Eigen::Matrix3f J;

  // Error signals phi_x and phi_y
  struct EnuCoor_f *pos_enu = stateGetPositionEnu_f();
  float x = pos_enu->x;
  float y = pos_enu->y;

  float phi1 = L * (x - f1);
  float phi2 = L * (y - f2);

  gvf_parametric_trajectory.phi_errors[0] = phi1; // Error signals for the telemetry
  gvf_parametric_trajectory.phi_errors[1] = phi2;
  gvf_parametric_elen = 2;

  // Chi
  X(0) = L * beta * f1d - kx * phi1;
  X(1) = L * beta * f2d - ky * phi2;
  X(2) = L + beta * (kx * phi1 * f1d + ky * phi2 * f2d);
  X *= L;

  // Coordination if needed for multi vehicles
  float consensus_term_w = 0;

  if (gvf_parametric_coordination.coordination)
  {
    for (int i = 0; i < GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS; i++)
    {
      if (gvf_parametric_coordination_tables.tableNei[i].nei_id != -1)
      {
        uint32_t timeout = now - (uint32_t)(gvf_parametric_coordination_tables.last_comm[i]);
        gvf_parametric_coordination_tables.tableNei[i].delta_t = timeout;
        if (timeout <= gvf_parametric_coordination.timeout)
        {

          float wi = gvf_parametric_control.w;
          float wj = gvf_parametric_coordination_tables.tableNei[i].w;
          float delta_t_j = gvf_parametric_coordination_tables.tableNei[i].delta_t;
          float wdot_j = gvf_parametric_coordination_tables.tableNei[i].w_dot;
          float desired_dw = gvf_parametric_coordination_tables.tableNei[i].desired_dw;

          float error_w = -beta * (wi - (wj + wdot_j * delta_t_j)) + desired_dw;

          consensus_term_w += error_w;

          gvf_parametric_coordination_tables.error_deltaw[i] = error_w;
        }
      }
    }
  }

  X(2) += gvf_parametric_coordination.kc * consensus_term_w;

  // Jacobian
  J.setZero();
  J(0, 0) = -kx * L;
  J(1, 1) = -ky * L;
  J(2, 0) = (beta * L) * (beta * f1dd + kx * f1d);
  J(2, 1) = (beta * L) * (beta * f2dd + ky * f2d);
  J(2, 2) = beta * beta * (kx * (phi1 * f1dd - L * f1d * f1d) + ky * (phi2 * f2dd - L * f2d * f2d));
  J *= L;

  // Guidance algorithm
  float ground_speed = stateGetHorizontalSpeedNorm_f();
  float w_dot = (ground_speed * X(2)) / sqrtf(X(0) * X(0) + X(1) * X(1));

  Eigen::Vector3f xi_dot;
  struct EnuCoor_f *vel_enu = stateGetSpeedEnu_f();
  float course = stateGetHorizontalSpeedDir_f();

  xi_dot << vel_enu->x, vel_enu->y, w_dot;

  Eigen::Matrix3f G;
  Eigen::Matrix3f Gp;
  Eigen::Matrix<float, 2, 3> Fp;
  Eigen::Vector2f h;
  Eigen::Matrix<float, 1, 2> ht;

  G << 1, 0, 0,
      0, 1, 0,
      0, 0, 0;
  Fp << 0, -1, 0,
      1, 0, 0;
  Gp << 0, -1, 0,
      1, 0, 0,
      0, 0, 0;

  h << sinf(course), cosf(course);
  ht = h.transpose();

  Eigen::Matrix<float, 1, 3> Xt = X.transpose();
  Eigen::Vector3f Xh = X.normalized();
  Eigen::Matrix<float, 1, 3> Xht = Xh.transpose();
  Eigen::Matrix3f I;
  I.setIdentity();

  float aux = ht * Fp * X;
  Eigen::Vector3f aux2 = J * xi_dot;

  // Coordination if needed for multi vehicles
  float consensus_term_wdot = 0;

  if (gvf_parametric_coordination.coordination)
  {
    for (int i = 0; i < GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS; i++)
    {
      if (gvf_parametric_coordination_tables.tableNei[i].nei_id != -1)
      {
        uint32_t timeout = now - uint32_t(gvf_parametric_coordination_tables.last_comm[i]);
        gvf_parametric_coordination_tables.tableNei[i].delta_t = timeout;
        if (timeout > gvf_parametric_coordination.timeout)
        {

          float wi_dot = gvf_parametric_control.w_dot;
          float wj_dot = gvf_parametric_coordination_tables.tableNei[i].w_dot;

          consensus_term_wdot += -beta * (wi_dot - wj_dot);
        }
      }
    }
  }

  aux2(2) += gvf_parametric_coordination.kc * consensus_term_wdot;

  float heading_rate = -1 / (Xt * G * X) * Xt * Gp * (I - Xh * Xht) * aux2 - (gvf_parametric_control.k_psi * aux /
                                                                              sqrtf(Xt * G * X));

  // From gvf_common.h TODO: implement d/dt of kppa and ori_err
  gvf_c_omega.omega = heading_rate;
  gvf_c_info.kappa = (f1d * f2dd - f1dd * f2d) / powf(f1d * f1d + f2d * f2d, 1.5);
  gvf_c_info.ori_err = 1 - (Xh(0) * cosf(course) + Xh(1) * sinf(course));

  // Virtual coordinate update, even if the vehicle is not in autonomous mode, the parameter w will get "closer" to
  // the vehicle. So it is not only okei but advisable to update it.
  gvf_parametric_control.w += w_dot * gvf_parametric_control.delta_T * 1e-3;

  gvf_parametric_low_level_control_2d(heading_rate, 0.);

  if ((gvf_parametric_coordination.coordination) && (now - last_transmision > gvf_parametric_coordination.broadtime) && (autopilot_get_mode() == AP_MODE_AUTO2))
  {
    gvf_parametric_coordination_send_w_to_nei();
    last_transmision = now;
  }
}

// Core 3D control algorithm
#ifdef FIXEDWING_FIRMWARE
void gvf_parametric_control_3d(float kx, float ky, float kz, float f1, float f2, float f3, float f1d, float f2d,
                               float f3d, float f1dd, float f2dd, float f3dd)
{
  // ----- Setup ----- //
  uint32_t now = get_sys_time_msec();
  gvf_parametric_control.delta_T = now - gvf_parametric_t0;
  gvf_parametric_t0 = now;

  if (gvf_parametric_control.delta_T > 300)
  {                               // We need at least two iterations for Delta_T
    gvf_parametric_control.w = 0; // Reset w since we assume the algorithm starts
    return;
  }

  gvf_parametric_control.kx = kx;
  gvf_parametric_control.ky = ky;
  gvf_parametric_control.kz = kz;

#if PERIODIC_TELEMETRY
  static uint32_t last_manual_send = 0;
  if (gvf_parametric_coordination.coordination && now - last_manual_send > 200)
  {
    manual_send_gvf_parametric_coordination();
    last_manual_send = now;
  }
#endif

  if (gvf_parametric_control.delta_T == 0)
  {
    // Avoid potential ill-formed edge case
    return;
  }

  // ----- Pre-transformations ----- //

  Eigen::Vector3f f_vec(f1, f2, f3);
  Eigen::Vector3f fd_vec(f1d, f2d, f3d);
  Eigen::Vector3f fdd_vec(f1dd, f2dd, f3dd);

  gvf_parametric_update_tr();

  // Apply the rotation + translation
  f_vec = gvf_parametric_affine_tr.t * f_vec;
  // After derivation, only rotation remains
  fd_vec = gvf_parametric_affine_tr.t.linear() * fd_vec;
  fdd_vec = gvf_parametric_affine_tr.t.linear() * fdd_vec;

  f1 = f_vec(0);
  f2 = f_vec(1);
  f3 = f_vec(2);

  f1d = fd_vec(0);
  f2d = fd_vec(1);
  f3d = fd_vec(2);

  f1dd = fdd_vec(0);
  f2dd = fdd_vec(1);
  f3dd = fdd_vec(2);

  // Additional values required for using step adaptation

  float f_v2 = f1d * f1d + f2d * f2d + f3d * f3d;
  float f_v = sqrtf(f_v2);

  float f_d_dot_f_dd = f1d * f1dd + f2d * f2dd + f3d * f3dd;

  float og_f1d = f1d;
  float og_f2d = f2d;
  float og_f3d = f3d;

  float og_f1dd = f1dd;
  float og_f2dd = f2dd;
  float og_f3dd = f3dd;

  if (gvf_parametric_control.step_adaptation)
  {
    if (f_v < 1e-6) // i.e, f'(w) = 0
    {
      // Use an arbitrary direction
      f1d = 1 / sqrtf(3.);
      f2d = 1 / sqrtf(3.);
      f3d = 1 / sqrtf(3.);

      f1dd = 0.;
      f2dd = 0.;
      f3dd = 0.;
    }
    else
    {
      f1dd = (f1dd - f_d_dot_f_dd * f1d / f_v2) / f_v2;
      f2dd = (f2dd - f_d_dot_f_dd * f2d / f_v2) / f_v2;
      f3dd = (f3dd - f_d_dot_f_dd * f3d / f_v2) / f_v2;

      f1d = f1d / f_v;
      f2d = f2d / f_v;
      f3d = f3d / f_v;
    }
  }

  // ----- Core algorithm ----- //
  
  float L = gvf_parametric_control.L;
  float beta = gvf_parametric_control.beta * gvf_parametric_control.s;

  // Error signals phi_x phi_y and phi_z
  struct EnuCoor_f *pos_enu = stateGetPositionEnu_f();
  float x = pos_enu->x;
  float y = pos_enu->y;
  float z = pos_enu->z;

  float phi1 = L * (x - f1);
  float phi2 = L * (y - f2);
  float phi3 = L * (z - f3);

  // Error signals in meters for the telemetry
  gvf_parametric_trajectory.phi_errors[0] = phi1 / L; 
  gvf_parametric_trajectory.phi_errors[1] = phi2 / L;
  gvf_parametric_trajectory.phi_errors[2] = phi3 / L;
  gvf_parametric_elen = 3;

  // Chi
  Eigen::Vector4f X;
  X(0) = -f1d * L * L * beta - kx * phi1;
  X(1) = -f2d * L * L * beta - ky * phi2;
  X(2) = -f3d * L * L * beta - kz * phi3;
  X(3) = -L * L + beta * (kx * phi1 * f1d + ky * phi2 * f2d + kz * phi3 * f3d);

  X *= L;

  // Coordination if needed for multi vehicles
  float consensus_term_w = 0;
  int neighbors_count = 0;

  if (gvf_parametric_coordination.coordination)
  {

    for (int i = 0; i < GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS; i++)
    {
      if (gvf_parametric_coordination_tables.tableNei[i].nei_id != -1)
      {
        uint32_t timeout = now - gvf_parametric_coordination_tables.last_comm[i];
        gvf_parametric_coordination_tables.tableNei[i].delta_t = timeout;
        if (timeout < gvf_parametric_coordination.timeout)
        {

          float wi = gvf_parametric_control.w;
          float wj = gvf_parametric_coordination_tables.tableNei[i].w;
          float delta_t_j = gvf_parametric_coordination_tables.tableNei[i].delta_t;
          float wdot_j = gvf_parametric_coordination_tables.tableNei[i].w_dot;
          float desired_dw = gvf_parametric_coordination_tables.tableNei[i].desired_dw;

          float error_w = -beta * (wi - (wj + wdot_j * delta_t_j * 1e-3)) + desired_dw;

          consensus_term_w += error_w;

          gvf_parametric_coordination_tables.error_deltaw[i] = error_w;
          neighbors_count++;
        }
      }
    }
  }

  // Virtual coordinate update
  float ground_speed = stateGetHorizontalSpeedNorm_f();
  float w_dot;
  float w_coordination_adjusted;

std::cout << "AC ID: " << AC_ID << " || ";

  if (gvf_parametric_control.step_adaptation)
  {
    // Custom rules to limit unwanted behavior

    // Average the consensus value to keep it comparable to individual contribution
    if (neighbors_count > 0)
    {
      consensus_term_w /= neighbors_count;
    }

    // Switch strategy depending if control in speed is allowed or not
    if (gvf_parametric_coordination.speed_ctl)
    {
      // When adding speed control, bound using the individual speed (given by X(3), the virtual field)
      // and apply only when coordination pushes forward (prevent coordination-issued oscillation for fixed-wings)
      w_coordination_adjusted = consensus_term_w;
      BoundAbs(w_coordination_adjusted, ABS(X(3)*gvf_parametric_coordination.kc));

      if (w_coordination_adjusted > 0)
      {
        X(3) += w_coordination_adjusted*SIGN(X(3));
      }
    }
    else
    {

      // Avoid pathological case (div by 0)
      if( X(3) == 0)
      {
        w_coordination_adjusted = 0;
      }
      else
      {
        float d = ABS(X(3) * gvf_parametric_coordination.ktol);

        // First Activation function u such that u(x) ~ x at infinity and u(0) = 0, u'(0) = 0
        // u(x) = x*(1-e^(-(xs)²))
        // With s = sqrtf(0.5*(3-sqrtf(6)))/d
        // This achieves the properties u(d) ~= d * 0.24 and u'''(d) = 0
        float s = sqrtf(0.5*(3-sqrtf(6)))/d;

        // Second activation function is for soft bounds: v(x) = tanh(x)
        // The resulting activation function is a composition with appropriate rescaling: w(x) = v(u(x)/t)*t,
        // With w(0) = 0, w'(0) = 0, w(x) ~ sgn(x) at infinity,
        // t = (gvf_parametric_coordination.kc)  /atanh(0.9)
        float t = (gvf_parametric_coordination.kc) * ABS(X(3));///(0.5*logf(1.9/0.1));

        // The resulting behavior is:
        // _ w(x) ~= 0 for -d < x < d
        // _ w(x) ~= x for -kc < x < -d and d < x < kc
        // _ w(x) ~= sgn(x)*kc for x < -kc a d kc < x 
        w_coordination_adjusted = tanhf(t*(consensus_term_w)*(1-expf(-(consensus_term_w*consensus_term_w*s*s))))*t;

        // std::cout << "(d,s,t) : ( " << d << " , " << s << " , " << t << " ) | ";

        if (isnan(w_coordination_adjusted))
        {
          w_coordination_adjusted = SIGN(w_coordination_adjusted) *t;
        }
      }
      X(3) += w_coordination_adjusted*SIGN(X(3));
    }

    

    w_dot = step_adaptation(ground_speed * X(3) * gvf_parametric_control.delta_T * 1e-3, og_f1d, og_f2d, og_f3d, og_f1dd, og_f2dd, og_f3dd) / (gvf_parametric_control.delta_T * 1e-3);
    
  }
  else
  {
    // Original GVF Parametric coordination scheme (linear consensus)
    w_coordination_adjusted = gvf_parametric_coordination.kc * consensus_term_w;
    X(3) += w_coordination_adjusted;
    w_dot = (ground_speed * X(3)) / sqrtf(X(0) * X(0) + X(1) * X(1));
  }
  std::cout << "Raw coord: " << consensus_term_w << " | ";
  std::cout << "Coordination contrib: " << w_coordination_adjusted << " | ";

  float coord_speed_mod;
  if (gvf_parametric_coordination.speed_ctl)
  {
    coord_speed_mod = w_coordination_adjusted;
  }
  else
  {
    coord_speed_mod = 0;
  }

  std::cout << "w   : " << gvf_parametric_control.w << " | ";
  std::cout << "wdot: " << w_dot << std::endl;


  // ----- Control algorithm ----- //
  // Carrot position
  desired_x = f1;
  desired_y = f2;

  // Jacobian
  Eigen::Matrix4f J;

  J.setZero();
  J(0, 0) = -kx * L;
  J(1, 1) = -ky * L;
  J(2, 2) = -kz * L;
  J(3, 0) = kx * f1d * beta * L;
  J(3, 1) = ky * f2d * beta * L;
  J(3, 2) = kz * f3d * beta * L;
  J(0, 3) = -(beta * L) * (beta * L * f1dd - kx * f1d);
  J(1, 3) = -(beta * L) * (beta * L * f2dd - ky * f2d);
  J(2, 3) = -(beta * L) * (beta * L * f3dd - kz * f3d);
  J(3, 3) = beta * beta * (kx * (phi1 * f1dd - L * f1d * f1d) + ky * (phi2 * f2dd - L * f2d * f2d) + kz * (phi3 * f3dd - L * f3d * f3d));
  J *= L;

  Eigen::Vector4f xi_dot;
  struct EnuCoor_f *vel_enu = stateGetSpeedEnu_f();
  float course = stateGetHorizontalSpeedDir_f();

  xi_dot << vel_enu->x, vel_enu->y, vel_enu->z, w_dot;

  Eigen::Matrix2f E;
  Eigen::Matrix<float, 2, 4> F;
  Eigen::Matrix<float, 2, 4> Fp;
  Eigen::Matrix4f G;
  Eigen::Matrix4f Gp;
  Eigen::Matrix4f I;
  Eigen::Vector2f h;
  Eigen::Matrix<float, 1, 2> ht;

  h << sinf(course), cosf(course);
  ht = h.transpose();
  I.setIdentity();
  F << 1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0;
  E << 0.0, -1.0,
      1.0, 0.0;
  G = F.transpose() * F;
  Fp = E * F;
  Gp = F.transpose() * E * F;

  Eigen::Matrix<float, 1, 4> Xt = X.transpose();
  Eigen::Vector4f Xh = X / X.norm();
  Eigen::Matrix<float, 1, 4> Xht = Xh.transpose();

  float aux = ht * Fp * X;
  Eigen::Vector4f aux2 = J * xi_dot;

  // Coordination if needed for multi vehicles
  float consensus_term_wdot = 0;

  if (gvf_parametric_coordination.coordination)
  {
    for (int i = 0; i < GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS; i++)
    {
      if (gvf_parametric_coordination_tables.tableNei[i].nei_id != -1)
      {
        uint32_t timeout = now - (uint32_t)(gvf_parametric_coordination_tables.last_comm[i]);
        gvf_parametric_coordination_tables.tableNei[i].delta_t = timeout;
        if (timeout > gvf_parametric_coordination.timeout)
        {

          float wi_dot = gvf_parametric_control.w_dot;
          float wj_dot = gvf_parametric_coordination_tables.tableNei[i].w_dot;

          consensus_term_wdot += -beta * (wi_dot - wj_dot);
        }
      }
    }
  }

  aux2(3) += gvf_parametric_coordination.kc * consensus_term_wdot;

  float heading_rate = -1 / (Xt * G * X) * Xt * Gp * (I - Xh * Xht) * aux2 - (gvf_parametric_control.k_psi * aux /
                                                                              sqrtf(Xt * G * X));
  float climbing_rate = (ground_speed * X(2)) / sqrtf(X(0) * X(0) + X(1) * X(1));

  // Virtual coordinate update, even if the vehicle is not in autonomous mode, the parameter w will get "closer" to
  // the vehicle. So it is not only okei but advisable to update it.
  gvf_parametric_control.w += w_dot * gvf_parametric_control.delta_T * 1e-3;
  gvf_parametric_control.w_dot = w_dot;

  if ((gvf_parametric_coordination.coordination) && (now - last_transmision > gvf_parametric_coordination.broadtime))
  {
    gvf_parametric_coordination_send_w_to_nei();
    last_transmision = now;
  }

  gvf_parametric_low_level_control_3d(heading_rate, climbing_rate, coord_speed_mod);
}
#endif // FIXED_WING FIRMWARE

/** 2D TRAJECTORIES **/
// 2D TREFOIL KNOT

bool gvf_parametric_2d_trefoil_XY(float xo, float yo, float w1, float w2, float ratio, float r, float alpha)
{
  gvf_parametric_trajectory.type = TREFOIL_2D;
  gvf_parametric_trajectory.p_parametric[0] = xo;
  gvf_parametric_trajectory.p_parametric[1] = yo;
  gvf_parametric_trajectory.p_parametric[2] = w1;
  gvf_parametric_trajectory.p_parametric[3] = w2;
  gvf_parametric_trajectory.p_parametric[4] = ratio;
  gvf_parametric_trajectory.p_parametric[5] = r;
  gvf_parametric_trajectory.p_parametric[6] = alpha;
  gvf_parametric_plen = 7 + gvf_parametric_plen_wps;
  gvf_parametric_plen_wps = 0;

  float f1, f2, f1d, f2d, f1dd, f2dd;

  gvf_parametric_2d_trefoil_info(&f1, &f2, &f1d, &f2d, &f1dd, &f2dd);
  gvf_parametric_control_2d(gvf_parametric_2d_trefoil_par.kx, gvf_parametric_2d_trefoil_par.ky, f1, f2, f1d, f2d, f1dd,
                            f2dd);

  return true;
}

bool gvf_parametric_2d_trefoil_wp(uint8_t wp, float w1, float w2, float ratio, float r, float alpha)
{
  gvf_parametric_trajectory.p_parametric[7] = wp;
  gvf_parametric_plen_wps = 1;
  gvf_parametric_2d_trefoil_XY(WaypointX(wp), WaypointY(wp), w1, w2, ratio, r, alpha);
  return true;
}

// 2D CUBIC BEZIER CURVE
bool gvf_parametric_2D_bezier_XY(void)
{
  gvf_parametric_trajectory.type = BEZIER_2D;
  float fx, fy, fxd, fyd, fxdd, fydd;
  gvf_parametric_2d_bezier_splines_info(gvf_bezier_2D, &fx, &fy, &fxd, &fyd, &fxdd, &fydd);
  gvf_parametric_control_2d(gvf_parametric_2d_bezier_par.kx, gvf_parametric_2d_bezier_par.ky, fx, fy, fxd, fyd, fxdd,
                            fydd);
  return true;
}

/* @param first_wp is the first waypoint of the Bézier Spline
 * there should be 3*GVF_PARAMETRIC_2D_BEZIER_N_SEG+1 points
 */
bool gvf_parametric_2D_bezier_wp(uint8_t first_wp)
{
  float x[3 * GVF_PARAMETRIC_2D_BEZIER_N_SEG + 1];
  float y[3 * GVF_PARAMETRIC_2D_BEZIER_N_SEG + 1];
  int k;
  for (k = 0; k < 3 * GVF_PARAMETRIC_2D_BEZIER_N_SEG + 1; k++)
  {
    x[k] = WaypointX(first_wp + k);
    y[k] = WaypointY(first_wp + k);
  }
  create_bezier_spline(gvf_bezier_2D, x, y);

  /* Send data piecewise. Some radio modules do not allow for a big data frame.*/

  // Send x points -> Indicate x with sign (+) in the first parameter
  if (gvf_parametric_splines_ctr == 0)
  {
    gvf_parametric_trajectory.p_parametric[0] = -GVF_PARAMETRIC_2D_BEZIER_N_SEG; // send x (negative value)
    for (k = 0; k < 3 * GVF_PARAMETRIC_2D_BEZIER_N_SEG + 1; k++)
    {
      gvf_parametric_trajectory.p_parametric[k + 1] = x[k];
    }
  }
  // Send y points -> Indicate y with sign (-) in the first parameter
  else if (gvf_parametric_splines_ctr == 1)
  {
    gvf_parametric_trajectory.p_parametric[0] = GVF_PARAMETRIC_2D_BEZIER_N_SEG; // send y (positive value)
    for (k = 0; k < 3 * GVF_PARAMETRIC_2D_BEZIER_N_SEG + 1; k++)
    {
      gvf_parametric_trajectory.p_parametric[k + 1] = y[k];
    }
  }
  // send kx, ky, beta and anything else needed..
  else
  {
    gvf_parametric_trajectory.p_parametric[0] = 0.0;
    gvf_parametric_trajectory.p_parametric[1] = gvf_parametric_2d_bezier_par.kx;
    gvf_parametric_trajectory.p_parametric[2] = gvf_parametric_2d_bezier_par.ky;
    gvf_parametric_trajectory.p_parametric[3] = gvf_parametric_control.beta;
  }
  gvf_parametric_plen = 16;
  gvf_parametric_plen_wps = 1;

  // restart the spline
  if (gvf_parametric_control.w >= (float)GVF_PARAMETRIC_2D_BEZIER_N_SEG)
  {
    gvf_parametric_control.w = 0;
  }
  else if (gvf_parametric_control.w < 0)
  {
    gvf_parametric_control.w = 0;
  }
  gvf_parametric_2D_bezier_XY();
  return true;
}

/** 3D TRAJECTORIES **/
#ifdef FIXEDWING_FIRMWARE

// 3D ELLIPSE
bool gvf_parametric_3d_ellipse_XYZ(float xo, float yo, float r, float zl, float zh, float alpha)
{
  horizontal_mode = HORIZONTAL_MODE_CIRCLE; //  Circle for the 2D GCS

  // Safety first! If the asked altitude is low
  if (zl > zh)
  {
    zl = zh;
  }
  if (zl < 1 || zh < 1)
  {
    zl = 10;
    zh = 10;
  }
  if (r < 1)
  {
    r = 60;
  }

  gvf_parametric_trajectory.type = ELLIPSE_3D;
  gvf_parametric_trajectory.p_parametric[0] = xo;
  gvf_parametric_trajectory.p_parametric[1] = yo;
  gvf_parametric_trajectory.p_parametric[2] = r;
  gvf_parametric_trajectory.p_parametric[3] = zl;
  gvf_parametric_trajectory.p_parametric[4] = zh;
  gvf_parametric_trajectory.p_parametric[5] = alpha;
  gvf_parametric_plen = 6 + gvf_parametric_plen_wps;
  gvf_parametric_plen_wps = 0;

  float f1, f2, f3, f1d, f2d, f3d, f1dd, f2dd, f3dd;

  gvf_parametric_3d_ellipse_info(&f1, &f2, &f3, &f1d, &f2d, &f3d, &f1dd, &f2dd, &f3dd);
  gvf_parametric_control_3d(gvf_parametric_3d_ellipse_par.kx, gvf_parametric_3d_ellipse_par.ky,
                            gvf_parametric_3d_ellipse_par.kz, f1, f2, f3, f1d, f2d, f3d, f1dd, f2dd, f3dd);

  return true;
}

bool gvf_parametric_3d_ellipse_wp(uint8_t wp, float r, float zl, float zh, float alpha)
{
  gvf_parametric_trajectory.p_parametric[6] = wp;
  gvf_parametric_plen_wps = 1;

  gvf_parametric_3d_ellipse_XYZ(WaypointX(wp), WaypointY(wp), r, zl, zh, alpha);
  return true;
}

bool gvf_parametric_3d_ellipse_wp_delta(uint8_t wp, float r, float alt_center, float delta, float alpha)
{
  float zl = alt_center - delta;
  float zh = alt_center + delta;

  gvf_parametric_3d_ellipse_XYZ(WaypointX(wp), WaypointY(wp), r, zl, zh, alpha);
  return true;
}

// 3D Lissajous

bool gvf_parametric_3d_lissajous_XYZ(float xo, float yo, float zo, float cx, float cy, float cz, float wx, float wy,
                                     float wz, float dx, float dy, float dz, float alpha)
{
  // Safety first! If the asked altitude is low
  if ((zo - cz) < 1)
  {
    zo = 10;
    cz = 0;
  }

  gvf_parametric_trajectory.type = LISSAJOUS_3D;
  gvf_parametric_trajectory.p_parametric[0] = xo;
  gvf_parametric_trajectory.p_parametric[1] = yo;
  gvf_parametric_trajectory.p_parametric[2] = zo;
  gvf_parametric_trajectory.p_parametric[3] = cx;
  gvf_parametric_trajectory.p_parametric[4] = cy;
  gvf_parametric_trajectory.p_parametric[5] = cz;
  gvf_parametric_trajectory.p_parametric[6] = wx;
  gvf_parametric_trajectory.p_parametric[7] = wy;
  gvf_parametric_trajectory.p_parametric[8] = wz;
  gvf_parametric_trajectory.p_parametric[9] = dx;
  gvf_parametric_trajectory.p_parametric[10] = dy;
  gvf_parametric_trajectory.p_parametric[11] = dz;
  gvf_parametric_trajectory.p_parametric[12] = alpha;
  gvf_parametric_plen = 13 + gvf_parametric_plen_wps;
  gvf_parametric_plen_wps = 0;

  float f1, f2, f3, f1d, f2d, f3d, f1dd, f2dd, f3dd;

  gvf_parametric_3d_lissajous_info(&f1, &f2, &f3, &f1d, &f2d, &f3d, &f1dd, &f2dd, &f3dd);
  gvf_parametric_control_3d(gvf_parametric_3d_lissajous_par.kx, gvf_parametric_3d_lissajous_par.ky,
                            gvf_parametric_3d_lissajous_par.kz, f1, f2, f3, f1d, f2d, f3d, f1dd, f2dd, f3dd);

  return true;
}

bool gvf_parametric_3d_lissajous_wp_center(uint8_t wp, float zo, float cx, float cy, float cz, float wx, float wy,
                                           float wz, float dx, float dy, float dz, float alpha)
{
  gvf_parametric_trajectory.p_parametric[13] = wp;
  gvf_parametric_plen_wps = 1;

  gvf_parametric_3d_lissajous_XYZ(WaypointX(wp), WaypointY(wp), zo, cx, cy, cz, wx, wy, wz, dx, dy, dz, alpha);
  return true;
}

// 3D sinusoid

bool gvf_parametric_3d_sin(float ay, float freq_y, float phase_y, float az, float freq_z, float phase_z)
{
  gvf_parametric_trajectory.type = SINUS_3D;
  gvf_parametric_trajectory.p_parametric[0] = ay;
  gvf_parametric_trajectory.p_parametric[1] = freq_y;
  gvf_parametric_trajectory.p_parametric[2] = phase_y * M_PI / 180;
  gvf_parametric_trajectory.p_parametric[3] = az;
  gvf_parametric_trajectory.p_parametric[4] = freq_z;
  gvf_parametric_trajectory.p_parametric[5] = phase_z * M_PI / 180;

  gvf_parametric_plen = 6 + gvf_parametric_plen_wps;
  gvf_parametric_plen_wps = 0;

  float f1, f2, f3, f1d, f2d, f3d, f1dd, f2dd, f3dd;

  gvf_parametric_3d_sin_info(&f1, &f2, &f3, &f1d, &f2d, &f3d, &f1dd, &f2dd, &f3dd);
  gvf_parametric_control_3d(gvf_parametric_3d_sin_par.kx, gvf_parametric_3d_sin_par.ky,
                            gvf_parametric_3d_sin_par.kz, f1, f2, f3, f1d, f2d, f3d, f1dd, f2dd, f3dd);

  return true;
}

bool gvf_parametric_3d_sin_XYZa(float xo, float yo, float zo, float alpha,
                                float ay, float freq_y, float phase_y, float az, float freq_z, float phase_z)
{
  gvf_parametric_set_offset(xo, yo, zo);
  gvf_paremetric_set_euler_rot(0, 0, alpha);

  return gvf_parametric_3d_sin(ay, freq_y, phase_y, az, freq_z, phase_z);
}

// 3D linear/log growth Lissajou curve

/**
 * @brief Set parameters for a 3D linear/log growth Lissajou trajectory
 *
 * @param ax Speed along the x-axis
 * @param ay Oscillations' amplitude along the y-axis
 * @param az Oscillations' amplitude along the z-axis
 * @param f_y Frequency along the y-axis (in radiants, i.e. before multiplication by 2*PI)
 * @param phi_y Phase for the y-oscillations
 * @param f_z Frequency along the z-axis (in radiants, i.e. before multiplication by 2*PI)
 * @param phi_z Phase for the z-oscillations
 *
 * @return true
 */
bool gvf_parametric_3d_growing_lissajou(float ax, float ay, float az, float f_y, float phi_y, float f_z, float phi_z)
{
  gvf_parametric_trajectory.type = GROWING_LISSAJOU;
  gvf_parametric_trajectory.p_parametric[0] = ax;
  gvf_parametric_trajectory.p_parametric[1] = ay;
  gvf_parametric_trajectory.p_parametric[2] = az;
  gvf_parametric_trajectory.p_parametric[3] = f_y;
  gvf_parametric_trajectory.p_parametric[4] = phi_y;
  gvf_parametric_trajectory.p_parametric[5] = f_z;
  gvf_parametric_trajectory.p_parametric[6] = phi_z;

  gvf_parametric_plen = 7 + gvf_parametric_plen_wps;
  gvf_parametric_plen_wps = 0;

  float f1, f2, f3, f1d, f2d, f3d, f1dd, f2dd, f3dd;

  gvf_parametric_3d_growing_lissajou_info(&f1, &f2, &f3, &f1d, &f2d, &f3d, &f1dd, &f2dd, &f3dd);

  gvf_parametric_control_3d(gvf_parametric_3d_growing_lissajou_par.kx, gvf_parametric_3d_growing_lissajou_par.ky,
                            gvf_parametric_3d_growing_lissajou_par.kz, f1, f2, f3, f1d, f2d, f3d, f1dd, f2dd, f3dd);

  return true;
}

// Drift ellipse

/**
 * @brief Set parameters for a 3D drifting ellipse trajectory
 *
 * @param v_x Speed along the x-axis
 * @param a_x Oscillations' amplitude along the x-axis
 * @param a_y Oscillations' amplitude along the y-axis
 * @param freq Frequency (in radiants, i.e. before multiplication by 2*PI)
 * @param phi Phase
 *
 * @return true
 */
bool gvf_parametric_3d_drift_ellipse(float v_x, float a_x, float a_y, float freq, float phase)
{
  gvf_parametric_trajectory.type = DRIFT_ELLIPSE;
  gvf_parametric_trajectory.p_parametric[0] = v_x;
  gvf_parametric_trajectory.p_parametric[1] = a_x;
  gvf_parametric_trajectory.p_parametric[2] = a_y;
  gvf_parametric_trajectory.p_parametric[3] = freq;
  gvf_parametric_trajectory.p_parametric[4] = phase;

  gvf_parametric_plen = 5 + gvf_parametric_plen_wps;
  gvf_parametric_plen_wps = 0;

  float f1, f2, f3, f1d, f2d, f3d, f1dd, f2dd, f3dd;

  gvf_parametric_drift_ellipse_info(&f1, &f2, &f3, &f1d, &f2d, &f3d, &f1dd, &f2dd, &f3dd);
  gvf_parametric_control_3d(gvf_parametric_drift_ellipse_par.kx, gvf_parametric_drift_ellipse_par.ky,
                            gvf_parametric_drift_ellipse_par.kz, f1, f2, f3, f1d, f2d, f3d, f1dd, f2dd, f3dd);

  return true;
}
#endif // FIXEDWING_FIRMWARE

// Coordination

void gvf_parametric_coordination_send_w_to_nei(void)
{

  struct pprzlink_msg msg;

  for (int i = 0; i < GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS; i++)
  {
    if (gvf_parametric_coordination_tables.tableNei[i].nei_id != -1)
    {

      msg.trans = &(DefaultChannel).trans_tx;
      msg.dev = &(DefaultDevice).device;
      msg.sender_id = AC_ID;
      msg.receiver_id = gvf_parametric_coordination_tables.tableNei[i].nei_id;
      msg.component_id = 0;
      pprzlink_msg_v2_send_GVF_PARAMETRIC_G_W(&msg, &(gvf_parametric_control.w), &(gvf_parametric_control.w_dot));
    }
  }
}

void gvf_parametric_coordination_parseRegTable(uint8_t *buf)
{

  uint8_t ac_id = DL_GVF_PARAMETRIC_REG_TABLE_ac_id(buf);
  if (ac_id == AC_ID)
  {
    uint8_t nei_id = (uint8_t)(DL_GVF_PARAMETRIC_REG_TABLE_nei_id(buf));
    float desired_deltaw = DL_GVF_PARAMETRIC_REG_TABLE_desired_deltaw(buf);

    if (nei_id == 0)
    {
      for (int i = 0; i < GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS; i++)
      {
        gvf_parametric_coordination_tables.tableNei[i].nei_id = -1;
      }
    }
    else
    {
      for (int i = 0; i < GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS; i++)
      {
        if (gvf_parametric_coordination_tables.tableNei[i].nei_id == nei_id)
        {
          gvf_parametric_coordination_tables.tableNei[i].nei_id = nei_id;
          gvf_parametric_coordination_tables.tableNei[i].desired_dw = desired_deltaw;
          return;
        }
      }

      for (int i = 0; i < GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS; i++)
      {
        if (gvf_parametric_coordination_tables.tableNei[i].nei_id == -1)
        {
          gvf_parametric_coordination_tables.tableNei[i].nei_id = nei_id;
          gvf_parametric_coordination_tables.tableNei[i].desired_dw = desired_deltaw;
          return;
        }
      }
    }
  }
}

void gvf_parametric_coordination_parseWTable(uint8_t *buf)
{

  int16_t sender_id = (int16_t)(DL_GVF_PARAMETRIC_W_ac_id(buf));

  for (int i = 0; i < GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS; i++)
  {
    if (gvf_parametric_coordination_tables.tableNei[i].nei_id == sender_id)
    {
      gvf_parametric_coordination_tables.last_comm[i] = get_sys_time_msec();
      gvf_parametric_coordination_tables.tableNei[i].w = DL_GVF_PARAMETRIC_W_w(buf);
      gvf_parametric_coordination_tables.tableNei[i].w_dot = DL_GVF_PARAMETRIC_W_w_dot(buf);
      return;
    }
  }
}
