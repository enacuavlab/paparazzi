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
#include <Eigen/Dense> // https://eigen.tuxfamily.org/dox/GettingStarted.html
#include <Eigen/Geometry>

struct gvf_parametric_affine_transform
{
  Eigen::Quaternion<float> rot = Eigen::Quaternion<float>::Identity();
  Eigen::Translation3f transalation = Eigen::Translation3f(0., 0., 0.);
  Eigen::Transform<float, 3, Eigen::TransformTraits::Isometry> t = transalation * rot;
} gvf_parametric_affine_tr;

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

      float gvf_parametric_config[8] = {gvf_parametric_control.w,
                                       (float)now,
                                       gvf_parametric_control.s * gvf_parametric_control.beta,
                                       gvf_parametric_control.k_roll,
                                       gvf_parametric_control.k_climb,
                                       gvf_parametric_control.k_psi,
                                       gvf_parametric_control.L,
                                       gvf_parametric_control.w_dot};

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
    send_gvf_parametric_coordination(&(DefaultChannel).trans_tx,&(DefaultDevice).device);
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

  gvf_parametric_coordination.coordination = GVF_PARAMETRIC_COORDINATION_COORDINATION;
  gvf_parametric_coordination.kc = GVF_PARAMETRIC_COORDINATION_KC;
  gvf_parametric_coordination.timeout = GVF_PARAMETRIC_COORDINATION_TIMEOUT;
  gvf_parametric_coordination.broadtime = GVF_PARAMETRIC_COORDINATION_BROADTIME;

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
  gvf_parametric_set_offset(waypoints[wp].x, waypoints[wp].y, waypoints[wp].a);
}

void gvf_parametric_set_offset_wpa(uint8_t wp, float alt)
{
  gvf_parametric_set_offset(waypoints[wp].x, waypoints[wp].y, alt);
}

void gvf_paremetric_set_euler_rot(float rz, float ry, float rzbis)
{
  gvf_parametric_affine_tr.rot = Eigen::AngleAxisf(rzbis * M_PI / 180, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(ry * M_PI / 180, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(rz * M_PI / 180, Eigen::Vector3f::UnitZ());

  gvf_parametric_affine_tr.t = gvf_parametric_affine_tr.transalation * gvf_parametric_affine_tr.rot;
}

void gvf_paremetric_set_cardan_rot(float rx, float ry, float rz)
{
  gvf_parametric_affine_tr.rot = Eigen::AngleAxisf(rz * M_PI / 180, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(ry * M_PI / 180, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(rx * M_PI / 180, Eigen::Vector3f::UnitX());

  gvf_parametric_affine_tr.t = gvf_parametric_affine_tr.transalation * gvf_parametric_affine_tr.rot;
}

void gvf_paremetric_set_quaternion_rot(float x, float y, float z, float w)
{
  gvf_parametric_affine_tr.rot = Eigen::Quaternion<float>(w, x, y, z);

  gvf_parametric_affine_tr.t = gvf_parametric_affine_tr.transalation * gvf_parametric_affine_tr.rot;
}

void gvf_parametric_set_wps_rot(uint8_t wp1, uint8_t wp2)
{
  Eigen::Vector3f U = Eigen::Vector3f(waypoints[wp2].x - waypoints[wp1].x,
                                      waypoints[wp2].y - waypoints[wp1].y,
                                      waypoints[wp2].a - waypoints[wp1].a);

  Eigen::Vector3f Base = Eigen::Vector3f(1, 0, 0);
  gvf_parametric_affine_tr.rot = Eigen::Quaternion<float>::FromTwoVectors(Base, U).normalized();

  gvf_parametric_affine_tr.t = gvf_parametric_affine_tr.transalation * gvf_parametric_affine_tr.rot;
}

void gvf_parametric_set_wp_rot(uint8_t wp)
{
  Eigen::Vector3f U = Eigen::Vector3f(waypoints[wp].x,
                                      waypoints[wp].y,
                                      waypoints[wp].a);

  Eigen::Vector3f Base = Eigen::Vector3f(1, 0, 0);
  gvf_parametric_affine_tr.rot = Eigen::Quaternion<float>::FromTwoVectors(Base, U).normalized();

  gvf_parametric_affine_tr.t = gvf_parametric_affine_tr.transalation * gvf_parametric_affine_tr.rot;
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
  gvf_parametric_set_offset(waypoints[wp1].x, waypoints[wp1].y, waypoints[wp1].a);
  gvf_parametric_set_wps_rot(wp1, wp2);
}

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

  // Carrot position
  desired_x = f1;
  desired_y = f2;

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
          float desired_dw = gvf_parametric_coordination_tables.tableNei[i].desired_dw;

          float error_w = -beta * (wi - wj) + desired_dw;

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

  // Virtual coordinate update, even if the vehicle is not in autonomous mode, the parameter w will get "closer" to
  // the vehicle. So it is not only okei but advisable to update it.
  gvf_parametric_control.w += w_dot * gvf_parametric_control.delta_T * 1e-3;

  gvf_parametric_low_level_control_2d(heading_rate);

  if ((gvf_parametric_coordination.coordination) && (now - last_transmision > gvf_parametric_coordination.broadtime) && (autopilot_get_mode() == AP_MODE_AUTO2))
  {
    gvf_parametric_coordination_send_w_to_nei();
    last_transmision = now;
  }
}

void gvf_parametric_control_3d(float kx, float ky, float kz, float f1, float f2, float f3, float f1d, float f2d,
                               float f3d, float f1dd, float f2dd, float f3dd)
{
  uint32_t now = get_sys_time_msec();
  gvf_parametric_control.delta_T = now - gvf_parametric_t0;
  gvf_parametric_t0 = now;

  if (gvf_parametric_control.delta_T > 300)
  {                               // We need at least two iterations for Delta_T
    gvf_parametric_control.w = 0; // Reset w since we assume the algorithm starts
    return;
  }

  #if PERIODIC_TELEMETRY
  static uint32_t last_manual_send = 0;
  if (now - last_manual_send > 200)
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

  Eigen::Vector3f f_vec(f1, f2, f3);
  Eigen::Vector3f fd_vec(f1d, f2d, f3d);
  Eigen::Vector3f fdd_vec(f1dd, f2dd, f3dd);

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


  #if defined(GVF_PARAMETRIC_STEP_ADAPTATION)
  float f_v2 = f1d * f1d + f2d * f2d + f3d * f3d;
  float f_v = sqrtf(f_v2);

  float f_d_dot_f_dd = f1d*f1dd + f2d*f2dd + f3d*f3dd;

  float og_f1d = f1d;
  float og_f2d = f2d;
  float og_f3d = f3d;

  float og_f1dd = f1dd;
  float og_f2dd = f2dd;
  float og_f3dd = f3dd;


  if (f_v < 1e-6)
  {
    // Use an arbitrary direction
    f1d = 1/sqrtf(3.);
    f2d = 1/sqrtf(3.);
    f3d = 1/sqrtf(3.);

    f1dd = 0.;
    f2dd = 0.;
    f3dd = 0.;
  }
  else
  {
    f1dd = (f1dd - f_d_dot_f_dd * f1d / f_v2)/f_v2;
    f2dd = (f2dd - f_d_dot_f_dd * f2d / f_v2)/f_v2;
    f3dd = (f3dd - f_d_dot_f_dd * f3d / f_v2)/f_v2;

    f1d = f1d/f_v;
    f2d = f2d/f_v;
    f3d = f3d/f_v;
  }

  #endif


  // Carrot position
  desired_x = f1;
  desired_y = f2;

  float L = gvf_parametric_control.L;
  float beta = gvf_parametric_control.beta * gvf_parametric_control.s;

  Eigen::Vector4f X;
  Eigen::Matrix4f J;

  // Error signals phi_x phi_y and phi_z
  struct EnuCoor_f *pos_enu = stateGetPositionEnu_f();
  float x = pos_enu->x;
  float y = pos_enu->y;
  float z = pos_enu->z;

  float phi1 = L * (x - f1);
  float phi2 = L * (y - f2);
  float phi3 = L * (z - f3);

  gvf_parametric_trajectory.phi_errors[0] = phi1 / L; // Error signals in meters for the telemetry
  gvf_parametric_trajectory.phi_errors[1] = phi2 / L;
  gvf_parametric_trajectory.phi_errors[2] = phi3 / L;
  gvf_parametric_elen = 3;

  // Chi
  X(0) = -f1d * L * L * beta - kx * phi1;
  X(1) = -f2d * L * L * beta - ky * phi2;
  X(2) = -f3d * L * L * beta - kz * phi3;
  X(3) = -L * L + beta * (kx * phi1 * f1d + ky * phi2 * f2d + kz * phi3 * f3d);


  X *= L;

  // std::cout << "|| ------------------------------ ||" << std::endl;
  // std::cout << "Chi : " << X << std::endl;
  // std::cout << "Normalized Chi(3): " << normalized_parametric_error << std::endl;

  // Coordination if needed for multi vehicles
  float consensus_term_w = 0;

  

  if (gvf_parametric_coordination.coordination)
  {

    // std::cout << AC_ID << " Neighbors' : " << std::endl;

    for (int i = 0; i < GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS; i++)
    {
      if (gvf_parametric_coordination_tables.tableNei[i].nei_id != -1)
      {
        // std::cout << "- " << gvf_parametric_coordination_tables.tableNei[i].nei_id << " : ";
        uint32_t timeout = now - gvf_parametric_coordination_tables.last_comm[i];
        gvf_parametric_coordination_tables.tableNei[i].delta_t = timeout;
        if (timeout < gvf_parametric_coordination.timeout)
        {

          float wi = gvf_parametric_control.w;
          float wj = gvf_parametric_coordination_tables.tableNei[i].w;
          float desired_dw = gvf_parametric_coordination_tables.tableNei[i].desired_dw;

          float error_w = -beta * (wi - wj) + desired_dw;
          // std::cout << ", contributes " << error_w;

          consensus_term_w += error_w;

          gvf_parametric_coordination_tables.error_deltaw[i] = error_w;
        }
        // else
        // {
        //   std::cout << ", but timeout " << timeout << " out of " << gvf_parametric_coordination.timeout << " (last seen at " << gvf_parametric_coordination_tables.last_comm[i] << " )";
        // }
      }
      // std::cout << std::endl;
    }
  }
  else
  {
    std::cout << AC_ID << ": No coordination !!!" << std::endl;
  }

  // Limit impact of coordination using appropriate activation functions
  // See https://en.wikipedia.org/wiki/Activation_function
  // float w_coordination = gvf_parametric_coordination.kc * consensus_term_w;
  // float w_coordination_adjust = 1;
  
  // float norm_phi = sqrtf(phi1*phi1 + phi2*phi2 + phi3*phi3);
  float w_coordination_adjusted = gvf_parametric_coordination.kc * beta *(expf(consensus_term_w) - expf(-consensus_term_w))/(expf(consensus_term_w) + expf(-consensus_term_w));
  
  // Handle Limit cases
  if (!isfinite(w_coordination_adjusted))
  {
    if (consensus_term_w > 0)
    {
      w_coordination_adjusted = gvf_parametric_coordination.kc;
    }
    else
    {
      w_coordination_adjusted = -gvf_parametric_coordination.kc;
    }
  }
  

  std::cout << AC_ID << " W_dots : Individual " << X(3);
  std::cout << " | Consensus "<< consensus_term_w;
  // std::cout << " | Phi Norm " << norm_phi;
  std::cout << " | Coordination " << w_coordination_adjusted << std::endl;

  X(3) += w_coordination_adjusted;

  

  // Jacobian
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

  // Guidance algorithm
  float ground_speed = stateGetHorizontalSpeedNorm_f();

  /*
  std::cout << "Ground speed: " << ground_speed << std::endl;

  std::cout << std::endl << "Curve's params: " << std::endl;
  for (int i = 0; i < 7; i++)
  {
    std::cout << gvf_parametric_trajectory.p_parametric[i] << " ; ";
  }
  std::cout << std::endl;
  std::cout << "Beta: " << gvf_parametric_control.beta << std::endl;
  std::cout << "s   : " << (int)(gvf_parametric_control.s) << std::endl;
  std::cout << "L   : " << gvf_parametric_control.L << std::endl;
  std::cout << "w   : " << gvf_parametric_control.w << std::endl;
  std::cout << "Delta_t: " << gvf_parametric_control.delta_T << std::endl;
  std::cout << std::endl << std::endl;

  std::cout << "f(w) = [ " << f1 << "; " << f2 << "; " << f3 << " ] " << std::endl;
  std::cout << "f'(w) = [ " << f1d << "; " << f2d << "; " << f3d << " ] " << std::endl;
  std::cout << "f''(w) = [ " << f1dd << "; " << f2dd << "; " << f3dd << " ] " << std::endl;

  std::cout << std::endl;
  */

  float w_dot;
  #if defined(GVF_PARAMETRIC_STEP_ADAPTATION)
  w_dot = step_adaptation(ground_speed * X(3) * gvf_parametric_control.delta_T * 1e-3, og_f1d, og_f2d, og_f3d, og_f1dd, og_f2dd, og_f3dd) / (gvf_parametric_control.delta_T * 1e-3);
  // std::cout << "Requested step: " << ground_speed * normalized_parametric_error * gvf_parametric_control.delta_T * 1e-3 << std::endl;
  #else
  w_dot = (ground_speed * X(3)) / sqrtf(X(0) * X(0) + X(1) * X(1));
  #endif

  // std::cout << "w   : " << gvf_parametric_control.w << std::endl;
  // std::cout << "wdot: " << w_dot << std::endl;

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

  gvf_parametric_low_level_control_3d(heading_rate, climbing_rate);

  if ((gvf_parametric_coordination.coordination) && (now - last_transmision > gvf_parametric_coordination.broadtime) && (autopilot_get_mode() == AP_MODE_AUTO2))
  {
    // std::cout << "Transmitting coordination msg !" << std::endl;
    gvf_parametric_coordination_send_w_to_nei();
    last_transmision = now;
  }
}

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

  gvf_parametric_2d_trefoil_XY(waypoints[wp].x, waypoints[wp].y, w1, w2, ratio, r, alpha);
  return true;
}

/** 3D TRAJECTORIES **/
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

  gvf_parametric_3d_ellipse_XYZ(waypoints[wp].x, waypoints[wp].y, r, zl, zh, alpha);
  return true;
}

bool gvf_parametric_3d_ellipse_wp_delta(uint8_t wp, float r, float alt_center, float delta, float alpha)
{
  float zl = alt_center - delta;
  float zh = alt_center + delta;

  gvf_parametric_3d_ellipse_XYZ(waypoints[wp].x, waypoints[wp].y, r, zl, zh, alpha);
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

  gvf_parametric_3d_lissajous_XYZ(waypoints[wp].x, waypoints[wp].y, zo, cx, cy, cz, wx, wy, wz, dx, dy, dz, alpha);
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

/*
static void print_gvf_parametric_coordination_regTable()
{
  for (int i = 0; i < GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS; i++)
  {
    std::cout << "For slot n° " << i << " : " << ((int)gvf_parametric_coordination_tables.tableNei[i].nei_id) << " , " << gvf_parametric_coordination_tables.tableNei[i].desired_dw << std::endl << std::endl;
  }
}
*/

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
          // print_gvf_parametric_coordination_regTable();
          return;
        }
      }

      for (int i = 0; i < GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS; i++)
      {
        if (gvf_parametric_coordination_tables.tableNei[i].nei_id == -1)
        {
          gvf_parametric_coordination_tables.tableNei[i].nei_id = nei_id;
          gvf_parametric_coordination_tables.tableNei[i].desired_dw = desired_deltaw;
          // print_gvf_parametric_coordination_regTable();
          return;
        }
      }
    }
  }
}

void gvf_parametric_coordination_parseWTable(uint8_t *buf)
{

  int16_t sender_id = (int16_t)(DL_GVF_PARAMETRIC_W_ac_id(buf));

  // std::cout << "***** " << AC_ID << " Msg received from " << sender_id;

  for (int i = 0; i < GVF_PARAMETRIC_COORDINATION_MAX_NEIGHBORS; i++)
  {
    if (gvf_parametric_coordination_tables.tableNei[i].nei_id == sender_id)
    {
      gvf_parametric_coordination_tables.last_comm[i] = get_sys_time_msec();
      gvf_parametric_coordination_tables.tableNei[i].w = DL_GVF_PARAMETRIC_W_w(buf);
      gvf_parametric_coordination_tables.tableNei[i].w_dot = DL_GVF_PARAMETRIC_W_w_dot(buf);
      // std::cout << " at " << gvf_parametric_coordination_tables.last_comm[i] << " *****" << std::endl;
      return;
    }
  }
}
