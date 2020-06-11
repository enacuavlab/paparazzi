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
 * @file modules/guidance/gvf_advanced/gvf_advanced.cpp
 *
 * Guiding vector field algorithm for 2D and 3D complex trajectories.
 */

#include <iostream>
#include <Eigen/Dense>

#include "gvf_advanced.h"
#include "./trajectories/gvf_advanced_3d_ellipse.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "subsystems/navigation/common_nav.h"
#include "autopilot.h"

// Control
uint32_t t0 = 0; // We need it for calculting the time lapse delta_T
gvf_advanced_con gvf_advanced_control;

// Trajectory
gvf_advanced_tra gvf_advanced_trajectory;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
static void send_gvf_advanced(struct transport_tx *trans, struct link_device *dev)
{
  // Do not know whether is a good idea to do this check here or to include
  // this plen in gvf_trajectory
  int plen;

  switch (gvf_advanced_trajectory.type) {
    case ELLIPSE_3D:
      plen = 6;
      break;
    default:
      plen = 1;
      break;
  }

  uint8_t traj_type = (uint8_t)gvf_advanced_trajectory.type;

  pprz_msg_send_GVF_ADVANCED(trans, dev, AC_ID, &traj_type,
                    &gvf_advanced_control.s, plen, gvf_advanced_trajectory.p_advanced);
}

static void send_circle_advanced(struct transport_tx *trans, struct link_device *dev)
{
  if (gvf_advanced_trajectory.type == ELLIPSE_3D){
    pprz_msg_send_CIRCLE(trans, dev, AC_ID,
                         &gvf_advanced_trajectory.p_advanced[0], &gvf_advanced_trajectory.p_advanced[1],
                         &gvf_advanced_trajectory.p_advanced[2]);
  }
}

#endif // PERIODIC TELEMETRY

#ifdef __cplusplus
}
#endif

void gvf_advanced_init(void)
{
  gvf_advanced_control.w = 0;
  gvf_advanced_control.delta_T = 0;
  gvf_advanced_control.s = 1;
  gvf_advanced_control.k_roll = GVF_ADVANCED_KROLL;
  gvf_advanced_control.k_climb = GVF_ADVANCED_KCLIMB;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GVF_ADVANCED, send_gvf_advanced);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CIRCLE, send_circle_advanced);
#endif

}

void gvf_advanced_control_2D(float, Eigen::Vector3f *, Eigen::Matrix3f *);
void gvf_advanced_control_2D(float ktheta, Eigen::Vector3f *Chi2d, Eigen::Matrix3f *J2d)
{
    ktheta += ktheta;
    (*Chi2d).setZero();
    (*J2d).setZero();
}

void gvf_advanced_control_3D(float, Eigen::Vector4f *, Eigen::Matrix4f *);
void gvf_advanced_control_3D(float k_psi, Eigen::Vector4f *Chi3d, Eigen::Matrix4f *J3d)
{
    uint32_t now = get_sys_time_msec();
    gvf_advanced_control.delta_T = now - t0;
    t0 = now;

    if(gvf_advanced_control.delta_T > 300) // We need at least two iterations for Delta_T
      return;

    Eigen::Vector4f X = *Chi3d;
    float ground_speed = stateGetHorizontalSpeedNorm_f();

    ground_speed = 10;
    float w_dot = (ground_speed*X(3)) / sqrtf(X(0)*X(0) + X(1)*X(1));
    gvf_advanced_control.w += w_dot*gvf_advanced_control.delta_T*1e-3;

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
    E << 0.0, -1.0*gvf_advanced_control.s,
         1.0*gvf_advanced_control.s,  0.0;
    G = F.transpose()*F;
    Fp = E*F;
    Gp = F.transpose()*E*F;

    Eigen::Matrix4f J = *J3d;
    Eigen::Matrix<float, 1, 4> Xt = X.transpose();
    Eigen::Vector4f Xh = X/X.norm();
    Eigen::Matrix<float, 1, 4> Xht = Xh.transpose();

    float aux = ht*Fp*X;

    float heading_rate = -1/(Xt*G*X)*Xt*Gp*(I-Xh*Xht)*J*xi_dot - (k_psi*aux/sqrtf(Xt*G*X));
    float climbing_rate = (ground_speed*X(2)) / sqrtf(X(0)*X(0) + X(1)*X(1));

    // Low-level control
    if (autopilot_get_mode() == AP_MODE_AUTO2) {
    // Vertical
    v_ctl_mode = V_CTL_MODE_AUTO_CLIMB;
    v_ctl_speed_mode = V_CTL_SPEED_THROTTLE;

    v_ctl_climb_setpoint = gvf_advanced_control.k_climb*climbing_rate;

    // Lateral
    lateral_mode = LATERAL_MODE_ROLL;

    struct FloatEulers *att = stateGetNedToBodyEulers_f();

    h_ctl_roll_setpoint =
      -gvf_advanced_control.k_roll*atanf(heading_rate * ground_speed / GVF_ADVANCED_GRAVITY / cosf(att->theta));
    BoundAbs(h_ctl_roll_setpoint, h_ctl_roll_max_setpoint);
    }

    std::cout << "Heading rate: " << heading_rate << std::endl;
    std::cout << "Climbing rate: " << climbing_rate << std::endl;
    std::cout << "w dot: " << w_dot << std::endl;
}

// 3D ELLIPSE

bool gvf_advanced_3D_ellipse_XY(float xo, float yo, float r, float zl, float zh, float alpha)
{

  horizontal_mode = HORIZONTAL_MODE_CIRCLE; // "2D Cylinder" is a circle for the GCS

  Eigen::Vector4f Chi3d;
  Eigen::Matrix4f J3d;

  gvf_advanced_trajectory.type = ELLIPSE_3D;
  gvf_advanced_trajectory.p_advanced[0] = xo;
  gvf_advanced_trajectory.p_advanced[1] = yo;
  gvf_advanced_trajectory.p_advanced[2] = r;
  gvf_advanced_trajectory.p_advanced[3] = zl;
  gvf_advanced_trajectory.p_advanced[4] = zh;
  gvf_advanced_trajectory.p_advanced[5] = alpha;

  // SAFE MODE
  if (zl > zh)
    zl = zh;
  if (zl < 1 || zh < 1){
    zl = 10;
    zh = 10;
  }
  if (r < 1)
    r = 60;

  struct EnuCoor_f *pos_enu = stateGetPositionEnu_f();
  float x = pos_enu->x;
  float y = pos_enu->y;
  float z = pos_enu->z;
  float w = gvf_advanced_control.w;

  float f1 = r*cosf(w) + xo;
  float f2 = r*sinf(w) + yo;
  float f3 = 0.5*(zh + zl + (zl-zh)*sinf(alpha-w));

  float f1d = -r*sinf(w);
  float f2d = r*cosf(w);
  float f3d = -0.5*(zl-zh)*cosf(alpha-w);

  float f1dd = -r*cosf(w);
  float f2dd = -r*sinf(w);
  float f3dd = -0.5*(zl-zh)*sinf(alpha-w);

  float phi1 = x - f1;
  float phi2 = y - f2;
  float phi3 = z - f3;

  std::cout << "phi1: " << phi1 << std::endl;
  std::cout << "phi2: " << phi2 << std::endl;
  std::cout << "phi3: " << phi3 << std::endl;

  Chi3d(0) = -f1d - gvf_advanced_3d_ellipse_par.kx*phi1*1e-2;
  Chi3d(1) = -f2d - gvf_advanced_3d_ellipse_par.ky*phi2*1e-2;
  Chi3d(2) = -f3d - gvf_advanced_3d_ellipse_par.kz*phi3*1e-2;
  Chi3d(3) =   -1 + (gvf_advanced_3d_ellipse_par.kx*phi1*f1d*1e-2
      + gvf_advanced_3d_ellipse_par.ky*phi2*f2d*1e-2
      + gvf_advanced_3d_ellipse_par.kz*phi3*f3d*1e-2);

  J3d.setZero();

  J3d(0,0) = -gvf_advanced_3d_ellipse_par.kx*1e-2;
  J3d(1,1) = -gvf_advanced_3d_ellipse_par.ky*1e-2;
  J3d(2,2) = -gvf_advanced_3d_ellipse_par.kz*1e-2;
  J3d(3,0) = gvf_advanced_3d_ellipse_par.kx*f1d*1e-2;
  J3d(3,1) = gvf_advanced_3d_ellipse_par.ky*f2d*1e-2;
  J3d(3,2) = gvf_advanced_3d_ellipse_par.kz*f3d*1e-2;
  J3d(0,3) = -f1dd + gvf_advanced_3d_ellipse_par.kx*f1d*1e-2;
  J3d(1,3) = -f2dd + gvf_advanced_3d_ellipse_par.ky*f2d*1e-2;
  J3d(2,3) = -f3dd + gvf_advanced_3d_ellipse_par.kz*f3d*1e-2;
  J3d(3,3) =  gvf_advanced_3d_ellipse_par.kx*(phi1*f1dd-f1d*f1d)*1e-2
      + gvf_advanced_3d_ellipse_par.ky*(phi2*f2dd-f2d*f2d)*1e-2
      + gvf_advanced_3d_ellipse_par.kz*(phi3*f3dd-f3d*f3d)*1e-2;

  std::cout << "Chi3d: " << std::endl << Chi3d << std::endl;
  std::cout << "J3d: " << std::endl << J3d << std::endl;

  gvf_advanced_control_3D(gvf_advanced_3d_ellipse_par.k_psi, &Chi3d, &J3d);

  return true;
}

bool gvf_advanced_3D_ellipse_wp(uint8_t wp, float r, float zl, float zh, float alpha)
{
  gvf_advanced_3D_ellipse_XY(waypoints[wp].x, waypoints[wp].y, r, zl, zh, alpha);
  return true;
}

