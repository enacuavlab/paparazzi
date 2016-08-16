/*
 * Copyright (C) 2016 Hector Garcia de Marina
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
 *
 */

#include <math.h>
#include "std.h"

#include "gvf.h"

#include "./trajectories/gvf_ellipse.h"
#include "subsystems/navigation/common_nav.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/autopilot.h"

// Control
float gvf_error;
float gvf_ke;
float gvf_kd;
float gvf_kn;

// Trajectory
uint8_t gvf_traj_type;
float gvf_p1;
float gvf_p2;
float gvf_p3;
float gvf_p4;
float gvf_p5;
float gvf_p6;
float gvf_p7;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
static void send_gvf(struct transport_tx *trans, struct link_device *dev)
{
    pprz_msg_send_GVF(trans, dev, AC_ID, &gvf_error, &gvf_traj_type,
            &gvf_p1, &gvf_p2, &gvf_p3, &gvf_p4, &gvf_p5, &gvf_p6, &gvf_p7);
}

#endif

void gvf_init(void)
{
    gvf_ke = 1;
    gvf_kn = 1;
    gvf_kd = 1;
    gvf_traj_type = 0;
    gvf_p1 = 0;
    gvf_p2 = 0;
    gvf_p3 = 0;
    gvf_p4 = 0;
    gvf_p5 = 0;
    gvf_p6 = 0;
    gvf_p7 = 0;

#if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GVF, send_gvf);
#endif
}

bool gvf_ellipse(uint8_t wp, float a, float b, float alpha)
{

    alpha = alpha*M_PI/180;

    gvf_traj_type = 1;
    gvf_p1 = waypoints[wp].x;
    gvf_p2 = waypoints[wp].y;
    gvf_p3 = a;
    gvf_p4 = b;
    gvf_p5 = alpha;

    // SAFE MODE (TODO)
    if(a == 0 || b == 0){
        a = 60;
        b = 60;
    }

    float ke = gvf_ke;
    float kn = gvf_kn;
    float kd = gvf_kd;

    // State
    struct EnuCoor_f *p = stateGetPositionEnu_f();
    float px = p->x;
    float py = p->y;
    float wx = waypoints[wp].x;
    float wy = waypoints[wp].y;
    float ground_speed = stateGetHorizontalSpeedNorm_f();
    float course = stateGetHorizontalSpeedDir_f();
    float px_dot = ground_speed*sinf(course);
    float py_dot = ground_speed*cosf(course);
    struct FloatEulers *att = stateGetNedToBodyEulers_f();
    float psi = att->psi;
    float air_speed = stateGetAirspeed_f();

    // Phi(x,y)
    float xel = (px-wx)*cosf(alpha) - (py-wy)*sinf(alpha);
    float yel = (px-wx)*sinf(alpha) + (py-wy)*cosf(alpha);
    float e = (xel/a)*(xel/a) + (yel/b)*(yel/b) - 1;

    // grad Phi
    float nx = (2*xel/(a*a))*cosf(alpha) + (2*yel/(b*b))*sinf(alpha);
    float ny = (2*yel/(b*b))*cosf(alpha) - (2*xel/(a*a))*sinf(alpha);

    // Hessian Phi
    float H11 = 2*(cosf(alpha)*cosf(alpha)/(a*a)
            + sinf(alpha)*sinf(alpha)/(b*b));
    float H12 = 2*sinf(alpha)*cosf(alpha)*(1/(b*b) - 1/(a*a));
    float H21 = H12;
    float H22 = 2*(sinf(alpha)*sinf(alpha)/(a*a)
            + cosf(alpha)*cosf(alpha)/(b*b));

    // tangent to Phi
    float tx = ny;
    float ty = -nx;

    // Calculation of the desired angular velocity in the vector field
    float pdx_dot = tx - ke*e*nx;
    float pdy_dot = ty - ke*e*ny;

    float norm_pd_dot = sqrtf(pdx_dot*pdx_dot + pdy_dot*pdy_dot);
    float md_x = pdx_dot / norm_pd_dot;
    float md_y = pdy_dot / norm_pd_dot;

    float Apd_dot_dot_x = -ke*e*(nx*px_dot + ny*py_dot)*nx;
    float Apd_dot_dot_y = -ke*e*(nx*px_dot + ny*py_dot)*ny;

    float Bpd_dot_dot_x = ((-ke*e*H11)+H21)*px_dot + ((-ke*e*H12)+H22)*py_dot;
    float Bpd_dot_dot_y = -(H11+(ke*e*H21))*px_dot - (H12+(ke*e*H22))*py_dot;

    float pd_dot_dot_x = Apd_dot_dot_x + Bpd_dot_dot_x;
    float pd_dot_dot_y = Apd_dot_dot_y + Bpd_dot_dot_y;

    float md_dot_const = -(md_x*pd_dot_dot_y - md_y*pd_dot_dot_x)/norm_pd_dot;
    float md_dot_x = md_y * md_dot_const;
    float md_dot_y = -md_x * md_dot_const;

    float omega_d = -(md_dot_x*md_y - md_dot_y*md_x);

    float mr_x = sinf(course);
    float mr_y = cosf(course);

    // Calculation of the setting point of omega
    // float omega = ground_speed/(air_speed*(cosf(course)*cosf(psi) +
    //            sinf(course)*sinf(psi))) * (omega_d + kn*mr_x*md_y 
    //        -kn*mr_y*md_x);

    float omega = omega_d + kn*mr_x*md_y -kn*mr_y*md_x;
    
    // Coordinated turn, it is minus since in NED the positive is clockwise
    h_ctl_roll_setpoint =
        -atanf(kd*omega*ground_speed/GVF_GRAVITY/cosf(att->theta));
    BoundAbs(h_ctl_roll_setpoint, h_ctl_roll_max_setpoint);

    lateral_mode = LATERAL_MODE_ROLL;

    gvf_error = e;

    gvf_p6 = H12;
    return true;
}

bool gvf_ellipse_set(uint8_t wp)
{
    float a = gvf_ellipse_a;
    float b = gvf_ellipse_b;
    float alpha = gvf_ellipse_alpha*M_PI/180;

    gvf_ellipse(wp, a, b, alpha);

    return true;
}

