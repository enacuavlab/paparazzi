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

#include "gvf.h"
#include "std.h"
#include "./trajectories/gvf_ellipse.h"
#include "subsystems/navigation/common_nav.h"

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
    gvf_ke = 0;
    gvf_kn = 0;
    gvf_kd = 0;
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
    gvf_traj_type = 1;
    gvf_p1 = waypoints[wp].x;
    gvf_p2 = waypoints[wp].y;
    gvf_p3 = a;
    gvf_p4 = b;
    gvf_p5 = alpha;

    gvf_error = 123.56;
    return true;
}

bool gvf_ellipse_set(uint8_t wp)
{
    float a = gvf_ellipse_a;
    float b = gvf_ellipse_b;
    float alpha = gvf_ellipse_alpha;

    gvf_traj_type = 1;
    gvf_p1 = waypoints[wp].x;
    gvf_p2 = waypoints[wp].y;
    gvf_p3 = gvf_ellipse_a;
    gvf_p4 = gvf_ellipse_b;
    gvf_p5 = gvf_ellipse_alpha;
    gvf_error = 100;
    return true;
}

