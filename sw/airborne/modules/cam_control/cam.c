/*
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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
/** \file cam.c
 *  \brief Pan/Tilt camera library
 *
 */

#include "cam.h"
#include "autopilot.h"
#include "generated/flight_plan.h"
#include "generated/modules.h"
#include "modules/core/commands.h"
#include "state.h"
#include "modules/datalink/telemetry.h"

// Minimum and maximum angles
// used to convert angles to commands, assuming a linear interpolation
#ifndef CAM_PAN_MAX
#define CAM_PAN_MAX RadOfDeg(90.f)
#endif
#ifndef CAM_PAN_MIN
#define CAM_PAN_MIN -CAM_PAN_MAX
#endif
#ifndef CAM_TILT_MAX
#define CAM_TILT_MAX RadOfDeg(90.f)
#endif
#ifndef CAM_TILT_MIN
#define CAM_TILT_MIN -CAM_TILT_MAX
#endif

// Default position and orientation of the gimbal in body frame
#ifndef CAM_GIMBAL_POS_X
#define CAM_GIMBAL_POS_X 0.f
#endif
#ifndef CAM_GIMBAL_POS_Y
#define CAM_GIMBAL_POS_Y 0.f
#endif
#ifndef CAM_GIMBAL_POS_Z
#define CAM_GIMBAL_POS_Z 0.f
#endif
#ifndef CAM_GIMBAL_TO_BODY_PHI
#define CAM_GIMBAL_TO_BODY_PHI 0.f
#endif
#ifndef CAM_GIMBAL_TO_BODY_THETA
#define CAM_GIMBAL_TO_BODY_THETA 0.f
#endif
#ifndef CAM_GIMBAL_TO_BODY_PSI
#define CAM_GIMBAL_TO_BODY_PSI 0.f
#endif

// Global cam structure
struct CamControl cam_control;

static void send_cam(struct transport_tx *trans, struct link_device *dev)
{
  int16_t x = cam_control.target_pos.x;
  int16_t y = cam_control.target_pos.y;
  int16_t phi = DegOfRad(cam_control.pan_angle);
  int16_t theta = DegOfRad(cam_control.tilt_angle);
  pprz_msg_send_CAM(trans, dev, AC_ID, &phi, &theta, &x, &y);
}

#ifdef SHOW_CAM_COORDINATES
static void send_cam_point(struct transport_tx *trans, struct link_device *dev)
{
  // TODO target enu to lla
  //pprz_msg_send_CAM_POINT(trans, dev, AC_ID,
  //                        &cam_point_distance_from_home, &cam_point_lat, &cam_point_lon);
}
#endif

void cam_init(void)
{
  // apply default settings
  struct FloatEulers gimbal_to_body = {
    CAM_GIMBAL_TO_BODY_PHI,
    CAM_GIMBAL_TO_BODY_THETA,
    CAM_GIMBAL_TO_BODY_PSI
  };
  struct FloatVect3 gimbal_pos = {
    CAM_GIMBAL_POS_X,
    CAM_GIMBAL_POS_Y,
    CAM_GIMBAL_POS_Z
  };
  cam_setup(&cam_control,
      CAM_PAN_MAX, CAM_PAN_MIN,
      CAM_TILT_MAX, CAM_TILT_MIN,
      gimbal_to_body, gimbal_pos);

  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CAM, send_cam);
#ifdef SHOW_CAM_COORDINATES
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CAM_POINT, send_cam_point);
#endif
}

void cam_periodic(void)
{
  // cam_run(&cam_control, compute_angles); // TODO run from module using this API ???
}

void cam_run(struct CamControl *cam, cam_angles_from_dir compute_angles)
{
  //TODO
  //
#if defined(CAM_FIXED_FOR_FPV_IN_AUTO1) && CAM_FIXED_FOR_FPV_IN_AUTO1 == 1
  //Position the camera for straight view.
  if (autopilot_get_mode() == AP_MODE_AUTO2) {
#endif
    switch (cam_mode) {
      case CAM_MODE_OFF:
        cam_pan_c = RadOfDeg(CAM_PAN0);
        cam_tilt_c = RadOfDeg(CAM_TILT0);
        cam_angles();
        break;
      case CAM_MODE_ANGLES:
        cam_angles();
        break;
      case CAM_MODE_NADIR:
        cam_nadir();
        break;
      case CAM_MODE_XY_TARGET:
        cam_target();
        break;
      case CAM_MODE_WP_TARGET:
        cam_waypoint_target();
        break;
      case CAM_MODE_AC_TARGET:
        cam_ac_target();
        break;
        // In this mode the target coordinates are calculated continuously from the pan and tilt radio channels.
        // The "TARGET" waypoint coordinates are not used.
        // If the "-DSHOW_CAM_COORDINATES" is defined then the coordinates of where the camera is looking are calculated.
      case CAM_MODE_STABILIZED:
        cam_waypoint_target();
        break;
        // In this mode the angles come from the pan and tilt radio channels.
        // The "TARGET" waypoint coordinates are not used but i need to call the "cam_waypoint_target()" function
        // in order to calculate the coordinates of where the camera is looking.
      case CAM_MODE_RC:
        cam_waypoint_target();
        break;
      default:
        break;
    }
#if defined(CAM_FIXED_FOR_FPV_IN_AUTO1) && CAM_FIXED_FOR_FPV_IN_AUTO1 == 1
  } else if (autopilot_get_mode() == AP_MODE_AUTO1) {
    //Position the camera for straight view.

#if defined(CAM_TILT_POSITION_FOR_FPV)
    cam_tilt_c = RadOfDeg(CAM_TILT_POSITION_FOR_FPV);
#else
    cam_tilt_c = RadOfDeg(90);
#endif
#if defined(CAM_PAN_POSITION_FOR_FPV)
    cam_pan_c = RadOfDeg(CAM_PAN_POSITION_FOR_FPV);
#else
    cam_pan_c = RadOfDeg(0);
#endif
    cam_angles();
#ifdef SHOW_CAM_COORDINATES
    cam_point_lon = 0;
    cam_point_lat = 0;
    cam_point_distance_from_home = 0;
#endif
  }
#endif


}

/** Computes the servo values from cam_pan_c and cam_tilt_c */
void cam_angles(void)
{
  float cam_pan = 0;
  float cam_tilt = 0;
  if (cam_pan_c > RadOfDeg(CAM_PAN_MAX)) {
    cam_pan_c = RadOfDeg(CAM_PAN_MAX);
  } else {
    if (cam_pan_c < RadOfDeg(CAM_PAN_MIN)) {
      cam_pan_c = RadOfDeg(CAM_PAN_MIN);
    }
  }

  if (cam_tilt_c > RadOfDeg(CAM_TILT_MAX)) {
    cam_tilt_c = RadOfDeg(CAM_TILT_MAX);
  } else {
    if (cam_tilt_c < RadOfDeg(CAM_TILT_MIN)) {
      cam_tilt_c = RadOfDeg(CAM_TILT_MIN);
    }
  }

#ifdef CAM_PAN_NEUTRAL
  float pan_diff = cam_pan_c - RadOfDeg(CAM_PAN_NEUTRAL);
  if (pan_diff > 0) {
    cam_pan = MAX_PPRZ * (pan_diff / (RadOfDeg(CAM_PAN_MAX - CAM_PAN_NEUTRAL)));
  } else {
    cam_pan = MIN_PPRZ * (pan_diff / (RadOfDeg(CAM_PAN_MIN - CAM_PAN_NEUTRAL)));
  }
#else
  cam_pan = ((float)RadOfDeg(cam_pan_c - CAM_PAN_MIN)) * ((float)MAX_PPRZ / (float)RadOfDeg(CAM_PAN_MAX - CAM_PAN_MIN));
#endif

#ifdef CAM_TILT_NEUTRAL
  float tilt_diff = cam_tilt_c - RadOfDeg(CAM_TILT_NEUTRAL);
  if (tilt_diff > 0) {
    cam_tilt = MAX_PPRZ * (tilt_diff / (RadOfDeg(CAM_TILT_MAX - CAM_TILT_NEUTRAL)));
  } else {
    cam_tilt = MIN_PPRZ * (tilt_diff / (RadOfDeg(CAM_TILT_MIN - CAM_TILT_NEUTRAL)));
  }
#else
  cam_tilt = ((float)RadOfDeg(cam_tilt_c - CAM_TILT_MIN))  * ((float)MAX_PPRZ / (float)RadOfDeg(
               CAM_TILT_MAX - CAM_TILT_MIN));
#endif

  cam_pan = TRIM_PPRZ(cam_pan);
  cam_tilt = TRIM_PPRZ(cam_tilt);

  cam_phi_c = cam_pan_c;
  cam_theta_c = cam_tilt_c;

#ifdef COMMAND_CAM_PAN
  command_set(COMMAND_CAM_PAN, cam_pan);
#endif
#ifdef COMMAND_CAM_TILT
  command_set(COMMAND_CAM_TILT, cam_tilt);
#endif
}

/** Computes the right angles from target_x, target_y, target_alt */
void cam_target(void)
{
#ifdef TEST_CAM
  vPoint(test_cam_estimator_x, test_cam_estimator_y, test_cam_estimator_z,
         test_cam_estimator_phi, test_cam_estimator_theta, test_cam_estimator_hspeed_dir,
         cam_target_x, cam_target_y, cam_target_alt,
         &cam_pan_c, &cam_tilt_c);
#else
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  vPoint(pos->x, pos->y, stateGetPositionUtm_f()->alt,
         att->phi, att->theta, stateGetHorizontalSpeedDir_f(),
         cam_target_x, cam_target_y, cam_target_alt,
         &cam_pan_c, &cam_tilt_c);
#endif
  cam_angles();
}

/** Point straight down */
void cam_nadir(void)
{
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
#ifdef TEST_CAM
  cam_target_x = test_cam_estimator_x;
  cam_target_y = test_cam_estimator_y;
#else
  cam_target_x = pos->x;
  cam_target_y = pos->y;
#endif
  cam_target_alt = -10;
  cam_target();
}


void cam_waypoint_target(void)
{
  if (cam_target_wp < nb_waypoint) {
    cam_target_x = WaypointX(cam_target_wp);
    cam_target_y = WaypointY(cam_target_wp);
  }
  cam_target_alt = ground_alt;
  cam_target();
}

#ifdef TRAFFIC_INFO
#include "modules/multi/traffic_info.h"

void cam_ac_target(void)
{
  struct EnuCoor_f ac_pos *ac = acInfoGetPositionEnu_f(cam_target_ac);
  cam_target_x = ac->x;
  cam_target_y = ac->y;
  cam_target_alt = acInfoGetPositionUtm_f()->alt;
  cam_target();
}
#else
void cam_ac_target(void) {}
#endif // TRAFFIC_INFO
