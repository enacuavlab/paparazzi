/*
 * Copyright (C) 2005-  Pascal Brisset, Antoine Drouin
 *               2025 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
/** \file cam.h
 *  \brief Pan/Tilt camera API
 *
 */

#ifndef CAM_H
#define CAM_H

#include "std.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_float.h"

#define CAM_MODE_OFF        0  // Do nothing
#define CAM_MODE_ANGLES     1  // Input: servo angles
#define CAM_MODE_NADIR      2  // Input: look down
#define CAM_MODE_XY_TARGET  3  // Input: target_x, target_y
#define CAM_MODE_WP_TARGET  4  // Input: waypoint no
#define CAM_MODE_AC_TARGET  5  // Input: ac id
#define CAM_MODE_STABILIZED 6  // Stabilized mode, input: camera angles from the pan and tilt radio channels, output pointing coordinates.
#define CAM_MODE_RC         7  // Manual mode, input: camera angles from the pan and tilt radio channels, output servo positions.

/** Function pointer to return cam angle from a specified direction
 *
 * The direction is the unit vector from the camera position to the target
 * expressed in the gimbal frame.
 * The resulting angles depends on the type of gimbal that is used,
 * in particular the number and order of rotations.
 * This function is provided by the user as it is specific to each mounting.
 * It returns the pan and tilt angles.
 */
typedef void (*cam_angles_from_dir)(struct FloatVect3 dir, float *pan, float *tilt);

struct CamControl {
  uint8_t mode;                 ///< cam control mode
  bool lock;                    ///< lock current command FIXME really needed ?

  int16_t pan_cmd;              ///< pan command [pprz]
  int16_t tilt_cmd;             ///< tilt command [pprz]

  float pan_max;                ///< pan angle at maximum command
  float pan_min;                ///< pan angle at minimum command
  float tilt_max;               ///< tilt angle at maximum command
  float tilt_min;               ///< tilt angle at minimum command
  struct FloatRMat gimbal_to_body;  ///< rotation matrix from gimbal to body frame
  struct FloatVect3 gimbal_pos;     ///< position of the gimbal in body NED frame [m]

  float pan_angle;              ///< pan angle [rad]
  float tilt_angle;             ///< tilt angle [rad]
  struct EnuCoor_f target_pos;  ///< target point in ENU world frame [m]
  uint8_t target_wp_id;         ///< waypoint ID to track
  uint8_t target_ac_id;         ///< aircraft ID to track
};

extern struct CamControl cam_control;

extern void cam_init(void);
extern void cam_periodic(void);

// API for internal and external use
extern void cam_setup(struct CamControl *cam,
    float pan_max, float pan_min,
    float tilt_max, float tilt_min,
    struct FloatEulers gimbal_to_body,
    struct FloatVect3 gimbal_pos);
extern void cam_run(struct CamControl *cam, cam_angles_from_dir compute_angles);
extern void cam_set_mode(struct CamControl *cam, uint8_t mode);
extern void cam_set_lock(struct CamControl *cam, bool lock);
extern void cam_set_pan_command(struct CamControl *cam, int16_t pan);
extern void cam_set_tilt_command(struct CamControl *cam, int16_t pan);
extern void cam_set_angles_rad(struct CamControl *cam, float pan, float tilt);
extern void cam_set_angles_deg(struct CamControl *cam, float pan, float tilt);
extern void cam_set_target_pos(struct CamControl *cam, struct NedCoor_f target);
extern void cam_set_wp_id(struct CamControl *cam, uint8_t wp_id);
extern void cam_set_ac_id(struct CamControl *cam, uint8_t ac_id);

// settings handler
#define cam_SetMode(x) cam_set_mode(&cam_control,x)
#define cam_SetLock(x) cam_set_lock(&cam_control, x)
#define cam_SetPanCommand(x) cam_set_pan_command(&cam_control, x)
#define cam_SetTiltCommand(x) cam_set_pan_command(&cam_control, x)

#endif // CAM_H
