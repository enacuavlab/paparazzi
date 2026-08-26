/*
 * Copyright (C) 2026 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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

/** @file "modules/imav_rescue_people/imav_rescue_people.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Indentify people to be rescued for IMAV2026
 */

#include "modules/imav_rescue_people/imav_rescue_people.h"
#include "modules/core/abi.h"
#include "modules/nav/waypoints.h"
#include "modules/datalink/downlink.h"
#include <string.h>
#include "state.h"

#ifndef IMAV_RESCUE_JEVOIS_MSG_ID
#define IMAV_RESCUE_JEVOIS_MSG_ID ABI_BROADCAST
#endif

#ifndef IMAV_RESCUE_SPOTS_NB
#define IMAV_RESCUE_SPOTS_NB 3
#endif

#define IMAV_RESCUE_DRAW_OFFSET 10

// Default parameters
// Camera is looking down and is placed at the center of the frame
// With cam X axis pointing to the right, Y down and Z forward of image frame,
// the camera is just rotated of pi/2 around body Z axis

#ifndef IMAV_RESCUE_BODY_TO_CAM_PHI
#define IMAV_RESCUE_BODY_TO_CAM_PHI 0.f
#endif

#ifndef IMAV_RESCUE_BODY_TO_CAM_THETA
#define IMAV_RESCUE_BODY_TO_CAM_THETA 0.f
#endif

#ifndef IMAV_RESCUE_BODY_TO_CAM_PSI
#define IMAV_RESCUE_BODY_TO_CAM_PSI M_PI_2
#endif

#ifndef IMAV_RESCUE_CAM_POS_X
#define IMAV_RESCUE_CAM_POS_X 0.f
#endif

#ifndef IMAV_RESCUE_CAM_POS_Y
#define IMAV_RESCUE_CAM_POS_Y 0.f
#endif

#ifndef IMAV_RESCUE_CAM_POS_Z
#define IMAV_RESCUE_CAM_POS_Z 0.f
#endif

// Convert pixel unit to m in image plane on x axis
// from "mm" to meter by default
#ifndef IMAV_RESCUE_PIXEL_TO_IMAGE_X
#define IMAV_RESCUE_PIXEL_TO_IMAGE_X (1.f / 1000.f)
#endif

// Convert pixel unit to m in image plane on y axis (same as x by default)
#ifndef IMAV_RESCUE_PIXEL_TO_IMAGE_Y
#define IMAV_RESCUE_PIXEL_TO_IMAGE_Y IMAV_RESCUE_PIXEL_TO_IMAGE_X
#endif

// Detection of targets
struct target_loc_t {
  int16_t px;                   ///< Target in camera frame
  int16_t py;                   ///< Target in camera frame

  struct FloatRMat body_to_cam; ///< Body to camera rotation
  struct FloatVect3 cam_pos;    ///< Position of camera in body frame

  struct NedCoor_f target;      ///< Target position in LTP-NED frame
  struct LlaCoor_f pos_lla;     ///< Target global position in LLA

  bool valid;                   ///< True if a target have been seen
  uint8_t type;                 ///< Type of target
};

static struct target_loc_t target_loc;

// Abi event
static abi_event imav_rescue_jevois_msg_ev;

// Rescue spots
struct rescue_spot {
  uint8_t cluster_idx;
  int32_t lat;
  int32_t lon;
  float score;
  int count;
};

static struct rescue_spot rescue_spots[IMAV_RESCUE_SPOTS_NB] = {0};

static void rescue_send_draw(struct rescue_spot* spot);
static void update_spots(struct rescue_spot* spot);


static void imav_rescue_jevois_msg_cb(uint8_t sender_id UNUSED,
    uint8_t type, char * id, uint8_t nb UNUSED, int16_t * coord,
    uint16_t * dim UNUSED, struct FloatQuat quat UNUSED, char * extra UNUSED)
{
  if (type != JEVOIS_MSG_N2) {
    return; // invalid frame
  }

  char type[20];
  float score = 0.f;
  char *tk = strtok(id, ":");
  bool valid = false;

  // parse string id with format "type:score"
  if (tk != NULL) {
    strncpy(type, tk, 20);
    tk = strtok(NULL, ":");
    if (tk != NULL) {
      score = strtof(tk, NULL);
    }
    if (strncmp(type, "person", 20) == 0) {
      valid = true;
    } else if (strncmp(type, "sofa", 20) == 0) {
      valid = true;
      score *= 0.8; // apply penalty ?
    }
  }
  if (!valid) {
    return; // invalid format
  }

  // compute absolute position
  target_loc.px = coord[0];
  target_loc.py = coord[1];

  // Prepare rotation matrices
  struct FloatRMat *ltp_to_body_rmat = stateGetNedToBodyRMat_f();
  struct FloatRMat ltp_to_cam_rmat;
  float_rmat_comp(&ltp_to_cam_rmat, ltp_to_body_rmat, &target_loc.body_to_cam);
  // Prepare cam world position
  // C_w = P_w + R_w2b * C_b
  struct FloatVect3 cam_pos_ltp;
  float_rmat_vmult(&cam_pos_ltp, ltp_to_body_rmat, &target_loc.cam_pos);
  VECT3_ADD(cam_pos_ltp, *stateGetPositionNed_f());

  // Compute target position here (pixels in "mm" to meters)
  struct FloatVect3 target_img = {
    .x = (float)target_loc.px * IMAV_RESCUE_PIXEL_TO_IMAGE_X,
    .y = (float)target_loc.py * IMAV_RESCUE_PIXEL_TO_IMAGE_Y,
    .z = 1.f
  };
  struct FloatVect3 tmp; // before scale factor
  float_rmat_transp_vmult(&tmp, &ltp_to_cam_rmat, &target_img); // R^-1 * v_img

  if (fabsf(tmp.z) > 0.1f) {
    float scale = fabsf(cam_pos_ltp.z / tmp.z); // scale factor
    VECT3_SUM_SCALED(target_loc.target, cam_pos_ltp, tmp, scale); // T_w = C_w + s*tmp
    // now, T_w.z should be equal to zero as it is assumed that the target is on a flat ground
    // compute absolute position
    struct EcefCoor_f target_ecef;
    ecef_of_ned_point_f(&target_ecef, stateGetNedOrigin_f(), &target_loc.target);
    lla_of_ecef_f(&target_loc.pos_lla, &target_ecef);

    target_loc.valid = true;
    //if (target_localization_update_wp) {
    //  // look for waypoint to update
    //  uint8_t i = 0;
    //  while (target_loc_wp_tab[i][0] != 0) {
    //    if (target_loc_wp_tab[i][0] == target_loc.type) {
    //      // update WP (ENU) from target (NED)
    //      waypoint_move_xy_i(target_loc_wp_tab[i][1], POS_BFP_OF_REAL(target_loc.target.y), POS_BFP_OF_REAL(target_loc.target.x));
    //    }
    //    i++;
    //  }
    //}
  }
  else {
    // if too close from ground, don't do anything
    target_loc.valid = false;
  }

  // TODO store rescue spot if score is high enough, update if close to existing spot and increase counter
  // cross check with sound and light sensors
}

void imav_rescue_init(void)
{
  target_loc.px = 0;
  target_loc.py = 0;

  struct FloatEulers euler = {
    IMAV_RESCUE_BODY_TO_CAM_PHI,
    IMAV_RESCUE_BODY_TO_CAM_THETA,
    IMAV_RESCUE_BODY_TO_CAM_PSI
  };
  float_rmat_of_eulers(&target_loc.body_to_cam, &euler);
  VECT3_ASSIGN(target_loc.cam_pos,
      IMAV_RESCUE_CAM_POS_X,
      IMAV_RESCUE_CAM_POS_Y,
      IMAV_RESCUE_CAM_POS_Z);

  FLOAT_VECT3_ZERO(target_loc.target);

  target_loc.valid = false;

  // Abi messages bindings
  AbiBindMsgJEVOIS_MSG(IMAV_RESCUE_JEVOIS_MSG_ID, &imav_rescue_jevois_msg_ev, imav_rescue_jevois_msg_cb);
}


static void update_spots(struct rescue_spot* spot) {
  int coldest_idx = 0;
  for(int i=0; i<IMAV_RESCUE_SPOTS_NB; i++) {
    struct rescue_spot *sp = &rescue_spots[i];

    // cluster found in hottests, just update it.
    if (sp->cluster_idx == spot->cluster_idx) {
      memcpy(sp, spot, sizeof(struct rescue_spot));
      return;
    }

    // find the lowest score amongst the array.
    if (sp->score < rescue_spots[coldest_idx].score) {
      coldest_idx = i;
    }
  }

  // the new spot has higher score, save it in place of the lowest.
  if (spot->score > rescue_spots[coldest_idx].score) {
    memcpy(&rescue_spots[coldest_idx], spot, sizeof(struct rescue_spot));
  }
}


static void rescue_send_draw(struct rescue_spot* spot)
{
  uint8_t color = 0 << 6 | 5 << 3 | 5;  // Transparent, Magenta, Magenta
  uint8_t shape = 0;
  uint8_t status = 0;
  float radius = 2;
  uint8_t idx = spot->cluster_idx + IMAV_RESCUE_DRAW_OFFSET;

  char text[10];
  int nb_text = snprintf(text, 10, "%.2f (%d)", spot->score, spot->count);

  DOWNLINK_SEND_DRAW(DefaultChannel, DefaultDevice, &idx, &color, &shape, &status, &radius, 1, &spot->lat, 1, &spot->lon, nb_text, text);
}


