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
#include "modules/sensors/cameras/jevois.h"
#include <stdio.h>
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

#ifndef IMAV_RESCUE_PROXIMITY
#define IMAV_RESCUE_PROXIMITY 3
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
  struct NedCoor_f pos;
  float score;
  int count;
  uint8_t wp_id;
};

static struct rescue_spot rescue_spots[IMAV_RESCUE_SPOTS_NB] = {0};

#if (defined IMAV_RESCUE_SPOTS_WPS)
static uint8_t imav_rescue_wps[] = IMAV_RESCUE_SPOTS_WPS;
const uint8_t imav_rescue_wps_len = sizeof(imav_rescue_wps);
#else
static uint8_t imav_rescue_wps[] = {};
const uint8_t imav_rescue_wps_len = 0;
#endif

static void rescue_send_draw(struct rescue_spot* spot) UNUSED;
static void update_spots(struct NedCoor_f pos, float score);


static void imav_rescue_jevois_msg_cb(uint8_t sender_id UNUSED,
    uint8_t type, char * id, uint8_t nb UNUSED, int16_t * coord,
    uint16_t * dim UNUSED, struct FloatQuat quat UNUSED, char * extra UNUSED)
{
  if (type != JEVOIS_MSG_N2) {
    return; // invalid frame
  }

  char category[21];
  float score = 0.f;
  char *tk = strtok(id, ":");
  bool valid = false;

  // parse string id with format "category:score"
  if (tk != NULL) {
    strncpy(category, tk, 20);
    tk = strtok(NULL, ":");
    if (tk != NULL) {
      score = strtof(tk, NULL);
    }
    if (strncmp(category, "person", 20) == 0) {
      valid = true;
    } else if (strncmp(category, "sofa", 20) == 0) {
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
    // update closest spot or create a new one
    // TODO cross check with sound and light sensors
    update_spots(target_loc.target, score);
  }
  else {
    // if too close from ground, don't do anything
    target_loc.valid = false;
  }

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

  for (int i = 0; i < Min(imav_rescue_wps_len,IMAV_RESCUE_SPOTS_NB); i++) {
    rescue_spots[i].wp_id = imav_rescue_wps[i];
  }

  // Abi messages bindings
  AbiBindMsgJEVOIS_MSG(IMAV_RESCUE_JEVOIS_MSG_ID, &imav_rescue_jevois_msg_ev, imav_rescue_jevois_msg_cb);
}


static void update_spots(struct NedCoor_f pos, float score) {
  // find closest and merge position and score,
  for (int i = 0; i < IMAV_RESCUE_SPOTS_NB; i++) {
    struct rescue_spot *sp = &rescue_spots[i];
    struct FloatVect2 d_pos;
    VECT2_DIFF(d_pos, pos, sp->pos);
    float dist = float_vect2_norm2(&d_pos);
    // is it close enough
    if (dist < IMAV_RESCUE_PROXIMITY) {
      float new_x = (pos.x + sp->pos.x) / 2.f;
      float new_y = (pos.y + sp->pos.y) / 2.f;
      waypoint_move_xy_i(sp->wp_id, POS_BFP_OF_REAL(new_x), POS_BFP_OF_REAL(new_y));
      sp->pos.x = new_x;
      sp->pos.y = new_y;
      sp->score = (score + sp->score) / 2.f;
      sp->count++;
      return;
    }
  }

  // if not find empty slot,
  for (int i = 0; i < IMAV_RESCUE_SPOTS_NB; i++) {
    struct rescue_spot *sp = &rescue_spots[i];
    if (sp->count == 0) {
      sp->pos = pos;
      sp->score = score;
      sp->count = 1;
      waypoint_move_xy_i(sp->wp_id, POS_BFP_OF_REAL(pos.x), POS_BFP_OF_REAL(pos.y));
      return;
    }
  }

  // if not replace if score is better
  float lowest_score = 100.f;
  int lowest_idx = 0;
  for (int i = 0; i < IMAV_RESCUE_SPOTS_NB; i++) {
    struct rescue_spot *sp = &rescue_spots[i];
    if (sp->score < lowest_score) {
      lowest_score = sp->score;
      lowest_idx = i;
    }
  }
  rescue_spots[lowest_idx].pos = pos;
  rescue_spots[lowest_idx].score = score;
  rescue_spots[lowest_idx].count = 1;
  waypoint_move_xy_i(rescue_spots[lowest_idx].wp_id, POS_BFP_OF_REAL(pos.x), POS_BFP_OF_REAL(pos.y));
  return;
}


static void rescue_send_draw(struct rescue_spot* spot)
{
  uint8_t color = 0 << 6 | 5 << 3 | 5;  // Transparent, Magenta, Magenta
  uint8_t shape = 0;
  uint8_t status = 0;
  float radius = 2;
  uint8_t idx = spot->wp_id + IMAV_RESCUE_DRAW_OFFSET;
  struct NedCoor_i ned;
  NED_BFP_OF_REAL(ned, spot->pos);
  struct EcefCoor_i ecef;
  ecef_of_ned_pos_i(&ecef, stateGetNedOrigin_i(), &ned);
  struct LlaCoor_i lla;
  lla_of_ecef_i(&lla, &ecef);

  char text[10];
  int nb_text = snprintf(text, 10, "%.2f (%d)", spot->score, spot->count);

  DOWNLINK_SEND_DRAW(DefaultChannel, DefaultDevice, &idx, &color, &shape, &status, &radius, 1, &lla.lat, 1, &lla.lon, nb_text, text);
}

void imav_rescue_set_best_wp(uint8_t wp_id)
{
  // find best
  float best_score = 0.f;
  int best_idx = -1;
  for (int i = 0; i < IMAV_RESCUE_SPOTS_NB; i++) {
    struct rescue_spot *sp = &rescue_spots[i];
    if (sp->score > best_score && sp->count > 0) {
      best_score = sp->score;
      best_idx = i;
    }
  }
  if (best_idx != -1 && best_idx < IMAV_RESCUE_SPOTS_NB) {
    float pos_x = rescue_spots[best_idx].pos.x;
    float pos_y = rescue_spots[best_idx].pos.y;
    waypoint_move_xy_i(wp_id, POS_BFP_OF_REAL(pos_x), POS_BFP_OF_REAL(pos_y));
  }
}

