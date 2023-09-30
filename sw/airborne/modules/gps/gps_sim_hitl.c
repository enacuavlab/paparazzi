/*
 * Copyright (C) 2014 Sergey Krukowski <softsr@yahoo.de>
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
 */

/**
 * @file modules/gps/gps_sim_hitl.c
 * GPS subsystem simulation from rotorcrafts horizontal/vertical reference system
 */

#include "modules/gps/gps_sim_hitl.h"
#include "modules/gps/gps.h"
#include "modules/core/abi.h"
#include "modules/datalink/datalink.h"

struct GpsState gps_sim_hitl;
bool gps_has_fix;

void gps_sim_hitl_init(void)
{
  gps_has_fix = true;
}

void gps_sim_hitl_parse_HITL_GPS(uint8_t *buf)
{
  if (DL_HITL_GPS_ac_id(buf) != AC_ID) {
    return;
  }

  gps_sim_hitl.week = 1794;
  gps_sim_hitl.tow = DL_HITL_GPS_time(buf) * 1000;

  gps_sim_hitl.ecef_vel.x = DL_HITL_GPS_ecef_vel_x(buf) * 100.;
  gps_sim_hitl.ecef_vel.y = DL_HITL_GPS_ecef_vel_y(buf) * 100.;
  gps_sim_hitl.ecef_vel.z = DL_HITL_GPS_ecef_vel_z(buf) * 100.;
  SetBit(gps_sim_hitl.valid_fields, GPS_VALID_VEL_ECEF_BIT);

  gps_sim_hitl.lla_pos.lat = DL_HITL_GPS_lat(buf) * 1e7;
  gps_sim_hitl.lla_pos.lon = DL_HITL_GPS_lon(buf) * 1e7;
  gps_sim_hitl.lla_pos.alt = DL_HITL_GPS_alt(buf) * 1000.;
  SetBit(gps_sim_hitl.valid_fields, GPS_VALID_POS_LLA_BIT);
  //ecef_of_lla_i(&gps_sim_hitl.ecef_pos, &gps_sim_hitl.lla_pos);
  //SetBit(gps_sim_hitl.valid_fields, GPS_VALID_POS_ECEF_BIT);

  gps_sim_hitl.hmsl        = DL_HITL_GPS_hmsl(buf) * 1000.;
  SetBit(gps_sim_hitl.valid_fields, GPS_VALID_HMSL_BIT);

  /* calc NED speed from ECEF */
  struct LtpDef_i ref_ltp;
  ltp_def_from_lla_i(&ref_ltp, &gps_sim_hitl.lla_pos);
  struct NedCoor_i ned_vel_i;
  ned_of_ecef_vect_i(&ned_vel_i, &ref_ltp, &gps_sim_hitl.ecef_vel);
  gps_sim_hitl.ned_vel.x = ned_vel_i.x;
  gps_sim_hitl.ned_vel.y = ned_vel_i.y;
  gps_sim_hitl.ned_vel.z = ned_vel_i.z;
  SetBit(gps_sim_hitl.valid_fields, GPS_VALID_VEL_NED_BIT);
  struct NedCoor_f ned_vel_f;
  VECT3_FLOAT_OF_CM(ned_vel_f, gps_sim_hitl.ned_vel);

  /* horizontal and 3d ground speed in cm/s */
  gps_sim_hitl.gspeed = sqrtf(ned_vel_f.x * ned_vel_f.x + ned_vel_f.y * ned_vel_f.y) * 100;
  gps_sim_hitl.speed_3d = sqrtf(ned_vel_f.x * ned_vel_f.x + ned_vel_f.y * ned_vel_f.y + ned_vel_f.z * ned_vel_f.z) * 100;

  /* ground course in radians * 1e7 */
  gps_sim_hitl.course = atan2f(ned_vel_f.y, ned_vel_f.x) * 1e7;
  SetBit(gps_sim_hitl.valid_fields, GPS_VALID_COURSE_BIT);

  gps_sim_hitl.pacc = 650;
  gps_sim_hitl.hacc = 450;
  gps_sim_hitl.vacc = 200;
  gps_sim_hitl.sacc = 100;
  gps_sim_hitl.pdop = 650;

  if (gps_has_fix) {
    gps_sim_hitl.num_sv = 11;
    gps_sim_hitl.fix = GPS_FIX_3D;
  } else {
    gps_sim_hitl.num_sv = 1;
    gps_sim_hitl.fix = GPS_FIX_NONE;
  }

  // publish gps data
  uint32_t now_ts = get_sys_time_usec();
  gps_sim_hitl.last_msg_ticks = sys_time.nb_sec_rem;
  gps_sim_hitl.last_msg_time = sys_time.nb_sec;
  if (gps_sim_hitl.fix == GPS_FIX_3D) {
    gps_sim_hitl.last_3dfix_ticks = sys_time.nb_sec_rem;
    gps_sim_hitl.last_3dfix_time = sys_time.nb_sec;
  }
  AbiSendMsgGPS(GPS_SIM_ID, now_ts, &gps_sim_hitl);
}

void gps_feed_value() {}

