/*
 * Copyright (C) 2025 Jean-Baptiste FORESTIER <jean-baptiste.forestier@enac.fr>
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
/**
 * @file "modules/sensors/trisonica.c"
 * @author Jean-Baptiste FORESTIER
 * Decoder for standardized messages from the Trisonica Mini LI-550F
 * 
 * CALIBRATION :
 * Calibration Anemo, compas, and Level has to be done with an operator
 * In CLI mode, set your output rate : (ctrl + C, ASCII 3), then , outputrate your_freq
 * In CLI mode, select the parameters you want 
 * 
 * TELEMETRY :
 * Using name="AEROPROBE" id="179", use it just for control.
 * Correpondance to trisonica value :
 * counter = mag_head_ang (degree)
 * velocity = absolute_value(pitch) (degree)
 * a_attack = absolute_value(roll) (degree)
 * a_slideslip = 100 * temperature (degC)
 * altitude = absolute_value(winddirhor) (degree)
 * dynamic_p = absolute_value(winddirver) (degree)
 * static_p = 100 * windspeed2d (m/s)
 * checksum = 100 * windspeed3d (m/s)
 * 
 * CONNECTION ON TAWAKIV2 : 
 * Work on UART 3.
 * 
 * SD LOG :
 * Log on SD card in /PPRZ/pprzlog_xxxx.LOG
 * It will create a new pprzlog file at each startup.
 * Data will be written here at each line reception and we timestamp it with the Tawaki onboard time on CSV format with ',' delimiter.
 * All parameters are not sent with the nominal configuration, you will therefore see 0. You can add through the CLI the supplementary parameters you want to log.
 * Time is in us since the Tawaki startup.
 * 
 * PARSER :
 * ASCII parser
 * Exemple of line received : S  00.04 S2  00.02 D  013 DV -063 U -00.00 V -00.02 W -00.04 T  22.53 H  58.56 P  1003.34 AD  1.1724622 PI -004.3 RO -002.7 MD  010 TD  010
 */

#ifndef TRISONICA_H
#define TRISONICA_H

#include "std.h"

//TODO Should I change this define ?
#define TRISONCIA_ID 18
// max string length
#define TRISONICA_MAX_LEN 300

// generic trisonica message structure _ 24 parameters
struct trisonica_msg_t {
  float windspeed3d;
  float windspeed2d;
  int16_t winddirhor;
  int16_t winddirver;
  float u_vector;
  float v_vector;
  float w_vector;
  float speed_sound;
  float temp;
  float relative_humidity;
  float dew_point;
  float abs_pressure;
  float airdensity;
  float accel_x;
  float accel_y;
  float accel_z;
  float pitch;
  float roll;
  float mag_x;
  float mag_y;
  float mag_z;
  float mag_head_ang;
  float true_head;
  uint32_t now_ts;
};

struct trisonica_tags {
  char  windspeed3d[2];
  char  windspeed2d[2];
  char  winddirhor[2];
  char  winddirver[2];
  char  u_vector[2];
  char  v_vector[2];
  char  w_vector[2];
  char  speed_sound[2];
  char  temp[2];
  char  relative_humidity[2];
  char  dew_point[2];
  char  abs_pressure[2];
  char  airdensity[2];
  char  accel_x[2];
  char  accel_y[2];
  char  accel_z[2];
  char  pitch[2];
  char  roll[2];
  char  mag_x [2];
  char  mag_y[2];
  char  mag_z[2];
  char  mag_head_ang[2];
  char  true_head[2];
};

// decoder state
enum trisonica_state {
  TS_SYNC = 0,
  TS_TYPE,
};

// trisonica struct
struct trisonica_t {
  enum trisonica_state state; // decoder state
  char buf[TRISONICA_MAX_LEN]; // temp buffer
  uint8_t idx; // temp buffer index
  struct trisonica_msg_t msg; // last decoded message
  bool data_available; // new data to report
  struct trisonica_tags tags;
  bool log_ptu_started;
  uint32_t now_ts;
};

void trisonica_log_data_ascii(struct trisonica_t *ts);
void trisonica_parse(struct trisonica_t *ts, char c);
void process_line(struct trisonica_t *ts);
int process_character_uint(struct trisonica_t *ts, int local_idx, int16_t *parameter);
int process_character(struct trisonica_t *ts, int local_idx, float *parameter);
int16_t fast_atoi16(const char *s);
float fast_atof(const char *s);

extern void trisonica_init(void);
extern void trisonica_event(void);
extern void trisonica_report(void);
extern void trisonica_send_string(char *s);



#endif

