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
 * @file "modules/sensors/trisonica_mini_LI_550F.c"
 * @author Jean-Baptiste FORESTIER
 * Decoder for standardized messages from the Trisonica Mini LI-550F
 */

#include "modules/sensors/trisonica_mini_LI_550F.h"

#include "std.h"
#include "mcu_periph/uart.h"
#include "modules/core/abi.h"
#include "math/pprz_algebra_float.h"
#include "modules/datalink/downlink.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

//TODO Calibration Anemo, compas, and Level has to be done with an operator
//TODO Should I keep this define ?
#define TRISONCIA_ID 18
// max string length
#define TRISONICA_MAX_LEN 32
// check delimiter
#define TRISONICA_CHECK_DELIM(_c) (_c == '\n' || _c == '\r' || _c == ' ')

// generic trisonica message structure _ 23 parameters
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
};

struct trisonica_tags {
  char  windspeed3d       ='S ';
  char  windspeed2d       ='S2';
  char  winddirhor        ='D ';
  char  winddirver        ='DV';
  char  u_vector          ='U ';
  char  v_vector          ='V ';
  char  w_vector          ='W ';
  char  speed_sound       ='C ';
  char  temp              ='T ';
  char  relative_humidity ='H ';
  char  dew_point         ='DP';
  char  abs_pressure      ='P ';
  char  airdensity        ='AD';
  char  accel_x           ='AX';
  char  accel_y           ='AY';
  char  accel_z           ='AZ';
  char  pitch             ='PI';
  char  roll              ='RO';
  char  mag_x             ='MX';
  char  mag_y             ='MY';
  char  mag_z             ='MZ';
  char  mag_head_ang      ='MD';
  char  true_head         ='TD';
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
  uint8_t n; // temp coordinates/dimension index
  struct trisonica_msg_t msg; // last decoded message
  bool data_available; // new data to report
  struct trisonica_tags tags;
};

struct trisonica_t trisonica;

// reporting function, send telemetry message
void trisonica_report(void)
{
  if (trisonica.data_available == false) {
    // no new data, return
    return;
  }

  DOWNLINK_SEND_TRISONICA(DefaultChannel, DefaultDevice,
      &trisonica.msg.abs_pressure,
      &trisonica.msg.accel_x,
      &trisonica.msg.accel_y,
      &trisonica.msg.accel_z,
      &trisonica.msg.airdensity,
      &trisonica.msg.dew_point,
      &trisonica.msg.mag_head_ang,
      &trisonica.msg.mag_x,
      &trisonica.msg.mag_y,
      &trisonica.msg.mag_z,
      &trisonica.msg.pitch,
      &trisonica.msg.relative_humidity,
      &trisonica.msg.roll,
      &trisonica.msg.speed_sound,
      &trisonica.msg.temp,
      &trisonica.msg.true_head,
      &trisonica.msg.u_vector,
      &trisonica.msg.v_vector,
      &trisonica.msg.w_vector,
      &trisonica.msg.winddirhor,
      &trisonica.msg.winddirver,
      &trisonica.msg.windspeed2d,
      &trisonica.msg.windspeed3d,
      );
  trisonica.data_available = false;
}

// initialization
void trisonica_init(void)
{
  trisonica.state = TS_SYNC;
  trisonica.idx = 0;
  trisonica.n = 0;
  trisonica.data_available = false;
  memset(trisonica.buf, 0,TRISONICA_MAX_LEN);
  
  	// Set Outputrate to 40 Hz
	trisonica_send_string("outputrate 40");
	//TODO read acqu
	//TODO Send all param OK ?
}


static void trisonica_handle_msg(struct trisonica_t *ts) {
  // send ABI message
  AbiSendMsgTRISONCIA_MSG(TRISONCIA_ID,
      ts->msg.abs_pressure,
      ts->msg.accel_x,
      ts->msg.accel_y,
      ts->msg.accel_z,
      ts->msg.airdensity,
      ts->msg.dew_point,
      ts->msg.mag_head_ang,
      ts->msg.mag_x,
      ts->msg.mag_y,
      ts->msg.mag_z,
      ts->msg.pitch,
      ts->msg.relative_humidity,
      ts->msg.roll,
      ts->msg.speed_sound,
      ts->msg.temp,
      ts->msg.true_head,
      ts->msg.u_vector,
      ts->msg.v_vector,
      ts->msg.w_vector,
      ts->msg.winddirhor,
      ts->msg.winddirver,
      ts->msg.windspeed2d,
      ts->msg.windspeed3d,
      );
  // Here can add the send of specific msg
  ts->data_available = true;
  ts->idx = 0;
  ts->n = 0;
}

// Process for float
static void process_character(struct trisonica_t *ts, char c, float *parameter) {
    if (TRISONICA_CHECK_DELIM(c)) { // End of parameter, we save it
        ts->buf[ts->idx] = '\0';  // End the buffer
        parameter = (float)atof(ts->buf); // Convert
        ts->idx = 0; // Reset index for next acquisition
    } else {
        //ts->buf[ts->idx++] = c; // Fill buffer
        if (ts->idx > TRISONICA_MAX_LEN) {
            ts->state = TS_SYNC; // Error
        }
    }
}

// Process for int16_t
static void process_character_uint(struct trisonica_t *ts, char c, int16_t *parameter) {
    if (TRISONICA_CHECK_DELIM(c)) { // End of parameter, we save it
        ts->buf[ts->idx] = '\0';  // End the buffer
        parameter = (int16_t)atoi(ts->buf); // Convert
        ts->idx = 0; // Reset index for next acquisition
    } else {
        //ts->buf[ts->idx++] = c; // Fill buffer
        if (ts->idx > TRISONICA_MAX_LEN) {
            ts->state = TS_SYNC; // Error
        }
    }
}

// raw message parsing function
// The LI-550 outputs data in an ASCII character string ending with carriage return
// and line feed characters. Each line is a single record of all the measured parameters
// contained in a single sample.
static void trisonica_parse(struct trisonica_t *ts, char c)
{
  switch (ts->state) {
    case TS_SYNC:
      // wait for sync (Line feed character == Last line char)
      if (c == '\n') {
    	ts->state = TS_TYPE;
    	ts->idx = 0;
    	ts->n = 0;
      }
      break;
    case TS_TYPE:
      ts->buf[ts->idx++] = c; // fill buffer
      // parse type : 2 letter or 1 letter + 1 space
      if (ts->idx > 1) {
        if (ts->buf[0] == ts->tags.abs_pressure[0] && ts->buf[1] == ts->tags.abs_pressure[1]) {
        	process_character(ts, c, ts->msg.abs_pressure);
        	break;
        }
        if (ts->buf[0] == ts->tags.accel_x[0] && ts->buf[1] == ts->tags.accel_x[1]) {
        	process_character(ts, c, ts->msg.accel_x);
        	break;
        }
        if (ts->buf[0] == ts->tags.accel_y[0] && ts->buf[1] == ts->tags.accel_y[1]) {
        	process_character(ts, c, ts->msg.accel_y);
        	break;
        }
        if (ts->buf[0] == ts->tags.accel_z[0] && ts->buf[1] == ts->tags.accel_z[1]) {
        	process_character(ts, c, ts->msg.accel_z);
        	break;
        }
        if (ts->buf[0] == ts->tags.airdensity[0] && ts->buf[1] == ts->tags.airdensity[1]) {
        	process_character(ts, c, ts->msg.airdensity);
        	break;
        }
        if (ts->buf[0] == ts->tags.dew_point[0] && ts->buf[1] == ts->tags.dew_point[1]) {
        	process_character(ts, c, ts->msg.dew_point);
        	break;
        }
        if (ts->buf[0] == ts->tags.mag_head_ang[0] && ts->buf[1] == ts->tags.mag_head_ang[1]) {
        	process_character(ts, c, ts->msg.mag_head_ang);
        	break;
        }
        if (ts->buf[0] == ts->tags.mag_x[0] && ts->buf[1] == ts->tags.mag_x[1]) {
        	process_character(ts, c, ts->msg.mag_x);
        	break;
        }
        if (ts->buf[0] == ts->tags.mag_y[0] && ts->buf[1] == ts->tags.mag_y[1]) {
        	process_character(ts, c, ts->msg.mag_y);
        	break;
        }
        if (ts->buf[0] == ts->tags.mag_z[0] && ts->buf[1] == ts->tags.mag_z[1]) {
        	process_character(ts, c, ts->msg.mag_z);
        	break;
        }
        if (ts->buf[0] == ts->tags.pitch[0] && ts->buf[1] == ts->tags.pitch[1]) {
        	process_character(ts, c, ts->msg.pitch);
        	break;
        }
        if (ts->buf[0] == ts->tags.relative_humidity[0] && ts->buf[1] == ts->tags.relative_humidity[1]) {
        	process_character(ts, c, ts->msg.relative_humidity);
        	break;
        }
        if (ts->buf[0] == ts->tags.roll[0] && ts->buf[1] == ts->tags.roll[1]) {
        	process_character(ts, c, ts->msg.roll);
        	break;
        }
        if (ts->buf[0] == ts->tags.speed_sound[0] && ts->buf[1] == ts->tags.speed_sound[1]) {
        	process_character(ts, c, ts->msg.speed_sound);
        	break;
        }
        if (ts->buf[0] == ts->tags.temp[0] && ts->buf[1] == ts->tags.temp[1]) {
        	process_character(ts, c, ts->msg.temp);
        	break;
        }
        if (ts->buf[0] == ts->tags.true_head[0] && ts->buf[1] == ts->tags.true_head[1]) {
        	process_character(ts, c, ts->msg.true_head);
        	break;
        }
        if (ts->buf[0] == ts->tags.u_vector[0] && ts->buf[1] == ts->tags.u_vector[1]) {
        	process_character(ts, c, ts->msg.u_vector);
        	break;
        }
        if (ts->buf[0] == ts->tags.v_vector[0] && ts->buf[1] == ts->tags.v_vector[1]) {
        	process_character(ts, c, ts->msg.v_vector);
        	break;
        }
        if (ts->buf[0] == ts->tags.w_vector[0] && ts->buf[1] == ts->tags.w_vector[1]) {
        	process_character(ts, c, ts->msg.w_vector);
        	break;
        }
        if (ts->buf[0] == ts->tags.winddirhor[0] && ts->buf[1] == ts->tags.winddirhor[1]) {
        	process_character_uint(ts, c, ts->msg.winddirhor);
        	break;
        }
        if (ts->buf[0] == ts->tags.winddirver[0] && ts->buf[1] == ts->tags.winddirver[1]) {
        	process_character_uint(ts, c, ts->msg.winddirver);
        	break;
        }
        if (ts->buf[0] == ts->tags.windspeed2d[0] && ts->buf[1] == ts->tags.windspeed2d[1]) {
        	process_character(ts, c, ts->msg.windspeed2d);
        	break;
        }
        if (ts->buf[0] == ts->tags.windspeed3d[0] && ts->buf[1] == ts->tags.windspeed3d[1]) {
        	process_character(ts, c, ts->msg.windspeed3d);
        	break;
        }
      }
      break;

    default:
      // error, back to SYNC
    	ts->state = TS_SYNC;
      break;
  }
}


// UART polling function
void trisonica_event(void)
{
  // Look for data on serial link and send to parser
  while (uart_char_available(&(TRISONICA_DEV))) {
    uint8_t ch = uart_getch(&(TRISONICA_DEV));
    trisonica_parse(&trisonica, ch);
  }
}

// utility function to send a string
void trisonica_send_string(char *s)
{
  uint8_t i = 0;
  while (s[i]) {
    uart_put_byte(&(JEVOIS_DEV), 0, (uint8_t)(s[i]));
    i++;
  }
}


