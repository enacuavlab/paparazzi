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
 

#include "modules/sensors/trisonica.h"
#include "mcu_periph/uart.h"
#include "modules/core/abi.h"
#include "math/pprz_algebra_float.h"
#include "modules/datalink/downlink.h"
#include "pprzlink/messages.h"
#include "generated/airframe.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "std.h"


// ** Declaration ** // 


struct trisonica_t trisonica;


// ** Tool functions ** // 

float fast_atof(const char *s) {
    float result = 0.0f;
    float sign = 1.0f;
    float fraction = 0.0f;
    int divisor = 1;
    int seen_dot = 0;

    if (*s == '-') {
        sign = -1.0f;
        s++;
    }

    while (*s) {
        if (*s == '.') {
            seen_dot = 1;
        } else if (*s >= '0' && *s <= '9') {
            if (seen_dot) {
                fraction = fraction * 10.0f + (*s - '0');
                divisor *= 10;
            } else {
                result = result * 10.0f + (*s - '0');
            }
        } else {
            break; // stop at first non-numeric char
        }
        s++;
    }

    return sign * (result + fraction / divisor);
}

int16_t fast_atoi16(const char *s) {
    int16_t result = 0;
    int negative = 0;

    if (*s == '-') {
        negative = 1;
        s++;
    }

    while (*s >= '0' && *s <= '9') {
        result = result * 10 + (*s - '0');
        s++;
    }

    return negative ? -result : result;
}


// ** Init and Parse functions ** // 

// Initialization
void trisonica_init(void)
{
  trisonica.state = TS_SYNC;
  trisonica.idx = 0;
  trisonica.data_available = false;
  memset(trisonica.buf, 0,TRISONICA_MAX_LEN);
  trisonica.log_ptu_started = false;

  strcpy(trisonica.tags.windspeed3d, "S ");
  strcpy(trisonica.tags.windspeed2d, "S2");
  strcpy(trisonica.tags.winddirhor,  "D ");
  strcpy(trisonica.tags.winddirver,  "DV");
  strcpy(trisonica.tags.u_vector,    "U ");
  strcpy(trisonica.tags.v_vector,    "V ");
  strcpy(trisonica.tags.w_vector,    "W ");
  strcpy(trisonica.tags.temp,        "T ");
  strcpy(trisonica.tags.relative_humidity, "H ");
  strcpy(trisonica.tags.dew_point,    "DP");
  strcpy(trisonica.tags.abs_pressure, "P ");
  strcpy(trisonica.tags.airdensity,   "AD");
  strcpy(trisonica.tags.accel_x,      "AX");
  strcpy(trisonica.tags.accel_y,      "AY");
  strcpy(trisonica.tags.accel_z,      "AZ");
  strcpy(trisonica.tags.pitch,        "PI");
  strcpy(trisonica.tags.roll,         "RO");
  strcpy(trisonica.tags.mag_x,        "MX");
  strcpy(trisonica.tags.mag_y,        "MY");
  strcpy(trisonica.tags.mag_z,        "MZ");
  strcpy(trisonica.tags.mag_head_ang, "MD");
  strcpy(trisonica.tags.true_head,    "TD");
}

// Process char for float param
int process_character(struct trisonica_t *ts, int local_idx, float *parameter) {

	char local_buff[20];
	int i=0;
	int flag_smthg_writ = 0;

	for(int k=local_idx+2;k<TRISONICA_MAX_LEN;k++){ //We start to check data value after the tag character (+2)

		if(ts->buf[k] != 0x20 && ts->buf[k] != '\r'){ //We remove empty char (associated to 0x20 hexa),at beginning, after the 4 first empty or tag char
			local_buff[i]=ts->buf[k];
			i++;
			flag_smthg_writ = 1;
		} else if((ts->buf[k] == 0x20 || ts->buf[k] == '\r')&& flag_smthg_writ == 1){ //End of the parameter
			local_buff[i] = '\0';  // End the buffer
			*parameter = fast_atof(local_buff); // Save the data
			return k;
		}
	}
	return TRISONICA_MAX_LEN; //In case of error
}

// Process char for int16_t param
int process_character_uint(struct trisonica_t *ts, int local_idx, int16_t *parameter) {

	char local_buff[20];
	int i=0;
	int flag_smthg_writ = 0;

	for(int k=local_idx+2;k<TRISONICA_MAX_LEN;k++){ //We start to check data value after the tag character (+2)

		if(ts->buf[k] != 0x20){ //We remove empty char (associated to 0x20 hexa),at beginning, after the 4 first empty or tag char
			local_buff[i]=ts->buf[k];
			i++;
			flag_smthg_writ = 1;
		} else if(ts->buf[k] == 0x20 && flag_smthg_writ == 1){ //End of the parameter
			local_buff[i] = '\0';  // End the buffer
			*parameter = fast_atoi16(local_buff); // Save the data
			return k;
		}
	}
	return TRISONICA_MAX_LEN; //In case of error
}

//Function for line processing
//Exemple of line : S  00.04 S2  00.02 D  013 DV -063 U -00.00 V -00.02 W -00.04 T  22.53 H  58.56 P  1003.34 AD  1.1724622 PI -004.3 RO -002.7 MD  010 TD  010
void process_line(struct trisonica_t *ts){

	int loc_idx =0;

	//Loop on all char of the line
	for(int i =0; i < ts->idx;i++){
		loc_idx = i;

        if (ts->buf[loc_idx] == ts->tags.abs_pressure[0] && ts->buf[loc_idx+1] == ts->tags.abs_pressure[1]) {
        	i = process_character(ts, loc_idx, &ts->msg.abs_pressure); //We set the iteration parameter to the past index position in the fonction
        }
        if (ts->buf[loc_idx] == ts->tags.accel_x[0] && ts->buf[loc_idx+1] == ts->tags.accel_x[1]) {
        	i = process_character(ts, loc_idx, &ts->msg.accel_x);
        }
        if (ts->buf[loc_idx] == ts->tags.accel_y[0] && ts->buf[loc_idx+1] == ts->tags.accel_y[1]) {
        	i = process_character(ts, loc_idx, &ts->msg.accel_y);
        }
        if (ts->buf[loc_idx] == ts->tags.accel_z[0] && ts->buf[loc_idx+1] == ts->tags.accel_z[1]) {
        	i = process_character(ts, loc_idx, &ts->msg.accel_z);
        }
        if (ts->buf[loc_idx] == ts->tags.airdensity[0] && ts->buf[loc_idx+1] == ts->tags.airdensity[1]) {
        	i = process_character(ts, loc_idx, &ts->msg.airdensity);
        }
        if (ts->buf[loc_idx] == ts->tags.dew_point[0] && ts->buf[loc_idx+1] == ts->tags.dew_point[1]) {
        	i = process_character(ts, loc_idx, &ts->msg.dew_point);
        }
        if (ts->buf[loc_idx] == ts->tags.mag_head_ang[0] && ts->buf[loc_idx+1] == ts->tags.mag_head_ang[1]) {
        	i = process_character(ts, loc_idx, &ts->msg.mag_head_ang);
        }
        if (ts->buf[loc_idx] == ts->tags.mag_x[0] && ts->buf[loc_idx+1] == ts->tags.mag_x[1]) {
        	i = process_character(ts, loc_idx, &ts->msg.mag_x);
        }
        if (ts->buf[loc_idx] == ts->tags.mag_y[0] && ts->buf[loc_idx+1] == ts->tags.mag_y[1]) {
        	i = process_character(ts, loc_idx, &ts->msg.mag_y);
        }
        if (ts->buf[loc_idx] == ts->tags.mag_z[0] && ts->buf[loc_idx+1] == ts->tags.mag_z[1]) {
        	i = process_character(ts, loc_idx, &ts->msg.mag_z);
        }
        if (ts->buf[loc_idx] == ts->tags.pitch[0] && ts->buf[loc_idx+1] == ts->tags.pitch[1]) {
        	i = process_character(ts, loc_idx, &ts->msg.pitch);
        }
        if (ts->buf[loc_idx] == ts->tags.relative_humidity[0] && ts->buf[loc_idx+1] == ts->tags.relative_humidity[1]) {
        	i = process_character(ts, loc_idx, &ts->msg.relative_humidity);
        }
        if (ts->buf[loc_idx] == ts->tags.roll[0] && ts->buf[loc_idx+1] == ts->tags.roll[1]) {
        	i = process_character(ts, loc_idx, &ts->msg.roll);
        }
        if (ts->buf[loc_idx] == ts->tags.speed_sound[0] && ts->buf[loc_idx+1] == ts->tags.speed_sound[1]) {
        	i = process_character(ts, loc_idx, &ts->msg.speed_sound);
        }
        if (ts->buf[loc_idx] == ts->tags.temp[0] && ts->buf[loc_idx+1] == ts->tags.temp[1]) {
        	i = process_character(ts, loc_idx, &ts->msg.temp);
        }
        if (ts->buf[loc_idx] == ts->tags.true_head[0] && ts->buf[loc_idx+1] == ts->tags.true_head[1]) {
        	i = process_character(ts, loc_idx, &ts->msg.true_head);
        }
        if (ts->buf[loc_idx] == ts->tags.u_vector[0] && ts->buf[loc_idx+1] == ts->tags.u_vector[1]) {
        	i = process_character(ts, loc_idx, &ts->msg.u_vector);
        }
        if (ts->buf[loc_idx] == ts->tags.v_vector[0] && ts->buf[loc_idx+1] == ts->tags.v_vector[1]) {
        	i = process_character(ts, loc_idx, &ts->msg.v_vector);
        }
        if (ts->buf[loc_idx] == ts->tags.w_vector[0] && ts->buf[loc_idx+1] == ts->tags.w_vector[1]) {
        	i = process_character(ts, loc_idx, &ts->msg.w_vector);
        }
        if (ts->buf[loc_idx] == ts->tags.winddirhor[0] && ts->buf[loc_idx+1] == ts->tags.winddirhor[1]) {
        	i = process_character_uint(ts, loc_idx, &ts->msg.winddirhor);
        }
        if (ts->buf[loc_idx] == ts->tags.winddirver[0] && ts->buf[loc_idx+1] == ts->tags.winddirver[1]) {
        	i = process_character_uint(ts, loc_idx, &ts->msg.winddirver);
        }
        if (ts->buf[loc_idx] == ts->tags.windspeed2d[0] && ts->buf[loc_idx+1] == ts->tags.windspeed2d[1]) {
        	i = process_character(ts, loc_idx, &ts->msg.windspeed2d);
        }
		if (ts->buf[loc_idx] == ts->tags.windspeed3d[0] && ts->buf[loc_idx+1] == ts->tags.windspeed3d[1]) {
			i = process_character(ts, loc_idx, &ts->msg.windspeed3d); 
		}
	}
}

// The LI-550 outputs data in an ASCII character string ending with carriage return
// and line feed characters. Each line is a single record of all the measured parameters
// contained in a single sample.
void trisonica_parse(struct trisonica_t *ts, char c)
{
  switch (ts->state) {

    case TS_SYNC:
      // wait for sync (Line feed character == Last line char)
      if (c == '\n') {
    	ts->state = TS_TYPE;
    	ts->idx = 0;
      }
      break;

    case TS_TYPE:
    	if(c != '\n'){
    		ts->buf[ts->idx++] = c; // fill buffer until end of carriage char
    	} else if(c == '\n'){
    		process_line(ts);
    		ts->now_ts = get_sys_time_usec(); // current timestamp
    		ts->data_available = true; // New data
    		trisonica_report();
    		trisonica_log_data_ascii(ts);
    	    //trisonica_handle_msg(ts); // Send message to ABI 
    	    memset(ts->buf, 0,TRISONICA_MAX_LEN); // Reset Buffer
    	    ts->idx = 0; // Reset index for next acquisition
    	}
    	break;

    default:
      // error, back to SYNC
    	ts->state = TS_SYNC;
      break;
  }
}


// ** UART receive and send functions ** // 

// UART polling function
void trisonica_event(void)
{
	uint8_t ch;
  // Look for data on serial link and send to parser
  while (uart_char_available(&TRISONICA_UART)) {
    ch = uart_getch(&TRISONICA_UART);
    trisonica_parse(&trisonica, ch);
  }
}

// utility function to send a string
void trisonica_send_string(char *s)
{
  uint8_t i = 0;
  while (s[i]) {
    uart_put_byte(&(TRISONICA_UART), 0, (uint8_t)(s[i]));
    i++;
  }
}

// ** Report et event functions ** // 

// reporting function, send telemetry message
void trisonica_report() {
	
  if (trisonica.data_available == false) {
    // no new data, return
    return;
  }

	//We cast our parameter to fit in an existing telemetry named AEROPROBE
	uint32_t arg1 = (uint32_t)trisonica.msg.mag_head_ang;
	int16_t arg2 = (int16_t)abs((int)trisonica.msg.pitch);
	int16_t arg3 = (int16_t)abs((int)trisonica.msg.roll);
	int16_t arg4 = (int16_t)(100 * trisonica.msg.temp);
	int32_t arg5 = (int32_t)abs(trisonica.msg.winddirhor);
	int32_t arg6 = (int32_t)abs(trisonica.msg.winddirver);
	int32_t arg7 = (int32_t)(100 * trisonica.msg.windspeed2d);
	uint8_t arg8 = (uint8_t)(100 * trisonica.msg.windspeed3d);
	
  DOWNLINK_SEND_AEROPROBE(DefaultChannel, DefaultDevice,
       &arg1,
       &arg2,
       &arg3,
       &arg4,
       &arg5,
       &arg6,
       &arg7,
       &arg8
      );

    trisonica.data_available = false;
}

/*
static void trisonica_handle_msg(struct trisonica_t *ts) {

  // send ABI message
  AbiSendMsgTRISONICA_MSG(TRISONCIA_ID,
      ts->now_ts,
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
      ts->msg.windspeed3d
      );
  // Here can add the send of specific msg
}*/

// ** Log SD function ** // 

void trisonica_log_data_ascii(struct trisonica_t *ts) {
	
  if (pprzLogFile != -1) {
	  
    if (!ts->log_ptu_started) {
      sdLogWriteLog(pprzLogFile, "#\n");
      sdLogWriteLog(pprzLogFile,"time(us),abs_pressure(hPa),accel_x(m/s2),accel_y,accel_z,airdensity(kg/m3),dew_point(degC),mag_head_ang(deg),mag_x,mag_y,mag_z,pitch(deg),relative_humidity,roll(deg),speed_sound(m/s),temp(degC),true_head(deg),u_vec,v_vec,w_vec,winddirhor(deg),winddirver(deg),windspeed2d(m/s),windspeed3d(m/s)\n");
      ts->log_ptu_started = true;
    } else {
      sdLogWriteLog(pprzLogFile, "%lu,%.2f,%.2f,%.2f,%.2f,%.7f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%.2f,%.2f\n",
		  ts->now_ts,
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
		  ts->msg.windspeed3d
		  );
    }
    
  }
}

