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
 * @file "modules/gps/gps_nmea_send.h"
 * @author Jean-Baptiste FORESTIER
 * @brief module used to send GPS data over a Tawaki UART for extern instrument using NMEA protocol
 * Exemple of use : MAPIR camera stores GPS data in metadata on each frame 
 */


#ifndef GPS_NMEA_SEND_H
#define GPS_NMEA_SEND_H

#include "std.h"


struct gps_nmea_send_msg_t {
	  uint32_t now_ts;
	  int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t phi;
    int32_t theta;
    int32_t psi;
    int32_t vground;
    float course;
    int32_t groundalt;
};


struct Gps_Nmea_Send
{
  bool initialized;
  bool error_init; // Flag to indicate if there was an error during initialization
  struct gps_nmea_send_msg_t msg;
};

void gps_nmea_send_init(void);
void gps_nmea_send_periodic(void);
void recover_gps_data(void);
void build_nmea_sentence(void);
uint8_t nmea_checksum(const char *sentence, int length);
void nmea_convert_deg_to_DDMM(double deg, char *buf, int is_lat);
void nmea_time_from_timestamp(uint32_t ts, char *buf);
void get_system_date_str(char *buf, size_t buf_size);
void nmea_send(const char *payload, int payload_length);


#endif
