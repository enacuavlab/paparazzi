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
 * @file "modules/gps/gps_nmea_send.c"
 * @author Jean-Baptiste FORESTIER
 * @brief module used to send GPS data over a Tawaki UART for extern instrument using NMEA protocol
 * Exemple of use : MAPIR camera stores GPS data in metadata on each frame 
 */



#include "modules/gps/gps_nmea_send.h"
#include "mcu_periph/uart.h"
#include "generated/airframe.h"
#include "state.h" // State interface for rotation compensation
#include "modules/gps/gps.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "std.h"
#include <math.h>
#include <inttypes.h>
#include <time.h>

// ** Declaration ** // 
struct Gps_Nmea_Send gps_nmea_send;


/**
 * Initialization function
 */
void gps_nmea_send_init(void) {

	gps_nmea_send.error_init = false;
	gps_nmea_send.initialized = false;
	
	gps_nmea_send.msg.now_ts = 0;
	gps_nmea_send.msg.lat = 0;
	gps_nmea_send.msg.lon = 0;
	gps_nmea_send.msg.alt = 0;			
	gps_nmea_send.msg.phi = 0;
	gps_nmea_send.msg.theta = 0;
	gps_nmea_send.msg.psi = 0;
	gps_nmea_send.msg.vground = 0;		
	gps_nmea_send.msg.course = 0;
	gps_nmea_send.msg.groundalt = 0;	   
        	
	gps_nmea_send.initialized = true;
}



/**
 * Periodic function to send data
 */
void gps_nmea_send_periodic(void) {

  if (gps_nmea_send.initialized) {   
		recover_gps_data();
		build_nmea_sentence();	
  } else {
		gps_nmea_send_init();
  }
  
}


/**
 * Function to recover data available onboard from GPS
 */
void recover_gps_data(void) {	
	
	  gps_nmea_send.msg.now_ts = get_sys_time_usec(); // current timestamp
      gps_nmea_send.msg.lat = stateGetPositionLla_f()->lat;
      gps_nmea_send.msg.lon = stateGetPositionLla_f()->lon;
      //gps_nmea_send.msg.alt = stateGetPositionLla_f()->alt;
      gps_nmea_send.msg.phi = stateGetNedToBodyEulers_f()->phi;
      gps_nmea_send.msg.theta = stateGetNedToBodyEulers_f()->theta;
      gps_nmea_send.msg.psi = stateGetNedToBodyEulers_f()->psi;
      gps_nmea_send.msg.vground = stateGetHorizontalSpeedNorm_f();
      gps_nmea_send.msg.course = DegOfRad(stateGetHorizontalSpeedDir_f());
      gps_nmea_send.msg.groundalt = POS_BFP_OF_REAL(state.alt_agl_f);		
}

/**
 * Function to build sentence for NMEA Protocol
 */
void build_nmea_sentence(void) {	



    // Latitude and Longitude
    char lat_buf[16], lon_buf[16],time_nmea [16], gga[120], rmc[120], date_sys[6];
    int len_gga = 0;
    int len_rmc = 0;
    uint8_t cs;
    nmea_convert_deg_to_DDMM(gps_nmea_send.msg.lat, lat_buf, 1);
    nmea_convert_deg_to_DDMM(gps_nmea_send.msg.lon, lon_buf, 0);
    char lat_hemi = (gps_nmea_send.msg.lat >= 0) ? 'N' : 'S';
    char lon_hemi = (gps_nmea_send.msg.lon >= 0) ? 'E' : 'W';
    nmea_time_from_timestamp(gps_nmea_send.msg.now_ts, time_nmea);


    // -------------------------------
    //  GPGGA Frame
    // -------------------------------
    for (int i=0; i<120; i++) {gga[i] = '\0';}

    if(gps.fix >= GPS_FIX_3D ) {

    sprintf( gga,
            "$GPGGA,%s,%s,%c,%s,%c,1,%02u,%04u,%09.3f,M,0,M,,",
            time_nmea, lat_buf, lat_hemi, lon_buf, lon_hemi, gps.num_sv, gps.pdop, (float)gps.hmsl/1000);
    len_gga = 7;   // $GPGGA, = 7
    len_gga += 7;  // time_nmea, = 7 
    len_gga += 10; // lat_buf = 9 + comma 
    len_gga += 2;  // lat_hemi = N,
    len_gga += 11; // lon_buf = 10 + comma
    len_gga += 2;  // lon_hemi = W,
    len_gga += 2;  // GPS Quality indicator GPS fix = 1,
    len_gga += 3;  // number of sats = 08,
    len_gga += 5;  // position dilution of precision scaled by 100 = 1020,
    len_gga += 10; // height above mean sea level (MSL) in m = 02945.127,
    len_gga += 2;  // height above mean sea level unit = M,
    len_gga += 2;  // Ellipsoid to geoid distance set 0, don't where to find it = 0,
    len_gga += 2;  // Distance above unit = M,
    len_gga += 1;  // Empty = ,

    cs = nmea_checksum(gga, len_gga);
    sprintf(gga + len_gga, "*%02X\r\n", cs);
    len_gga += 5;  // * + Checksum + \r\n = *4F\r\n

    nmea_send(gga, len_gga);   
    
    } else {
        //If no fix, empty GGA
        sprintf( gga,"$GPGGA,,,,,,0,00,99.99,,,,,,*68");
        nmea_send(gga, 31);
    }

    // -------------------------------
    //  GPRMC Frame
    // -------------------------------
    for (int i=0; i<120; i++) {rmc[i] = '\0';}

    if(gps.fix >= GPS_FIX_3D ) {

            
    get_system_date_str(date_sys, sizeof(date_sys));
    sprintf( rmc,
            "$GPRMC,%s,A,%s,%c,%s,%c,%05.1f,%05.1f,%s,000.0,W",
            time_nmea, lat_buf, lat_hemi, lon_buf, lon_hemi, (float)gps_nmea_send.msg.vground, gps_nmea_send.msg.course, date_sys);
    len_rmc = 7;   // $GPRMC, = 7
    len_rmc += 7;  // time_nmea, = 7 
    len_rmc += 2;  // Status A=active or V=void = A,
    len_rmc += 10; // lat_buf = 9 + comma 
    len_rmc += 2;  // lat_hemi = N,
    len_rmc += 11; // lon_buf = 10 + comma
    len_rmc += 2;  // lon_hemi = W,
    len_rmc += 6;  // vground = 050.4,
    len_rmc += 6;  // track angle course = 050.4,    
    len_rmc += 7;  // Date = 230394, 
    len_rmc += 7;  // Magnetic variation, in degrees = 003.1,W, 

    cs = nmea_checksum(rmc, len_rmc);
    sprintf(rmc + len_rmc, "*%02X\r\n", cs);
    len_rmc += 5;  // * + Checksum + \r\n = *4F\r\n

    nmea_send(rmc, len_rmc);   
    
    } else {
        //If no fix, empty RMC
        sprintf( rmc,"$GPRMC,,V,,,,,,,,,,*53");
        nmea_send(rmc, 22);
    }
}






/**
 * Function to calculate checksum 
 */
uint8_t nmea_checksum(const char *sentence, int length) {
    uint8_t cs = 0;
    for (int i = 0; i < length; i++) {
        cs ^= (uint8_t)sentence[i];
    }
    return cs;
}


/**
 * Function to convert lat long in DDLL.MMMM Format
 * Lat and long don't have the same length
 */
void nmea_convert_deg_to_DDMM(double deg, char *buf, int is_lat) {
    double abs_deg = fabs(deg);
    int d = (int)abs_deg;
    double minutes = (abs_deg - d) * 60.0;

    if (is_lat)
        sprintf(buf, "%02d%07.4f", d, minutes); // 2 integers for degrees 0 to 90 without sign => %02d | 2 integers for minutes, 1 for point, 4 for minutes decimals 
    else
        sprintf(buf, "%03d%07.4f", d, minutes); // 3 integers for degrees 0 to 180 without sign => %023 | 2 integers for minutes, 1 for point, 4 for minutes decimals 
}

/**
 * Function to convert timestamp to NMEA time
 */
void nmea_time_from_timestamp(uint32_t ts, char *buf)
{
    uint32_t s = ts % 60;
    uint32_t m = (ts / 60) % 60;
    uint32_t h = (ts / 3600) % 24;

    sprintf(buf, "%02" PRIu32 "%02" PRIu32 "%02" PRIu32, h, m, s);
}

/**
 * Get Date from system 
 */
void get_system_date_str(char *buf, size_t buf_size) {


    const int64_t gps_epoch_days = 3657; //day frm 1970-01-01
    uint64_t gps_sec = (uint64_t)gps.week * 7 * 24 * 3600 + gps.tow / 1000; //Total time gps
    int64_t days = gps_epoch_days + gps_sec / 86400; //Day gps + gps epoch

    //Algo to get year month day from Week and tow
    days += 719468; // Shift day count so that day 0 corresponds to the Gregorian epoch (0000-03-01)
    int64_t era = (days >= 0 ? days : days - 146096) / 146097; // Determine the 400-year Gregorian cycle, One Gregorian era = 400 years = 146097 days.
    uint64_t doe = (uint64_t)(days - era * 146097); // Day Of Era: number of days elapsed within the current 400-year cycle.
    uint64_t yoe = (doe - doe/1460 + doe/36524 - doe/146096) / 365; // Year Of Era: compute the year number inside the cycle, correcting for leap years (every 4 years, except centuries, except 400-year multiples).
    int64_t y = (int64_t)yoe + era * 400; // Convert "year in era" to an absolute Gregorian year.
    uint64_t doy = doe - (365*yoe + yoe/4 - yoe/100); // Day Of Year: subtract all full days from completed years to get day index within the year.
    uint64_t mp = (5*doy + 2)/153; // Month part: day-of-year into a month index in a calendar where March = 0 and February = 11
    uint64_t d = doy - (153*mp+2)/5 + 1; // Compute the day-of-month
    uint64_t m = mp + (mp < 10 ? 3 : -9); // Convert shifted month index back to standard month  0-12   

    // Format : YYMMDD
    snprintf(buf, buf_size, "%02d%02d%02d",
             (uint8_t)(y + (m <= 2))%100,
             (uint8_t)m,
             (uint8_t)d);
}


/**
 * Function to send payload to UART
 */
void nmea_send(const char *payload, int payload_length) {

    uart_put_buffer(&NMEA_SEND_UART, 0, (const uint8_t *)payload, payload_length);

}







