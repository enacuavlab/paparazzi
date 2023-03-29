/*
 * Copyright (C) 2023 Fabien-B <fabien.bonneval@enac.fr>
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

/** @file "modules/sensors/panache_sensor.c"
 * @author Fabien-B <fabien.bonneval@enac.fr>
 * Get data from Panache sensors, or datalink (for simulation)
 */

#include "modules/sensors/panache_sensor.h"
#include "generated/airframe.h"
#include "pprzlink/messages.h"
#include "modules/datalink/datalink.h"
#include "modules/datalink/downlink.h"
#include "modules/loggers/flight_recorder.h"


void panache_init(void)
{
  // your init code here
}

static float float_from_buf(uint8_t* buf) {
  if(DL_PAYLOAD_COMMAND_command_length(buf) != 4) {return -1;}
  
  union {
    uint8_t c[4];
    float f;
  } tmp;

  for (int i=0; i<4; i++) {
    tmp.c[i] = DL_PAYLOAD_COMMAND_command(buf)[i];
  }

  return tmp.f;
}

static void send_float(float f) {
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 1, &f);

}

void panache_dl_cb(uint8_t* buf)
{
  // PAYLOAD_COMMAND
  if (DL_PAYLOAD_COMMAND_ac_id(dl_buffer) == AC_ID) {
    float res = float_from_buf(buf);
    send_float(res);
#ifdef SITL
    printf("(%2d) panache sensor: %e\n",AC_ID, res);
#endif
  }
}

void panache_sensors_cb(uint8_t* buf) {
    float* values = pprzlink_get_DL_PAYLOAD_FLOAT_values(buf);
    uint8_t len = pprzlink_get_PAYLOAD_FLOAT_values_length(buf);
    
    if(sizeof(SensorsData_t) == len*sizeof(float)) {

#if FLIGHTRECORDER_SDLOG
    // log to SD card
    pprz_msg_send_PAYLOAD_FLOAT(&pprzlog_tp.trans_tx, &(flightrecorder_sdlog).device, AC_ID, len, values);
#endif
    
    SensorsData_t* data = (SensorsData_t*) values;
                
#ifdef SITL
    printf("[panache] lpl:%e\tspl:%e\tmpl:%e\tpm0:%e\tpm1:%e\tpm2:%e\n", data->gas.lpl, data->gas.spl, data->gas.mpl, data->particle.pm[0], data->particle.pm[1], data->particle.pm[2]);
#endif

    float resume[6] = {data->gas.lpl, data->gas.spl, data->gas.mpl, data->particle.pm[0], data->particle.pm[1], data->particle.pm[2]};
    DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 6, resume);
    }
}


