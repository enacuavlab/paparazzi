/*
 * Copyright (C) 2026 Fabien-B <fabien-B@github.com>
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

/** @file "modules/imav_rescue_people/scoutector.c"
 * @author Fabien-B <fabien-B@github.com>
 * Scoutector driver, specially made for IMAV 2026 !
 */

#include "modules/imav_rescue_people/scoutector.h"
#include "string.h"
#include "uavcan/uavcan.h"
#include "uavcan.protocol.debug.KeyValue.h"
#include "modules/datalink/downlink.h"

static uavcan_event scoutector_uavcan_ev;

scoutector_t scout_data;

static void scoutector_uavcan_cb(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer) {
  struct uavcan_protocol_debug_KeyValue msg;

  if(uavcan_protocol_debug_KeyValue_decode(transfer, &msg)) {
    return;
  }

  if(msg.key.len != 3) {
    return;
  }

  if(strncmp("det", (const char*)msg.key.data, 3) == 0) {
    scout_data.det = msg.value;
  } else if(strncmp("snr", (const char*)msg.key.data, 3) == 0) {
    scout_data.snr = msg.value;
  } else if(strncmp("lit", (const char*)msg.key.data, 3) == 0) {
    scout_data.lit = msg.value;
  }
  //debug
  else if(strncmp("lct", (const char*)msg.key.data, 3) == 0) {
    //TODO log ?
  } else if(strncmp("ltu", (const char*)msg.key.data, 3) == 0) {
    //TODO log ?
  }
  
}


void scoutector_init(void)
{
  uavcan_bind(UAVCAN_PROTOCOL_DEBUG_KEYVALUE_ID, UAVCAN_PROTOCOL_DEBUG_KEYVALUE_SIGNATURE, &scoutector_uavcan_ev, &scoutector_uavcan_cb);
}

void scoutector_report(void) {
  float f[3] = {scout_data.det, scout_data.snr, scout_data.lit};
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 3, f);
}
