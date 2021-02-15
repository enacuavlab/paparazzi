/*
 * Copyright (C) 2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ahrs/ahrs_datalink.c
 *
 * Receive and parse AHRS data from PPRZLINK message
 */

#include "modules/ahrs/ahrs_datalink.h"
#include "subsystems/ahrs.h"
#include "math/pprz_algebra_float.h"
#include "mcu_periph/sys_time.h"
#include "state.h"

#ifndef AHRS_DATALINK_OUTPUT_ENABLED
#define AHRS_DATALINK_OUTPUT_ENABLED TRUE
#endif
PRINT_CONFIG_VAR(AHRS_DATALINK_OUTPUT_ENABLED)

#ifndef AHRS_DATALINK_TIMEOUT
#define AHRS_DATALINK_TIMEOUT 0.5f // timeout in seconds
#endif
PRINT_CONFIG_VAR(AHRS_DATALINK_TIMEOUT)

#define AHRS_DATALINK_QUAT    0
#define AHRS_DATALINK_EULER   1

// bitmask
#define AHRS_DATALINK_ATT_BIT     0
#define AHRS_DATALINK_HEADING_BIT 1
#define AHRS_DATALINK_RATES_BIT   2

struct ahrs_datalink_t {
  bool output_enabled;      ///< if TRUE with push the estimation results to the state interface
  bool valid;               ///< solution is valid
  float last_time;
  struct FloatQuat quat;
  struct FloatEulers eulers;
  struct FloatRates rates;
  uint8_t flag;
  uint8_t type;             ///< quat or euler
};

static struct ahrs_datalink_t ahrs_datalink;

static uint8_t ahrs_datalink_id = AHRS_COMP_ID_DATALINK;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_filter_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t mde = 3;
  uint16_t val = 0;
  if (!ahrs_datalink.valid) { mde = 5; }
  pprz_msg_send_STATE_FILTER_STATUS(trans, dev, AC_ID, &ahrs_datalink_id, &mde, &val);
}
#endif


static bool ahrs_datalink_enable_output(bool enable)
{
  ahrs_datalink.output_enabled = enable;
  return ahrs_datalink.output_enabled;
}

/**
 * Compute body orientation and rates from imu orientation and rates
 */
static void set_body_orientation_and_rates(void)
{
  if (ahrs_datalink.output_enabled && ahrs_datalink.valid) {
    if (bit_is_set(ahrs_datalink.flag, AHRS_DATALINK_RATES_BIT)) {
      stateSetBodyRates_f(&ahrs_datalink.rates);
    }

    // FIXME use correctly ATT and HEADING flag, set all attitude for now
    if (bit_is_set(ahrs_datalink.flag, AHRS_DATALINK_ATT_BIT)) {
      if (ahrs_datalink.type == AHRS_DATALINK_QUAT) {
        stateSetNedToBodyQuat_f(&ahrs_datalink.quat);
      }
      else if (ahrs_datalink.type == AHRS_DATALINK_EULER) {
        stateSetNedToBodyEulers_f(&ahrs_datalink.eulers);
      }
    }
  }
}

void ahrs_datalink_init(void)
{
  ahrs_datalink.valid = false;
  ahrs_datalink.last_time = get_sys_time_float();
  ahrs_datalink.flag = 0;
  ahrs_datalink.type = AHRS_DATALINK_QUAT;
  float_quat_identity(&ahrs_datalink.quat);
  FLOAT_EULERS_ZERO(ahrs_datalink.eulers);
}

void ahrs_datalink_register(void)
{
  ahrs_datalink.output_enabled = AHRS_DATALINK_OUTPUT_ENABLED;
  ahrs_datalink_init();
  ahrs_register_impl(ahrs_datalink_enable_output);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STATE_FILTER_STATUS, send_filter_status);
#endif
}

void ahrs_datalink_parse_quat(uint8_t *buf)
{
  if (DL_REMOTE_AHRS_QUAT_ac_id(buf) != AC_ID) { return; } // not for this aircraft

  ahrs_datalink.type = AHRS_DATALINK_QUAT;
  ahrs_datalink.flag = DL_REMOTE_AHRS_QUAT_flag(buf);
  ahrs_datalink.quat.qi = DL_REMOTE_AHRS_QUAT_qi(buf);
  ahrs_datalink.quat.qx = DL_REMOTE_AHRS_QUAT_qx(buf);
  ahrs_datalink.quat.qy = DL_REMOTE_AHRS_QUAT_qy(buf);
  ahrs_datalink.quat.qz = DL_REMOTE_AHRS_QUAT_qz(buf);
  ahrs_datalink.rates.p = DL_REMOTE_AHRS_QUAT_p(buf);
  ahrs_datalink.rates.q = DL_REMOTE_AHRS_QUAT_q(buf);
  ahrs_datalink.rates.r = DL_REMOTE_AHRS_QUAT_r(buf);

  // check timeout
  float t = get_sys_time_float();
  if (t - ahrs_datalink.last_time > AHRS_DATALINK_TIMEOUT) {
    ahrs_datalink.valid = false;
  } else {
    ahrs_datalink.valid = true;
  }
  ahrs_datalink.last_time = t;
  // update state
  set_body_orientation_and_rates();
}

void ahrs_datalink_parse_eulers(uint8_t *buf)
{
  if (DL_REMOTE_AHRS_EULER_ac_id(buf) != AC_ID) { return; } // not for this aircraft

  ahrs_datalink.type = AHRS_DATALINK_EULER;
  ahrs_datalink.flag = DL_REMOTE_AHRS_EULER_flag(buf);
  ahrs_datalink.eulers.phi = DL_REMOTE_AHRS_EULER_phi(buf);
  ahrs_datalink.eulers.theta = DL_REMOTE_AHRS_EULER_theta(buf);
  ahrs_datalink.eulers.psi = DL_REMOTE_AHRS_EULER_psi(buf);
  ahrs_datalink.rates.p = DL_REMOTE_AHRS_EULER_p(buf);
  ahrs_datalink.rates.q = DL_REMOTE_AHRS_EULER_q(buf);
  ahrs_datalink.rates.r = DL_REMOTE_AHRS_EULER_r(buf);

  // check timeout
  float t = get_sys_time_float();
  if (t - ahrs_datalink.last_time > AHRS_DATALINK_TIMEOUT) {
    ahrs_datalink.valid = false;
  } else {
    ahrs_datalink.valid = true;
  }
  ahrs_datalink.last_time = t;
  // update state
  set_body_orientation_and_rates();
}

