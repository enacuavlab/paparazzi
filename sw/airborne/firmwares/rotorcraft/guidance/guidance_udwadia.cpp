/*
 * Copyright (C) Florian Sansou <fl.sansou@enac.fr>
 * ENAC uav lab
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
 * @file firmwares/rotorcraft/guidance/guidance_udwadia.cpp
 *
 * A guidance mode
 */
#include "generated/airframe.h"
// #include "firmwares/rotorcraft/guidance/guidance_indi.h"
// #include "modules/ins/ins_int.h"
#include "modules/radio_control/radio_control.h"
#include "state.h"
#include "modules/imu/imu.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"
// #include "stabilization/stabilization_attitude_ref_quat_int.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "filters/low_pass_filter.h"
#include "modules/core/abi.h"



#if PERIODIC_TELEMETRY
// #include "modules/datalink/telemetry.h"
// static void send_indi_guidance(struct transport_tx *trans, struct link_device *dev)
// {
//   pprz_msg_send_GUIDANCE_INDI_HYBRID(trans, dev, AC_ID,
//                               &sp_accel.x,
//                               &sp_accel.y,
//                               &sp_accel.z,
//                               &control_increment.x,
//                               &control_increment.y,
//                               &control_increment.z,
//                               &filt_accel_ned[0].o[0],
//                               &filt_accel_ned[1].o[0],
//                               &filt_accel_ned[2].o[0],
//                               &speed_sp.x,
//                               &speed_sp.y,
//                               &speed_sp.z);
// }
#endif

/**
 * @brief Init function
 */
void guidance_udwadia_init(void)
{
//   AbiBindMsgACCEL_SP(GUIDANCE_INDI_ACCEL_SP_ID, &accel_sp_ev, accel_sp_cb);

// #if PERIODIC_TELEMETRY
//   register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE_INDI_HYBRID, send_indi_guidance);
// #endif
}

/**
 *
 * Call upon entering indi guidance
 */
void guidance_udwadia_enter(void)
{
  
}

/**
 * @param accel_sp accel setpoint in NED frame [m/s^2]
 * @param heading_sp the desired heading [rad]
 * @return stabilization setpoint structure
 *
 * main indi guidance function
 */
// struct StabilizationSetpoint guidance_udwadia_run(struct FloatVect3 *accel_sp, float heading_sp)
// {
 
// }

// struct StabilizationSetpoint guidance_udwadia_run_mode(bool in_flight UNUSED, struct HorizontalGuidance *gh, struct VerticalGuidance *gv)
// {
 
// }



// struct StabilizationSetpoint guidance_h_run_pos(bool __attribute__((unused)) in_flight, struct HorizontalGuidance __attribute__((unused)) *gh)
// {
//   // return guidance_udwadia_run_mode(in_flight, gh, _gv, GUIDANCE_INDI_H_POS, _v_mode);
// }
// 
// struct StabilizationSetpoint guidance_h_run_speed(bool __attribute__((unused)) in_flight, struct HorizontalGuidance __attribute__((unused)) *gh)
// {
//   // return guidance_udwadia_run_mode(in_flight, gh, _gv, GUIDANCE_INDI_H_SPEED, _v_mode);
// }
// 
// struct StabilizationSetpoint guidance_h_run_accel(bool __attribute__((unused)) in_flight, struct HorizontalGuidance __attribute__((unused)) *gh)
// {
//   // return guidance_udwadia_run_mode(in_flight, gh, _gv, GUIDANCE_INDI_H_ACCEL, _v_mode);
// }

// int32_t guidance_v_run_pos(bool in_flight UNUSED, struct VerticalGuidance *gv)
// {
//   // _gv = gv;
//   // _v_mode = GUIDANCE_INDI_V_POS;
//   return 0; // nothing to do
// }

// int32_t guidance_v_run_speed(bool in_flight UNUSED, struct VerticalGuidance *gv)
// {
//   // _gv = gv;
//   // _v_mode = GUIDANCE_INDI_V_SPEED;
//   return 0; // nothing to do
// }

// int32_t guidance_v_run_accel(bool in_flight UNUSED, struct VerticalGuidance *gv)
// {
//   // _gv = gv;
//   // _v_mode = GUIDANCE_INDI_V_ACCEL;
//   return 0; // nothing to do
// }


