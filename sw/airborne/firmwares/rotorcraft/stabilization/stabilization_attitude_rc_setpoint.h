/*
 * Copyright (C) 2012 Felix Ruess <felix.ruess@gmail.com>
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

/** @file stabilization_attitude_rc_setpoint.h
 *  Read an attitude setpoint from the RC.
 */

#ifndef STABILIZATION_ATTITUDE_RC_SETPOINT_H
#define STABILIZATION_ATTITUDE_RC_SETPOINT_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "modules/radio_control/radio_control.h"

extern int32_t transition_theta_offset; // Pitch offset added for hybrid vehicle when in forward mode

extern void stabilization_attitude_reset_care_free_heading(void);
extern int32_t stabilization_attitude_get_heading_i(void);
extern float stabilization_attitude_get_heading_f(void);
extern void stabilization_attitude_read_rc_roll_pitch_quat_f(struct FloatQuat *q, struct RadioControl *rc);
extern void stabilization_attitude_read_rc_roll_pitch_earth_quat_f(struct FloatQuat *q, struct RadioControl *rc);

/** Read attitude setpoint from RC as euler angles
 * @param[out] sp                attitude setpoint as euler angles
 * @param[in]  in_flight         true if in flight
 * @param[in]  in_carefree       true if in carefree mode
 * @param[in]  coordinated_turn  true if in horizontal mode forward
 * @param[in]  rc                pointer to radio control structure
 */
extern void stabilization_attitude_read_rc_setpoint_eulers(struct Int32Eulers *sp, bool in_flight,
    bool in_carefree, bool coordinated_turn, struct RadioControl *rc);

/** Read attitude setpoint from RC as float euler angles
 * @param[out] sp                attitude setpoint as euler angles
 * @param[in]  in_flight         true if in flight
 * @param[in]  in_carefree       true if in carefree mode
 * @param[in]  coordinated_turn  true if in horizontal mode forward
 * @param[in]  rc                pointer to radio control structure
 */
extern void stabilization_attitude_read_rc_setpoint_eulers_f(struct FloatEulers *sp, bool in_flight,
    bool in_carefree, bool coordinated_turn, struct RadioControl *rc);

/** Read attitude setpoint from RC as quaternion
 * Interprets the stick positions as axes.
 * @param[out] q_sp              attitude setpoint as quaternion
 * @param[in]  in_flight         true if in flight
 * @param[in]  in_carefree       true if in carefree mode
 * @param[in]  coordinated_turn  true if in horizontal mode forward
 * @param[in]  rc                pointer to radio control structure
 */
extern void stabilization_attitude_read_rc_setpoint_quat_f(struct FloatQuat *q_sp, bool in_flight,
    bool in_carefree, bool coordinated_turn, struct RadioControl *rc);

/** Read attitude setpoint from RC as quaternion in earth bound frame
 * @param[out] q_sp              attitude setpoint as quaternion
 * @param[in]  in_flight         true if in flight
 * @param[in]  in_carefree       true if in carefree mode
 * @param[in]  coordinated_turn  true if in horizontal mode forward
 * @param[in]  rc                pointer to radio control structure
 */
extern void stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(struct FloatQuat *q_sp, bool in_flight,
    bool in_carefree, bool coordinated_turn, struct RadioControl *rc);

#endif /* STABILIZATION_ATTITUDE_RC_SETPOINT_H */

