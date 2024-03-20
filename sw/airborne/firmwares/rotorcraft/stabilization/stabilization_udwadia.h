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

#ifndef STABILIZATION_UDWADIA
#define STABILIZATION_UDWADIA

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"



extern struct Int32Quat   stab_att_sp_quat;  ///< with #INT32_QUAT_FRAC
extern struct Int32Eulers stab_att_sp_euler; ///< with #INT32_ANGLE_FRAC
extern float g1g2[INDI_OUTPUTS][INDI_NUM_ACT];
extern float actuator_state_filt_vect[INDI_NUM_ACT];
extern bool act_is_servo[INDI_NUM_ACT];

extern bool indi_use_adaptive;

extern float du_min_stab_indi[INDI_NUM_ACT];
extern float du_max_stab_indi[INDI_NUM_ACT];
extern float du_pref_stab_indi[INDI_NUM_ACT];
extern float *Bwls[INDI_OUTPUTS];

extern float thrust_bx_eff;
extern float thrust_bx_act_dyn;
extern float actuator_thrust_bx_pprz;
extern float thrust_bx_state_filt;

extern float act_pref[INDI_NUM_ACT];

extern float indi_Wu[INDI_NUM_ACT];

struct Indi_gains {
  struct FloatRates att;
  struct FloatRates rate;
};

extern struct Indi_gains indi_gains;

extern void stabilization_udwadia_init(void);
extern void stabilization_udwadia_enter(void);
extern void stabilization_attitude_udwadia_run(bool in_flight)
extern void stabilization_udwadia_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn);


#endif /* STABILIZATION_INDI */

