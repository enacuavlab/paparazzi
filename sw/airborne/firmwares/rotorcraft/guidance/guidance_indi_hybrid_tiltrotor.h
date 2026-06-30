/*
 * Copyright (C) 2026 Mauro VA <mauro.villanueva-aguado@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file firmwares/rotorcraft/guidance/guidance_indi_hybrid_tiltrotor.h
 */

#ifndef GUIDANCE_INDI_HYBRID_TILTROTOR_H
#define GUIDANCE_INDI_HYBRID_TILTROTOR_H

#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"
#include "modules/core/abi.h"
#include "math/pprz_algebra_float.h"


extern float guidance_indi_get_lift(struct FloatVect3 vel, float theta);
extern void guidance_indi_calcg_wing(float Gmat[GUIDANCE_INDI_HYBRID_V][GUIDANCE_INDI_HYBRID_U], struct FloatVect3 a_diff, float v_gih[GUIDANCE_INDI_HYBRID_V]);
extern void guidance_indi_hybrid_set_wls_settings(float body_v[3], float roll_angle, float pitch_angle);


#define GIHT_X 0
#define GIHT_Y 1
#define GIHT_Z 2

#define GIHT_CMD_ROLL   0
#define GIHT_CMD_PITCH  1
#define GIHT_CMD_TZ     2
#define GIHT_CMD_TX     3


#ifndef GUIDANCE_INDI_PUSHER_INDEX
#define GUIDANCE_INDI_PUSHER_INDEX 4
#endif

#ifndef GUIDANCE_INDI_MASS
#define GUIDANCE_INDI_MASS 1.0f
#endif

#ifndef GUIDANCE_INDI_MAX_ACC_BODY_X
#define GUIDANCE_INDI_MAX_ACC_BODY_X 2.0f
#endif

#ifndef GUIDANCE_INDI_MAX_ACC_BODY_Z
#define GUIDANCE_INDI_MAX_ACC_BODY_Z 3.0f
#endif

#ifndef GUIDANCE_INDI_WING_AREA
#define GUIDANCE_INDI_WING_AREA 0.5f
#endif

#ifndef GUIDANCE_INDI_CL_0
#define GUIDANCE_INDI_CL_0 0.1f
#endif

#ifndef GUIDANCE_INDI_CL_ALPHA
#define GUIDANCE_INDI_CL_ALPHA 5.0f
#endif

#ifndef GUIDANCE_INDI_PITCH_PREF_DEG
#define GUIDANCE_INDI_PITCH_PREF_DEG 5.0f         /* preferred pitch angle, positive nose up [deg] */
#endif

#ifndef GUIDANCE_INDI_AIRSPEED_IMPORTANCE
#define GUIDANCE_INDI_AIRSPEED_IMPORTANCE 2.0f /* forward-velocity weight boost in cruise */
#endif

#ifndef GUIDANCE_INDI_WU_PITCH
#define GUIDANCE_INDI_WU_PITCH 100.0f  /* pitch preference weight (vs gamma_sq * Wv = 1e5 on the objectives) */
#endif


extern float guidance_indi_max_thr_z;
extern float guidance_indi_max_thr_x;


#endif /* GUIDANCE_INDI_HYBRID_TILTROTOR_H */