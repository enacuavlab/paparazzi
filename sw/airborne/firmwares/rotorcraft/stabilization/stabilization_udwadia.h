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

#ifdef __cplusplus
extern "C" {
#endif

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"

//extern struct FloatEulers stab_att_sp_euler;

extern void stabilization_udwadia_init(void);
extern void stabilization_udwadia_enter(void);
extern void stabilization_udwadia_run(bool in_flight);
extern void stabilization_udwadia_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn);

#ifdef __cplusplus
}
#endif

#endif /* STABILIZATION_UDWADIA */

