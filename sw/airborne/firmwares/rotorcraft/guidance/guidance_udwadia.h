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
 * @file firmwares/rotorcraft/guidance/guidance_udwadia.h
 *
 * A guidance mode
 */

#ifndef GUIDANCE_UDWADIA_H
#define GUIDANCE_UDWADIA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "firmwares/rotorcraft/guidance.h"
#include "firmwares/rotorcraft/stabilization.h"

extern void guidance_udwadia_init(void);
extern void guidance_udwadia_enter(void);



extern struct StabilizationSetpoint guidance_udwadia_run(struct FloatVect3 *accep_sp, float heading_sp);
// extern struct StabilizationSetpoint guidance_udwadia_run_mode(bool in_flight, struct HorizontalGuidance *gh, struct VerticalGuidance *gv, enum GuidanceIndi_HMode h_mode, enum GuidanceIndi_VMode v_mode);



extern struct FloatVect3 sp_accel;
// extern struct ThrustSetpoint thrust_sp;

#ifdef __cplusplus
}
#endif

#endif /* GUIDANCE_INDI_H */
