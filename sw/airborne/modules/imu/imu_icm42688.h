<<<<<<<< HEAD:sw/airborne/modules/imu/imu_icm42688.h
/*
 * Copyright (C) 2022 Jesús Bautista Villar <jesbauti20@gmail.com>
========
(*
 * Copyright (C) 2004 CENA/ENAC, Yann Le Fablec
>>>>>>>> refs/remotes/origin/panache_mfeurgard:sw/lib/ocaml/platform.mli
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
 *
 *)

<<<<<<<< HEAD:sw/airborne/modules/imu/imu_icm42688.h
/**
 * @file modules/imu/imu_icm42688.h
 * Driver for the IMU ICM42688.
 */

#ifndef IMU_ICM42688_H
#define IMU_ICM42688_H

#include "std.h"

extern void imu_icm42688_init(void);
extern void imu_icm42688_periodic(void);
extern void imu_icm42688_event(void);

#endif /* IMU_ICM42688_H */
========
(** Renvoie le nom de la plateforme : Unix ou Win32 *)
val platform_name : string

(** Teste si la plateforme courante est Unix *)
val platform_is_unix : bool

(** Teste si la plateforme courante est Windows (Win32) *)
val platform_is_win32 : bool
>>>>>>>> refs/remotes/origin/panache_mfeurgard:sw/lib/ocaml/platform.mli
