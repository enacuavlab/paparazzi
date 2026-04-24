/*
 * Copyright (C) 2026  ENAC, Mael Feurgard
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
 * @file modules/nav/nav_extended_dubins.h
 *
 * Navigation module to follow Extended Dubins Paths
 * (Dubins Paths with added straights at start and end)
 */

#ifndef NAV_EXTENDED_DUBINS_H
#define NAV_EXTENDED_DUBINS_H

typedef enum DubinsType
{
  RSR = 0,
  LSL = 1,
  RSL = 2,
  LSR = 3,
  RLR = 4,
  LRL = 5,
  SLS = 6,
  SRS = 7,

  S_RSR = 8+0,
  S_LSL = 8+1,
  S_RSL = 8+2,
  S_LSR = 8+3,
  S_RLR = 8+4,
  S_LRL = 8+5,
  S_SLS = 8+6,
  S_SRS = 8+7,

  RSR_S = 2*8+0,
  LSL_S = 2*8+1,
  RSL_S = 2*8+2,
  LSR_S = 2*8+3,
  RLR_S = 2*8+4,
  LRL_S = 2*8+5,
  SLS_S = 2*8+6,
  SRS_S = 2*8+7,

  S_RSR_S = 3*8+0,
  S_LSL_S = 3*8+1,
  S_RSL_S = 3*8+2,
  S_LSR_S = 3*8+3,
  S_RLR_S = 3*8+4,
  S_LRL_S = 3*8+5,
  S_SLS_S = 3*8+6,
  S_SRS_S = 3*8+7,
};

bool NotExtendedDubins(DubinsType t);
bool StartExtendedDubins(DubinsType t);
bool EndExtendedDubins(DubinsType t);
bool BothExtendedDubins(DubinsType t);


#endif // NAV_EXTENDED_DUBINS_H