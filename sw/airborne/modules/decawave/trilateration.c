/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
/**
 * @file "modules/decawave/trilateration.c"
 * @author Gautier Hattenberger
 * Trilateration algorithm
 * https://en.wikipedia.org/wiki/Trilateration
 */

#include "trilateration.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_simple_matrix.h"


#include "generated/airframe.h"


#include "generated/uwb.c"



// base original locations
static float P[3][3];
// base unit vector on x, y and z axis
static float Ex[3], Ey[3], Ez[3];
// base inter distances
static float D, I, J;

bool init_failed;


float multilat_b[UWB_POSITIONING_NB_ANCHORS][1] = {0};
// float mtl_A [UWB_POSITIONING_NB_ANCHORS][4] = {0};
// float mtl_L [4][UWB_POSITIONING_NB_ANCHORS] = {0};

int trilateration_init(struct Anchor *anchors)
{
  float tmp[3], n;
  init_failed = false;

  // store original points
  for (int i = 0; i < 3; i++) {
    P[i][0] = anchors[i].pos.x;
    P[i][1] = anchors[i].pos.y;
    P[i][2] = anchors[i].pos.z;
  }
  // Ex = (P1 - P0) / ||P1 - P0||
  float_vect_diff(tmp, P[1], P[0], 3);
  n = float_vect_norm(tmp, 3);
  if (n > 0.f) {
    float_vect_sdiv(Ex, tmp, n, 3);
  } else {
    init_failed = true;
    return -1;
  }
  // I = Ex . (P2 - P0)
  float_vect_diff(tmp, P[2], P[0], 3);
  I = float_vect_dot_product(Ex, tmp, 3);
  // Ey = (P2 - P0 - I.Ex) / ||P2 - P0 - I.Ex||
  float_vect_smul(tmp, Ex, -I, 3);
  float_vect_diff(tmp, tmp, P[0], 3);
  float_vect_add(tmp, P[2], 3);
  n = float_vect_norm(tmp, 3);
  if (n > 0.f) {
    float_vect_sdiv(Ey, tmp, n, 3);
  } else {
    init_failed = true;
    return -1;
  }
  // Ez = Ex x Ey
  Ez[0] = Ex[1]*Ey[2] - Ex[2]*Ey[1];
  Ez[1] = Ex[2]*Ey[0] - Ex[0]*Ey[2];
  Ez[2] = Ex[0]*Ey[1] - Ex[1]*Ey[0];
  // D = ||P1 - P0||
  float_vect_diff(tmp, P[1], P[0], 3);
  D = float_vect_norm(tmp, 3);
  // J = Ey . (P2 - P0)
  float_vect_diff(tmp, P[2], P[0], 3);
  J = float_vect_dot_product(Ey, tmp, 3);

  return 0;
}

int trilateration_compute(struct Anchor *anchors, struct EnuCoor_f *pos)
{
  if (init_failed) {
    return -1;
  }
  const float r02 = anchors[0].distance * anchors[0].distance;
  const float r12 = anchors[1].distance * anchors[1].distance;
  const float r22 = anchors[2].distance * anchors[2].distance;
  const float d2 = D * D;
  const float i2 = I * I;
  const float j2 = J * J;
  float tmp[3];
  tmp[0] = (r02 - r12 + d2) / (2.f * D);
  tmp[1] = (r02 - r22 + i2 + j2) / (2.f * J) - ((I * tmp[0]) / J);
  const float d0 = r02 - (tmp[0] * tmp[0]) - (tmp[1] * tmp[1]);
  if (d0 < 0.f) {
    // impossible solution
    // might happen if position of the anchors are not correct
    // or if reported distances are completely wrong
    return -1;
  }
  tmp[2] = sqrtf(d0); // only keep positive value
  // restore original frame
  pos->x = P[0][0] + tmp[0] * Ex[0] + tmp[1] * Ey[0] + tmp[2] * Ez[0];
  pos->y = P[0][1] + tmp[0] * Ex[1] + tmp[1] * Ey[1] + tmp[2] * Ez[1];
  pos->z = P[0][2] + tmp[0] * Ex[2] + tmp[1] * Ey[2] + tmp[2] * Ez[2];
  pos->z = fabsf(pos->z); // in case the base is not matching, keep positive z
  return 0;
}

// void multilat_init() {
//   for(int i=0; i<UWB_POSITIONING_NB_ANCHORS; i++) {
//     mtl_A[i][0] = 1;
//     mtl_A[i][1] = -2*anchors[i].pos.x;
//     mtl_A[i][2] = -2*anchors[i].pos.y;
//     mtl_A[i][3] = -2*anchors[i].pos.z;
//   }
//   float mtl_At[4][UWB_POSITIONING_NB_ANCHORS] = {0};
//   float mtl_AtA [4][4] = {0};


  
//   float_mat_inv_4d(float invOut[4][4], const float mat_in[4][4]);
// }

int multilat_compute(struct Anchor *anchors, struct EnuCoor_f *pos) {
  for(int i=0; i<UWB_POSITIONING_NB_ANCHORS; i++) {
    multilat_b[i][0] = anchors[i].distance * anchors[i].distance - B_c[i][0];
  }

  float x[4][1] = {0};

  MAKE_MATRIX_PTR(_L, L, 4);
  MAKE_MATRIX_PTR(_b, multilat_b, UWB_POSITIONING_NB_ANCHORS);
  MAKE_MATRIX_PTR(_x, x, 4);

  // C = A*B   A:(i,k) B:(k,j) C:(i,j)
  //MAT_MUL(_i, _k, _j, C, A, B)
  MAT_MUL(4, UWB_POSITIONING_NB_ANCHORS, 1, _x, _L, _b);
  //MAT_MUL2(4, UWB_POSITIONING_NB_ANCHORS, 1, x, L, multilat_b);

  pos->x = x[1][0];
  pos->y = x[2][0];
  pos->z = x[3][0];

  return 0;

}