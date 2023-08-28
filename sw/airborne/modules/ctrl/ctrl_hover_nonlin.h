#ifndef CTRL_HOVER_NONLIN_H
#define CTRL_HOVER_NONLIN_H

#include "std.h"
#include "math/pprz_algebra_float.h"
#include "state.h"

struct CtrlHoverNonlin {
  //Drone
  float m;
  struct FloatMat33 J;

  // setpoint
  struct FloatVect3 target;

  //Gain controlleur
  float kap;
  float kad;
  float kpp;
  float kpd;
  float kdelta;

  struct FloatVect3 d_etoile;

  //etat du controlleur
  float f;
  struct FloatQuat qd;

  //matrice de controle
  float Mk[4*3];
  float u_bare[4*1];

  struct FloatVect3 ep, ev, omega, f_r, f_delta, nu, omega_d, tau_r, omega_dotdot;
  float f_dot;
  struct FloatQuat q;
  float u[4*1];
  float u_scale[4];

  float rc_x;
  float rc_y;
  float rc_z;

  struct NedCoor_f pos_target;
  float kf;
  float mot_max_speed;
};

extern struct CtrlHoverNonlin ctrl_hover_nonlin;

extern void ctrl_hover_nonlin_init(void);
extern void ctrl_hover_nonlin_run(bool in_flight);

#endif /* CTRL_HOVER_NONLIN_H */
