#include "modules/ctrl/ctrl_hover_nonlin.h"
#include "math/pprz_isa.h"
#include "math/pprz_algebra_float.h"
#include "std.h"
#include "paparazzi.h"
#include "state.h"
#include "mcu_periph/sys_time.h"
#include "modules/radio_control/radio_control.h"
#include "modules/actuators/actuators.h"

#include <stdio.h>

#include "generated/flight_plan.h"
#include "generated/airframe.h"

#define MAX(a,b) ((a) > (b) ? (a) : (b))

#define POS_INC 0.005



#ifndef CTRL_HOVER_NONLIN_DRONE_MASS
#define CTRL_HOVER_NONLIN_DRONE_MASS 0.519
#endif

#ifndef CTRL_HOVER_NONLIN_DRONE_J1
#define CTRL_HOVER_NONLIN_DRONE_J1 0.007249858398151
#endif

#ifndef CTRL_HOVER_NONLIN_DRONE_J2
#define CTRL_HOVER_NONLIN_DRONE_J2 0.000407672840112
#endif
#ifndef CTRL_HOVER_NONLIN_DRONE_J3
#define CTRL_HOVER_NONLIN_DRONE_J3 0.008567110783194
#endif

#ifndef CTRL_HOVER_NONLIN_KAP
#define CTRL_HOVER_NONLIN_KAP 0.08f
#endif

#ifndef CTRL_HOVER_NONLIN_KAD
#define CTRL_HOVER_NONLIN_KAD 0.1f
#endif

#ifndef CTRL_HOVER_NONLIN_KPP
#define CTRL_HOVER_NONLIN_KPP 0.5f
#endif

#ifndef CTRL_HOVER_NONLIN_KPD
#define CTRL_HOVER_NONLIN_KPD 1.2f
#endif

#ifndef CTRL_HOVER_NONLIN_KDELTA
#define CTRL_HOVER_NONLIN_KDELTA 1.f
#endif

static const float ctrl_dt = (1.f / PERIODIC_FREQUENCY);

#define MAT33_VECT3_SKEW(_c, _a) {  \
    (_c).m[0] = 0;          \
    (_c).m[1] = -(_a).z;    \
    (_c).m[2] = (_a).y;     \
    (_c).m[3] = (_a).z;     \
    (_c).m[4] = 0;          \
    (_c).m[5] = -(_a).x;    \
    (_c).m[6] = -(_a).y;    \
    (_c).m[7] = (_a).x;     \
    (_c).m[8] = 0;          \
}

static const float Mk[] = { 0, 0,  3.458982807987897,
                            0, 0, -3.458982807987897,
                            1.350214971288870,  -13.178271493492982, 0.104845049184770,
                            -1.350214971288870, -13.178271493492982, -0.104845049184770
  };

static const float u_bare[] = {0.565621212259045, 0.565621212259045, 0, 0};



static void p_v_stab(struct FloatVect3 *fr, struct FloatVect3 ep, struct FloatVect3 ev);
static void trans_mis(struct FloatVect3 *f_delta, struct FloatVect3 fr, float f, struct FloatQuat qd); //
static void f_delta_stab(struct FloatVect3 *nu, struct FloatVect3 f_delta,  struct FloatVect3 ep, struct FloatVect3 ev);
static void dstar_split(float *f_dot, struct FloatVect3 *omega_d, struct FloatVect3 nu, float f, struct FloatQuat qd);//
static void controller_state(float *f, struct FloatQuat *qd, float f_dot, struct FloatVect3 omega_d); //
static void q_o_stab(struct FloatVect3 *tau_r, struct FloatQuat qd, struct FloatQuat q, struct FloatVect3 omega, struct FloatVect3 omega_d, struct FloatVect3 omega_dotdot);
static void distribution(float *u, struct FloatVect3 tau_r, float f); //
static void feedforward(struct FloatVect3 *omega_dotdot, struct FloatVect3 f_delta, struct FloatVect3 ep, struct FloatVect3 ev, struct FloatQuat q, struct FloatQuat qd, struct FloatVect3 omega, struct FloatVect3 omega_d, struct FloatVect3 nu, float f); //

struct CtrlHoverNonlin ctrl_hover_nonlin;






void ctrl_hover_nonlin_init(void)
{
  ctrl_hover_nonlin.m = CTRL_HOVER_NONLIN_DRONE_MASS;

  FLOAT_MAT33_ZERO(ctrl_hover_nonlin.J);
  MAT33_ELMT(ctrl_hover_nonlin.J, 0, 0) = CTRL_HOVER_NONLIN_DRONE_J1;
  MAT33_ELMT(ctrl_hover_nonlin.J, 1, 1) = CTRL_HOVER_NONLIN_DRONE_J2;
  MAT33_ELMT(ctrl_hover_nonlin.J, 2, 2) = CTRL_HOVER_NONLIN_DRONE_J3;

  ctrl_hover_nonlin.kap = CTRL_HOVER_NONLIN_KAP;
  ctrl_hover_nonlin.kad = CTRL_HOVER_NONLIN_KAD;
  ctrl_hover_nonlin.kpp = CTRL_HOVER_NONLIN_KPP;
  ctrl_hover_nonlin.kpd = CTRL_HOVER_NONLIN_KPD;

  ctrl_hover_nonlin.kdelta = CTRL_HOVER_NONLIN_KDELTA;

  VECT3_ASSIGN(ctrl_hover_nonlin.d_etoile, 1, 0, 0);

  //intialisation des etats du ctrl_hover_nonlin
  ctrl_hover_nonlin.f = CTRL_HOVER_NONLIN_DRONE_MASS * PPRZ_ISA_GRAVITY;
  QUAT_ASSIGN(ctrl_hover_nonlin.qd, 1.f/sqrtf(2.f), 0, -1.f/sqrtf(2.f), 0);

  memcpy(ctrl_hover_nonlin.Mk, Mk, sizeof(Mk));
//  ctrl_hover_nonlin.Mk = {0, 0, -3.22580645161290
//    -0.880620907001274, -27.7471520515246, -0.292377437525412
//      0, 0, 3.22580645161290
//      0.880620907001274, -27.7471520515246, 0.292377437525425
//  };
  memcpy(ctrl_hover_nonlin.u_bare, u_bare, sizeof(u_bare));

  //initialisation des variables du ctrl_hover_nonlin

  ctrl_hover_nonlin.f_dot = 0;
  FLOAT_VECT3_ZERO(ctrl_hover_nonlin.f_r);
  FLOAT_VECT3_ZERO(ctrl_hover_nonlin.f_delta);
  FLOAT_VECT3_ZERO(ctrl_hover_nonlin.nu);
  FLOAT_VECT3_ZERO(ctrl_hover_nonlin.omega_d);
  FLOAT_VECT3_ZERO(ctrl_hover_nonlin.tau_r);
  FLOAT_VECT3_ZERO(ctrl_hover_nonlin.omega_dotdot);
  FLOAT_VECT3_ZERO(ctrl_hover_nonlin.omega);
  float_quat_identity(&ctrl_hover_nonlin.q);
  float_vect_zero(ctrl_hover_nonlin.u, 4);

  ctrl_hover_nonlin.pos_target = *stateGetPositionNed_f(); 

  ctrl_hover_nonlin.kf = 0.000000018767000;
  ctrl_hover_nonlin.mot_max_speed = 16066.000000000000000;
}


void ctrl_hover_nonlin_run(bool in_flight)
{
  /* A transformer pour recuperer la position, la vitesse, l'orientation et la vitesse angulaire du drone
     pour calculer ep et ev en fonction de la consigne.
     */

  ctrl_hover_nonlin.rc_x = -(radio_control.values[RADIO_PITCH]/9600.0); //[-1, 1]
  ctrl_hover_nonlin.rc_y = (radio_control.values[RADIO_ROLL]/9600.0); //[-1, 1]
  ctrl_hover_nonlin.rc_z = (radio_control.values[RADIO_THROTTLE]/9600.0); 

 
  if(fabs(ctrl_hover_nonlin.rc_x) >= 0.5){
    ctrl_hover_nonlin.pos_target.x += ctrl_hover_nonlin.rc_x*POS_INC;
  }
  
  if(fabs(ctrl_hover_nonlin.rc_y) >= 0.5){
     ctrl_hover_nonlin.pos_target.y += ctrl_hover_nonlin.rc_y*POS_INC;
  }

  //VECT3_DIFF(ctrl_hover_nonlin.ep, *stateGetPositionNed_f(), pos_cible);
  VECT3_ASSIGN(ctrl_hover_nonlin.ep, stateGetPositionNed_f()->x - ctrl_hover_nonlin.pos_target.x, stateGetPositionNed_f()->y - ctrl_hover_nonlin.pos_target.y, stateGetPositionNed_f()->z - ctrl_hover_nonlin.pos_target.z);
  //VECT3_COPY(ctrl_hover_nonlin.ev, *stateGetSpeedNed_f());
  VECT3_ASSIGN(ctrl_hover_nonlin.ev, stateGetSpeedNed_f()->x, stateGetSpeedNed_f()->y, stateGetSpeedNed_f()->z);

  ctrl_hover_nonlin.q = *stateGetNedToBodyQuat_f();
  VECT3_ASSIGN(ctrl_hover_nonlin.omega, stateGetBodyRates_f()->p, stateGetBodyRates_f()->q, stateGetBodyRates_f()->r);


  p_v_stab(&ctrl_hover_nonlin.f_r, ctrl_hover_nonlin.ep, ctrl_hover_nonlin.ev);
  trans_mis(&ctrl_hover_nonlin.f_delta, ctrl_hover_nonlin.f_r, ctrl_hover_nonlin.f, ctrl_hover_nonlin.qd);
  f_delta_stab(&ctrl_hover_nonlin.nu, ctrl_hover_nonlin.f_delta, ctrl_hover_nonlin.ep, ctrl_hover_nonlin.ev);
  dstar_split(&ctrl_hover_nonlin.f_dot, &ctrl_hover_nonlin.omega_d, ctrl_hover_nonlin.nu, ctrl_hover_nonlin.f, ctrl_hover_nonlin.qd);
  controller_state(&ctrl_hover_nonlin.f, &ctrl_hover_nonlin.qd, ctrl_hover_nonlin.f_dot, ctrl_hover_nonlin.omega_d);
  feedforward(&ctrl_hover_nonlin.omega_dotdot, ctrl_hover_nonlin.f_delta, ctrl_hover_nonlin.ep, ctrl_hover_nonlin.ev, ctrl_hover_nonlin.q, ctrl_hover_nonlin.qd, ctrl_hover_nonlin.omega, ctrl_hover_nonlin.omega_d, ctrl_hover_nonlin.nu, ctrl_hover_nonlin.f);
  q_o_stab(&ctrl_hover_nonlin.tau_r, ctrl_hover_nonlin.qd, ctrl_hover_nonlin.q, ctrl_hover_nonlin.omega, ctrl_hover_nonlin.omega_d, ctrl_hover_nonlin.omega_dotdot);
  distribution(ctrl_hover_nonlin.u, ctrl_hover_nonlin.tau_r, ctrl_hover_nonlin.f);

   // LEFT_MOTOR 
  if (ctrl_hover_nonlin.u[0]>0){
    ctrl_hover_nonlin.u_scale[0] = (sqrtf(MAX(ctrl_hover_nonlin.u[0],0)/ctrl_hover_nonlin.kf) / ctrl_hover_nonlin.mot_max_speed)*MAX_PPRZ; 
  }
  else{
    ctrl_hover_nonlin.u_scale[0] = 0; 
  }

  // RIGHT_MOTOR 
  if (ctrl_hover_nonlin.u[1]>0){
    ctrl_hover_nonlin.u_scale[1] = (sqrtf(MAX(ctrl_hover_nonlin.u[1],0)/ctrl_hover_nonlin.kf) / ctrl_hover_nonlin.mot_max_speed)*MAX_PPRZ; 
  }
  else{
    ctrl_hover_nonlin.u_scale[1] = 0; 
  }

 //ELEVON_LEFT  
  ctrl_hover_nonlin.u_scale[2] = (ctrl_hover_nonlin.u[2]*6/M_PI)*MAX_PPRZ;

  // ELEVON_RIGHT  
  ctrl_hover_nonlin.u_scale[3] = (ctrl_hover_nonlin.u[3]*6/M_PI)*MAX_PPRZ;

   // actuators_pprz[0] -> ELEVON_LEFT, actuators_pprz[1] -> ELEVON_RIGHT,  actuators_pprz[2] -> RIGHT_MOTOR,  actuators_pprz[3] -> LEFT_MOTOR
  actuators_pprz[0]=TRIM_PPRZ(ctrl_hover_nonlin.u_scale[2]); 
  actuators_pprz[1]=TRIM_PPRZ(-ctrl_hover_nonlin.u_scale[3]);  


  actuators_pprz[2]=TRIM_UPPRZ(ctrl_hover_nonlin.u_scale[1]);
  actuators_pprz[3]=TRIM_UPPRZ(ctrl_hover_nonlin.u_scale[0]); 

  
  
  if (in_flight) {
    stabilization_cmd[COMMAND_THRUST] = (actuators_pprz[2]+actuators_pprz[3]); // for in_flight detection
    //printf("in_flight\n");
  } else {
    stabilization_cmd[COMMAND_THRUST] = 1000;
    //printf("Not in_flight\n");
  };
  //
}

static void p_v_stab(struct FloatVect3 *fr, struct FloatVect3 ep, struct FloatVect3 ev){
  //erreur de stabilisation
  //f_r = drone.MASS * [0;0;drone.G] - ctrl_hover_nonlin.kpp * ep - ctrl_hover_nonlin.kpd * ev;

  struct FloatVect3 mg = { 0.f, 0.f,  ctrl_hover_nonlin.m * PPRZ_ISA_GRAVITY};
  struct FloatVect3 ep_prod, ev_prod;

  VECT3_SMUL(ep_prod, ep, -ctrl_hover_nonlin.kpp);
  VECT3_SMUL(ev_prod, ev, -ctrl_hover_nonlin.kpd);

  VECT3_ADD(*fr, mg);
  VECT3_ADD(*fr, ep_prod);
  VECT3_ADD(*fr, ev_prod);
}

static void trans_mis(struct FloatVect3 *f_delta, struct FloatVect3 fr, float f, struct FloatQuat qd){
  //transitionnal mismatch
  //f_delta = q2dcm(q_d) * ctrl_hover_nonlin.d_etoile * f - f_r;

  struct FloatRMat R_qd;
  float_rmat_of_quat(&R_qd, &qd);

  struct FloatVect3 _d_star_f, prod_R_d_f;
  VECT3_SMUL(_d_star_f, ctrl_hover_nonlin.d_etoile, f);
  RMAT_VECT3_MUL(prod_R_d_f, R_qd, _d_star_f);

  VECT3_DIFF(*f_delta, prod_R_d_f, fr);
}


static void f_delta_stab(struct FloatVect3 *nu, struct FloatVect3 f_delta, struct FloatVect3 ep, struct FloatVect3 ev){

  float gain_ep = ctrl_hover_nonlin.kpd * ctrl_hover_nonlin.kpp/ctrl_hover_nonlin.m;
  float gain_ev = ctrl_hover_nonlin.kpd*ctrl_hover_nonlin.kpd/ctrl_hover_nonlin.m - ctrl_hover_nonlin.kpp;
  float gain_fdelta = -ctrl_hover_nonlin.kpd/ctrl_hover_nonlin.m + ctrl_hover_nonlin.kdelta;

  struct FloatVect3 ep_gain_prod, ev_gain_prod, fdelta_gain_prod;
  VECT3_SMUL(ep_gain_prod, ep, gain_ep);
  VECT3_SMUL(ev_gain_prod, ev, gain_ev);
  VECT3_SMUL(fdelta_gain_prod, f_delta, gain_fdelta);

  VECT3_ADD(*nu, ep_gain_prod);
  VECT3_ADD(*nu, ev_gain_prod);
  VECT3_ADD(*nu, fdelta_gain_prod);
}

static void dstar_split(float *f_dot, struct FloatVect3 *omega_d, struct FloatVect3 nu, float f, struct FloatQuat qd){

  //f_dot

  struct FloatRMat R_qd;

  float_rmat_of_quat(&R_qd, &qd);

  struct FloatVect3 _matrot_d_star;
  RMAT_VECT3_MUL(_matrot_d_star, R_qd, ctrl_hover_nonlin.d_etoile);

  *f_dot = VECT3_DOT_PRODUCT(_matrot_d_star, nu);

  //omega_d
  struct FloatVect3 mat_prim_nu;
  struct FloatMat33 d_etoile_skew_sym;

  RMAT_VECT3_TRANSP_MUL(mat_prim_nu, R_qd, nu);
  MAT33_VECT3_SKEW(d_etoile_skew_sym, ctrl_hover_nonlin.d_etoile);
  MAT33_VECT3_MUL(mat_prim_nu, d_etoile_skew_sym, mat_prim_nu);
  VECT3_SMUL(*omega_d, mat_prim_nu, 1/f);

}

static void controller_state(float *f, struct FloatQuat *qd, float f_dot, struct FloatVect3 omega_d){

  *f += f_dot * ctrl_dt;

  float qi, qx, qy, qz;
  qi = -0.5 *  (qd->qx * omega_d.x + qd->qy * omega_d.y + qd->qz * omega_d.z) * ctrl_dt;
  qx = 0.5 * (qd->qi* omega_d.x - qd->qz * omega_d.y + qd->qy * omega_d.z) * ctrl_dt;
  qy = 0.5 * (qd->qz* omega_d.x + qd->qi * omega_d.y - qd->qx * omega_d.z) * ctrl_dt;
  qz = 0.5 * (-qd->qy* omega_d.x + qd->qx * omega_d.y + qd->qi * omega_d.z) * ctrl_dt;

  QUAT_ASSIGN(*qd, qd->qi + qi, qd->qx + qx, qd->qy + qy, qd->qz + qz);

  // struct FloatRates omega = { omega_d.x; omega_d.y; omega_d.z };
  // float_quat_integrate_fi(qd, &omega, ctrl_dt); // TODO check if the same as above
}

static void q_o_stab(struct FloatVect3 *tau_r, struct FloatQuat qd, struct FloatQuat q, struct FloatVect3 omega, struct FloatVect3 omega_d, struct FloatVect3 omega_dotdot){

  struct FloatVect3 epsilon_delta, omega_delta, Jomega, omega_J_omega, J_omegadd;

  //-kap * epsilon_delta
  //VECT3_ASSIGN(epsilon_delta, -q.qi * qd.qx + qd.qi * q.qx + qd.qz * q.qy - qd.qy * q.qz, -q.qi * qd.qy + qd.qi * q.qy - qd.qz * q.qx + qd.qx  q.qz, -q.qi * qd.qz + qd.qi * q.qz + qd.qz * q.qx - qd.qx * q.qy);
  struct FloatQuat tmp;
  float_quat_inv_comp(&tmp, &qd, &q);
  VECT3_ASSIGN(epsilon_delta, tmp.qx, tmp.qy, tmp.qz); // TODO check
  VECT3_SMUL(epsilon_delta, epsilon_delta, -ctrl_hover_nonlin.kap);

  //-kad * omega_delta
  VECT3_DIFF(omega_delta, omega, omega_d);
  VECT3_SMUL(omega_delta, omega_delta, -ctrl_hover_nonlin.kad);

  //omega x J*omega
  MAT33_VECT3_MUL(Jomega, ctrl_hover_nonlin.J, omega);
  VECT3_CROSS_PRODUCT(omega_J_omega, omega, Jomega);

  //J*omega_dotdot
  MAT33_VECT3_MUL(J_omegadd, ctrl_hover_nonlin.J, omega_dotdot);

  //tau_r
  VECT3_ADD(*tau_r, epsilon_delta);
  VECT3_ADD(*tau_r, omega_delta);
  VECT3_ADD(*tau_r, omega_J_omega);
  VECT3_ADD(*tau_r, J_omegadd);
}

static void distribution(float *u, struct FloatVect3 tau_r, float f){
  u[0] =  ctrl_hover_nonlin.Mk[0] * tau_r.x + ctrl_hover_nonlin.Mk[1] * tau_r.y + ctrl_hover_nonlin.Mk[2] * tau_r.z + ctrl_hover_nonlin.u_bare[0] * f;
  u[1] =  ctrl_hover_nonlin.Mk[3] * tau_r.x + ctrl_hover_nonlin.Mk[4] * tau_r.y + ctrl_hover_nonlin.Mk[5] * tau_r.z + ctrl_hover_nonlin.u_bare[1] * f;
  u[2] =  ctrl_hover_nonlin.Mk[6] * tau_r.x + ctrl_hover_nonlin.Mk[7] * tau_r.y + ctrl_hover_nonlin.Mk[8] * tau_r.z + ctrl_hover_nonlin.u_bare[2] * f;
  u[3] =  ctrl_hover_nonlin.Mk[9] * tau_r.x + ctrl_hover_nonlin.Mk[10] * tau_r.y + ctrl_hover_nonlin.Mk[11] * tau_r.z + ctrl_hover_nonlin.u_bare[3] * f;
}

static void feedforward(struct FloatVect3 *omega_dotdot, struct FloatVect3 f_delta, struct FloatVect3 ep, struct FloatVect3 ev, struct FloatQuat q, struct FloatQuat qd, struct FloatVect3 omega, struct FloatVect3 omega_d, struct FloatVect3 nu, float f){
  struct FloatRMat R_qd, R_q;

  float_rmat_of_quat(&R_qd, &qd);
  float_rmat_of_quat(&R_q, &q);

  struct FloatVect3 mat_prim_nu, omega1_dot, omega2_dot, omega3_dot;
  RMAT_VECT3_TRANSP_MUL(mat_prim_nu, R_qd, nu);

  struct FloatMat33 d_etoile_skew_sym;

  MAT33_VECT3_SKEW(d_etoile_skew_sym, ctrl_hover_nonlin.d_etoile);
  MAT33_VECT3_MUL(omega1_dot, d_etoile_skew_sym, mat_prim_nu);
  VECT3_SMUL(omega1_dot, omega1_dot, VECT3_DOT_PRODUCT(ctrl_hover_nonlin.d_etoile, mat_prim_nu));
  VECT3_SMUL(omega1_dot, omega1_dot, -1/(pow(f,2)));

  VECT3_COPY(omega2_dot, omega1_dot);

  float gain = (powf(ctrl_hover_nonlin.kpd, 2) /powf(ctrl_hover_nonlin.m,2) - ctrl_hover_nonlin.kpp/ctrl_hover_nonlin.m);

  struct FloatVect3 ep_term, ev_term, f_delta_term, f_term, nu_term;

  VECT3_SMUL(ep_term, ep, -gain* ctrl_hover_nonlin.kpp); //ep_term = - Gain*kpp*ep;
  VECT3_SMUL(ev_term, ev, ctrl_hover_nonlin.kpp*ctrl_hover_nonlin.kpd/ctrl_hover_nonlin.m - gain * ctrl_hover_nonlin.kpp); //ev_term = (kpp*kpd/m - Gain*kpd)*ev;
  VECT3_SMUL(f_delta_term, f_delta, gain + (ctrl_hover_nonlin.kpd/ctrl_hover_nonlin.m + ctrl_hover_nonlin.kdelta)*ctrl_hover_nonlin.kdelta);//f_delta_term = (Gain + (kpd/m + k_delta)*k_delta)*f_delta;

  struct FloatMat33 omegad_skew_sym, omega_skew_sym, diff_matrot;
  struct FloatVect3 d_star_f, term1, term2, term3;
  VECT3_SMUL(d_star_f, ctrl_hover_nonlin.d_etoile, f);

  MAT33_VECT3_SKEW(omegad_skew_sym, omega_d);
  MAT33_VECT3_SKEW(omega_skew_sym, omega);

  //Gain*(R-Rd)*d_etoile* f
  MAT33_MAT33_DIFF(diff_matrot, R_q, R_qd);
  MAT33_VECT3_MUL(term1, diff_matrot, d_star_f);
  VECT3_SMUL(term1, term1, gain);

  // kpd/m * Rd*skew_sym(omega_d)*d_etoile* f
  MAT33_VECT3_MUL(term2, omegad_skew_sym, d_star_f);
  MAT33_VECT3_MUL(term2, R_qd, term2);
  VECT3_SMUL(term2, term2, ctrl_hover_nonlin.kpd/ctrl_hover_nonlin.m);

  //- kpd/m *R*skew_sym(omega)*d_etoile* f
  MAT33_VECT3_MUL(term3, omega_skew_sym, d_star_f);
  MAT33_VECT3_MUL(term3, R_q, term2);
  VECT3_SMUL(term3, term3, -ctrl_hover_nonlin.kpd/ctrl_hover_nonlin.m);

  //f_term = Gain*(R-Rd)*d_etoile* f + kpd/m * Rd*skew_sym(omega_d)*d_etoile* f - kpd/m *R*skew_sym(omega)*d_etoile* f;
  FLOAT_VECT3_ZERO(f_term);
  VECT3_ADD(f_term, term1);
  VECT3_ADD(f_term, term2);
  VECT3_ADD(f_term, term3);

  //nu_term = kpd/m * (Rd-R) * d_etoile * (Rd*d_etoile)'* nu;
  struct FloatMat33 Rd_R_diff;
  struct FloatVect3 R_dstar;

  MAT33_VECT3_MUL(R_dstar, R_qd, ctrl_hover_nonlin.d_etoile);

  MAT33_MAT33_DIFF(Rd_R_diff, R_qd, R_q);
  MAT33_VECT3_MUL(nu_term, Rd_R_diff, ctrl_hover_nonlin.d_etoile);
  VECT3_SMUL(nu_term, nu_term, VECT3_DOT_PRODUCT(R_dstar, nu));
  VECT3_SMUL(nu_term, nu_term, ctrl_hover_nonlin.kpd/ctrl_hover_nonlin.m);

  //omega_d3_p = (1/f) * skew_sym(d_etoile) * Rd'* (ep_term + ev_term + f_delta_term + f_term + nu_term);
  struct FloatVect3 somme_term;
  FLOAT_VECT3_ZERO(somme_term);
  VECT3_ADD(somme_term, ep_term);
  VECT3_ADD(somme_term, ev_term);
  VECT3_ADD(somme_term, f_delta_term);
  VECT3_ADD(somme_term, f_term);
  VECT3_ADD(somme_term, nu_term);

  MAT33_VECT3_TRANSP_MUL(omega3_dot, R_qd, somme_term);
  MAT33_VECT3_MUL(omega3_dot, d_etoile_skew_sym, omega3_dot);
  VECT3_SMUL(omega3_dot, omega3_dot, 1/f);

  VECT3_ADD(*omega_dotdot, omega1_dot);
  VECT3_ADD(*omega_dotdot, omega2_dot);
  VECT3_ADD(*omega_dotdot, omega3_dot);
}

