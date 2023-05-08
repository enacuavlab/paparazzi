/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: colibri_v1.h
 *
 * Code generated for Simulink model 'colibri_v1'.
 *
 * Model version                  : 4.32
 * Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
 * C/C++ source code generated on : Thu May  4 13:41:43 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_colibri_v1_h_
#define RTW_HEADER_colibri_v1_h_
#ifndef colibri_v1_COMMON_INCLUDES_
#define colibri_v1_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* colibri_v1_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

#ifndef DEFINED_TYPEDEF_FOR_struct_s2gCMPZDg0ZN3PbxqDfEwH_
#define DEFINED_TYPEDEF_FOR_struct_s2gCMPZDg0ZN3PbxqDfEwH_

typedef struct {
  real_T G;
  real_T RHO;
  real_T WING_MASS;
  real_T WING_INERTIA[9];
  real_T P_UR_CG[3];
  real_T P_BR_CG[3];
  real_T P_BL_CG[3];
  real_T P_UL_CG[3];
  real_T INERTIA_PROP_X;
  real_T INERTIA_PROP_N;
  real_T PROP_RADIUS;
  real_T PROP_KP;
  real_T PROP_KM;
  real_T MOTOR_SPEED_MAX;
  real_T MOTOR_RATE_CHANGE;
  real_T MOTOR_ANGLE;
  real_T WING_SURFACE;
  real_T CHORD;
  real_T WINGSPAN;
  real_T THICKNESS;
  real_T ELEVON_MEFFICIENCY;
  real_T ELEVON_FEFFICIENCY;
  real_T ELEVON_ANGLE_MAX;
  real_T ELEVON_RATE_CHANGE;
  real_T WING_ROT_POS[3];
  real_T PEND_PROP_KP;
  real_T PEND_MOTOR_SPEED_MAX;
  real_T PEND_MASS;
  real_T PEND_INERTIA[9];
  real_T PEND_MOT_POS[3];
  real_T PEND_ELEVATOR_POS[3];
  real_T PEND_ROT_POS[3];
} struct_s2gCMPZDg0ZN3PbxqDfEwH;

#endif

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T DiscreteTimeIntegrator_DSTATE[28];/* '<Root>/Discrete-Time Integrator' */
  real_T PrevY;                        /* '<S3>/Rate Limiter2' */
  real_T PrevY_a;                      /* '<S3>/Rate Limiter3' */
  real_T PrevY_ah;                     /* '<S3>/Rate Limiter4' */
  real_T PrevY_h;                      /* '<S3>/Rate Limiter5' */
  real_T PrevY_as;                     /* '<S3>/Rate Limiter' */
  real_T PrevY_l;                      /* '<S3>/Rate Limiter1' */
  real_T PrevY_j;                      /* '<S3>/Rate Limiter7' */
  real_T PrevY_d;                      /* '<S3>/Rate Limiter6' */
} DW;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: initial_state
   * Referenced by: '<Root>/Discrete-Time Integrator'
   */
  real_T DiscreteTimeIntegrator_IC[28];
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T u[8];                         /* '<Root>/u' */
  real_T w[3];                         /* '<Root>/w' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T p[3];                         /* '<Root>/p' */
  real_T v[3];                         /* '<Root>/v' */
  real_T q[4];                         /* '<Root>/q' */
  real_T omega[3];                     /* '<Root>/omega' */
  real_T accel[3];                     /* '<Root>/accel' */
  real_T rotaccel[3];                  /* '<Root>/rotaccel' */
  real_T pendulum_angle;               /* '<Root>/pendulum_angle' */
  real_T pendulum_ang_speed;           /* '<Root>/pendulum_ang_speed' */
  real_T pendulum_ang_acc;             /* '<Root>/pendulum_ang_acc' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void colibri_v1_initialize(void);
extern void colibri_v1_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'colibri_v1'
 * '<S1>'   : 'colibri_v1/Drone'
 * '<S2>'   : 'colibri_v1/MATLAB Function1'
 * '<S3>'   : 'colibri_v1/Saturation'
 * '<S4>'   : 'colibri_v1/state'
 */
#endif                                 /* RTW_HEADER_colibri_v1_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
