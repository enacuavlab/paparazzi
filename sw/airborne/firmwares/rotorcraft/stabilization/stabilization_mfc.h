/*
 * This control algorithm is Model Free Control (MFC)
 */

#ifndef STABILIZATION_MFC_H
#define STABILIZATION_MFC_H

#include <inttypes.h>
#include "std.h"
#include "paparazzi.h"
#include "generated/airframe.h"

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"

// commands: roll, pitch, yaw, and thrust
#define MFC_NUM_CMD 4

// mfc estimator: roll, pitch, and yaw
#define MFC_NUM_STATES 3

extern struct Int32Quat   stab_att_sp_quat;  ///< with #INT32_QUAT_FRAC
extern struct Int32Eulers stab_att_sp_euler; ///< with #INT32_ANGLE_FRAC

struct MfcParameters{
  int8_t id;
  float setpoint;
  float setpoint_trajec[3];
  uint8_t use_trajec_sp;
  float measure;
  float error[3];
  float command[2];
  float estimator_num[3];
  float estimator_den[3];
  float estimator;
  // Static values
  float time_trajec;
  float int_window;
  float command_filter;
  float alpha;
  float kp;
  float kd;
};

struct Mfc{
  float sample_time;
  float time;
  float start_time;
  bool in_hover;
  bool in_transition;
  bool in_forward;
};

extern struct Mfc mfc;
extern struct MfcParameters mfc_roll;
extern struct MfcParameters mfc_pitch;
extern struct MfcParameters mfc_yaw;
extern int16_t stabilization_mfc_cmd[MFC_NUM_CMD];

// RC setpoint variables
extern struct FloatQuat q_sp;
extern struct FloatEulers stab_rc_sp;

extern int16_t motor_left;   // only for tests
extern int16_t motor_right;  // only for tests
extern float counter_test; // only for tests
extern float vel_x_gs; // only for tests
extern float vel_y_gs; // only for tests
extern float vel_z_gs; // only for tests

// MFC functions
extern void stabilization_attitude_mfc_init(void);
extern void stabilization_attitude_mfc_enter(void);
extern void stabilization_attitude_mfc_run(bool in_flight);
extern void stabilization_attitude_mfc_mixing(void);
extern void stabilization_attitude_mfc_set_failsafe_setpoint(void);
extern void stabilization_attitude_mfc_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn);

#endif
