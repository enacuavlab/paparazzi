
#pragma once

#include "subsystems/datalink/telemetry_common.h"

// prototypes of all callbacks;
void send_gps(struct transport_tx *trans, struct link_device *dev);
void send_desired(struct transport_tx *trans, struct link_device *dev);



///////////////// BEGIN Ap process ////////////////////////


// Ap, default mode

// example with b=57600, th=gcd(Ti)=0.1s=5760 ticks, Th=lcm(Ti)=0.4s=23040 ticks, f=100Hz=> Tp=0.01s=576 ticks
#define PROCESS_PARAMS_Ap { \
  .hypertick = 5760, \
  .hyperperiod = 23040, \
  .Tp = 576, \
  .n = 5760/576, \
  .hyper_n = 23040/576, \
  .C = 0 \
}

#define PARAMS_Ap_defaut_GPS { \
  .msg_id = PPRZ_MSG_ID_GPS, \
  .class_id = DL_telemetry_CLASS_ID, \
  .cb = send_gps, \
  .Oh = 0, \
  .Oa = 0, \
  .Og = 0, \
  .Ts = 4  \
}


#define PARAMS_Ap_defaut_DESIRED  { \
  .msg_id = PPRZ_MSG_ID_DESIRED, \
  .class_id = DL_telemetry_CLASS_ID, \
  .cb = send_desired, \
  .Oh = 0, \
  .Oa = 0, \
  .Og = 0, \
  .Ts = 1  \
}

//struct message_params plop[] = {PARAMS_Ap_defaut_GPS, PARAMS_Ap_defaut_DESIRED};

#define MODE_default_Ap { \
  .nb_messages = 2, \
  .msgp = (struct message_params[]) {PARAMS_Ap_defaut_GPS, PARAMS_Ap_defaut_DESIRED} \
}

// Ap, minimal mode


#define PARAMS_Ap_minimal_GPS { \
  .msg_id = PPRZ_MSG_ID_GPS, \
  .class_id = DL_telemetry_CLASS_ID, \
  .cb = send_gps, \
  .Oh = 0, \
  .Oa = 0, \
  .Og = 0, \
  .Ts = 8  \
}


#define MODE_minimal_Ap { \
  .nb_messages = 1, \
  .msgp = (struct message_params[]) {PARAMS_Ap_minimal_GPS} \
}



struct telemetry_process process_Ap = {
  .process_id = 0,
  .modes = (struct telemetry_mode[]) {MODE_default_Ap, MODE_minimal_Ap},
  .current_mode = 0,
  .params = PROCESS_PARAMS_Ap
};

///////////////////// END Ap process ////////////////////////


// same thing again for all other processes

