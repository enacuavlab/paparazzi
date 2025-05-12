#include <stdlib.h>
#include "micolink_mtf01p.h"
#include "mcu_periph/uart.h"
#include "modules/core/abi.h"

// Messages
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

#ifndef USE_MICOLINK_MTF01FD_AGL
#define USE_MICOLINK_MTF01FD_AGL 1
#endif

#ifndef USE_MICOLINK_MTF01FD_OPTICAL_FLOW
#define USE_MICOLINK_MTF01FD_OPTICAL_FLOW 1
#endif

#define MICOLINK_MTF01FD_FLOW_RATE_HZ 100 

// Max flow : 7 rad/s
//   1 x 7 =  7 m/s @ 1 m
//   12 x 7 = 84 m/s @ 12 m
//
#define MICOLINK_MTF01FD_MAX_FLOW 7
#define MICOLINK_MTF01FD_MAX_FLOW_CM 700

// TOF Range: 12m
#define MICOLINK_MTF01FD_MAX_DISTANCE 12000000



#define MICOLINK_MTF01P_MSG_HEAD            0xEF
#define MICOLINK_MTF01P_MAX_PAYLOAD_LEN     64
#define MICOLINK_MTF01P_MAX_LEN             MICOLINK_MTF01P_MAX_PAYLOAD_LEN + 7

enum MicolinkMTF01PParseStatus {
  MICOLINK_MTF01P_INITIALIZE,
  MICOLINK_MTF01P_PARSE_HEAD,
  MICOLINK_MTF01P_PARSE_DEV,
  MICOLINK_MTF01P_PARSE_SYS,
  MICOLINK_MTF01P_PARSE_MSG,
  MICOLINK_MTF01P_PARSE_SEQ,
  MICOLINK_MTF01P_PARSE_PAYLEN,
  MICOLINK_MTF01P_PARSE_PAYLOAD,
  MICOLINK_MTF01P_PARSE_CHECKSUM
};


struct micolinkmtf01p_msg_t {
  uint8_t head;
  uint8_t dev_id;
  uint8_t sys_id;
  uint8_t msg_id;
  uint8_t seq;
  uint8_t paylen;
  uint8_t payload[MICOLINK_MTF01P_MAX_PAYLOAD_LEN];
  uint8_t checksum;

  uint8_t status;                           
  uint8_t payidx;

} micolinkmtf01p_msg = {
  .status = MICOLINK_MTF01P_INITIALIZE
};


struct micolinkmtf01p_pay_t {
  uint32_t time_ms;	 // System time in ms
  uint32_t distance;     // distance(mm), 0 Indicates unavailable
  uint8_t  strength;     // signal strength
  uint8_t  precision;    // distance precision
  uint8_t  dis_status;   // distance status
  uint8_t  reserved1;    // reserved
  int16_t  flow_vel_x;   // optical flow velocity in x, cm/s @ 1m
  int16_t  flow_vel_y;	 // optical flow velocity in y, cm/s @ 1m
  uint8_t  flow_quality; // optical flow quality, 0-255, bigger = higher quality
  uint8_t  flow_status;	 // optical flow status
  uint16_t reserved2;	 // reserved
} micolinkmtf01p_pay;


struct opticflow_t { // Extract of micolinkmtf01p_pay_t
  uint32_t time_usec;	
  uint32_t distance;             
  uint32_t distance_compensated; // adding computed value 
  int16_t flow_x;
  int16_t flow_y;
  uint8_t quality;
} opticflow; 


struct link_device *micolinkmtf01p_device;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
/******************************************************************************/
static void send_optical_flow(struct transport_tx *trans, struct link_device *dev) {
  float time_sec=0;
  uint8_t sensor_id=12;
  int32_t flow_x=0;
  int32_t flow_y=0;
  float flow_comp_m_x=0;
  float flow_comp_m_y=0;
  uint8_t flow_quality=0;
  float ground_distance=0;
  float distance_compensated=0;
  uint8_t distance_quality=0;

  time_sec = (float)opticflow.time_usec / 1e6f;
  flow_x = (int32_t)opticflow.flow_x;
  flow_y = (int32_t)opticflow.flow_y;
  flow_quality = opticflow.quality;

  ground_distance = (float)(opticflow.distance) * 0.001f;
  distance_compensated = (float)(opticflow.distance_compensated);

  pprz_msg_send_OPTICAL_FLOW(trans, dev, AC_ID,
                              &time_sec,
                              &sensor_id,
                              &flow_x,
                              &flow_y,
                              &flow_comp_m_x,        // m
                              &flow_comp_m_y,        // m 
                              &flow_quality, 
                              &ground_distance,      // m
                              &distance_compensated, // mm
                              &distance_quality);
}
#endif

/******************************************************************************/
void micolinkmtf01p_init(void) {

  memset(&micolinkmtf01p_pay,0,sizeof(struct micolinkmtf01p_pay_t));
  memset(&micolinkmtf01p_msg,0,sizeof(struct micolinkmtf01p_msg_t));
  micolinkmtf01p_msg.status = MICOLINK_MTF01P_PARSE_HEAD;

  micolinkmtf01p_device = &((MICOLINK_MFT01P_PORT).device);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OPTICAL_FLOW, send_optical_flow);
#endif
}

/******************************************************************************/
static void micolinkmtf01p_parse(uint8_t byte) {
  switch (micolinkmtf01p_msg.status) {
    
    case MICOLINK_MTF01P_INITIALIZE:
      break;

    case MICOLINK_MTF01P_PARSE_HEAD:
      if (byte == MICOLINK_MTF01P_MSG_HEAD) {
        micolinkmtf01p_msg.head = byte;
        micolinkmtf01p_msg.status++;
      }
      break;

    case MICOLINK_MTF01P_PARSE_DEV:
      micolinkmtf01p_msg.dev_id = byte;
      micolinkmtf01p_msg.status++;
      break;

    case MICOLINK_MTF01P_PARSE_SYS:
      micolinkmtf01p_msg.sys_id = byte;
      micolinkmtf01p_msg.status++;
      break;

    case MICOLINK_MTF01P_PARSE_MSG:
      micolinkmtf01p_msg.msg_id = byte;
      micolinkmtf01p_msg.status++;
      break;

    case MICOLINK_MTF01P_PARSE_SEQ:
      micolinkmtf01p_msg.seq = byte;
      micolinkmtf01p_msg.status++;
      break;

    case MICOLINK_MTF01P_PARSE_PAYLEN:
      micolinkmtf01p_msg.paylen = byte;
      if (micolinkmtf01p_msg.paylen == 0) micolinkmtf01p_msg.status += 2;
      else if (micolinkmtf01p_msg.paylen > MICOLINK_MTF01P_MAX_PAYLOAD_LEN) 
        micolinkmtf01p_msg.status = MICOLINK_MTF01P_PARSE_HEAD;
      else micolinkmtf01p_msg.status++;
      break;

    case MICOLINK_MTF01P_PARSE_PAYLOAD:
      micolinkmtf01p_msg.payload[micolinkmtf01p_msg.payidx++] = byte;
      if (micolinkmtf01p_msg.payidx == micolinkmtf01p_msg.paylen) {
        micolinkmtf01p_msg.payidx = 0;
	micolinkmtf01p_msg.status++;
      }
      break;

    case MICOLINK_MTF01P_PARSE_CHECKSUM:
      micolinkmtf01p_msg.checksum = byte;
      uint8_t length = 6 + micolinkmtf01p_msg.paylen;
      uint8_t temp[MICOLINK_MTF01P_MAX_LEN];
      uint8_t checksum = 0;
      memcpy(temp, &micolinkmtf01p_msg, length);
      for(uint8_t i=0; i<length; i++) checksum += temp[i];
      if(checksum == micolinkmtf01p_msg.checksum) {

        memset(&micolinkmtf01p_pay, 0, sizeof(micolinkmtf01p_pay));
        memcpy(&micolinkmtf01p_pay, micolinkmtf01p_msg.payload, micolinkmtf01p_msg.paylen);

	if ((0 < micolinkmtf01p_pay.distance) && (micolinkmtf01p_pay.distance <=  MICOLINK_MTF01FD_MAX_DISTANCE) 
	  && (abs(micolinkmtf01p_pay.flow_vel_x) <=  MICOLINK_MTF01FD_MAX_FLOW_CM) 
	  && (abs(micolinkmtf01p_pay.flow_vel_y) <=  MICOLINK_MTF01FD_MAX_FLOW_CM)) {

          //opticflow.time_usec = 1000 * micolinkmtf01p_pay.time_ms;
	  opticflow.time_usec = get_sys_time_usec();

	  // Value inverted according to the orientation on the drone, should be configurable
          opticflow.flow_x = micolinkmtf01p_pay.flow_vel_y; // ROLL
          opticflow.flow_y = micolinkmtf01p_pay.flow_vel_x; // PITCH

          opticflow.quality = micolinkmtf01p_pay.flow_quality;
          opticflow.distance = micolinkmtf01p_pay.distance;

          float phi = stateGetNedToBodyEulers_f()->phi;
          float theta = stateGetNedToBodyEulers_f()->theta;
	  opticflow.distance_compensated = (float)(opticflow.distance * cos(phi) * cos(theta));
	  float agl_m = opticflow.distance_compensated * 0.001f;

          if (USE_MICOLINK_MTF01FD_AGL) {
            AbiSendMsgAGL(AGL_LIDAR_MATEKSYS_3901_L0X_ID, 
                        opticflow.time_usec, 
                        agl_m);
          } 
          if (USE_MICOLINK_MTF01FD_OPTICAL_FLOW) {
            AbiSendMsgOPTICAL_FLOW(FLOW_OPTICFLOW_MATEKSYS_3901_L0X_ID, 
                                opticflow.time_usec, 
                                opticflow.flow_x,
                                opticflow.flow_y,
                                0,
                                0,
                                opticflow.quality,
                                0.0);
          }
	}
      }
      micolinkmtf01p_msg.status = MICOLINK_MTF01P_PARSE_HEAD;
      break;

    default:
      micolinkmtf01p_msg.status = MICOLINK_MTF01P_PARSE_HEAD;
      micolinkmtf01p_msg.payidx = 0;
      break;
  }
}

/******************************************************************************/
void micolinkmtf01p_event(void) {
  while (micolinkmtf01p_msg.status != MICOLINK_MTF01P_INITIALIZE 
    && micolinkmtf01p_device->char_available(micolinkmtf01p_device->periph)) {
      micolinkmtf01p_parse(micolinkmtf01p_device->get_byte(micolinkmtf01p_device->periph));
  }
}
