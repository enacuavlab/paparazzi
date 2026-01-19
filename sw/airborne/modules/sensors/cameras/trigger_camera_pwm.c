/*
 * Copyright (C) 2025 Jean-Baptiste Forestier <jean-baptiste.forestier@enac.fr>
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
 * @file "modules/sensors/cameras/trigger_camera_pwm.c"
 * @author Jean-Baptiste Forestier
 * Trigger using pwm cameras like MAPIR (https://www.mapir.camera/en-gb) 
 * or ThermalCapture 2.0 from TeAx (https://thermalcapture.com/thermalcapture-2-0/)
 * 
 *                       **** MAPIR ****
 * Duty cycle == 2000us => TRIGGER_ON
 * Duty cycle == 1500us => SD Unmount
 * Duty cycle == 1000us => TRIGGER_OFF
 * Image capture frequency = 1.5s for JPG and 2.5-3.0s RAW+JPG
 * 
 * The quickest the 2000us command should be sent is about once every 1.5s as the camera cannot 
 * capture JPG images more quickly than 1.5s. For RAW+JPG mode we recommend a 2.5-3.0s wait time.
 * 
 * RAW images are used for capturing data for reflectance measurements, otherwise the pixels in the JPG are not usable.
 * im_freq_timer should therefore be > 1.5s (=>TRIGGER_CAMERA_CAPTURE_IMAGE_PERIOD = 1.5 seconds)
 * 
 * NMEA GPS data are stored in metadata
 * 
 *                   **** ThermalCapture ****
 * Duty cycle < 1500us => TRIGGER_OFF
 * Duty cycle > 1500us => TRIGGER_ON
 * Image capture frequency up to 30 Hz
 * Possibility to capture one frame per trigger in configurator : Trigger frame mode
 * Convert 14-bit data into temperature values : 
 * High gain mode: temp [°C] = raw * 0.04 - 273.15
 * Low gain mode : temp [°C] = raw * 0.4  - 273.15
 * 
 * NMEA GPS data are stored in metadata
 *  
 *                   **** Trigger  system ****
 *  
 * TIME            ────────────────────────────────────────────────────────────>
 * 
 *     PWM             1000us         2000us         1000us         2000us
 *     Vcc             ┌────┐        ┌───────┐       ┌────┐        ┌───────┐
 *                     │ OFF│        │  ON   │       │ OFF│        │  ON   │
 *     0V          ────┘    └────────┘       └───────┘    └────────┘       └────
 * im_freq_timer:  ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 0 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 0 ↓↓↓↓↓↓↓↓↓↓
 * shutter_timer:                    ↓↓↓↓ 0                        ↓↓↓↓ 0
 * 
 *                                     ↑                             ↑
 *                                  Picture 1                     Picture 2
 * 
 * 
 *  Telemetry ROTORCRAFT_CAM used : 
 *    - arg1 : image capture period in seconds
 *    - arg2 : image counter 
 */




#include "modules/sensors/cameras/trigger_camera_pwm.h"

#include "generated/airframe.h"
#include "generated/modules.h"
#include "modules/actuators/actuators.h"
#include "pprzlink/messages.h"


/** how long to push shutter in seconds */
#ifndef TRIGGER_CAMERA_SHUTTER_DELAY
#define TRIGGER_CAMERA_SHUTTER_DELAY 0.5
#endif

/** Period of image capture in seconds */
#ifndef TRIGGER_CAMERA_CAPTURE_IMAGE_PERIOD
#define TRIGGER_CAMERA_CAPTURE_IMAGE_PERIOD 3.0
#endif

/** Periodic function call frequency : needs to be set at the same frequency as the periodic call in xml */
#ifndef TRIGGER_CAMERA_PERIODIC_FREQ
#define TRIGGER_CAMERA_PERIODIC_FREQ 10.0
#endif

/** Max PWM Value */
#ifndef TRIGGER_CAMERA_ON_VALUE
#define TRIGGER_CAMERA_ON_VALUE MAX_PPRZ
#endif

/** Min PWM Value */
#ifndef TRIGGER_CAMERA_OFF_VALUE
#define TRIGGER_CAMERA_OFF_VALUE MIN_PPRZ
#endif

/** Servo destination */
#ifndef TRIGGER_CAMERA_SERVO
#define TRIGGER_CAMERA_SERVO TRIGGER_CAMERA
#endif

#define ActSet(chan, val) ActuatorSet(chan, val)

/**
 * Timer used for Shutter delay control and Image Capture Frequency + image counter
 */
uint8_t shutter_timer;
uint8_t im_freq_timer;
uint8_t im_counter;

/**
 * Initialization function
 */
void trigger_camera_pwm_init(void) {

  shutter_timer = 0;
  im_counter = 0;
  im_freq_timer = TRIGGER_CAMERA_CAPTURE_IMAGE_PERIOD * TRIGGER_CAMERA_PERIODIC_FREQ;;
  ActSet(TRIGGER_CAMERA_SERVO, TRIGGER_CAMERA_OFF_VALUE);
}


/**
 * Periodic function to send data
 */
void trigger_camera_pwm_periodic(void) {

  //Manage the image capturation each TRIGGER_CAMERA_CAPTURE_IMAGE_PERIOD seconds
  if (im_freq_timer>0) {
    im_freq_timer--;
  } else {
    ActSet(TRIGGER_CAMERA_SERVO, TRIGGER_CAMERA_ON_VALUE);
    im_counter++;
    shutter_timer = TRIGGER_CAMERA_SHUTTER_DELAY * TRIGGER_CAMERA_PERIODIC_FREQ;
    im_freq_timer = TRIGGER_CAMERA_CAPTURE_IMAGE_PERIOD * TRIGGER_CAMERA_PERIODIC_FREQ;
  }

  //Manage the shutter opening time each TRIGGER_CAMERA_SHUTTER_DELAY seconds
  if (shutter_timer>0) {
    shutter_timer--;
  } else {
    ActSet(TRIGGER_CAMERA_SERVO, TRIGGER_CAMERA_OFF_VALUE);
    shutter_timer = 0;
  }

  //Send telemetry with image counter and image perdiod
  int16_t arg1 = (int16_t)TRIGGER_CAMERA_CAPTURE_IMAGE_PERIOD;
	int16_t arg2  = (int16_t)im_counter;

  DOWNLINK_SEND_ROTORCRAFT_CAM(DefaultChannel, DefaultDevice,
    &arg1,
    &arg2
     );
}
