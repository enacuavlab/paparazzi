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
 * The quickest the 2000us command should be sent is about once every 1.5s as the camera cannot 
 * capture JPG images more quickly than 1.5s. For RAW+JPG mode we recommend a 2.5-3.0s wait time.
 * 
 * RAW images are used for capturing data for reflectance measurements, otherwise the pixels in the JPG are not usable.
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
 * im_freq_timer should therefore be > 1.5s (=>TRIGGER_CAMERA_CAPTURE_IMAGE_FREQ = 1.5 seconds)
 */




#include "modules/sensors/cameras/trigger_camera_pwm.h"

#include "generated/airframe.h"
#include "generated/modules.h"
#include "modules/actuators/actuators.h"


/** how long to push shutter in seconds */
#ifndef TC_SHUTTER_DELAY
#define TC_SHUTTER_DELAY 0.5
#endif

/** Frequency of image capture */
#ifndef TRIGGER_CAMERA_CAPTURE_IMAGE_FREQ
#define TRIGGER_CAMERA_CAPTURE_IMAGE_FREQ 1.5
#endif

/** Periodic function call frequency */
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

/**
 * Timer used for Shutter delay control and Image Capture Frequency
 */
uint8_t shutter_timer;
uint8_t im_freq_timer;

/**
 * Initialization function
 */
void trigger_camera_pwm_init(void) {

  shutter_timer = 0;
  im_freq_timer = TRIGGER_CAMERA_CAPTURE_IMAGE_FREQ * TRIGGER_CAMERA_PERIODIC_FREQ;;
  ActuatorSet(TRIGGER_CAMERA_SERVO, TRIGGER_CAMERA_OFF_VALUE);
}


/**
 * Periodic function to send data
 */
void trigger_camera_pwm_periodic(void) {

  //Manage the image capturation each TRIGGER_CAMERA_CAPTURE_IMAGE_FREQ seconds
  if (im_freq_timer>0) {
    im_freq_timer--;
  } else {
    ActuatorSet(TRIGGER_CAMERA_SERVO, TRIGGER_CAMERA_ON_VALUE);
    shutter_timer = TC_SHUTTER_DELAY * TRIGGER_CAMERA_PERIODIC_FREQ;
    im_freq_timer = TRIGGER_CAMERA_CAPTURE_IMAGE_FREQ * TRIGGER_CAMERA_PERIODIC_FREQ;
  }

  //Manage the shutter opening time each TC_SHUTTER_DELAY seconds
  if (shutter_timer>0) {
    shutter_timer--;
  } else {
    ActuatorSet(TRIGGER_CAMERA_SERVO, TRIGGER_CAMERA_OFF_VALUE);
    shutter_timer = 0;
  }
}