/*
 * Copyright (C) 2023 Flo&Fab <name.surname@enac.fr>
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

/** @file "modules/sensors/encoder_amt22.c"
 * @author Flo&Fab <name.surname@enac.fr>
 * Driver for AMT22 encoder from CUI devices.
 */

#include "modules/sensors/encoder_amt22.h"

#include "modules/datalink/downlink.h"

/** Default alpha0 old : 2.4*/
#ifndef ENC_AMT22_ALPHA0
#define ENC_AMT22_ALPHA0 2.4f 
#endif
PRINT_CONFIG_VAR(ENC_AMT22_ALPHA0)

/** Default alpha1 old : 2.17*/
#ifndef ENC_AMT22_ALPHA1
#define ENC_AMT22_ALPHA1 2.08f
#endif
PRINT_CONFIG_VAR(ENC_AMT22_ALPHA1)

/** Default alpha2 old : 0.7120*/
#ifndef ENC_AMT22_ALPHA2
#define ENC_AMT22_ALPHA2 0.64f
#endif
PRINT_CONFIG_VAR(ENC_AMT22_ALPHA2)

/** Default epsilon old : not enought: 0.06, too much 0.03  0.009*/
#ifndef ENC_AMT22_EPS
#define ENC_AMT22_EPS 0.02f
#endif
PRINT_CONFIG_VAR(ENC_AMT22_EPS)

#ifndef BYPASS_HG_FILTER_SIM
#define BYPASS_HG_FILTER_SIM FALSE
#endif
PRINT_CONFIG_VAR(BYPASS_HG_FILTER_SIM)



struct EncoderAmt22 encoder_amt22;

float sim_angle;
float sim_ag_speed;
float sim_ag_accel;

void encoder_amt22_init(void)
{
  #if !USE_NPS
    amt22_init(&encoder_amt22.amt22, &AMT22_SPI_DEV, AMT22_SPI_SLAVE_IDX);
  #else
    sim_angle = 0;
    sim_ag_speed = 0;
    sim_ag_accel = 0;
  #endif
  float alpha_gain[3] = {ENC_AMT22_ALPHA0, ENC_AMT22_ALPHA1, ENC_AMT22_ALPHA2};
  high_gain_filter_init(&encoder_amt22.H_g_filter, alpha_gain, ENC_AMT22_EPS, PERIODIC_FREQUENCY);
}

void encoder_amt22_periodic(void)
{
  #if !USE_NPS
    //amt22_request(&amt22, AMT22_READ_TURNS);
    amt22_request(&encoder_amt22.amt22, AMT22_READ_POSITION);
  #endif
  if(!BYPASS_HG_FILTER_SIM){
    high_gain_filter_process(&encoder_amt22.H_g_filter, encoder_amt22.amt22.angle_rad);
  }
}

void encoder_amt22_event(void)
{
  #if !USE_NPS
    amt22_event(&encoder_amt22.amt22);
  #else
    if(BYPASS_HG_FILTER_SIM){
      encoder_amt22.H_g_filter.hatx[0] = sim_angle;
      encoder_amt22.H_g_filter.hatx[1] = sim_ag_speed;
      encoder_amt22.H_g_filter.hatx[2] = sim_ag_accel;
    }
    else{
      encoder_amt22.amt22.angle_rad = sim_angle;
    }
  #endif
}

extern void encoder_amt22_update_alpha0(float alpha0){
  high_gain_filter_update_alpha0(&encoder_amt22.H_g_filter, alpha0);
  high_gain_filter_reset(&encoder_amt22.H_g_filter);
}
extern void encoder_amt22_update_alpha1(float alpha1){
  high_gain_filter_update_alpha1(&encoder_amt22.H_g_filter, alpha1);
  high_gain_filter_reset(&encoder_amt22.H_g_filter);
}
extern void encoder_amt22_update_alpha2(float alpha2){
  high_gain_filter_update_alpha2(&encoder_amt22.H_g_filter, alpha2);
  high_gain_filter_reset(&encoder_amt22.H_g_filter);
}
extern void encoder_amt22_update_epsilon(float epsilon){
  high_gain_filter_update_epsilon(&encoder_amt22.H_g_filter, epsilon);
  high_gain_filter_reset(&encoder_amt22.H_g_filter);
}

extern void encoder_amt22_sim_update(float simulink_angle, float simulink_ag_speed, float simulink_ag_accel){
  sim_angle = simulink_angle;
  sim_ag_speed = simulink_ag_speed;
  sim_ag_accel = simulink_ag_accel;
}

extern void encoder_amt22_report(void){

  float f[5] = {encoder_amt22.amt22.position, 
                encoder_amt22.amt22.angle_rad,
                encoder_amt22.H_g_filter.hatx[0],
                encoder_amt22.H_g_filter.hatx[1],
                encoder_amt22.H_g_filter.hatx[2]};
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 5, f);

}


