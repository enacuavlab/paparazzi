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

/** Default alpha0 */
#ifndef ENC_AMT22_ALPHA0
#define ENC_AMT22_ALPHA0 1.5f
#endif
PRINT_CONFIG_VAR(ENC_AMT22_ALPHA0)

/** Default alpha1 */
#ifndef ENC_AMT22_ALPHA1
#define ENC_AMT22_ALPHA1 1.8f
#endif
PRINT_CONFIG_VAR(ENC_AMT22_ALPHA1)

/** Default alpha2 */
#ifndef ENC_AMT22_ALPHA2
#define ENC_AMT22_ALPHA2 0.65f
#endif
PRINT_CONFIG_VAR(ENC_AMT22_ALPHA2)

/** Default epsilon */
#ifndef ENC_AMT22_EPS
#define ENC_AMT22_EPS 0.06f
#endif
PRINT_CONFIG_VAR(ENC_AMT22_EPS)



struct EncoderAmt22 encoder_amt22;

void encoder_amt22_init(void)
{
  amt22_init(&encoder_amt22.amt22, &AMT22_SPI_DEV, AMT22_SPI_SLAVE_IDX);
  float alpha_gain[3] = {ENC_AMT22_ALPHA0, ENC_AMT22_ALPHA1, ENC_AMT22_ALPHA2};
  high_gain_filter_init(&encoder_amt22.H_g_filter, alpha_gain, ENC_AMT22_EPS, PERIODIC_FREQUENCY);
}

void encoder_amt22_periodic(void)
{
  //amt22_request(&amt22, AMT22_READ_TURNS);
  amt22_request(&encoder_amt22.amt22, AMT22_READ_POSITION);
  high_gain_filter_process(&encoder_amt22.H_g_filter, encoder_amt22.amt22.angle_rad);

}

void encoder_amt22_event(void)
{
  amt22_event(&encoder_amt22.amt22);
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

extern void encoder_amt22_report(void){

  float f[5] = {encoder_amt22.amt22.position, 
                encoder_amt22.amt22.angle_rad,
                encoder_amt22.H_g_filter.hatx[0],
                encoder_amt22.H_g_filter.hatx[1],
                encoder_amt22.H_g_filter.hatx[2]};
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 5, f);

}


