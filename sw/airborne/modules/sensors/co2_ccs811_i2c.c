/*
 * Fabien Bonneval fabien.bonneval[at]gmail.com
 *
 * This file is part of paparazzi.
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
 *
 */

/**
 * @file modules/sensors/co2_ccs811_i2c.c
 * ScioSense CCS811 CO2 I2C sensor interface.
 *
 * This reads the values for CO2 from the ScioSense CCS811 sensor through I2C.
 */

#include "baro_bmp280_i2c.h"
#include "peripherals/ccs811_i2c.h"


//#include "modules/core/abi.h"
//#include "pprzlink/messages.h"
//#include "modules/datalink/downlink.h"

Ccs811_I2c co2_ccs811;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_co2(struct transport_tx *trans, struct link_device *dev)
{
  // Using CO2_SENSOR message
  if(co2_ccs811.data_available) {
    pprz_msg_send_CO2_SENSOR(trans, dev, AC_ID,
                      &co2_ccs811.co2, &co2_ccs811.tvoc, &co2_ccs811.resistance);
    co2_ccs811.data_available = false;
  }
}
#endif

void co2_ccs811_init(void)
{
  ccs811_i2c_init(&co2_ccs811, &CCS811_I2C_DEV, CCS811_I2C_ADDR);
  
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CO2_SENSOR, send_co2);
#endif
}

void co2_ccs811_periodic(void)
{
  static uint8_t hold_counter;
  if(co2_ccs811.state == CCS811_HOLD && hold_counter++ > 5) {
    co2_ccs811.state = CCS811_UNINIT;
  }
  
  ccs811_i2c_periodic(&co2_ccs811);
}

void co2_ccs811_event(void)
{
  ccs811_i2c_event(&co2_ccs811);
}



