/****************************************************************************
 * apps/examples/spi_test/spi_test_main.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "cubus_app_main.h"

// #include <nuttx/config.h>
// #include <stdio.h>

// #include <string.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/progmem.h>
#include <fcntl.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void print_satellite_health_data();

/****************************************************************************
 * Private Data
 ****************************************************************************/
extern ext_adc_s ext_adc_data[EXT_ADC_MAX_CHANNELS];
extern CRITICAL_FLAGS critic_flags;


char buffer[255] = {'\0'};

satellite_health_s sat_health = {'\0'};
satellite_health_s sat_health_rx_buf = {'\0'};

/****************************************************************************
 * Name: main
 ****************************************************************************/
int main(int argc, FAR char *argv[])
{
  Setup();
  check_flag_data();

  #if defined(CONFIG_CUSTOM_APPS_CUBUS_USE_EXT_ADC) || defined(CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC1) || defined(CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC3)
  RUN_ADC();
  #endif

  make_satellite_health();

  print_satellite_health_data(&sat_health);

  store_sat_health_data(&sat_health);

  retrieve_latest_sat_health_data(&sat_health_rx_buf);

  //TODO: after checking flags data are being written/read correctly, we'll enable satellite health things as well and have a basic complete work queue functions except UART 

  print_critical_flag_data(&critic_flags);

  critic_flags.ANT_DEP_STAT = DEPLOYED;
  store_flag_data(&critic_flags);
  print_critical_flag_data(&critic_flags);  
  return 0;

}


/****************************************************************************
 * Name: adc_main
 ****************************************************************************/

void make_satellite_health(){


  float int_adc1_temp[CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_GROUPSIZE] = {'\0'};  
  int_adc1_data_convert(int_adc1_temp);

  float int_adc3_temp[CONFIG_CUSTOM_APPS_CUBUS_INT_ADC3_GROUPSIZE] = {'\0'};
  int_adc3_data_convert(int_adc3_temp);

  /* External ADC data */
  sat_health.sol_t_v = (int16_t)ext_adc_data[0].processed_data;
  sat_health.raw_v = (int16_t)ext_adc_data[1].processed_data;
  sat_health.sol_p5_v = (int16_t)ext_adc_data[2].processed_data;
  sat_health.sol_p4_v = (int16_t)ext_adc_data[3].processed_data;
  sat_health.sol_p3_v = (int16_t)ext_adc_data[4].processed_data;
  sat_health.sol_p1_v = (int16_t)ext_adc_data[5].processed_data;
  sat_health.sol_p2_v = (int16_t)ext_adc_data[6].processed_data;

  sat_health.ant_temp_out = (int16_t)ext_adc_data[8].processed_data;
  // sat_health.temp_batt = (int16_t)ext_adc_data[9].processed_data;
  sat_health.temp_bpb = (int16_t)ext_adc_data[10].processed_data;
  sat_health.temp_z = (int16_t)ext_adc_data[11].processed_data;


  /* Internal ADC1 data */
  sat_health.batt_c = (int16_t)int_adc1_temp[9];
  sat_health.sol_t_c = (int16_t)int_adc1_temp[10];
  sat_health.raw_c = (int16_t)int_adc1_temp[11];

  sat_health.unreg_c = (int16_t)int_adc1_temp[0];
  sat_health.v3_main_c = (int16_t)int_adc1_temp[1];
  sat_health.v3_com_c = (int16_t)int_adc1_temp[2];
  sat_health.v5_c = (int16_t)int_adc1_temp[3];

  sat_health.batt_volt = (int16_t)int_adc1_temp[4];

  sat_health.sol_p1_c = (int16_t)int_adc1_temp[5];
  sat_health.v3_2_c = (int16_t)int_adc1_temp[6];
  sat_health.sol_p4_c = (int16_t)int_adc1_temp[7];
  sat_health.sol_p5_c = (int16_t)int_adc1_temp[8];

  sat_health.sol_p2_c = (int16_t)int_adc1_temp[12];
  sat_health.sol_p3_c = (int16_t)int_adc1_temp[13];

  /* internal adc2 data*/
  sat_health.v4_c = (int16_t)int_adc3_temp[0];
}


