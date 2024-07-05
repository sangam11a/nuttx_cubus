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

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/analog/ads7953.h>

#include "adc.h"

#include "math.h"

#include <nuttx/fs/fs.h>

#include <sys/mount.h>

#define IOCTL_MODE  1
// #define READ_MODE   1

#define EXT_ADC_MAX_CHANNELS  12

#define RES_LMP8640				0.025
#define GAIN_LMP8640			10

#define SENS_TMCS				0.265

#define MFM_MSN_STRPATH		    "/mnt/fs/mfm/mtd_mission"
#define MFM_MAIN_STRPATH        "/mnt/fs/mfm/mtd_mainstorage"

#define SFM_MSN_STRPATH		    "/mnt/fs/sfm/mtd_mission"
#define SFM_MAIN_STRPATH        "/mnt/fs/sfm/mtd_mainstorage"

typedef struct {
  size_t readsize;
  ssize_t nbytes;
  int fd;
  int errval;
  int ret;
}int_adc_config_s;


typedef struct {
  int fd;
}ext_adc_config_s;

typedef struct  {
	// uint64_t timestamp;
	// float accl_x;
	// float accl_y;
	// float accl_z;
	// float gyro_x;
	// float gyro_y;
	// float gyro_z;
	// float mag_x;
	// float mag_y;
	// float mag_z;
	int16_t temp_x;
	int16_t temp_x1;
	int16_t temp_y;
	int16_t temp_y1;
	int16_t temp_z;
	int16_t temp_z1;
	int16_t temp_bpb;
	int16_t temp_obc;
	int16_t temp_com;
	int16_t temp_batt;
	int16_t batt_volt;
	int16_t sol_p1_v;
	int16_t sol_p2_v;
	int16_t sol_p3_v;
	int16_t sol_p4_v;
	int16_t sol_p5_v;
	int16_t sol_t_v;
	int16_t raw_v;
	int16_t sol_p1_c;
	int16_t sol_p2_c;
	int16_t sol_p3_c;
	int16_t sol_p4_c;
	int16_t sol_p5_c;
	int16_t sol_t_c;
	int16_t rst_3v3_c;
	int16_t raw_c;
	int16_t v3_main_c;
	int16_t v3_com_c;
	int16_t v3_2_c;
	int16_t v5_c;
	int16_t unreg_c;
	int16_t v4_c;
	int16_t batt_c;
	int8_t rsv_cmd;
	int8_t ant_dep_stat;
	int8_t ul_state;
	int8_t oper_mode;
	int8_t msn_flag;
	int8_t rsv_flag;
	int8_t kill_switch;
    int16_t ant_temp_out;
}satellite_health_s;


typedef struct {
    uint8_t chan;
    float raw_data;
    float processed_data;
}ext_adc_s;

int ext_adc_main();
int read_int_adc1();
int read_int_adc3();