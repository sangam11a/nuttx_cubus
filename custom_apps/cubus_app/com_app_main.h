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

#ifndef __APPS_CUSTOM_APPS_COM_APP_H
#define __APPS_CUSTOM_APPS_COM_APP_H

#include <nuttx/config.h>
#include <sys/types.h>

#include <errno.h>
#include <debug.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
// #include </home/sushant/apn_cubus/CubeOS/nuttx/include/nuttx/serial/tioctl.h>
#include <nuttx/serial/serial.h>

#define PRINT_DELAY 500

#define COM_UART "/dev/ttyS0"
#define EPDM_UART "/dev/ttyS5"
#define CAM_UART "/dev/ttyS5"
#define ADCS_UART "/dev/ttyS5"

enum MSNS
{
    COM,
    EPDM,
    ADCS,
    CAM
} MSN_CHOICE;

#endif