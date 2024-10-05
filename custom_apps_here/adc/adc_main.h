/****************************************************************************
 * apps/examples/adc/adc.h
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

#ifndef __APPS_CUSTOM_APPS_ADC_H
#define __APPS_CUSTOM_APPS_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>


#include <sys/types.h>
#include <sys/ioctl.h>
#include <nuttx/analog/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>

#include <time.h>
#include <fcntl.h>

#include <poll.h>
#include <sensor/adc.h>
// #include <adc.h>
#include <uORB/uORB.h>
#include <sched.h>
#include <math.h>
#include <nuttx/analog/adc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CONFIG_NSH_BUILTIN_APPS - Build the ADC test as an NSH built-in function.
 *  Default: Built as a standalone program
 * CONFIG_EXAMPLES_ADC_DEVPATH - The default path to the ADC device.
 *   Default: /dev/adc0
 * CONFIG_EXAMPLES_ADC_NSAMPLES - This number of samples is
 *   collected and the program terminates.  Default:  Samples are collected
 *   indefinitely.
 * CONFIG_EXAMPLES_ADC_GROUPSIZE - The number of samples to read at once.
 *   Default: 4
 */

#ifndef CONFIG_CUSTOM_APPS_INT_ADC
  #error "ADC device support is not enabled (CONFIG_CUSTOM_APPS_ADC)"
#endif


/****************************************************************************
 * Public Types
 ****************************************************************************/


#define SENS_TMCS 0.265
#define RES_LMP8640 0.025
#define GAIN_LMP8640 10


struct adc_state_s
{
  bool      initialized;
  FAR char *devpath;
  int       count;
};

struct int_adc_config_s{
  size_t readsize;
  ssize_t nbytes;
  int fd;
  int errval;
  int ret;
};


/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __APPS_EXAMPLES_ADC_ADC_H */
