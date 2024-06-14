/****************************************************************************
 * custom_apps/custom_hello/custom_hello.c
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

#include <nuttx/config.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
// #include "write_to_file.h"



// #include <nuttx/arch/arm/src/stm32_serial.c>
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * custom_hello_main
 ****************************************************************************/
  static uint64_t gettime_nsecs(void)
{
  struct timespec ts;
  uint64_t nsecs;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  printf("value is %d\n",ts.tv_sec);
  nsecs  = ts.tv_sec;
  nsecs *= 1000 * 1000 * 1000;
  nsecs += ts.tv_nsec;

  return nsecs;
}

int  main(int argc, FAR char *argv[])
{
  // write("/dev/ttyS7","tes7",4);
  // write("/dev/ttyS6","tes6",4);
  // write("/dev/ttyS7","tes5",4);
  //syslog(LOG_INFO, "Hello World application for writing data to flash.\n");
  // printf("got %d number of arguments\n",argc);
  // int i ;
  // for(i = 0; i < argc; i++){
  //   printf("Argument is %s\n",argv[i]);
  // }
  // i=0;
  // do {
  //   write_to_file("/mnt/fs/mfm/mtd_mainstorage","/numbers.txt",argv[i++]);
  //   argc--;
  // }while(argc > 0);
  
// #ifdef CONFIG_RTC
  // up_rtc_gettime();

uint64_t i = gettime_nsecs();
printf("the value of i is %u",i);

// #endif
  
  return 0;
}
