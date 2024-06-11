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

#include <nuttx/mtd/mtd.h>

#include <nuttx/progmem.h>

#include <string.h>

#include <fcntl.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * custom_hello_main
 ****************************************************************************/

int  main(int argc, FAR char *argv[])
{
  uint8_t read_buf[100];
  char write_buf[100] = "Hello everybody ....";
  int size = 100;
  
  int fd  = open("/dev/intflash",O_RDONLY);
      if(fd < 0){
        printf("Error opening internal flash device\n");
      }else{
        printf("Opened internal flash device successfully\n");
      }
// #ifdef CONFIG_ARCH_HAVE_PROGMEM
      up_progmem_write(0x081C0000, write_buf, 30);

      up_progmem_read(0x081C0000, read_buf, 100);
      
      // printf("File read size: %d \n", size);
      for(int i=0;i<size;i++){
        printf("%x ", read_buf[i]);
      // }
// #endif
      printf("\n");

      close(fd);
}
