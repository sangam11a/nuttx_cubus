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
 * Preprocessor Definitions
 ****************************************************************************/
#define int_addr  0x081C0000  //address to read/write/erase

#define block   22    //sector corresponding to address 

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * custom_hello_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int buf_size = 30;  //size of data to read and write
  uint8_t read_buf[30];
  char write_buf[30] = "Hello everybody ....";
  
  int fd  = open("/dev/intflash", O_RDWR);   //opening the internal flash driver //LETS SEE IF WE CAN DO STUFF WITHOUT OPENING THE FILE OR NOT 
  if(fd < 0){
    // syslog(LOG_ERROR, "Error opening internal flash device\n");
    printf("Error opening internal flash device\n");
  }else{
    // syslog(LOG_INFO, "Opened internal flash device successfully\n");
    printf("Opened internal flash device successfully\n");
  }

  up_progmem_eraseblock(22);    //erasing the sector of internal flash (block means sector here)

  up_progmem_read(0x081C0000, read_buf, 30);  //reading after erasing
  
  /* reading after erasing */    
  for(int i=0;i<buf_size;i++){
    printf("%x ", read_buf[i]);
  }

  up_progmem_write(0x081C0000, write_buf, 30);  //writing data into the internal flash

  up_progmem_read(0x081C0000, read_buf, 30);   //reading data from internal flash

  /* reading after writing */    
  for(int i=0;i<buf_size;i++){
    printf("%x ", read_buf[i]);
  }
  printf("\n");

  close(fd);

  return 0;
}


/*
function to write data to internal flash memory
first the block/sector needs to be erased to rewrite data into the same address
this functions performs erase and write operations sequentially, nothing more... 

params:
  block:    sector number to erase corresponding to address 
  address:  address to start writing data from
  buffer:   data to write to flash memory
  size:     size of buffer data to be written 
*/
void WRT_TO_INT_FLASH(size_t blck, uint32_t address, uint8_t *buffer, uint32_t size){

  up_progmem_eraseblock(blck);

  up_progmem_write(address, buffer, size);

}
