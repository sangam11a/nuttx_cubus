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
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/ioctl.h>

#include <fcntl.h>
#include <syslog.h>
/****************************************************************************
 * Public Functions
 ****************************************************************************/

typedef struct
{
  uint8_t SATELLITE_HEALTH_1; // SAT HEALTH POINTER status
  uint8_t SATELLITE_HEALTH_2; // SAT HEALTH POINTER status

  uint8_t MSN1_DATA_1; // MSN1 DATA POINTER status
  uint8_t MSN1_DATA_2; // MSN1 DATA POINTER status

  uint8_t MSN2_DATA_1; // MSN2 DATA POINTER status
  uint8_t MSN2_DATA_2; // MSN2 DATA POINTER status

  uint8_t MSN3_DATA_1; // MSN3 DATA POINTER status
  uint8_t MSN3_DATA_2; // MSN3 DATA POINTER status
                       // to make sure data is stored in internal flash
} SEEK_POINTER;

#define SEEK_POINTER_INT_ADDR 0x081F0000

#define MFM_MAIN_STRPATH "/mnt/fs/mfm/mtd_mainstorage"
#define file_name_seek_pointer "/seek_pointer.txt"

int open_file_flash(struct file *file_pointer, char *flash_strpath, char *filename, int open_mode)
{

  const char file_name[] = {'\0'};
  // memcpy(file_name, filename, sizeof(filename));
  char path[65];
  sprintf(path, "%s%s", flash_strpath, filename);
  int fd = file_open(file_pointer, path, open_mode | O_CREAT);
  if (fd < 0)
  {
    syslog(LOG_ERR, "Error opening file: %s\n", path);
    return fd;
  }
  else
  {
    syslog(LOG_INFO, "Opened file: %s ...\n", path);
  }
  return fd;
}

/****************************************************************************
 * custom_hello_thread
 ****************************************************************************/
int main(int argc, FAR char *argv[])
{
  int data = 0x0a;
  int ret;
  if (argc > 1)
  {
    //  int fd = open()
    printf("number of argument received is %d\n", argc);
  }
  else
  {
    SEEK_POINTER seek_pointer = {'NULL'}, seek_pointer1 = {'NULL'};
    seek_pointer.SATELLITE_HEALTH_1 = data + 1;
    seek_pointer.SATELLITE_HEALTH_2 = data + 2;
    seek_pointer.MSN1_DATA_1 = data + 3;
    seek_pointer.MSN1_DATA_2 = data + 4;
    seek_pointer.MSN2_DATA_1 = data + 5;
    seek_pointer.MSN2_DATA_2 = data + 6;
    printf("[Cushello] print original pointer\n");
    print_seek_pointer(&seek_pointer);
    printf("[Cushello] store seek pointer\n");
    store_seek_pointer(&seek_pointer);

    printf("[Cushello] read seek pointer\n");
    read_seek_pointer(&seek_pointer1);
    print_seek_pointer(&seek_pointer1);

    printf("[Cushello] print original pointer\n");
    print_seek_pointer(&seek_pointer);
    printf("[Cushello] print data from ext flash\n");
    uint8_t data_retrieved[9];
    retrieve_data_from_flash(MFM_MAIN_STRPATH, "/seek_pointer.txt", data_retrieved, 1);
  }
  return EXIT_SUCCESS;
}

void print_retrieved_data(uint8_t *data_retrieved, ssize_t size)
{
  for (int i = 0; i < size; i++)
  {
    printf("%d ", data_retrieved[i]);
  }
}

void retrieve_data_from_flash(char *partition_name, char *filename, uint8_t *data_retrieved, int times)
{
  // struct stat st;
  struct file fptr;
  int fd = 0;
  fd = open("/mnt/fs/mfm/mtd_mainstorage/seek_pointer.txt", O_CREAT | O_RDONLY);
  // fd = open_file_flash(&fptr, partition_name, filename, O_RDONLY | O_CREAT);
  if (fd >= 0)
  {
    ssize_t bytes_read = read(fd, data_retrieved, sizeof(data_retrieved));
    if (bytes_read > 0)
    {
      print_retrieved_data(data_retrieved, bytes_read);
    }
    // file_syncfs(&fptr);
    // file_close(&fptr);
    close(fd);
  }
  else
  {
    close(fd);

    syslog(LOG_ERR, "Error opening file to read satellite health data..\n");
  }
  file_close(&fptr);
}

void read_seek_pointer(SEEK_POINTER *seek_pointer)
{
  SEEK_POINTER seek_pointer1;
  int fd = open("/dev/intflash", O_RDWR);
  if (fd >= 0)
  {
    syslog(LOG_INFO, "Printing Internal flash flag data.\n");
    up_progmem_read(SEEK_POINTER_INT_ADDR, seek_pointer, sizeof(seek_pointer));
    up_progmem_read(SEEK_POINTER_INT_ADDR, &seek_pointer1, sizeof(seek_pointer));
    // print_seek_pointer(&seek_pointer1);
  }
  else
  {
    syslog(LOG_ERR, "Error opening internal flash to store new flag data ... \n ");
  }
  close(fd);
}
void print_seek_pointer(SEEK_POINTER *seek_pointer)
{
  printf(" ********************************************\r\n");
  printf(" |   SATELLITE_HEALTH_1        \t %d \t|\r\n", seek_pointer->SATELLITE_HEALTH_1);
  printf(" |   SATELLITE_HEALTH_2        \t %d \t|\r\n", seek_pointer->SATELLITE_HEALTH_2);
  printf(" |   MSN1_DATA_1               \t %d \t|\r\n", seek_pointer->MSN1_DATA_1);
  printf(" |   MSN1_DATA_2               \t %d \t|\r\n", seek_pointer->MSN1_DATA_2);
  printf(" |   MSN2_DATA_1               \t %d \t|\r\n", seek_pointer->MSN2_DATA_1);
  printf(" |   MSN2_DATA_2               \t %d \t|\r\n", seek_pointer->MSN2_DATA_2);
  printf(" |   MSN3_DATA_1               \t %d \t|\r\n", seek_pointer->MSN3_DATA_1);
  printf(" |   MSN3_DATA_2               \t %d \t|\r\n", seek_pointer->MSN3_DATA_2);
  printf(" ********************************************\r\n");
}

int store_seek_pointer(SEEK_POINTER *seek_pointer)
{
  int bwr;
  int fd = open("/dev/intflash", O_RDWR);
  if (fd >= 0)
  { // internal flash file opened successfully
    // up_progmem_eraseblock(0x081F0000 );
    up_progmem_write(SEEK_POINTER_INT_ADDR, seek_pointer, sizeof(SEEK_POINTER));
    printf("******************data storing in internal flash memeory\n;");
    print_seek_pointer(seek_pointer);
  }
  else
  {
    syslog(LOG_ERR, "Error opening internal flash to store new flag data ... \n ");
  }
  close(fd);

  return 0;
}
