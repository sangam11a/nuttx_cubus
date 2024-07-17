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

#include "file_operations.h"

/****************************************************************************
 * Name: adc_main
 ****************************************************************************/

int open_file_flash(struct file *file_pointer, char *flash_strpath, char *filename, int open_mode){

  const char file_name[] = {'\0'};
  // memcpy(file_name, filename, sizeof(filename));
  char path[65];
  sprintf(path, "%s%s", flash_strpath, filename);
  int fd = file_open(file_pointer, path, open_mode);
  if(fd < 0){
     syslog(LOG_ERR, "Error opening file: %s\n",path);
     return fd;
  }else{
    syslog(LOG_INFO, "Opened file: %s ...\n",path);
  }
  return fd;
}

int get_file_size(char *filepath, char *filename){
  struct file fp;
  int fd = open_file_flash(&fp, filepath, filename, O_RDONLY);
  if(fd < 0){
    return -1;
  }
  int file_size = file_seek(&fp, 0, SEEK_END);
  file_close(&fp);
  return file_size;
}

int clear_file(char *fpath, char *fname){
  struct file f;
  int fd = open_file_flash(&f, fpath, fname, O_RDWR);
  if(fd < 0){
    return -1;
  }
  int size = file_seek(&f, 0, SEEK_END);
  file_truncate(&f, size);
  file_close(&f);
  return 0;
}