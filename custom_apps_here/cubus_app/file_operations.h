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
#ifndef __APPS_CUSTOM_APPS_CUBUS_APP_FILE_OPERATIONS_H
#define __APPS_CUSTOM_APPS_CUBUS_APP_FILE_OPERATIONS_H

#include <nuttx/config.h>

#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/statfs.h>

#include <stdio.h>
#include <fcntl.h>
#include <debug.h>

int open_file_flash(struct file *file_pointer, char *flash_strpath, char *filename, int open_mode);
int get_file_size(char *filepath, char *filename);
int clear_file(char *fpath, char *fname);

#endif