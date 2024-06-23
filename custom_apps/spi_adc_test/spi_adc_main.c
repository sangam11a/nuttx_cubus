/****************************************************************************
 * custom_apps/spi_adc_test/spi_test_main.c
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
#include <assert.h>
#include <fcntl.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/
// struct 

/****************************************************************************
 * spi_test_main
 ****************************************************************************/
static int task2(int argc, char *argv[]);
static int task1(int argc, char *argv[]);

int main(int argc, FAR char *argv[])
{
  printf("Multi-tasking application....\n");
  int ret = 0;
  ret = task_create("task1", 100, 528, task1, NULL);
  if(ret < 0){
    printf("Error starting task 1\n");
  }else{
    printf("task 1 started...\n");
  }
  ret = task_create("task2", 1, 528, task2, NULL);
  if(ret < 0){
    printf("Error starting task 2\n");
  }else{
    printf("task 2 started...\n");
  }
}

static int task1(int argc, char *argv[]){
  for(;;){
    printf("Inside Task 1 ... \n");
    usleep(1000*1000);
    printf("After delay...task 1\n");
    usleep(1000 * 1000);
  }
}

static int task2(int argc, char *argv[]){
  for(;;){
   printf("Inside Task 2 ... \n");
   usleep(1000*1000);
   printf("After delay...task 2\n");
    usleep(1000 * 100);
  }
}