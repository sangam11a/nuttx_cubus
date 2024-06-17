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
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <syslog.h>
#include "gpio_definitions.h"

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

#define ETX_LED_DRIVER_PATH "/dev/gpio_rw"    // LED Driver path

/****************************************************************************
 * Public Functions
 ****************************************************************************/
typedef struct{
    uint8_t gpio_val;
    uint32_t gpio_num;
    // void *data;
}gpio;

gpio gpio_numval;
/****************************************************************************
 * custom_hello_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
    int ret;

    printf("LED: starting application\n");

    int fd = open(ETX_LED_DRIVER_PATH, O_WRONLY);
    if(fd < 0){
        printf("failed to open: %s\n", ETX_LED_DRIVER_PATH);
    }
    if(!strcmp(argv[1], "COM_ON")){
        printf("GPIO output value: %d", GPIO_3V3_COM_EN);
        printf("Turning on COM mission...\n");
        gpio_numval.gpio_num = GPIO_3V3_COM_EN;
        gpio_numval.gpio_val = 1;
        // gpio_numval.data = NULL;
        ret = write(fd, (const void *)&gpio_numval, sizeof(gpio_numval));
    }
    else if(!strcmp(argv[1], "COM_OFF")){
        printf("Turning off COM mission...\n");
        gpio_numval.gpio_num = GPIO_3V3_COM_EN;
        gpio_numval.gpio_val = 0;
        ret = write(fd, (const void * )&gpio_numval, sizeof(gpio_numval));
        if(ret < 0){
            syslog(LOG_ERR,"Error enabling GPIO pin...\n");
        }
    }
    close(fd);
}

int Turn_gpio_on_off(uint32_t pin, uint8_t mode){

    int fd = open(ETX_LED_DRIVER_PATH, O_WRONLY);
    if(fd < 0){
        syslog(LOG_ERR, "Error opening %s for GPIO WRITE...", ETX_LED_DRIVER_PATH);
        close(fd);
        return -1;
    }
    gpio_numval.gpio_num = pin;
    gpio_numval.gpio_val = mode;
    if(gpio_numval.gpio_val > 1 || gpio_numval.gpio_num < 0){
        syslog(LOG_ERR,"Undefined GPIO pin or set mode selected...\n");
        return -2;
    }
    int ret = write(fd, (const void *)&gpio_numval, sizeof(gpio_numval));
    close(fd);
    if(ret < 0 ){
        syslog(LOG_ERR, "Unable to write to gpio pin...\n");
    }
    return ret;
}