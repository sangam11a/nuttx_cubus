/****************************************************************************
 * boards/arm/stm32/stm32f42a-minimal/src/stm32_wdg.c
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
#include <nuttx/board.h>
#include <nuttx/kthread.h> 

#include "stm32f427a.h"
#include "stm32.h"
#include<time.h>
#include <arch/board/board.h>
bool wdog_task_started = false;


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void wdt_toggle_task(void *arg)
{
  static bool gpio_state = true;
  stm32_gpiowrite(GPIO_WD_WDI, gpio_state);
  stm32_gpiowrite(GPIO_WD_WDI, !gpio_state);
  uint32_t start_time=0, end_time=0;
 
  printf("\n********************************************************************************\n\n\nSatellite resetting in  %d minutes\n", end_time);

  while (1)
  {
    // Toggle GPIO state every 500ms
    // flags = enter_critical_section();
    stm32_gpiowrite(GPIO_WD_WDI, gpio_state);
    // syslog(LOG_DEBUG,"\nGPio toggle state is %d\n", gpio_state);
    gpio_state = !gpio_state;
    // leave_critical_section(flags);
    start_time++;
    usleep(600000);
    
  }
}
/****************************************************************************
 * Name: Toggle watchdog
 *
 * Description:
 *   Directly access the wdog gpio pin
 *
 ****************************************************************************/
int toggle_wdg(){
  stm32_configgpio(GPIO_WD_WDI);
  stm32_gpiowrite(GPIO_WD_WDI, true);
  usleep(5000);
  // printf("____________________TOggled wdog_________\n");
  stm32_gpiowrite(GPIO_WD_WDI, false);
}

int stm32_wdg_setup(void)
{
  stm32_configgpio(GPIO_WD_WDI);
  stm32_gpiowrite(GPIO_WD_WDI, true);
  usleep(10000);
  stm32_gpiowrite(GPIO_WD_WDI, false);
  if (wdog_task_started == false)
  {
    // pid_t pid = task_create("[WDT_toggle_task]", 1, 904, wdt_toggle_task, NULL);
    pid_t pid = kthread_create(
    "WDT TOggle thread",      // Thread name
    50,  // Highest priority
    905,                  // Stack size
    wdt_toggle_task,     // Entry function
    NULL                 // Argument
    );

    if (pid < 0)
    {
      //printf("ERROR: Failed to create wdt_toggle_task\n");
      // return -1;
      pid = task_create("[WDT_toggle_task]", 1, 1204, wdt_toggle_task, NULL);
    }
    else
    {
      wdog_task_started = true;
      // syslog(LOG_DEBUG,"[WDT_Toggle_Task]WDT toggle task created successfully\n");
      stm32_gpiowrite(GPIO_WD_WDI, true);
      stm32_gpiowrite(GPIO_WD_WDI, false);
    }
  }
  else
  {
    printf("[WDT_Toggle_Task]WDT toggle task already created\n");
  }
  return OK;
}
