// /****************************************************************************
//  * custom_apps/custom_hello/custom_hello.c
//  *
//  * Licensed to the Apache Software Foundation (ASF) under one or more
//  * contributor license agreements.  See the NOTICE file distributed with
//  * this work for additional information regarding copyright ownership.  The
//  * ASF licenses this file to you under the Apache License, Version 2.0 (the
//  * "License"); you may not use this file except in compliance with the
//  * License.  You may obtain a copy of the License at
//  *
//  *   http://www.apache.org/licenses/LICENSE-2.0
//  *
//  * Unless required by applicable law or agreed to in writing, software
//  * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
//  * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
//  * License for the specific language governing permissions and limitations
//  * under the License.
//  *
//  ****************************************************************************/

// /****************************************************************************
//  * Included Files
//  ****************************************************************************/

// #include <nuttx/config.h>
// #include <stdio.h>
// #include <assert.h>
// #include <fcntl.h>
// #include <time.h>
// #include <poll.h>
// #include <unistd.h>
// #include <sys/ioctl.h>
// #include <mqueue.h>
// #include <nuttx/wqueue.h>
// #include <nuttx/clock.h>

// #include "gpio_control.h"
// #include "gpio_definitions.h"
// /****************************************************************************
//  * spi_driver_test_main
//  ****************************************************************************/

// #define QUEUE_NAME "/gpio"
// #define MAX_SIZE 1024
// #define MSG_STOP "//exit"

// char *gpio_name;
// uint8_t pin_mode;
// static struct work_s work_gpio;

// int gpio_write(uint32_t pin, uint8_t mode)
// {

//   gpio_config_s gpio_numval;
//   int fd = open(ETX_LED_DRIVER_PATH, O_WRONLY);
//   if (fd < 0)
//   {
//     syslog(LOG_ERR, "Error opening %s for GPIO WRITE...", ETX_LED_DRIVER_PATH);
//     close(fd);
//     return -1;
//   }
//   gpio_numval.gpio_num = pin;
//   gpio_numval.gpio_val = mode;
//   if (gpio_numval.gpio_val > 1 || gpio_numval.gpio_num < 0)
//   {
//     syslog(LOG_ERR, "Undefined GPIO pin or set mode selected...\n");
//     return -2;
//   }
//   int ret = write(fd, (const void *)&gpio_numval, sizeof(gpio_config_s));
//   close(fd);
//   if (ret < 0)
//   {
//     syslog(LOG_ERR, "Unable to write to gpio pin...\n");
//   }
//   return ret;
// }

// void reader_mq_1(char *gpio_name)
// {
//   struct mq_attr attr;
//   mqd_t mqd;

//   // Initialize attributes
//   attr.mq_flags = 0;
//   attr.mq_maxmsg = 10;
//   attr.mq_msgsize = 8192;
//   attr.mq_curmsgs = 0;

//   mqd_t mq;

//   char buffer[MAX_SIZE + 1];
//   gpio_name = buffer;
//   ssize_t bytes_read;

//   // Open the message queue
//   mq = mq_open(QUEUE_NAME, O_RDONLY);
//   if (mq == (mqd_t)-1)
//   {
//     perror("mq_open");
//     //exit(1);
//   }

//   printf("Waiting for messages...\n");

//   // while(1)
//   {
//     // Receive the message
//     bytes_read = mq_receive(mq, buffer, MAX_SIZE, NULL);
//     if (bytes_read == -1)
//     {
//       perror("mq_receive");
//       //exit(1);
//     }

//     buffer[bytes_read] = '\0'; // Null-terminate the string

//     printf("Received: %s\n", buffer);

//     // //exit if the received message is "//exit"
//   }

//   // Cleanup
//   if (mq_close(mq) == -1)
//   {
//     perror("mq_close");
//     //exit(1);
//   }

//   if (mq_unlink(QUEUE_NAME) == -1)
//   {
//     perror("mq_unlink");
//     //exit(1);
//   }
// }

// void CHECK_GPIO()
// {

//   while (1)
//   {
//     printf("called check gpio\n");
//     reader_mq_1(gpio_name);

//     if (!strcmp(gpio_name, "COM"))
//     {
//       printf("Enabling com\n");
//       gpio_write(GPIO_3V3_COM_EN, pin_mode);
//     }
//     else if (!strcmp(gpio_name, "MSN1"))
//     { // ADCS
//       printf("Enabling ADCS\n");

//       gpio_write(GPIO_MSN_3V3_EN, pin_mode);
//       gpio_write(GPIO_MSN1_EN, pin_mode);
//     }
//     else if (!strcmp(gpio_name, "MSN2"))
//     { // CAM
//       printf("Enabling CAM\n");

//       gpio_write(GPIO_MSN_3V3_EN, pin_mode);
//       gpio_write(GPIO_MSN2_EN, pin_mode);
//     }
//     else if (!strcmp(gpio_name, "MSN3"))
//     { // EPDM
//       gpio_write(GPIO_MSN_3V3_EN, pin_mode);
//       gpio_write(GPIO_MSN3_EN, pin_mode);
//     }
//     else if (!strcmp(gpio_name, "MUX"))
//     {
//       gpio_write(GPIO_MUX_EN, 1);
//       gpio_write(GPIO_SFM_MODE, pin_mode); // TODO: CHECK PULL UP PULL DOWN MODE FOR MSN and OBC access
//     }
//     else if (!strcmp(gpio_name, "ANT"))
//     {
//       printf("Starting antenna deployment sequence..\n Turning Burner EN line: %d\n Turning Unreg line: %d\n", pin_mode, pin_mode);
//       gpio_write(GPIO_BURNER_EN, pin_mode);
//       gpio_write(GPIO_UNREG_EN, pin_mode);
//     }
//     else
//     { // keep on adding other gpio pins as you go
//       printf("Unknown command \n");
//       return -2;
//     }
//     sleep(1);
//   }
// }

// void second()
// {
//   while (1)
//   {
//     printf("testing");
//     sleep(3);
//   }
// }
// // int main(int argc, FAR char *argv[])
// // {
// //   int ret;

// //   if (argc < 3)
// //   {
// //     printf("Enter both subsystem name and gpio mode\n Enter: Application Name, Subsystem name (CAM, MSN1, MSN2, MSN3 etc.),GPIO Pin Mode (1 or 0)\n ");
// //     printf("In case of MUX: 1 for OBC controls FLASH \t 0 to let MSN access FLASH\n");
// //     // reader_mq_1(gpio_name);
// //   }
// //   else
// //   {
// //     gpio_name = argv[1];
// //     printf("The argv[1] is %s", gpio_name);
// //     pin_mode = atoi(argv[2]);
// //   }

// //   ret = task_create("checkgpio_daemon", 1, CONFIG_CUSTOM_APPS_CUSTOM_HELLO_STACKSIZE, CHECK_GPIO, NULL);
// //   if (ret < 0)
// //   {
// //     printf("Failed to create checkgpio_daemon task\n");
// //     return -1;
// //   }

// //   ret = task_create("second", 1, CONFIG_CUSTOM_APPS_CUSTOM_HELLO_STACKSIZE, second, NULL);
// //   if (ret < 0)
// //   {
// //     printf("Failed to create second task\n");
// //     return -1;
// //   }

// //   return 0;
// // }

// int main(int argc, FAR char *argv[])
// {
//   if (argc < 3)
//   {
//     printf("Enter both subsystem name and gpio mode\n Enter: Application Name, Subsystem name (CAM, MSN1, MSN2, MSN3 etc.),GPIO Pin Mode (1 or 0)\n ");
//     printf("In case of MUX: 1 for OBC controls FLASH \t 0 to let MSN access FLASH\n");
//     // return -1;// edited dfor
//     // reader_mq_1(gpio_name);
//     // work_queue(HPWORK, &work_gpio, CHECK_GPIO, NULL, SEC2TICK(2));
//     int ret = task_create("checkgpio_daemon", 1,
//                           CONFIG_CUSTOM_APPS_CUSTOM_HELLO_STACKSIZE, CHECK_GPIO,
//                           NULL);
//     ret = task_create("second", 1,
//                       CONFIG_CUSTOM_APPS_CUSTOM_HELLO_STACKSIZE, second,
//                       NULL);
//   }

//   else
//   {
//     gpio_name = argv[1];
//     printf("The argv[1] is %s", gpio_name);
//     pin_mode = atoi(argv[2]);
//   }
// }


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
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the License for the specific language governing permissions and 
 * limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <assert.h>
#include <fcntl.h>
#include <time.h>
#include <poll.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <mqueue.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <nuttx/irq.h>
#include <time.h>

#include "gpio_control.h"
#include "gpio_definitions.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define QUEUE_NAME "/gpio"
#define MAX_SIZE 1024
#define MSG_STOP "//exit"
// #define SEC2TICK(sec) ((sec) * CLK_TCK)

/****************************************************************************
 * Private Data
 ****************************************************************************/

char *gpio_name;
uint8_t pin_mode;
static struct work_s work_gpio;
static struct work_s work_sec;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int gpio_write(uint32_t pin, uint8_t mode)
{
  gpio_config_s gpio_numval;
  int fd = open(ETX_LED_DRIVER_PATH, O_WRONLY);
  if (fd < 0)
  {
    syslog(LOG_ERR, "Error opening %s for GPIO WRITE...", ETX_LED_DRIVER_PATH);
    close(fd);
    return -1;
  }
  gpio_numval.gpio_num = pin;
  gpio_numval.gpio_val = mode;
  if (gpio_numval.gpio_val > 1 || gpio_numval.gpio_num < 0)
  {
    syslog(LOG_ERR, "Undefined GPIO pin or set mode selected...\n");
    return -2;
  }
  int ret = write(fd, (const void *)&gpio_numval, sizeof(gpio_config_s));
  close(fd);
  if (ret < 0)
  {
    syslog(LOG_ERR, "Unable to write to gpio pin...\n");
  }
  return ret;
}

void reader_mq_1(char *gpio_name)
{
  struct mq_attr attr;
  mqd_t mqd;

  // Initialize attributes
  attr.mq_flags = 0;
  attr.mq_maxmsg = 10;
  attr.mq_msgsize = 8192;
  attr.mq_curmsgs = 0;

  mqd_t mq;
  char buffer[MAX_SIZE + 1];
  gpio_name = buffer;
  ssize_t bytes_read;

  // Open the message queue
  mq = mq_open(QUEUE_NAME, O_RDONLY);
  if (mq == (mqd_t)-1)
  {
    perror("mq_open");
    //exit(1);
  }

  printf("Waiting for messages...\n");

  // while(1)
  {
    // Receive the message
    bytes_read = mq_receive(mq, buffer, MAX_SIZE, NULL);
    if (bytes_read == -1)
    {
      perror("mq_receive");
      //exit(1);
    }

    buffer[bytes_read] = '\0'; // Null-terminate the string

    printf("Received: %s\n", buffer);
  }

  // Cleanup
  if (mq_close(mq) == -1)
  {
    perror("mq_close");
    //exit(1);
  }

  if (mq_unlink(QUEUE_NAME) == -1)
  {
    perror("mq_unlink");
    //exit(1);
  }
}

void CHECK_GPIO(void *arg)
{
  // while (1)
  {
    printf("called check gpio\n");
    reader_mq_1(gpio_name);

    if (!strcmp(gpio_name, "COM"))
    {
      printf("Enabling com\n");
      gpio_write(GPIO_3V3_COM_EN, pin_mode);
    }
    else if (!strcmp(gpio_name, "MSN1"))
    { // ADCS
      printf("Enabling ADCS\n");

      gpio_write(GPIO_MSN_3V3_EN, pin_mode);
      gpio_write(GPIO_MSN1_EN, pin_mode);
    }
    else if (!strcmp(gpio_name, "MSN2"))
    { // CAM
      printf("Enabling CAM\n");

      gpio_write(GPIO_MSN_3V3_EN, pin_mode);
      gpio_write(GPIO_MSN2_EN, pin_mode);
    }
    else if (!strcmp(gpio_name, "MSN3"))
    { // EPDM
      gpio_write(GPIO_MSN_3V3_EN, pin_mode);
      gpio_write(GPIO_MSN3_EN, pin_mode);
    }
    else if (!strcmp(gpio_name, "MUX"))
    {
      gpio_write(GPIO_MUX_EN, 1);
      gpio_write(GPIO_SFM_MODE, pin_mode); // TODO: CHECK PULL UP PULL DOWN MODE FOR MSN and OBC access
    }
    else if (!strcmp(gpio_name, "ANT"))
    {
      printf("Starting antenna deployment sequence..\n Turning Burner EN line: %d\n Turning Unreg line: %d\n", pin_mode, pin_mode);
      gpio_write(GPIO_BURNER_EN, pin_mode);
      gpio_write(GPIO_UNREG_EN, pin_mode);
    }
    else
    { // keep on adding other gpio pins as you go
      printf("Unknown command \n");
      // return;
    }
    // usleep(1000000);
    // sleep(2);
  }
}

void second1(void *arg)
{
  // while (1)
  {
    printf("testing\n");
    // usleep(1000000);
  }
}

int main(int argc, FAR char *argv[])
{
  // if (argc < 3)
  // {
  //   printf("Enter both subsystem name and gpio mode\n Enter: Application Name, Subsystem name (CAM, MSN1, MSN2, MSN3 etc.),GPIO Pin Mode (1 or 0)\n ");
  //   printf("In case of MUX: 1 for OBC controls FLASH \t 0 to let MSN access FLASH\n");

  //   // Create tasks
 
  // }
  // else
  // {
  //   gpio_name = argv[1];
  //   printf("The argv[1] is %s\n", gpio_name);
  //   pin_mode = atoi(argv[2]);

  //   // Use work queue to schedule CHECK_GPIO
   
  //   // Start second task
  //   // ret = task_create("second", CONFIG_SCHED_PRIORITY_DEFAULT, CONFIG_CUSTOM_APPS_CUSTOM_HELLO_STACKSIZE, second, NULL);
    
  // }
  int ret ;
  ret = work_queue(HPWORK, &work_sec, second1, NULL, SEC2TICK(1));
    if (ret < 0)
    {
      printf("Failed to create second task\n");
      // return -1;
    }
   ret = work_queue(HPWORK, &work_gpio, CHECK_GPIO, NULL, SEC2TICK(2));
    if (ret < 0)
    {
      printf("Failed to queue work\n");
      // return -1;
    }

  

  return 0;
}
