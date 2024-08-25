#include <nuttx/wqueue.h>
#include <nuttx/config.h>
#include <mqueue.h>
#include <nuttx/config.h>
#include <stdio.h>
#include <assert.h>
#include <fcntl.h>

#define QUEUE_NAME "/gpio"
#define MAX_SIZE 1024
#define MSG_STOP "//exit"
static struct work_s work_gpio1;
static struct work_s work_gpio12;
uint8_t pin_mode_1;
static struct work_s work_sec1;

void handle_gpio(char *);
void reader_mq_2()
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
  ssize_t bytes_read;

  // Open the message queue
  mq = mq_open(QUEUE_NAME, O_RDONLY);
  // printf("mq : %d",mq);
  if (mq == (mqd_t)-1)
  {
    // perror("mq_open");
    // exit(1);
  }

  else
  {
    printf("Waiting for messages...\n");

    // while(1)

    // Receive the message
    bytes_read = mq_receive(mq, buffer, MAX_SIZE, NULL);
    if (bytes_read == -1)
    {
      perror("mq_receive");
      // exit(1);
    }

    buffer[bytes_read] = '\0'; // Null-terminate the string

    printf("Received: %s \n", buffer);
    handle_gpio(&buffer);
    // Cleanup
    if (mq_close(mq) == -1)
    {
      perror("mq_close");
      // exit(1);
    }

    if (mq_unlink(QUEUE_NAME) == -1)
    {
      perror("mq_unlink");
      // exit(1);
    }
  }
}
void handle_gpio(char *gpio_name_1)
{

  if (!strcmp(gpio_name_1, "COM"))
  {
    printf("Enabling com\n");
    // gpio_write(GPIO_3V3_COM_EN, pin_mode_1);
  }
  else if (!strcmp(gpio_name_1, "GPIO_COM_4V_EN"))
  {
    printf("Enaling GPIO_COM_4V_EN\n");
  }
  else if (!strcmp(gpio_name_1, "ADCS"))
  { // ADCS
    printf("Enabling ADCS\n");

    // gpio_write(GPIO_MSN_3V3_EN, pin_mode_1);
    // gpio_write(GPIO_MSN1_EN, pin_mode_1);
  }
  else if (!strcmp(gpio_name_1, "CAM"))
  { // CAM
    printf("Enabling CAM\n");

    // gpio_write(GPIO_MSN_3V3_EN, pin_mode_1);
    // gpio_write(GPIO_MSN2_EN, pin_mode_1);
  }
  else if (!strcmp(gpio_name_1, "EPDM"))
  { // EPDM
    // gpio_write(GPIO_MSN_3V3_EN, pin_mode_1);
    // gpio_write(GPIO_MSN3_EN, pin_mode_1);
  }
  else if (!strcmp(gpio_name_1, "MUX"))
  {
    // gpio_write(GPIO_MUX_EN, 1);
    // gpio_write(GPIO_SFM_MODE, pin_mode_1); // TODO: CHECK PULL UP PULL DOWN MODE FOR MSN and OBC access
  }
  else if (!strcmp(gpio_name_1, "ANT"))
  {
    printf("Starting antenna deployment sequence..\n Turning Burner EN line: %d\n Turning Unreg line: %d\n", pin_mode_1, pin_mode_1);
    // gpio_write(GPIO_BURNER_EN, pin_mode_1);
    // gpio_write(GPIO_UNREG_EN, pin_mode_1);
  }
}
void CHECK_GPIO_1()
{
  // char *gpio_name_1;

  while (1)
  {
    // printf("called check gpio\n");
    reader_mq_2();

    // else
    // { // keep on adding other gpio pins as you go
    //     printf("Unknown command \n");
    //     // return;
    // }
    // usleep(1000000);
    sleep(2);
  }
  // work_queue(HPWORK, &work_gpio12, CHECK_GPIO_1, NULL, SEC2TICK(2));
}

void first()
{

  int ret = work_queue(HPWORK, &work_gpio1, first, NULL, SEC2TICK(20));
  if (ret < 0)
  {
    printf("Failed to queue work\n");
    // return -1;
  }
  printf("First\n");
}

int main()
{
  int ret;
  // second();
  // printf("here in spi_driver_test\n");
  // first();
  ret = task_create("data reader", 100, 1500, CHECK_GPIO_1, NULL);
  if (ret < 0)
  {
    printf("Unable to create task data reader\n");
  }
  printf("SPi daemon running\n");
  // CHECK_GPIO_1();
}

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

// #include <nuttx/config.h>
// #include <stdio.h>
// #include <assert.h>
// #include <fcntl.h>
// #include <time.h>
// #include <poll.h>
// #include <unistd.h>
// #include <sys/ioctl.h>

// #include <nuttx/sensors/sensor.h>
// #include <nuttx/sensors/lis3mdl.h>

// #include "spi_driver_test.h"

// #define NB_LOWERHALFS 1

// struct data
// {
//     void *data_struct;
//     uint16_t data_size;
// };

// // #define IOCTL_MODE 1
// // #define READ_MODE   1

// // #define MAX_CHANNELS 12

// /****************************************************************************
//  * spi_driver_test_main
//  ****************************************************************************/

// int main(int argc, FAR char *argv[])
// {
//     int mag_fd;
//     uint16_t seconds;
//     int ret;

//     struct sensor_mag mag;
//     struct sensor_mag mag_sub;

//     int mag_afd, mag_sfd;

//     printf("SPI device LIS3MDL uorb test, World!\n");

//     /* Open SPI device driver */
//     mag_fd = open("/dev/uorb/sensor_mag0", O_RDONLY | O_NONBLOCK);
//     if (mag_fd < 0)
//     {
//         printf("Failed to open mag sensor\n");
//         return -1;
//     }

//     struct pollfd pfds[] = {
//         {.fd = mag_fd, .events = POLLIN}};

//     struct data sensor_data[] = {
//         {.data_struct = &mag, .data_size = sizeof(struct sensor_mag)}};

//     // seconds = 3;

//     // while (seconds > 0)
//     {
//         ret = poll(pfds, NB_LOWERHALFS, -1);
//         if (ret < 0)
//         {
//             perror("Could not poll sensor\n");
//             return ret;
//         }

//         for (int i = 0; i < NB_LOWERHALFS; i++)
//         {
//             if (pfds[i].revents & POLLIN)
//             {
//                 ret = read(pfds[i].fd, sensor_data[i].data_struct,
//                            sensor_data[i].data_size);

//                 if (ret != sensor_data[i].data_size)
//                 {
//                     perror("Could not read from sub-sensor.");
//                     return ret;
//                 }
//             }
//         }
//         seconds -= 3;
//     }

//     printf("Timestamp = %lli\n"
//            "Temperature [c] = %f\n"
//            "mag x axis = %f\n"
//            "mag y axis = %f\n"
//            "mag z axis = %f\n",
//            mag.timestamp, mag.temperature, mag.x, mag.y, mag.z);

//     close(mag_fd);

//     // mag_afd = orb_advertise(ORB_ID(sensor_mag),&mag);
//     // if (mag_afd < 0)
//     // {
//     //   printf("advertise failed: %d\n", errno);
//     // }

//     // orb_publish(ORB_ID(sensor_mag), mag_afd, &mag);

//     // mag_sfd = orb_subscribe(ORB_ID(sensor_mag));
//     // if (mag_sfd < 0)
//     // {
//     //   printf("subscribe failed: %d\n", errno);
//     // }

//     // if (OK != orb_copy(ORB_ID(sensor_mag), mag_sfd, &mag_sub))
//     // {
//     //   printf("copy failed: %d\n", errno);
//     // }

//     // if(mag_sub.timestamp != mag.timestamp)
//     // {
//     //   printf("mismatch adv val: %lli subb val: %lli\n", mag.timestamp, mag_sub.timestamp);
//     // }

//     return 0;
// }