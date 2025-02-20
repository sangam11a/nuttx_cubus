
/****************************************************************************
 * apps/examples/watchdog/watchdog_task.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <nuttx/timers/watchdog.h>
#include <time.h>
#include <nuttx/sched.h> // For task management
// #include <nuttx/task.h>   // For task APIs

#define DEVNAME "/dev/iwdg0"
#define TIMEOUT 29000    // 10 seconds in milliseconds
#define PING_INTERVAL 10 // 1 second for pinging
#define STACK_SIZE 848   // Stack size for the watchdog task

extern CRITICAL_FLAGS critic_flags;
int pet_counter = 0;
// Watchdog task function
static int watchdog_task(int argc, char *argv[])
{
  // if (critic_flags.ANT_DEP_STAT == DEPLOYED)
  {

    int fd = open(DEVNAME, O_RDONLY);
    if (fd < 0)
    {
      printf("Failed to open %s: %d\n", DEVNAME, errno);
      return -1;
    }

    // Set the watchdog timeout to 10 seconds
    if (ioctl(fd, WDIOC_SETTIMEOUT, (unsigned long)(TIMEOUT / 1000)) < 0)
    {
      printf("Failed to set timeout: %d\n", errno);
      close(fd);
      return -1;
    }
    else
    {
      printf("timeout set to %lu", (unsigned long)(TIMEOUT / 1000));
    }

    // Start the watchdog timer
    if (ioctl(fd, WDIOC_START, 0) < 0)
    {
      printf("Failed to start watchdog: %d\n", errno);
      close(fd);
      return -1;
    }
    else
    {
      syslog(LOG_DEBUG, "\n  &&&&&&&&&&&&&&&&&************Watchdog started &&&&&&&&&&&&&&&&&************\n");
    }

    // Ping the watchdog every second
    while (1)
    {
      // usleep(PING_INTERVAL * 1000); // Sleep for 1 second

      // Pet the watchdog
      if (pet_counter <= 370)
      {
        if (ioctl(fd, WDIOC_KEEPALIVE, 0) < 0)
        {
          printf("Failed to keep alive: %d\n", errno);
          // Handle the failure condition as needed
        }
        else
        {
          if(pet_counter % 50 == 0)
            printf("Watchdog petted! %d\n", pet_counter);
          pet_counter += 1;
          sleep(1);
        }
      }
      else
      {
        printf("WDOG timeout reached!!!!!!!Reset soon!");
        {
          sleep(3);
        }
      }
    }

    // Stop the watchdog (never reached in this case)
    ioctl(fd, WDIOC_STOP, 0);
    close(fd);
  }
  return 0; // Return success
}
