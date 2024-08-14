
#ifndef __CUSTOM_APPS_SPI_DRIVER_TEST_SPI_DRIVER_TEST_H
#define __CUSTOM_APPS_SPI_DRIVER_TEST_SPI_DRIVER_TEST_H

#include <uORB/uORB.h>
#include <sched.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <poll.h>
#include <sensor/mag.h>
#include <nuttx/sensors/sensor.h>
// #include "test2.h"
// #include "test.h"
struct orb_mag_scaled_s
{
  uint64_t timestamp;
  float x;
  float y;
  float z;
  float temperature;
};

ORB_DECLARE(orb_mag_scaled);

#endif //__CUSTOM_APPS_SPI_DRIVER_TEST_SPI_DRIVER_TEST_H
