
#ifndef __CUSTOM_APPS_SPI_DRIVER_TEST_SPI_DRIVER_TEST_H
#define __CUSTOM_APPS_SPI_DRIVER_TEST_SPI_DRIVER_TEST_H

#include <uORB/uORB.h>
#include <time.h>
#include <poll.h>
#include <sensor/mag.h>
#include <nuttx/sensors/sensor.h>

struct orb_mag_s
{
  uint64_t timestamp;
  float x;
  float y;
  float z;
  float temperature;
};

#endif //__CUSTOM_APPS_SPI_DRIVER_TEST_SPI_DRIVER_TEST_H
