/****************************************************************************
 * drivers/sensors/lis3mdl.c
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
#include <nuttx/nuttx.h>

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>


#include <math.h>

#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>
#include <nuttx/wqueue.h>

#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/ioctl.h>

#include <nuttx/sensors/lis3mdl.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_LIS3MDL_ORB)


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEV_ID 0x3D

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct lis3mdl_data_s
{
  uint64_t timestamp;   /* Units in microseconds */
  float x;              /* Magnetic field along X-Axis */
  float y;              /* Magnetic field along Y-Axis */
  float z;              /* Magnetic field along Z-Axis */
  float temperature;    /* Temperature in degree celsius */
};

struct lis3mdl_sensor_s
{
  /* Lower half sensor driver */
  struct sensor_lowerhalf_s lower;
  bool calibrated;                  /* Is device calibrated? */
};

struct lis3mdl_dev_s
{
  struct lis3mdl_sensor_s dev;    /* Sensor Private data */
  FAR struct spi_dev_s *spi;      /* SPI Interface */
  struct lis3mdl_config_s *config; /* configruration data */
  mutex_t dev_lock;               /* Manages exclusive access to the device */
  sem_t run;                      /* Locks the sensor thread */
  bool enabled;                   /* Enable/disable LIS3MDL */
};


typedef int (*push_data_func)(FAR struct lis3mdl_dev_s *priv,
                              FAR struct lis3mdl_data_s *data);
/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void lis3mdl_read_register(FAR struct lis3mdl_dev_s *dev,
                                  uint8_t const reg_addr,
                                  uint8_t *reg_data);
static void lis3mdl_write_register(FAR struct lis3mdl_dev_s *dev,
                                   uint8_t const reg_addr,
                                   uint8_t const reg_data);

static void lis3mdl_reset(FAR struct lis3mdl_dev_s *dev);

static void lis3mdl_read_measurement_data(FAR struct lis3mdl_dev_s *dev,
                                          FAR struct lis3mdl_data_s *data);
static void lis3mdl_read_magnetic_data(FAR struct lis3mdl_dev_s *dev,
                                       uint16_t *x_mag, uint16_t *y_mag,
                                       uint16_t *z_mag);
static void lis3mdl_read_temperature(FAR struct lis3mdl_dev_s *dev,
                                     uint16_t *temperature);
static int lis3mdl_push_data(FAR struct lis3mdl_dev_s *dev,
                             FAR struct lis3mdl_data_s *data);
/* sensor methods */

static int lis3mdl_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable);

static int lis3mdl_control(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           int cmd, unsigned long arg);              


/****************************************************************************
 * Private Data
 ****************************************************************************/

static const push_data_func deliver_data =
{
  lis3mdl_push_data
};

/* the lower half sensor driver operations for sensor register */
static const struct sensor_ops_s g_sensor_ops = 
{
  NULL,               /* Open */
  NULL,               /* Close */
  lis3mdl_activate,   /* activate */
  NULL,               /* set_interval */
  NULL,               /* batch */
  NULL,               /* fetch */
  NULL,               /* selftest */
  NULL,               /* set_calibvalue */
  NULL,               /* calibrate */
  lis3mdl_control     /* control */
};


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lis3mdl_read_register
 ****************************************************************************/

static void lis3mdl_read_register(FAR struct lis3mdl_dev_s *dev,
                                  uint8_t const reg_addr, uint8_t *reg_data)
{
  /* Lock the SPI bus so that only one device can access it at the same
   * time
   */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the LIS3MDL */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to read - the MSB
   * needs to be set to indicate the read indication.
   */

  SPI_SEND(dev->spi, reg_addr | 0x80);

  /* Write an idle byte while receiving the required data */

  *reg_data = (uint8_t)(SPI_SEND(dev->spi, 0));

  /* Set CS to high which deselects the LIS3MDL */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: lis3mdl_write_register
 ****************************************************************************/

static void lis3mdl_write_register(FAR struct lis3mdl_dev_s *dev,
                                   uint8_t const reg_addr,
                                   uint8_t const reg_data)
{
  /* Lock the SPI bus so that only one device can access it at the same
   * time
   */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the LIS3MDL */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to read */

  SPI_SEND(dev->spi, reg_addr);

  /* Transmit the content which should be written in the register */

  SPI_SEND(dev->spi, reg_data);

  /* Set CS to high which deselects the LIS3MDL */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: lis3mdl_checkid
 ****************************************************************************/

static int lis3mdl_checkid(FAR struct lis3mdl_dev_s *dev)
{
  uint8_t devid = 0;

  lis3mdl_read_register(dev, LIS3MDL_WHO_AM_I_REG, &devid);
  up_mdelay(1);
  sninfo("devid: 0x%02x\n", devid);

  if(devid != (uint8_t)DEV_ID)
  {
    /* ID is not correct */
    snerr("Wrong Device ID! %02x\n", devid);
    return -ENODEV;
  }
  return OK;
}

/****************************************************************************
 * Name: lis3mdl_reset
 ****************************************************************************/

static void lis3mdl_reset(FAR struct lis3mdl_dev_s *dev)
{
  lis3mdl_write_register(dev,
                         LIS3MDL_CTRL_REG_2,
                         LIS3MDL_CTRL_REG_2_SOFT_RST_BM);

  up_mdelay(100);
}

/****************************************************************************
 * Name: lis3mdl_interrupt_handler
 ****************************************************************************/

static void lis3mdl_read_measurement_data(FAR struct lis3mdl_dev_s *dev,
                                          FAR struct lis3mdl_data_s *data)
{
  /* Magnetic data */

  uint16_t x_mag = 0;
  uint16_t y_mag = 0;
  uint16_t z_mag = 0;

  lis3mdl_read_magnetic_data(dev, &x_mag, &y_mag, &z_mag);

  /* Temperature */

  uint16_t temperature = 0;

  lis3mdl_read_temperature(dev, &temperature);

  data->x = (float) (x_mag);
  data->y = (float) (y_mag);
  data->z = (float) (z_mag);
  data->temperature = (float) temperature;
  data->timestamp = sensor_get_timestamp();
}

static int lis3mdl_push_data(FAR struct lis3mdl_dev_s *dev,
                             FAR struct lis3mdl_data_s *data)
{
  struct sensor_mag mag_data;
  int ret;

  struct sensor_lowerhalf_s lower = dev->dev.lower;

  mag_data.timestamp = data->timestamp;
  mag_data.mag_x =  data->x;
  mag_data.mag_y = data->y;
  mag_data.mag_z = data->z;
  mag_data.temperature = data->temperature;

  ret = lower.push_event(lower.priv, &mag_data,
                         sizeof(struct sensor_mag));

  if (ret < 0)
  {
    snerr("Pushing mag data failed\n");
    return ret;
  }

  return OK;
}


/****************************************************************************
 * Name: lis3mdl_read_magnetic_data
 ****************************************************************************/

static void lis3mdl_read_magnetic_data(FAR struct lis3mdl_dev_s *dev,
                                       uint16_t *x_mag, uint16_t *y_mag,
                                       uint16_t *z_mag)
{
  /* Lock the SPI bus so that only one device can access it at the same
   * time
   */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the LIS3MDL */
  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to start reading
   * 0x80 -> MSB is set -> Read Indication 0x40 -> MSB-1 (MS-Bit) is
   * set -> auto increment of address when reading multiple bytes.
   */
  SPI_SEND(dev->spi, (LIS3MDL_OUT_X_L_REG | 0x80 | 0x40)); /* RX */
  *x_mag = ((uint16_t)(SPI_SEND(dev->spi, 0)) << 0);       /* LSB */
  *x_mag |= ((uint16_t)(SPI_SEND(dev->spi, 0)) << 8);      /* MSB */

  *y_mag = ((uint16_t)(SPI_SEND(dev->spi, 0)) << 0);  /* LSB */
  *y_mag |= ((uint16_t)(SPI_SEND(dev->spi, 0)) << 8); /* MSB */

  *z_mag = ((uint16_t)(SPI_SEND(dev->spi, 0)) << 0);  /* LSB */
  *z_mag |= ((uint16_t)(SPI_SEND(dev->spi, 0)) << 8); /* MSB */

  /* Set CS to high which deselects the LIS3MDL */
  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: lis3mdl_read_temperature
 ****************************************************************************/

static void lis3mdl_read_temperature(FAR struct lis3mdl_dev_s *dev,
                                     uint16_t *temperature)
{
  /* Lock the SPI bus so that only one device can access it at the same
   * time
   */

  SPI_LOCK(dev->spi, true);

  /* Set CS to low which selects the LIS3MDL */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to start reading
   * 0x80 -> MSB is set -> Read Indication 0x40 -> MSB-1 (MS-Bit) is
   * set -> auto increment of address when reading multiple bytes.
   */

  SPI_SEND(dev->spi, (LIS3MDL_TEMP_OUT_L_REG | 0x80 | 0x40));

  /* RX */

  *temperature = ((uint16_t)(SPI_SEND(dev->spi, 0)) << 0);  /* LSB */
  *temperature |= ((uint16_t)(SPI_SEND(dev->spi, 0)) << 8); /* MSB */

  /* Set CS to high which deselects the LIS3MDL */
  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: lis3mdl_activate
 ****************************************************************************/

static int lis3mdl_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable)
{
  int offset;
  switch (lower->type)
  {
    default:
      offset = 0;
      break;
  }
  
  FAR struct lis3mdl_sensor_s *dev = (FAR struct lis3mdl_sensor_s *)
                                     ((uintptr_t) lower - offset * sizeof(*lower));
  FAR struct lis3mdl_dev_s *priv = container_of(dev,
                                                FAR struct lis3mdl_dev_s,
                                                dev);
  struct sensor_mag temp;

  /**
   * Wake the thread
   */

  if(!priv->enabled && enable)
  {
    dev->calibrated = false;
    priv->enabled = enable;

    /* setup the sensor before polling thread */
    /* perform a reset */
    lis3mdl_reset(priv);

    /* Enable DRDY signal on INT 2 */

    /* Enable * - the maximum full scale mode. * Full scale = +/- 1.6 mT (16
     * Gauss).
     */

    lis3mdl_write_register(priv,
                           LIS3MDL_CTRL_REG_2,
                           LIS3MDL_CTRL_REG_2_FS_1_BM |
                               LIS3MDL_CTRL_REG_2_FS_0_BM);

    /* Enable - temperature sensor - ultra high performance mode (UMP) for X
     * and Y - fast output data rates This results in a output data rate of
     * 155 Hz for X and Y.
     */

    lis3mdl_write_register(priv,
                           LIS3MDL_CTRL_REG_1,
                           LIS3MDL_CTRL_REG_1_TEMP_EN_BM |
                               LIS3MDL_CTRL_REG_1_OM_1_BM |
                               LIS3MDL_CTRL_REG_1_OM_0_BM |
                               LIS3MDL_CTRL_REG_1_FAST_ODR_BM);

    /* Enable * - ultra high performance mode (UMP) for Z * This should result
     * to the same output data rate as for X and Y.
     */

    lis3mdl_write_register(priv,
                           LIS3MDL_CTRL_REG_4,
                           LIS3MDL_CTRL_REG_4_OMZ_1_BM |
                               LIS3MDL_CTRL_REG_4_OMZ_0_BM);

    /* Enable * - block data update for magnetic sensor data * This should
     * prevent race conditions when reading sensor data.
     */

    lis3mdl_write_register(priv,
                           LIS3MDL_CTRL_REG_5,
                           LIS3MDL_CTRL_REG_5_BDU_BM);

    /* Enable continuous conversion mode - the device starts measuring now. */

    lis3mdl_write_register(priv, LIS3MDL_CTRL_REG_3, 0);

    /* Read measurement data to ensure DRDY is low */

    lis3mdl_read_measurement_data(priv, &temp);

    /* Wake the polling thread */

    nxsem_post(&priv->run);

    return OK;
  }

  priv->enabled = enable;
  return OK;
}


static int lis3mdl_control(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           int cmd, unsigned long arg)
{
  FAR struct lis3mdl_sensor_s *dev = container_of(lower,
                                                  FAR struct lis3mdl_sensor_s,
                                                  lower);
  
  FAR struct lis3mdl_dev_s *priv = container_of(dev,
                                                FAR struct lis3mdl_dev_s,
                                                dev);
  int ret;

  switch (cmd)
  {
    case SNIOC_RESET:
      {
        /* Perform soft reset */
        lis3mdl_reset(priv);
      }
      break;

    default:
      break;
  }

  return OK;
}

static int lis3mdl_thread(int argc, char **argv)
{
  FAR struct lis3mdl_dev_s *priv =
    (FAR struct lis3mdl_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
  struct lis3mdl_data_s data;
  int ret;

  while (true)
    {
      int sensor;

      if (!priv->enabled)
        {
          /* Wait for the sensor to be enabled */

          nxsem_wait(&priv->run);
        }

      /* No measurements are done unless the sensor is calibrated */

      if (!priv->dev.calibrated)
        {
          sninfo("The sensor is not calibrated!\n");
        }

      lis3mdl_read_measurement_data(priv, &data);
      data.timestamp = sensor_get_timestamp();

      
      deliver_data(priv, &data);
       

    thread_sleep:
      nxsig_usleep(CONFIG_SENSORS_LIS3MDL_POLL_INTERVAL);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lis3mdl_register
 *
 * Description:
 *   Register the LIS3MDL MAG as a sensor device
 *
 * Input Parameters:
 *   devno   - The device number, used to bind the device path
 *             as /dev/uorb/sensor_mag_uncalN
 *   spi     - An instance of the SPI interface to use to communicate with
 *             LIS3MDL
 *   config  - configuration for the LIS3MDL driver. For details see
 *             description above.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lis3mdl_register(int devno, FAR struct spi_dev_s *spi,
                     FAR struct lis3mdl_config_s const *config)
{
  FAR struct lis3mdl_dev_s *priv;
  FAR struct sensor_lowerhalf_s *lower;
  int ret = OK;
  FAR char arg1[32];
  FAR char *argv[2];
  int i;

  /* Sanity check */

  DEBUGASSERT(spi != NULL);
  DEBUGASSERT(config != NULL);

  priv = kmm_zalloc(sizeof(struct lis3mdl_dev_s));
  if (priv == NULL)
  {
    snerr("ERROR: Failed to allocate instance\n");
    ret = -ENOMEM;
  }

  /* Initialize the LIS3MDL device structure */

  priv->spi = spi;
  priv->enabled = false;
  priv->config = config;
  nxmutex_init(&priv->dev_lock);
  nxsem_init(&priv->run, 0, 0);

  lower = &priv->dev.lower;
  lower->ops = &g_sensor_ops;
  lower->type = SENSOR_TYPE_MAGNETIC_FIELD;
  
  /* Setup SPI frequency and mode */

  SPI_SETFREQUENCY(spi, LIS3MDL_SPI_FREQUENCY);
  SPI_SETMODE(spi, LIS3MDL_SPI_MODE);

  ret = lis3mdl_checkid(priv);
  if(ret < 0)
  {
    snerr("ERROR: Wrong device ID\n");
    goto err_init;
  }

  ret = sensor_register(lower, devno);
  if (ret < 0)
  {
    snerr("Error: Failed to register mag driver, lis3mdl (err = %d)\n",
          ret);
    goto err_init;
  }

/* Create a thread from polling sensor data */

snprintf(arg1, 16, "%p", priv);
argv[0] = arg1;
argv[1] = NULL;
ret = kthread_create("lis3mdl_thread", SCHED_PRIORITY_DEFAULT,
                     CONFIG_SENSORS_LIS3MDL_THREAD_STACKSIZE,
                     lis3mdl_thread, argv);
if (ret < 0)
{
  snerr("ERROR: Failed to gcreated a pill thread (err = %d)\n", ret);
  goto err_register;
}

sninfo("LIS3MDL driver loaded successfully\n");
return OK;

err_register:
  sensor_unregister(&priv->dev.lower, devno);

err_init:
  nxsem_destroy(&priv->run);
  nxmutex_destroy(&priv->dev_lock);
  kmm_free(priv);
  return ret;
}

#endif /* CONFIG_SPI && CONFIG_SENSORS_LIS3MDL */
