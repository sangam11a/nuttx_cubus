/****************************************************************************
 * drivers/analog/ads7953.c
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

#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <string.h>
#include <fixedmath.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/analog/ads7953.h>
#include <nuttx/mutex.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADS7953_SPI_MODE       (SPIDEV_MODE0)
#define CONFIG_ADS7953_SPI_FREQUENCY    1000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ads7953_dev_s
{
  FAR const struct adc_callback_s *cb;
  FAR struct spi_dev_s *spi;
  int spidev;
  unsigned int devno;
  FAR struct ads7953_channel_config_s *channel_config;
  int channel_config_count;
  mutex_t lock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Standard character drivers */

static int ads7953_open(FAR struct file *filep);
static ssize_t ads7953_read(FAR struct file *filep, FAR char *buffer,size_t buflen);
static ssize_t ads7953_write(FAR struct file *filep, FAR const char *buffer,size_t buflen);
static int ads7953_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/* functions used to actually interact with the hardware (SPI) */

static void ads7953_write16(FAR struct ads7953_dev_s *priv, uint8_t *cmd);
static void ads7953_read16(FAR struct ads7953_dev_s *priv, uint8_t *cmd, uint8_t *data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is for upper level file operation to access from custom applications through standard read/write/open/ioctl methods*/

static const struct file_operations g_ads7953ops =
{
    ads7953_open,   /* open */
    NULL,           /* close not used */
    ads7953_read,   /* read */
    ads7953_write,  /* write */
    NULL,           /* seek not used */
    ads7953_ioctl,  /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Name: ads7953_configspi
 * 
 * Description:
 *    SPI configuration function to configure spi mode, frequency and everything. Only minimum configurations have been done; 
 *    can add other configurations or change existing configurations as per need.
 * 
 * Input Parameters:
 *    *spi -- spi device pointer
 * 
 * Return Value:
 *    None
 * 
 ****************************************************************************/

static inline void ads7953_configspi(FAR struct spi_dev_s *spi){
    SPI_SETMODE(spi, ADS7953_SPI_MODE);
    SPI_SETBITS(spi, 8);
    SPI_HWFEATURES(spi, 0);
    SPI_SETFREQUENCY(spi, CONFIG_ADS7953_SPI_FREQUENCY);
}

/****************************************************************************
 * Name: ads7953_write16
 * 
 * Description:
 *    this is the actual function that sends and receives data to and from ads7953 through SPI
 *    it can be called by either ioctl commands or write() file operations function
 * 
 * Input Parameters:
 *    *priv -- ads7953 structure pointer  
 *    *cmd --  command pointer to send data (2 bytes data)
 * 
 ****************************************************************************/

static void ads7953_write16(FAR struct ads7953_dev_s *priv, uint8_t *cmd){

  /* if preemption is enabled and IRQ is required, use enter_critical_section() function */

  SPI_LOCK(priv->spi, true);    //if thread lock then it is required, otherwise not

  ads7953_configspi(priv->spi); //configure SPI, set speed, type, HW features etc.

  SPI_SELECT(priv->spi, priv->spidev, true);  //selects the SPI to transfer data (including setting CS pin)
  SPI_EXCHANGE(priv->spi, cmd, NULL, 2);
  SPI_SELECT(priv->spi, priv->spidev, false);   //deselect the SPI after use

  SPI_LOCK(priv->spi, false);     //unlock the thread of spi2
  return;
}

/****************************************************************************
 * Name: ads7953_read16
 * 
 * Description:
 *    
 ****************************************************************************/

static void ads7953_read16(FAR struct ads7953_dev_s *priv, uint8_t *cmd, uint8_t *data){
  
  SPI_LOCK(priv->spi, true);
  ads7953_configspi(priv->spi);
  SPI_SELECT(priv->spi, priv->spidev, true);
  SPI_EXCHANGE(priv->spi,cmd, data, 2);
  SPI_SELECT(priv->spi, priv->spidev, false);
  SPI_LOCK(priv->spi, false);
  return;
}

/****************************************************************************
 * Name: ads7953_open
 * 
 * Description:
 *    character driver function to open the ADC device file to read/write data to and from external ADC
 *    This function is called whenever the ads7953 device driver is opened
 *    passes the file pointers to configure device
 *    
 * Input Parameters:
 *    *filep - file pointer structure
 * 
 * Return value:
 *    OK
 ****************************************************************************/

static int ads7953_open(FAR struct file *filep)
{
  ainfo("Opening the ads7953 file");
  FAR struct inode *inode = filep->f_inode;
  FAR struct ads7953_dev_s *priv = inode->i_private;
  ainfo("Opened the ADS7953 file \n");
  return OK;
}

/****************************************************************************
 * Name: ads7953_read
 * 
 * Description:
 *    executes standard read operation. 
 * 
 * Input Parameters:
 *    *filep -- pointer to the file of device which needs to be operated
 *    *buffer -- character pointer to store received data
 *    * buflem -- no of bytes to receive
 * 
 * Return Value:
 *    sizeof data read from ADS7953
 ****************************************************************************/

static ssize_t ads7953_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ads7953_dev_s *priv = inode->i_private;
  uint8_t temp[2] = {'\0'};
  uint8_t *buf = '\0';
  struct ads7953_data_config_s adc_data_struct[MAX_ADC_CHANNELS];
  temp[0] = AUTO_2_MODE2_1;
  temp[1] = AUTO_2_MODE2_2;
  // for(int i=0;i<8;i++){
  ads7953_read16(priv, temp, buf);
  // }
  ainfo("Buffer data: %d size: %d \n",*buf, sizeof(*buf));
  itoa(*buf,buffer,2);
  return sizeof(*buf);
}

/****************************************************************************
 * Name: ads7953_write
 * 
 * Description:
 *    standard file operation function to write data to the driver through write() method.
 * 
 * Input Parameters:
 *    *filep -- file pointer of device
 *    *buffer -- character pointer (command which needs to be written)
 *    *buflen -- size of command to send (not used since every command is of 2 bytes for ADS7953)
 * 
 * Return value:
 *    OK -- write operation successful
 ****************************************************************************/

static ssize_t ads7953_write(FAR struct file *filep, FAR const char *buffer,
                        size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ads7953_dev_s *priv = inode->i_private;
  uint16_t tempval = atoi(buffer);
  uint8_t cmd[2] = {'\0'};
  cmd[0] = tempval >> 8;
  cmd[1] = tempval;
  ads7953_write16(priv, cmd); //maybe mutex is required but the actual use of it is not well understood so skip for now
  return OK;
}


/****************************************************************************
 * Name: ads7953_ioctl
 * 
 * Description: 
 *   ioctl/special commands to allow application layer to access the hardware directly without using the read()/write() commands. 
 *   External ADC needs to use IOCTL commands because it needs to write into registers directly    
 * 
 * Input Parameters:
 *   Described below: 
 * 
 * Return Value:
 *   Zero on success; non-zero value on failure
 * 
 * Note: There are two ways to read ADC data: through IOCTL commands and by read/write methods. Figure out which one is better for our use.
 *        IOCTL gives more control while read/write methods are more streamlined with POSIX standard
 ****************************************************************************/

static int ads7953_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ads7953_dev_s *priv = inode->i_private;
  int ret = OK;
  uint8_t temp[2] = {'\0'};
  switch (cmd)
    {
      /* switch to manual select mode; arg: none*/
      case ANIOC_ADC_MANUAL_SELECT:
        {
          temp[0]  = MANUAL_MODE_1;
          temp[1] = MANUAL_MODE_2;
          ads7953_write16(priv, temp);
          ainfo("Manual Mode set: %x \n", temp[0] << 8 | temp[1]);
        }
        break;

      /* send the ADC2 auto select command; (refer to datasheet if confusion)*/
      case ANIOC_ADC_AUTO_2_SELECT:
        {
          temp[0] = AUTO_2_MODE_1; 
          temp[1] = AUTO_2_MODE_2;
          ads7953_write16(priv, temp);
          ainfo("Auto 2 Mode set: %x \n", temp[0] << 8 | temp[1]);
        }
        break;
      
      /* send the ADC2 auto program command */
      case ANIOC_ADC_AUTO_2_PROGRAM:
        {
          temp[0]  = ADC_AUTO_2_PROGRAM2_1;
          temp[1] = ADC_AUTO_2_PROGRAM2_2;
          ads7953_write16(priv, temp);
          ainfo("Auto 2 Program mode set: %x \n", temp[0] << 8 | temp[1]);
        }
        break;

      /* send the ADC2 select read command (perform auto_2_select, auto_2_program and run this command to get the channel data;
      * keep calling this ioctl to get new channel data every time)  */
      case ANIOC_ADC_AUTO_2_SELECT_READ:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          temp[0]  = AUTO_2_MODE2_1;
          temp[1] = AUTO_2_MODE2_2;
          ads7953_read16(priv, temp, ptr);
          ainfo("Auto 2 Read Mode: %x \n", temp[0] << 8 | temp[1]);
          ainfo("Data from read function: %d \n", *ptr);
        }
        break;

      /* wrong ioctl command */
      default:
        ainfo("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ads7953_register
 *
 * Description:
 *   Register the ADS7953 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/ext_adc1"
 *   spi - An instance of the SPI bus to use to communicate with ADS7953
 *   spidev - The SPI device number used to select the correct CS line
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
int ads7953_register(FAR const char *devpath,
                     FAR struct spi_dev_s *spi, int spidev)
{
  FAR struct ads7953_dev_s *priv = {'\0'};
  int ret;

  /* Sanity check */
  DEBUGASSERT(spi != NULL);

  /* Initialize the ADS7953 device structure */
  nxmutex_init(&priv->lock);
  priv = kmm_malloc(sizeof(struct ads7953_dev_s));
  if (priv == NULL)
    {
      aerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi        = spi;
  priv->spidev     = spidev;

  /* Register the character driver */
  ret = register_driver(devpath, &g_ads7953ops, 0666, priv);

  if (ret < 0)
    {
      aerr("ERROR: Failed to register driver: %d\n", ret);
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
    }
  return OK;
}