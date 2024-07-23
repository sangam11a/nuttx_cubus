/****************************************************************************
 * apps/examples/spi_test/spi_test_main.c
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

#include "cubus_app_main.h"
// #include<sys/ddi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/progmem.h>
#include <fcntl.h>
#include <stdio.h>
#include <nuttx/irq.h>
#include <stdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <time.h>

#include "com_app_main.h"
#include "gpio_definitions.h"

static int COM_TASK(int argc, char *argv[]);

uint8_t data[7] = {0x53, 0x01, 0x02, 0x03, 0x04, 0x7e};
uint8_t Msn_Start_Cmd[7] = {0x53, 0xad, 0xba, 0xcd, 0x9e, 0x7e};
uint8_t ACK[7] = {0x53, 0xaa, 0xcc, 0xaa, 0xcc, 0x7e};
uint8_t NACK[7] = {0x53, 0xee, 0xff, 0xee, 0xff, 0x7e};

uint8_t RX_DATA_EPDM[48] = {'\0'};
int Execute_EPDM();

#define BEACON_DELAY 90
#define BEACON_DATA_SIZE 85
#define ACK_DATA_SIZE 6 + 1
#define COM_RX_CMD_SIZE 29
#define COM_DG_MSG_SIZE 30
uint8_t beacon_status = 0;

uint8_t COM_BUSY = 0;
static struct work_s work_beacon;
uint8_t beacon_type = 0;

int handshake_COM(uint8_t *ack);
int handshake_MSN(uint8_t subsystem, uint8_t *ack);

/****************************************************************************
 * Send data from UART through any UART path
 ****************************************************************************/
int send_data_uart(char *dev_path, uint8_t *data, uint16_t size)
{
  double fd;
  int i;
  int count = 0, ret;
  printf("Opening uart dev path : %s\n", dev_path);
  fd = open(dev_path, O_WRONLY);

  if (fd < 0)
  {
    printf("error opening %s\n", dev_path);
    return fd;
  }
  int wr1 = write(fd, data, size);
  if (wr1 < 0)
  {
    printf("Unable to write data\n");
    return wr1;
  }
  printf("\n%d bytes written\n", wr1);
  ioctl(fd, TCFLSH, 2);
  printf("flused tx rx buffer\n");
  ioctl(fd, TCDRN, NULL);
  printf("drained tx rx buffer\n");
  close(fd);
  return wr1;
}

/****************************************************************************
 * Receive data from UART
 ****************************************************************************/
int receive_data_uart(char *dev_path, uint8_t *data, uint16_t size)
{
  int fd, ret;
  fd = open(dev_path, O_RDONLY);
  // ioctl(fd, TCFLSH, 2);    //check if we can receive data without flushing initially
  // ioctl(fd, TCDRN, NULL);
  // printf("drained and flushed tx rx buffer\n");
  if (fd < 0)
  {
    printf("Unable to open %s\n", dev_path);
    return fd;
  }

  printf("size of data to receive: %d\n", size);
  // ret = read(fd, data, sizeof(data)); //try receiving data without byte by byte method
  for (int i = 0; i < size; i++)
  {
    ret = read(fd, &data[i], 1);
  }
  if (ret < 0)
  {
    printf("data Not received from %s\n", dev_path);
    return ret;
  }
  printf("Data received from %s\n", dev_path);
  for (int i = 0; i < size; i++)
  {
    printf("%x ", data[i]);
  }
  printf("\n");
  ioctl(fd, TCFLSH, 2);
  ioctl(fd, TCDRN, NULL);
  printf("drained and flushed tx rx buffer\n");
  close(fd);
  return ret;
}

/****************************************************************************
 * Send beacon data to COM
 * To be done every 90 seconds
 ****************************************************************************/
int send_beacon_data()
{
  work_queue(HPWORK, &work_beacon, send_beacon_data, NULL, SEC2TICK(BEACON_DELAY));
  if (COM_BUSY == 1)
  {
    return -1;
  }
  uint8_t beacon_data[BEACON_DATA_SIZE];
  switch (beacon_type)
  {
  case 0:
    
    serialize_beacon_a(beacon_data);

    beacon_data[1] = 0xb1;
    beacon_data[2] = 0x51;
    beacon_data[83] = 0x7e;
    break;
  case 1:
    serialize_beacon_b(beacon_data);
    beacon_data[1] = 0xb2;
    beacon_data[2] = 0x51;
    break;
  default:
    printf("wrong case selected\n");
    return -1;
    break;
  }
    beacon_data[0] = 0x53;
    beacon_data[83] = 0x7e;


  beacon_data[84] = '0';
  int fd = open(COM_UART, O_WRONLY);
  if (fd < 0)
  {
    printf("unable to open: %s\n", COM_UART);
    return -1;
  }
  printf("Turning on COM 4V line..\n");
  gpio_write(GPIO_COM_4V_EN, 1);
  int ret = write(fd, beacon_data, BEACON_DATA_SIZE);
  usleep(10000);
  if (ret < 0)
  {
    printf("unable to send data\n");
    for (int i = 0; i < BEACON_DATA_SIZE; i++)
    {
      ret = write(fd, &beacon_data[i], 1);
      usleep(1000);
    }
    if (ret < 0)
    {
      printf("Unable to send data through byte method..\n");
      return -1;
    }
  }
  /*To delete*/
  if(beacon_status == 0){
    printf("\nbeacon 1:\n");
  }
  else{
    printf("\nbeacon 2:\n");

  }
  beacon_type = !beacon_type;

  for (int i = 0; i < 83; i++)
  {
    {
      printf("%d ", beacon_data[i]);
    }
  }
  printf("\n");
  /*To delete*/

  usleep(1000 * 1000 * 3); // 3 seconds
  gpio_write(GPIO_COM_4V_EN, 0);
  ioctl(fd, TCFLSH, 2);
  ioctl(fd, TCDRN, 2);
  printf("TX RX buffer flused\n");
  close(fd);
  printf("Turned off COM 4V line..\n");
  printf("Beacon Type %d sequence complete\n", beacon_type);
  return 0;
}

/****************************************************************************
 * COM RX telecommands
 *
 * COM works this way:
 * telecommand receive
 * ack send
 * execute command
 *
 * Useful commands (12 bytes)
 *  0:      MCU ID
 *  1-3:    main command
 *  4-5:    reservation table commands
 *  6-9:    address data (if data is being downloaded)
 *  10-11:  no of packets (if data is being downloaded)
 ****************************************************************************/
int receive_telecommand_rx(uint8_t *COM_RX_DATA)
{
    uint8_t useful_command[12];
    uint8_t main_cmd[3] = {'\0'};
    uint16_t rsv_table = 0;
    uint32_t address = 0;
    uint16_t pckt_no = 0;
    uint8_t ack[85] = {0x53, 0xac, 0x04, 0x01, 0x62, 0x63, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x70, 0x71, 0x72, 0x7e, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x80, 0x7e};
    printf("waiting for telecommands from COM\n");
    int ret = receive_data_uart(COM_UART, COM_RX_DATA, COM_RX_CMD_SIZE); // telecommand receive
    if (ret < 0)
    {
        printf("data not received from COM\n NACK sending\n");
        send_data_uart(COM_UART, NACK, 7);
        printf("data not received from COM\n NACK 1s55 ent\n");
        return ret;
    }
    else
    { if (COM_RX_DATA[16] == 0x02 |COM_RX_DATA[17] == 0x02 ){
      digipeater_mode(COM_RX_DATA);
      return 29;
    }
        // for (int i = 0; i < BEACON_DATA_SIZE; i++)
        // {
        //     send_data_uart(COM_UART, ack[i], 1);
        //     printf("%02x ",ack[i]);
        // }
    }
    printf("value of ret is %d\ndata received from COM\n sending ACK\n", ret);
    sleep(3);

    // ret = send_data_uart(COM_UART, ack, BEACON_DATA_SIZE);

    // ret = send_data_uart(COM_UART, ACK, 7); // ack send

    // print_rx_telecommand(COM_RX_DATA); // printing the received telecommand
    for (int i = 0; i < 12; i++)
    {
        useful_command[i] = COM_RX_DATA[i + 8]; // extracting useful commands
    }
    if (useful_command[6] != 0xff || useful_command[6] != 0x00 | useful_command[7] != 0xff || useful_command[7] != 0x00)
    {
        printf("Reservation command received\n"); // if reservation command is received then store the reservation command (do not execute)
        // send_data_uart(COM_UART, ack, BEACON_DATA_SIZE);
        int fd = open(COM_UART, O_WRONLY);
        if (fd < 0)
        {
            printf("unable to open: %s\n", COM_UART);
            return -1;
        }
        int ret = write(fd, ack, BEACON_DATA_SIZE);
        for(int i=0;i<85;i++){
            printf("%02x ", ack[i]);
        }
        close(fd);

        printf("\nACK sent success\n******Sleeping*******\n");
        sleep(2);
        ack[0] = 0x53;
        ack[1] = 0x0e;
        ack[2] = 0x51;
        for (int i = 3; i < 83; i++)
        {
            ack[i] = i;
        }
        ack[83] = 0x7e;
        fd = open(COM_UART, O_WRONLY);
        if (fd < 0)
        {
            printf("unable to open: %s\n", COM_UART);
            return -1;
        }
        ret = write(fd, ack, BEACON_DATA_SIZE);
        for(int i=0;i<85;i++){
                printf("%02x ", ack[i]);
            }
        close(fd);
        // send_data_uart(COM_UART, ack, BEACON_DATA_SIZE);
        // send_beacon_data();
        printf("\n EPDM data sent success\n ******Sleeping *******\n ");
        sleep(3);
    }
    else
    {
        switch (useful_command[0])
        {
        case 1:
            printf("command received for OBC\n");
            OBC_CMD_EXE(useful_command);
            break;
        case 2:
            printf("command received for CAM\n");
            break;
        default:
            printf("unknown command\n");
            break;
        }
    }
    return ret;
}


int OBC_CMD_EXE(uint8_t *cmd)
{
  printf("inside obc command execution ...\n");
}

int print_rx_telecommand(uint8_t *data)
{
  printf("Command from COM: \n");
  printf("Source call sign: ");
  for (int i = 0; i < 7; i++)
  {
    printf("%x", data[i]);
  }
  printf("\nSATELLITE ID: %d \n", data[7]);
  printf("Main command:");
  for (int i = 8; i < 12; i++)
  {
    printf("%x", data[i]);
  }
  printf("\n");
  printf("Reservation Table: %x\n", data[12] << 8 | data[13]);
  printf("Address data: %x \n", data[14] << 24 | data[15] << 16 | data[16] << 8 | data[17]);
  printf("No of packets: %x\n", data[18] << 8 | data[19]);
}
/****************************************************************************
 * COM RX telecommands parse
 ****************************************************************************/
int parse_telecommand(uint8_t *rx_data)
{
  printf("Data redirected to EPDM\n");
  // TODO: parse the command and execute
  // for now only epdm is executed
  if (Execute_EPDM() == 0)
  {
    printf("sending EPDM data to COM\n");
    send_data_uart(COM_UART, RX_DATA_EPDM, sizeof(RX_DATA_EPDM));
  }
}

/****************************************************************************
 * COM TASK task
 *
 * COM will be in digipeater mode till a digipeating message is received and digipeated
 ****************************************************************************/
void digipeater_mode(uint8_t *data)
{
  receive_data_uart(COM_UART, data, 29);
  for (int i = 29; i < 84; i++)
  {
    data[i] = 0xff;
  }
  // send_data_uart(COM_UART, data, 84);
  /*To delete*/
  if (send_data_uart(COM_UART, data, 84) > 0)
  {
    printf("digipeating data is \n ");
    for(int i =0 ;i<85;i++)
    printf("%02x ", data);
    printf("digipeating successful\n :"); 
  }
  /*To delete*/
}

/****************************************************************************
 * COM TASK task
 ****************************************************************************/
static int COM_TASK(int argc, char *argv[])
{
  int ret = -1;
  uint8_t rx_data[COM_RX_CMD_SIZE] = {'\0'};
  printf("Turning on COM MSN...\n");
  gpio_write(GPIO_3V3_COM_EN, 1);
  usleep(2000000);
  ret = handshake_COM(data); // tx rx data is flushed before closing the file
  usleep(PRINT_DELAY * 100);
  if (ret == 0)
  {
    printf("Successful handshake with COM\n");
  }
  if (ret != 0)
  {
    printf("Unable to handshake with COM\n");
  }
  usleep(1000);

  send_beacon_data();
  printf("Beacon 1 sent...\n");
  printf("Going to receiver mode...\n");
  receive_telecommand_rx(rx_data);
  sleep(10);
  send_beacon_data();
  printf("Beacon 2 sent...\n");
  receive_telecommand_rx(rx_data);
  sleep(2);
  printf("Startign digipeating mode:\n");
  // digipeater_mode(rx_data);
  for (;;)
  {
    receive_telecommand_rx(rx_data);
    usleep(1000);
  }
}

/****************************************************************************
 * COM handshake function
 ****************************************************************************/
int handshake_COM(uint8_t *ack)
{
  double fd;
  uint8_t data1[ACK_DATA_SIZE] = {'\0'};
  int i;
  int count = 0, ret;
  printf("Opening uart dev path : %s ret : %d", COM_UART, fd);
  usleep(PRINT_DELAY);
  fd = open(COM_UART, O_RDWR);
  if (fd < 0)
  {
    printf("error opening %s\n", COM_UART);
    usleep(PRINT_DELAY);
    return -1;
  }

  int wr1 = write(fd, data, ACK_DATA_SIZE); // writing handshake data
  if (wr1 < 0)
  {
    printf("Unable to send data through %d UART", COM_UART);
    usleep(PRINT_DELAY);
    return -1;
  }
  printf("\n%d bytes written\n", wr1);
  usleep(PRINT_DELAY);
  // ret = read(fd, data1, 10);   //try reading data from UART at once as well
  for (i = 0; i < ACK_DATA_SIZE; i++)
  {
    ret = read(fd, &data1[i], 1);
  }
  printf("data received from %s \n", COM_UART);
  usleep(PRINT_DELAY);
  for (int i = 0; i < ACK_DATA_SIZE; i++)
  {
    printf(" %x ", data1[i]);
  }
  printf("\n");
  usleep(PRINT_DELAY);
  if (data[0] == data1[0] && data[ACK_DATA_SIZE - 2] == data1[ACK_DATA_SIZE - 2])
  {
    printf("\n******Acknowledgement received******\n");
    usleep(PRINT_DELAY);
  }
  printf("handshake complete\n");
  printf("\n");
  ioctl(fd, TCFLSH, 2);
  ioctl(fd, TCDRN, NULL);
  printf("flused tx rx buffer\n");
  close(fd);
  return 0;
}

/****************************************************************************
 * MSN handshake function
 ****************************************************************************/
int handshake_MSN(uint8_t subsystem, uint8_t *ack)
{
  int fd;
  char devpath[15];
  uint8_t data1[7] = {'\0'};
  int i;
  int count = 0, ret;

  switch (subsystem)
  {
  case 1:
    strcpy(devpath, ADCS_UART);
    // gpio_write(GPIO_MSN1_EN, 1);// TODO uncomment later
    printf("Turned on power line for ADCS\n");
    break;
  case 2:
    strcpy(devpath, CAM_UART);
    // gpio_write(GPIO_MSN2_EN, 1);
    printf("Turned on power line for CAM\n");
    break;
  case 3:
    strcpy(devpath, EPDM_UART);
    // gpio_write(GPIO_BURNER_EN, 1);
    printf("Turned on power line for EPDM\n");
    break;
  default:
    printf("Unknown MSN subsystem selected\n");
    return -1;
    break;
  }

  printf("Opening uart dev path : %s \n", devpath);
  usleep(PRINT_DELAY);
  fd = open(devpath, O_RDWR);
  if (fd < 0)
  {
    printf("error opening %s\n", devpath);
    usleep(PRINT_DELAY);
    return -1;
  }

  int wr1 = write(fd, data, 6); // writing handshake data
  if (wr1 < 0)
  {
    printf("Unable to send data through %d UART", devpath);
    // usleep(PRINT_DELAY);
    return -1;
  }
  printf("\n%d bytes written\n", wr1);
  usleep(1000 * 3000);
  // ret = read(fd, data1, 7);
  for (i = 0; i < 6; i++)
  {
    ret = read(fd, &data1[i], 1);
  }
  printf("data received from %s \n", devpath);
  usleep(PRINT_DELAY);
  for (int i = 0; i < 7; i++)
  {
    printf(" %x ", data1[i]);
  }
  printf("\n");
  usleep(PRINT_DELAY);
  if (data[0] == data1[0] && data[5] == data1[5])
  {
    printf("\n******Acknowledgement received******\n");
    usleep(PRINT_DELAY);
  }
  printf("handshake complete\n");
  usleep(PRINT_DELAY);
  printf("\n");
  ioctl(fd, TCFLSH, 2);
  ioctl(fd, TCDRN, NULL);
  printf("flused tx rx buffer\n");
  close(fd);
  return 0;
}

/****************************************************************************
 * execute EPDM mission function
 ****************************************************************************/
int Execute_EPDM()
{
  int handshake_success = -1;
  // TODO uncomment later
  // gpio_write(GPIO_DCDC_MSN_3V3_2_EN, 1);
  // printf("MSNN 3v3 dc dc enabled \n");
  // usleep(1000 * 1000);

  // gpio_write(GPIO_MSN_3V3_EN, 1);
  // usleep(1000 * 1000);
  // printf("MSNN 3v3 dc dc enabled \n");
// TODO uncomment later
  for (int i = 0; i < 3; i++)
  {
    gpio_write(GPIO_BURNER_EN, 0); // Antenna deployment here
    usleep(1000 * 1000);
    handshake_success = handshake_MSN(3, data);
    if (handshake_success == 0)
    {
      break;
    }
  }
  if (handshake_success != 0)
  {
    printf("Handshake failure 3 times\n aborting EPDM mission execution\n");
    return -1;
  }
  printf("handshake success...\n");
  if (send_data_uart(EPDM_UART, Msn_Start_Cmd, 7) < 0)
  {
    printf("Unable to send Msn start command to EPDM\n Aborting mission..\n");
    return -1;
  }
  if (receive_data_uart(EPDM_UART, RX_DATA_EPDM, 48) >= 0)
  {
    printf("Data received from EPDM mission\n");
    send_data_uart(EPDM_UART, RX_DATA_EPDM, BEACON_DATA_SIZE);
  }
  // TODO uncomment later
  // gpio_write(GPIO_MSN3_EN, 0);
  // gpio_write(GPIO_DCDC_MSN_3V3_2_EN, 0);
  // gpio_write(GPIO_MSN_3V3_EN, 0);

// TODO uncomment later

  printf("EPDM Mission complete\n");
  return 0;
}

int turn_msn_on_off(uint8_t subsystem, uint8_t state)
{
  switch (subsystem)
  {
  case 1:
    printf("turning ADCS mission state: %d\n", state);
    // gpio_write(GPIO_MSN1_EN, state);
    break;
  case 2:
    printf("Turning CAM mission state: %d\n", state);
    // gpio_write(GPIO_MSN2_EN, state);
    break;
  case 3:
    printf("Turning EPDM mission state: %d\n", state);
    // gpio_write(GPIO_MSN3_EN, state);
    break;
  default:
    printf("Wrong subsystem selected\n");
    break;
  }
}
// #include

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
extern ext_adc_s ext_adc_data[EXT_ADC_MAX_CHANNELS];
extern CRITICAL_FLAGS critic_flags;

CRITICAL_FLAGS test_flags;

struct sensor_accel imu_acc_data;
struct sensor_gyro imu_gyro_data;
struct mpu6500_imu_msg raw_imu;

char buffer[255] = {'\0'};

satellite_health_s sat_health = {'\0'};
satellite_health_s sat_health_rx_buf = {'\0'};

S2S_BEACON_A s2s_beacon_type_a;
S2S_BEACON_TYPE_B s2s_beacon_type_b;

#define REG_LOW_MASK 0xFF00
#define REG_HIGH_MASK 0x00FF
#define MPU6050_FS_SEL 32.8f
#define MPU6050_AFS_SEL 4096.0f
#define DEVNAME_SIZE 32

// #define DELAY_ADC 4000
// #define ADC_DELAY 6000
#define HK_DELAY 90
#define ANT_DEP_DELAY 30 * 60
#define BEACON_DELAY 180

/*
Defing work structures for work_queue thread
*/
static struct work_s work_hk;
static struct work_s work_ant_dep;

void Antenna_Deployment();

/*
Declaring structure necessary for collecting HK data
*/

/****************************************************************************
 * Name: main
 ****************************************************************************/
int main(int argc, FAR char *argv[])
{

  Setup();
  RUN_HK();
  if (strcmp(argv[1], "com") == 0x00)
  {
    int retval = task_create("task1", 100, 1024, COM_TASK, NULL);
    if (retval < 0)
    {
      printf("unable to create COM task\n");
      return -1;
    }
  }
  printf("************************************************\n");

  //   if(critic_flags.ANT_DEP_STAT == UNDEPLOYED && critic_flags.UL_STATE == UL_NOT_RX){
  //     //TODO: add work queue to perform antenna deployment after 30 minutes
  //     work_queue(HPWORK, &work_ant_dep, Antenna_Deployment, NULL, SEC2TICK(ANT_DEP_DELAY));
  //   }else{
  //     printf("Antenna in Deployed State...\n Not entering antenna deployment sequence\n");
  //   }
  //   // work_queue(HPWORK, &work_hk, collect_hk, NULL, MSEC2TICK(HK_DELAY));
  // #if defined(CONFIG_CUSTOM_APPS_CUBUS_USE_EXT_ADC) || defined(CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC1) || defined(CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC3)
  //   RUN_HK();
  //   work_queue(HPWORK, &work_hk, RUN_HK, NULL, SEC2TICK(HK_DELAY));

  // #endif
  printf("************************************************\n");

  // TODO: after checking flags data are being written/read correctly, we'll enable satellite health things as well and have a basic complete work queue functions except UART

  return 0;
}

// TODO: add work queue to antenna deployment
void Antenna_Deployment()
{
  printf("Entering antenna deployment sequence\n");
  int retval, retval1 = 0;
  CRITICAL_FLAGS ant_check;
  ant_check.ANT_DEP_STAT = critic_flags.ANT_DEP_STAT;
  printf("Antenna Deployment Flag: %d\n", critic_flags.ANT_DEP_STAT);
  // TODO: add redundancy (check UL status along with antenna deployment status)
  if (critic_flags.ANT_DEP_STAT == UNDEPLOYED)
  {
    for (int i = 0; i < 3; i++)
    {
      printf("Turning on burner circut\nAttempt: %d\n", i + 1);
      retval = gpio_write1(GPIO_BURNER_EN, true);
      retval1 = gpio_write1(GPIO_UNREG_EN, true);
      RUN_ADC();
      usleep(1000 * 1000 * 6); // 6 seconds
      printf("Turning off burner circuit\n");
      gpio_write1(GPIO_UNREG_EN, false);
      gpio_write1(GPIO_BURNER_EN, false);
      usleep(1000 * 1000 * 2); // 2 seconds
    }
  }
  printf("Antenna deployment sequence complete\n");
  ant_check.ANT_DEP_STAT = DEPLOYED;
  store_flag_data(&ant_check);
  printf("Updated flag data...\n");
  check_flag_data();
  print_critical_flag_data(&critic_flags);
}

/****************************************************************************
 * Name: adc_main
 ****************************************************************************/

void RUN_HK()
{
  read_int_adc1(); // GET DATA FROM INTERNAL ADCs
  read_int_adc3();
  ext_adc_main();

  collect_imu_mag();

  make_satellite_health();
  store_sat_health_data(&sat_health);
  print_satellite_health_data(&sat_health);
  work_queue(HPWORK, &work_hk, RUN_HK, NULL, SEC2TICK(HK_DELAY));

  usleep(10000);
}

/*
 * this is to be used between any processes only...
 */
void RUN_ADC()
{
  // read_int_adc1();
  // read_int_adc3();
  ext_adc_main();
  make_satellite_health();
  store_sat_health_data(&sat_health);
  print_satellite_health_data(&sat_health);
}

void serialize_beacon_a(uint8_t beacon_data[BEACON_DATA_SIZE])
{
  for (int i = 0; i <= BEACON_DATA_SIZE; i++)
  {
    beacon_data[83] = 0x00;
    // }
    // beacon_data[84] = s2s_beacon_type_a.;
    // beacon_data[85] = s2s_beacon_type_a.;
  }
  // uint8_t beacon_data[BEACON_DATA_SIZE];
  beacon_data[0] = s2s_beacon_type_a.HEAD;
  beacon_data[1] = s2s_beacon_type_a.TYPE;
  beacon_data[2] = s2s_beacon_type_a.TIM_DAY;
  beacon_data[3] = s2s_beacon_type_a.TIM_HOUR;
  beacon_data[4] = (s2s_beacon_type_a.BAT_V >> 8) & 0Xff;
  beacon_data[5] = s2s_beacon_type_a.BAT_V & 0xff;
  beacon_data[6] = (s2s_beacon_type_a.BAT_C >> 8) & 0Xff;
  beacon_data[7] = (s2s_beacon_type_a.BAT_V) & 0Xff;
  beacon_data[8] = (s2s_beacon_type_a.BAT_T >> 8) & 0Xff;
  beacon_data[9] = (s2s_beacon_type_a.BAT_T) & 0Xff;
  beacon_data[10] = s2s_beacon_type_a.RAW_C;
  beacon_data[11] = (s2s_beacon_type_a.SOL_TOT_V >> 8) & 0Xff;
  beacon_data[12] = (s2s_beacon_type_a.SOL_TOT_V) & 0Xff;
  beacon_data[13] = (s2s_beacon_type_a.SOL_TOT_C >> 8) & 0Xff;
  beacon_data[14] = (s2s_beacon_type_a.SOL_TOT_C >> 8) & 0Xff;
  beacon_data[15] = s2s_beacon_type_a.ANT_P_T;
  beacon_data[16] = s2s_beacon_type_a.BPB_T;
  beacon_data[17] = s2s_beacon_type_a.OBC_T;
  beacon_data[18] = s2s_beacon_type_a.X_T;
  beacon_data[19] = s2s_beacon_type_a.X1_T;
  beacon_data[20] = s2s_beacon_type_a.Y_T;
  beacon_data[21] = s2s_beacon_type_a.Y1_T;
  // beacon_data[] = s2s_beacon_type_a.SOL_P5_T;
  beacon_data[22] = s2s_beacon_type_a.SOL_P1_STAT << 7 & s2s_beacon_type_a.SOL_P2_STAT << 6 & s2s_beacon_type_a.SOL_P3_STAT << 5 & s2s_beacon_type_a.SOL_P4_STAT << 4 & s2s_beacon_type_a.MSN1_STAT << 3 & s2s_beacon_type_a.MSN2_STAT << 2 & s2s_beacon_type_a.MSN3_STAT << 1 & 0xff;
  beacon_data[23] = s2s_beacon_type_a.ANT_STAT << 4 & s2s_beacon_type_a.UL_STAT << 4;
  beacon_data[24] = s2s_beacon_type_a.OPER_MODE;
  beacon_data[25] = (s2s_beacon_type_a.OBC_RESET_COUNT >> 8) & 0xff;
  beacon_data[26] = s2s_beacon_type_a.OBC_RESET_COUNT & 0xff;
  beacon_data[27] = s2s_beacon_type_a.RST_RESET_COUNT >> 8 & 0xff;
  beacon_data[28] = s2s_beacon_type_a.RST_RESET_COUNT & 0xff;
  beacon_data[29] = s2s_beacon_type_a.LAST_RESET;
  beacon_data[30] = s2s_beacon_type_a.CHK_CRC;
  // beacon_data[31] = s2s_beacon_type_a.;
  // beacon_data[32] = s2s_beacon_type_a.;
  // beacon_data[34] = s2s_beacon_type_a.;
  // beacon_data[35] = s2s_beacon_type_a.;
  // beacon_data[36] = s2s_beacon_type_a.;
  // beacon_data[37] = s2s_beacon_type_a.;
  // beacon_data[38] = s2s_beacon_type_a.;
  // beacon_data[39] = s2s_beacon_type_a.;
  // beacon_data[40] = s2s_beacon_type_a.;
  // beacon_data[41] = s2s_beacon_type_a.;
  // beacon_data[42] = s2s_beacon_type_a.;
  // beacon_data[43] = s2s_beacon_type_a.;
  // beacon_data[44] = s2s_beacon_type_a.;
  // beacon_data[45] = s2s_beacon_type_a.;
  // beacon_data[46] = s2s_beacon_type_a.;
  // beacon_data[47] = s2s_beacon_type_a.;
  // beacon_data[48] = s2s_beacon_type_a.;
  // beacon_data[49] = s2s_beacon_type_a.;
  // beacon_data[50] = s2s_beacon_type_a.;
  // beacon_data[51] = s2s_beacon_type_a.;
  // beacon_data[52] = s2s_beacon_type_a.;
  // beacon_data[53] = s2s_beacon_type_a.;
  // beacon_data[54] = s2s_beacon_type_a.;
  // beacon_data[55] = s2s_beacon_type_a.;
  // beacon_data[56] = s2s_beacon_type_a.;
  // beacon_data[57] = s2s_beacon_type_a.;
  // beacon_data[58] = s2s_beacon_type_a.;
  // beacon_data[59] = s2s_beacon_type_a.;
  // beacon_data[60] = s2s_beacon_type_a.;
  // beacon_data[61] = s2s_beacon_type_a.;
  // beacon_data[62] = s2s_beacon_type_a.;
  // beacon_data[63] = s2s_beacon_type_a.;
  // beacon_data[64] = s2s_beacon_type_a.;
  // beacon_data[65] = s2s_beacon_type_a.;
  // beacon_data[66] = s2s_beacon_type_a.;
  // beacon_data[67] = s2s_beacon_type_a.;
  // beacon_data[68] = s2s_beacon_type_a.;
  // beacon_data[69] = s2s_beacon_type_a.;
  // beacon_data[70] = s2s_beacon_type_a.;
  // beacon_data[71] = s2s_beacon_type_a.;
  // beacon_data[72] = s2s_beacon_type_a.;
  // beacon_data[73] = s2s_beacon_type_a.;
  // beacon_data[74] = s2s_beacon_type_a.;
  // beacon_data[75] = s2s_beacon_type_a.;
  // beacon_data[76] = s2s_beacon_type_a.;
  // beacon_data[77] = s2s_beacon_type_a.;
  // beacon_data[78] = s2s_beacon_type_a.;
  // beacon_data[79] = s2s_beacon_type_a.;
  // beacon_data[80] = s2s_beacon_type_a.;
  // beacon_data[81] = s2s_beacon_type_a.;
  // beacon_data[82] = s2s_beacon_type_a.;
  // beacon_data[83] = s2s_beacon_type_a.;
}
void serialize_beacon_b(uint8_t beacon_data[BEACON_DATA_SIZE])
{
  for (int i = 0; i <= BEACON_DATA_SIZE; i++)
  {
    beacon_data[83] = 0x00;
  }
  // uint8_t beacon_data[BEACON_DATA_SIZE];
  beacon_data[0] = s2s_beacon_type_b.HEAD;
  beacon_data[1] = s2s_beacon_type_b.TYPE;
  beacon_data[2] = s2s_beacon_type_b.TIM_DAY;
  beacon_data[3] = s2s_beacon_type_b.TIM_DAY;

  beacon_data[4] = s2s_beacon_type_b.SOL_P1_V;
  beacon_data[5] = s2s_beacon_type_b.SOL_P2_V;
  beacon_data[6] = s2s_beacon_type_b.SOL_P3_V;
  beacon_data[7] = s2s_beacon_type_b.SOL_P4_V;

  beacon_data[8] = s2s_beacon_type_b.SOL_P1_C;
  beacon_data[9] = s2s_beacon_type_b.SOL_P2_C;
  beacon_data[10] = s2s_beacon_type_b.SOL_P3_C;
  beacon_data[11] = s2s_beacon_type_b.SOL_P4_C;
  beacon_data[12] = (s2s_beacon_type_b.GYRO_X >> 8) & 0xff;
  beacon_data[13] = (s2s_beacon_type_b.GYRO_X) & 0xff;
  beacon_data[14] = (s2s_beacon_type_b.GYRO_Y >> 8) & 0xff;
  beacon_data[15] = (s2s_beacon_type_b.GYRO_Y) & 0xff;
  beacon_data[16] = s2s_beacon_type_b.GYRO_Z >> 8 & 0xff;
  beacon_data[17] = s2s_beacon_type_b.GYRO_Z & 0xff;

  beacon_data[18] = (s2s_beacon_type_b.ACCL_X >> 8) & 0xff;
  beacon_data[19] = (s2s_beacon_type_b.ACCL_X) & 0xff;
  beacon_data[20] = (s2s_beacon_type_b.ACCL_Y >> 8) & 0xff;
  beacon_data[21] = (s2s_beacon_type_b.ACCL_Y) & 0xff;
  beacon_data[22] = s2s_beacon_type_b.ACCL_Z >> 8 & 0xff;
  beacon_data[23] = s2s_beacon_type_b.ACCL_Z & 0xff;

  beacon_data[24] = (s2s_beacon_type_b.MAG_X >> 8) & 0xff;
  beacon_data[25] = (s2s_beacon_type_b.MAG_X) & 0xff;
  beacon_data[26] = (s2s_beacon_type_b.MAG_Y >> 8) & 0xff;
  beacon_data[27] = (s2s_beacon_type_b.MAG_Y) & 0xff;
  beacon_data[28] = s2s_beacon_type_b.MAG_Z >> 8 & 0xff;
  beacon_data[29] = s2s_beacon_type_b.MAG_Z & 0xff;
  beacon_data[30] = s2s_beacon_type_b.CHK_CRC;
}

void Make_Beacon_Data(uint8_t type)
{
  switch (type)
  {
  case 1:
    s2s_beacon_type_a.HEAD = 0x53;
    s2s_beacon_type_a.TYPE = 0;
    s2s_beacon_type_a.TIM_DAY = 01;
    s2s_beacon_type_a.TIM_HOUR = 01;
    s2s_beacon_type_a.BAT_V = sat_health.batt_volt;
    s2s_beacon_type_a.BAT_C = sat_health.batt_c;
    s2s_beacon_type_a.BAT_T = sat_health.temp_batt;

    s2s_beacon_type_a.RAW_C = sat_health.raw_c;
    s2s_beacon_type_a.SOL_TOT_V = sat_health.sol_t_v;
    s2s_beacon_type_a.SOL_TOT_C = sat_health.sol_t_c;

    s2s_beacon_type_a.BPB_T = sat_health.temp_bpb;
    s2s_beacon_type_a.OBC_T = sat_health.temp_obc;
    s2s_beacon_type_a.Y1_T = sat_health.temp_y1;
    s2s_beacon_type_a.Y_T = sat_health.temp_y;
    s2s_beacon_type_a.Z1_T = sat_health.temp_z1;
    s2s_beacon_type_a.Z_T = sat_health.temp_z;
    s2s_beacon_type_a.X1_T = sat_health.temp_x1;
    s2s_beacon_type_a.X_T = sat_health.temp_x;

    s2s_beacon_type_a.SOL_P1_STAT = sat_health.sol_p1_v >= 2 ? 1 : 0; // check from power, max power draw is 0.6 watt
    s2s_beacon_type_a.SOL_P2_STAT = sat_health.sol_p2_v >= 2 ? 1 : 0;
    s2s_beacon_type_a.SOL_P3_STAT = sat_health.sol_p3_v >= 2 ? 1 : 0;
    s2s_beacon_type_a.SOL_P4_STAT = sat_health.sol_p4_v >= 2 ? 1 : 0;

    s2s_beacon_type_a.ANT_STAT = critic_flags.ANT_DEP_STAT;
    s2s_beacon_type_a.KILL1_STAT = critic_flags.KILL_SWITCH_STAT;
    s2s_beacon_type_a.KILL2_STAT = critic_flags.KILL_SWITCH_STAT;
    s2s_beacon_type_a.UL_STAT = critic_flags.UL_STATE;

    // s2s_beacon_type_a.OBC_RESET_COUNT = ;  //TODO
    // s2s_beacon_type_a.LAST_RESET = ;       //TODO
    // s2s_beacon_type_a.CHK_CRC = ;          //TODO

  case 2:
    s2s_beacon_type_b.HEAD = 0x53;
    s2s_beacon_type_b.TYPE = 1;
    s2s_beacon_type_b.TIM_DAY = 01;

    s2s_beacon_type_b.SOL_P1_V = sat_health.sol_p1_v;
    s2s_beacon_type_b.SOL_P2_V = sat_health.sol_p2_v;
    s2s_beacon_type_b.SOL_P3_V = sat_health.sol_p3_v;
    s2s_beacon_type_b.SOL_P4_V = sat_health.sol_p4_v;

    s2s_beacon_type_b.SOL_P1_C = sat_health.sol_p1_c;
    s2s_beacon_type_b.SOL_P2_C = sat_health.sol_p2_c;
    s2s_beacon_type_b.SOL_P3_C = sat_health.sol_p3_c;
    s2s_beacon_type_b.SOL_P4_C = sat_health.sol_p4_c;

    s2s_beacon_type_b.MAG_X = (int16_t)sat_health.mag_x;
    s2s_beacon_type_b.MAG_Y = (int16_t)sat_health.mag_y;
    s2s_beacon_type_b.MAG_Z = (int16_t)sat_health.mag_z;

    s2s_beacon_type_b.GYRO_X = (int16_t)sat_health.gyro_x;
    s2s_beacon_type_b.GYRO_Y = (int16_t)sat_health.gyro_y;
    s2s_beacon_type_b.GYRO_Z = (int16_t)sat_health.gyro_z;

    s2s_beacon_type_b.ACCL_X = (int16_t)sat_health.accl_x;
    s2s_beacon_type_b.ACCL_Y = (int16_t)sat_health.accl_y;
    s2s_beacon_type_b.ACCL_Z = (int16_t)sat_health.accl_z;

    s2s_beacon_type_b.MAG_X = (int16_t)sat_health.mag_x;
    s2s_beacon_type_b.MAG_Y = (int16_t)sat_health.mag_y;
    s2s_beacon_type_b.MAG_Z = (int16_t)sat_health.mag_z;

    // s2s_beacon_type_b.CHK_CRC = ;  //TODO
  }
}

/****************************************************************************
 * Name: adc_main
 ****************************************************************************/

void make_satellite_health()
{

  float int_adc1_temp[CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_GROUPSIZE] = {'\0'};
  int_adc1_data_convert(int_adc1_temp);

  float int_adc3_temp[CONFIG_CUSTOM_APPS_CUBUS_INT_ADC3_GROUPSIZE] = {'\0'};
  int_adc3_data_convert(int_adc3_temp);

  /* External ADC data */
  sat_health.sol_t_v = (int16_t)ext_adc_data[0].processed_data;
  sat_health.raw_v = (int16_t)ext_adc_data[1].processed_data;
  sat_health.sol_p5_v = (int16_t)ext_adc_data[2].processed_data;
  sat_health.sol_p4_v = (int16_t)ext_adc_data[3].processed_data;
  sat_health.sol_p3_v = (int16_t)ext_adc_data[4].processed_data;
  sat_health.sol_p1_v = (int16_t)ext_adc_data[5].processed_data;
  sat_health.sol_p2_v = (int16_t)ext_adc_data[6].processed_data;

  sat_health.ant_temp_out = (int16_t)ext_adc_data[8].processed_data;
  // sat_health.temp_batt = (int16_t)ext_adc_data[9].processed_data;
  sat_health.temp_bpb = (int16_t)ext_adc_data[10].processed_data;
  sat_health.temp_z = (int16_t)ext_adc_data[11].processed_data;

  /* Internal ADC1 data */
  sat_health.batt_c = (int16_t)int_adc1_temp[9];
  sat_health.sol_t_c = (int16_t)int_adc1_temp[10];
  sat_health.raw_c = (int16_t)int_adc1_temp[11];

  sat_health.unreg_c = (int16_t)int_adc1_temp[0];
  sat_health.v3_main_c = (int16_t)int_adc1_temp[1];
  sat_health.v3_com_c = (int16_t)int_adc1_temp[2];
  sat_health.v5_c = (int16_t)int_adc1_temp[3];

  sat_health.batt_volt = (int16_t)int_adc1_temp[4];

  sat_health.sol_p1_c = (int16_t)int_adc1_temp[5];
  sat_health.v3_2_c = (int16_t)int_adc1_temp[6];
  sat_health.sol_p4_c = (int16_t)int_adc1_temp[7];
  sat_health.sol_p5_c = (int16_t)int_adc1_temp[8];

  sat_health.sol_p2_c = (int16_t)int_adc1_temp[12];
  sat_health.sol_p3_c = (int16_t)int_adc1_temp[13];

  /* internal adc2 data*/
  sat_health.v4_c = (int16_t)int_adc3_temp[0];
}

void collect_imu_mag()
{
  float acq_period = CONFIG_EXAMPLES_SENSOR_FUSION_SAMPLE_RATE / 1000.0f;
  printf("Sensor Fusion example\n");
  printf("Sample Rate: %.2f Hz\n", 1.0 / acq_period);
  printf("got inside sensor_work");
  int fd, fd_mag;

  int16_t mag_data[4];

  fd = open("/dev/mpu6500", O_RDONLY);
  if (fd < 0)
  {
    printf("Failed to open mpu6500\n");
    return; // This might create an issue
  }

  fd_mag = open("/dev/mag0", O_RDONLY);
  if (fd_mag < 0)
  {
    printf("Failed to open magnetometer\n");
    close(fd_mag);
    // return;// This might create an issue
  }
  printf("************************************************\n");

  read_mpu6050(fd, &imu_acc_data, &imu_gyro_data, &raw_imu);
  read_lis3mdl(fd_mag, &raw_imu, mag_data);

  printf("Timestamp: %f  Temperature: %f\n"
         "Accelerometer X: %f | Y: %f | Z: %f\n"
         "Gyroscope X: %f | Y: %f | Z: %f\n"
         "Magnetometer X: %f | Y: %f | Z: %f\n",
         imu_acc_data.timestamp, imu_acc_data.temperature,
         imu_acc_data.x, imu_acc_data.y, imu_acc_data.z,
         imu_gyro_data.x, imu_gyro_data.y, imu_gyro_data.z,
         raw_imu.mag_x, raw_imu.mag_y, raw_imu.mag_z);
  printf("************************************************\n");

  close(fd);
  close(fd_mag);

  sat_health.accl_x = ((imu_acc_data.x));
  sat_health.accl_y = ((imu_acc_data.y));
  sat_health.accl_z = ((imu_acc_data.z));

  sat_health.gyro_x = ((imu_gyro_data.x));
  sat_health.gyro_y = ((imu_gyro_data.y));
  sat_health.gyro_z = ((imu_gyro_data.z));

  sat_health.mag_x = (raw_imu.mag_x);
  sat_health.mag_y = (raw_imu.mag_y);
  sat_health.mag_z = (raw_imu.mag_z);

  printf("Accelerometer X: %d | Y: %d | Z: %d\n  Magnetometer X: %d | Y: %d | Z: %d\n", sat_health.accl_x, sat_health.accl_y,
         sat_health.accl_z, sat_health.mag_x, sat_health.mag_y, sat_health.mag_z);
}

void read_mpu6050(int fd, struct sensor_accel *acc_data, struct sensor_gyro *gyro_data, struct mpu6500_imu_msg *raw_imu)
{
  int16_t raw_data[7];
  memset(raw_imu, 0, sizeof(struct mpu6500_imu_msg));
  int ret = read(fd, raw_data, sizeof(raw_data));
  if (ret <= 0) //!= sizeof(raw_data))
  {
    printf("Failed to read accelerometer data\n");
  }
  else
  {
    raw_imu->acc_x = ((raw_data[0] & REG_HIGH_MASK) << 8) + ((raw_data[0] & REG_LOW_MASK) >> 8);
    raw_imu->acc_y = ((raw_data[1] & REG_HIGH_MASK) << 8) + ((raw_data[1] & REG_LOW_MASK) >> 8);
    raw_imu->acc_z = ((raw_data[2] & REG_HIGH_MASK) << 8) + ((raw_data[2] & REG_LOW_MASK) >> 8);
    raw_imu->gyro_x = ((raw_data[4] & REG_HIGH_MASK) << 8) + ((raw_data[4] & REG_LOW_MASK) >> 8);
    raw_imu->gyro_y = ((raw_data[5] & REG_HIGH_MASK) << 8) + ((raw_data[5] & REG_LOW_MASK) >> 8);
    raw_imu->gyro_z = ((raw_data[6] & REG_HIGH_MASK) << 8) + ((raw_data[6] & REG_LOW_MASK) >> 8);
  }

  acc_data->x = raw_imu->acc_x / MPU6050_AFS_SEL;
  acc_data->y = raw_imu->acc_y / MPU6050_AFS_SEL;
  acc_data->z = raw_imu->acc_z / MPU6050_AFS_SEL;

  gyro_data->x = raw_imu->gyro_x / MPU6050_FS_SEL;
  gyro_data->y = raw_imu->gyro_y / MPU6050_FS_SEL;
  gyro_data->z = raw_imu->gyro_z / MPU6050_FS_SEL;
  printf("Timestamp: %f  Temperature: %f\n"
         "Accelerometer X: %f | Y: %f | Z: %f\n"
         "Gyroscope X: %f | Y: %f | Z: %f\n",

         imu_acc_data.timestamp, imu_acc_data.temperature,
         acc_data->x, acc_data->y, acc_data->z,
         gyro_data->x, gyro_data->y, gyro_data->z);
}

void read_lis3mdl(int fd_mag, struct mpu6500_imu_msg *raw_imu, int16_t mag_data[4])
{
  assert(fd_mag >= 0);
  int data_size = read(fd_mag, mag_data, 8);
  if (data_size > 0)
  {
    printf("read sensor data from Mag. Len %i\n", data_size);
    raw_imu->mag_x = mag_data[0];
    raw_imu->mag_y = mag_data[1];
    raw_imu->mag_z = mag_data[2];
    printf("Magnetometer func: x:%d y:%d z:%d\n", mag_data[0], mag_data[1], mag_data[2]);
  }
  else
  {
    printf("Failed to read from sensor.\n");
  }
}