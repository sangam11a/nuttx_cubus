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
#include "mission_operations.h"

uint8_t data[7] = {0x53, 0x01, 0x02, 0x03, 0x04, 0x7e};
uint8_t Msn_Start_Cmd[7] = {0x53, 0xad, 0xba, 0xcd, 0x9e, 0x7e};
uint8_t ACK[7] = {0x53, 0xaa, 0xcc, 0xaa, 0xcc, 0x7e};
uint8_t NACK[7] = {0x53, 0xee, 0xff, 0xee, 0xff, 0x7e};
uint8_t RX_DATA_EPDM[48] = {'\0'};

/****************************************************************************
 * Send data from UART through any UART path
 ****************************************************************************/
int send_data_uart(char *dev_path, uint8_t *data, uint16_t size)
{
    double fd;
    uint8_t data1[7] = {'\0'};
    int i;
    int count = 0, ret;
    printf("Opening uart dev path : %s\n", dev_path);
    // usleep(1000);
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
    // ioctl(fd, TCFLSH, 2);
    // printf("flused tx rx buffer\n");
    // ioctl(fd, TCDRN, NULL);
    // printf("drained tx rx buffer\n");
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
    ioctl(fd, TCFLSH, 2);
    printf("flused tx rx buffer\n");
    ioctl(fd, TCDRN, NULL);
    printf("drained tx rx buffer\n");
    if (fd < 0)
    {
        printf("Unable to open %s\n", dev_path);
        return fd;
    }

    // int ret = read(fd, data, sizeof(data));
    printf("size of data to receive: %d\n", size);
    for (int i = 0; i < size; i++)
    {
        ret = read(fd, &data[i], 1);
    }
    if (ret < 0)
    {
        printf("data Not received from %s\n", dev_path);
        return ret;
    }
    for (int i = 0; i < size; i++)
    {
        printf("%x ", data[i]);
    }
    printf("\n");
    ioctl(fd, TCFLSH, 2);
    printf("flused tx rx buffer\n");
    ioctl(fd, TCDRN, NULL);
    printf("drained tx rx buffer\n");
    close(fd);
    return ret;
}

/****************************************************************************
 * Send beacon data to COM
 * To be done every 90 seconds
 ****************************************************************************/
int send_beacon_data()
{
    // work_queue(HPWORK, &work_beacon, send_beacon_data, NULL, SEC2TICK(BEACON_DELAY));
    beacon_type = !beacon_type;
    if (COM_BUSY == 1)
    {
        return -1;
    }
    uint8_t beacon_data[BEACON_DATA_SIZE];
    switch (beacon_type)
    {
    case 0:
        beacon_data[0] = 0x53;
        beacon_data[1] = 0x01;
        beacon_data[2] = 0x54;
        for (int i = 3; i < 83; i++)
        {
            beacon_data[i] = i;
        }
        beacon_data[83] = 0x7e;
        break;
    case 1:
        beacon_data[0] = 0x53;
        beacon_data[1] = 0x02;
        beacon_data[2] = 0x54;
        for (int i = 3; i < 83; i++)
        {
            beacon_data[i] = i;
        }
        beacon_data[83] = 0x73;
        break;
    default:
        printf("wrong case selected\n");
        return -1;
        break;
    }
    beacon_data[84] = '0';
    int fd = open(COM_UART, O_WRONLY);
    if (fd < 0)
    {
        printf("unable to open: %s\n", COM_UART);
        return -1;
    }
    ioctl(fd, TCFLSH, 2); // flusing tx/rx buffer
    printf("tx buffer flushed\n");
    ioctl(fd, TCDRN, NULL);
    printf("RX buffer flused\n");
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
    usleep(1000 * 1000 * 4); // 4 seconds
    gpio_write(GPIO_COM_4V_EN, 0);
    ioctl(fd, TCFLSH, 2); // flusing tx/rx buffer
    printf("tx buffer flushed\n");
    ioctl(fd, TCDRN, 2);
    printf("RX buffer flused\n");
    close(fd);
    printf("Turned off COM 4V line..\n");
    printf("Beacon Type %d sequence complete\n", beacon_type);
    return 0;
}

/****************************************************************************
 * COM RX telecommands
 ****************************************************************************/
int receive_telecommand_rx(uint8_t *COM_RX_DATA)
{
    printf("waiting for telecommands from COM\n");
    int ret = receive_data_uart(COM_UART, COM_RX_DATA, 35);
    if (ret < 0)
    {
        // send_data_uart(COM_UART, NACK, 7);
        printf("data not received from COM\n");
        return ret;
    }
    // ret = send_data_uart(COM_UART, ACK, 7);
    return ret;
}

/****************************************************************************
 * COM RX telecommands parse
 ****************************************************************************/
int parse_telecommand(uint8_t *rx_data)
{
    printf("Data redirected to EPDM\n");
    //TODO: parse the command and execute
    //for now only epdm is executed
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
    receive_data_uart(COM_UART, data, 35);
    for (int i = 35; i < 84; i++)
    {
        data[i] = 0xff;
    }
    send_data_uart(COM_UART, data, 84);
}

/****************************************************************************
 * COM TASK task
 ****************************************************************************/
static int COM_TASK(int argc, char *argv[])
{
    int ret = -1;
    uint8_t rx_data[35] = {'\0'};
    printf("Turning on COM MSN...\n");
    gpio_write(GPIO_3V3_COM_EN, 1);
    usleep(2000000);
    ret = handshake_COM(data);
    usleep(PRINT_DELAY * 100);
    if (ret == 0)
    {
        printf("Successful handshake with COM\n");
        // break;
    }
    // }
    if (ret != 0)
    {
        printf("Unable to handshake with COM\n");
    }
    usleep(10000);
    beacon_type = 1;
    ret = send_beacon_data();
    // ret = send_beacon_data();
    usleep(PRINT_DELAY * 100);
    if (ret < 0)
    {
        printf("Unable to send beacon Type 1 data\n");
    }
    printf("Going to receiver mode...\n");
    receive_telecommand_rx(rx_data);
    parse_telecommand(rx_data);
    // usleep(1000 * 1000 * 10);
    // digipeater_mode(rx_data);
    printf("digipeating completed\n");
    for (;;)
    {

        usleep(1000);
    }
}

/****************************************************************************
 * COM handshake function
 ****************************************************************************/
int handshake_COM(uint8_t *ack)
{
    double fd;
    uint8_t data1[7] = {'\0'};
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

    int wr1 = write(fd, data, 7); // writing handshake data
    if (wr1 < 0)
    {
        printf("Unable to send data through %d UART", COM_UART);
        usleep(PRINT_DELAY);
        return -1;
    }
    printf("\n%d bytes written\n", wr1);
    usleep(PRINT_DELAY);
    // ret = read(fd, data1, 10);
    for (i = 0; i < 7; i++)
    {
        ret = read(fd, &data1[i], 1);
    }
    printf("data received from %s \n", COM_UART, fd);
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
    printf("\n");
    ioctl(fd, TCFLSH, 2);
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
        gpio_write(GPIO_MSN1_EN, 1);
        printf("Turned on power line for ADCS\n");
        break;
    case 2:
        strcpy(devpath, CAM_UART);
        gpio_write(GPIO_MSN2_EN, 1);
        printf("Turned on power line for CAM\n");
        break;
    case 3:
        strcpy(devpath, EPDM_UART);
        gpio_write(GPIO_BURNER_EN, 1);
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
    for (int i = 0; i < 3; i++)
    {
        handshake_success = handshake_MSN(3, data);
        if (handshake_success == 0)
        {
            break;
        }
        gpio_write(GPIO_BURNER_EN, 0);
        usleep(1000 * 1000);
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
    }
    gpio_write(GPIO_MSN3_EN, 0);
    printf("EPDM Mission complete\n");
    return 0;
}

int turn_msn_on_off(uint8_t subsystem, uint8_t state){
    switch(subsystem){
        case 1:
            printf("turning ADCS mission state: %d\n", state);
            gpio_write(GPIO_MSN1_EN, state);
            break;
        case 2:
            printf("Turning CAM mission state: %d\n", state);
            gpio_write(GPIO_MSN2_EN, state);
            break;
        case 3:
            printf("Turning EPDM mission state: %d\n", state);
            gpio_write(GPIO_MSN3_EN, state);
            break;
        default:
            printf("Wrong subsystem selected\n");
            break;
    }
}