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

#include "com_app_main.h"
#include "gpio_definitions.h"

static int COM_TASK(int argc, char *argv[]);

uint8_t data[7] = {0x53, 0x01, 0x02, 0x03, 0x04, 0x7e};
uint8_t ACK[7] = {0x53, 0xaa, 0xcc, 0xaa, 0xcc, 0x7e};
uint8_t NACK[7] = {0x53, 0xee, 0xff, 0xee, 0xff, 0x7e};

#define BEACON_DELAY 90
#define BEACON_DATA_SIZE 84
uint8_t COM_BUSY = 0;
static struct work_s work_beacon;
uint8_t beacon_type = 0;

int handshake_COM(uint8_t *ack);

int send_data_uart(char *dev_path, uint8_t *data)
{
    double fd;
    uint8_t data1[7] = {'\0'};
    int i;
    int count = 0, ret;
    printf("Opening uart dev path : %s", dev_path);
    usleep(1000);
    fd = open(dev_path, O_RDWR);
    if (fd < 0)
    {
        printf("error opening %s\n", CAM_UART);
        return fd;
    }

    int wr1 = write(fd, data, 7);
    if(wr1 < 0){
        printf("Unable to write data\n");
        return wr1;
    }
    printf("\n%d bytes written\n", wr1);
    return wr1;
}

int receive_data_uart(char *dev_path, uint8_t *data)
{
    int fd;
    fd = open(dev_path, O_RDONLY);
    if (fd < 0)
    {
        printf("Unable to open %s\n", dev_path);
        return fd;
    }

    int ret = read(fd, data, sizeof(data));
    if (ret < 0)
    {
        printf("data Not received from %s\n", dev_path);
        return ret;
    }
    printf("size of data variable: %d\n", sizeof(data));
    for (int i = 0; i < sizeof(data); i++)
    {
        printf("%x ", data[i]);
    }
    printf("\n");
    close(fd);
    return ret;
}

/*
Beacon type 1 is of 84 including header+packet_type+ data + information/data + footer
*/
int send_beacon_data()
{
    work_queue(HPWORK, &work_beacon, send_beacon_data, NULL, SEC2TICK(BEACON_DELAY));
    beacon_type = !beacon_type;
    if (COM_BUSY == 1)
    {
        return -1;
    }
    uint8_t beacon_data[BEACON_DATA_SIZE];
    switch (beacon_type)
    {
    case 0:
        beacon_data[84]; //={0x53,0x01,0x54};
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

    int fd = open(COM_UART, O_RDWR);
    if (fd < 0)
    {
        printf("unable to open: %s\n", COM_UART);
        return -1;
    }
    printf("Turning on COM 4V line..\n");
    gpio_write(GPIO_COM_4V_EN, 1);
    int ret = write(fd, beacon_data, 84);
    usleep(10000);
    if (ret < 0)
    {
        printf("unable to send data\n");
        for (int i = 0; i < 84; i++)
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
    close(fd);
    printf("Turned off COM 4V line..\n");
    printf("Beacon Type %d sequence complete\n", beacon_type);
    return 0;
}

/*
Receive telecommand from GS for 90 seconds
*/
int receive_telecommand_rx(uint8_t *data)
{
    int ret = receive_data_uart(COM_UART, data);
    if(ret < 0){
        send_data_uart(COM_UART, NACK);
        return ret;
    }
    ret = send_data_uart(COM_UART, ACK);
    if(ret < 0){
        ret = send_data_uart(COM_UART, ACK);  //TRYING SECOND TIME
    }
    return ret;
}

int parse_telecommand(){
    // uint8_t cmd[];
}
/*
COM will be in digipeater mode till a digipeating message is received and digipeated
*/
void digipeater_mode(uint8_t *data)
{
    receive_data_uart(COM_UART, data);
}

/*
Handshake command is to be provided 6bytes with a header 0x53 and footer 0x7e
*/
void handshake(int choice, uint8_t ack[])
{
    float counter = 0, expected = 90000 * 1000;

    uint8_t data[100];
    printf("handshake fun called choice is %d\n", choice);

    switch (choice)
    {
    case EPDM:
        printf("COM epdm");
        send_data_uart(EPDM_UART, ack);
        break;
    case ADCS:
        printf("ADCS uart");
        send_data_uart(ADCS_UART, ack);
        break;
    case CAM:
        printf("CAM uart");
        send_data_uart(CAM_UART, ack);
        break;
    default:
        break;
    };
}

/****************************************************************************
 * custom_hello_thread
 ****************************************************************************/
int main(int argc, FAR char *argv[])
{
    // printf("Custom com cmd %d", argc);
    if (argc > 1)
    {
        printf("\n");
        if (strcmp(argv[1], "com") == 0x00)
        {
            int retval = task_create("task1", 100, 1024, COM_TASK, NULL);
            if (retval < 0)
            {
                printf("unable to create COM task\n");
                return -1;
            }
        }
        else if (strcmp(argv[1], "epdm") == 0x00)
        {
            handshake(EPDM, data);
        }
        else if (strcmp(argv[1], "cam") == 0x00)
        {
            handshake(CAM, data);
        }
        else if (strcmp(argv[1], "adcs") == 0x00)
        {
            handshake(ADCS, data);
        }
        else
        {
            handshake(77, data);
        }
    }
    else
    {
        printf("too few argumnets\n");
    }
    return 0;
}

static int COM_TASK(int argc, char *argv[])
{
    int ret = -1;
    uint8_t rx_data[80];
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
    usleep(PRINT_DELAY * 100);
    if (ret < 0)
    {
        printf("Unable to send beacon Type 1 data\n");
    }
    printf("Going to receiver mode...\n");
    receive_telecommand_rx(rx_data);
    for (;;)
    {

        usleep(1000);
    }
}

int handshake_COM(uint8_t *ack)
{
    double fd, elapsed = 0, required = 15000 * 1000;
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
    usleep(PRINT_DELAY);
    printf("\n");
    usleep(PRINT_DELAY);
    return 0;
}

int handshake_MSN(uint8_t subsystem, uint8_t *ack)
{
    double fd, elapsed = 0, required = 15000 * 1000;
    uint8_t data1[7] = {'\0'};
    int i;
    int count = 0, ret;

    switch(subsystem){
        case 1:
            
    }

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
    usleep(PRINT_DELAY);
    printf("\n");
    usleep(PRINT_DELAY);
    return 0;
}