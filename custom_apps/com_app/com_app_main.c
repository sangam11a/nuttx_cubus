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
uint8_t Msn_Start_Cmd[7] = {0x53, 0xad, 0xba, 0xcd, 0x9e, 0x7e};
uint8_t ACK[7] = {0x53, 0xaa, 0xcc, 0xaa, 0xcc, 0x7e};
uint8_t NACK[7] = {0x53, 0xee, 0xff, 0xee, 0xff, 0x7e};

uint8_t RX_DATA_EPDM[48] = {'\0'};
int Execute_EPDM();

#define BEACON_DELAY 90
#define BEACON_DATA_SIZE 85
#define ACK_DATA_SIZE 6 + 1
#define COM_RX_CMD_SIZE 20
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
    usleep(1000 * 1000 * 3); // 4 seconds
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

    printf("waiting for telecommands from COM\n");
    int ret = receive_data_uart(COM_UART, COM_RX_DATA, COM_RX_CMD_SIZE); // telecommand receive
    if (ret < 0)
    {
        printf("data not received from COM\n NACK sending\n");
        send_data_uart(COM_UART, NACK, 7);
        printf("data not received from COM\n NACK sent\n");
        return ret;
    }
    printf("data received from COM\n sending ACK\n");
    ret = send_data_uart(COM_UART, ACK, 7); // ack send
    print_rx_telecommand(COM_RX_DATA);      // printing the received telecommand
    for (int i = 0; i < 12; i++)
    {
        useful_command[i] = COM_RX_DATA[i + 8]; // extracting useful commands
    }
    if (useful_command[5] != 0xff || useful_command[5] != 0x00)
    {
        printf("Reservation command received\n"); // if reservation command is received then store the reservation command (do not execute)
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
    receive_data_uart(COM_UART, data, 35);
    for (int i = 35; i < 84; i++)
    {
        data[i] = 0xff;
    }
    send_data_uart(COM_UART, data, 84);
}

/****************************************************************************
 * main function
 ****************************************************************************/
int main(int argc, FAR char *argv[])
{
    // printf("Custom com cmd %d", argc);
    if (argc > 1)
    {
        printf("\n");
        // if (strcmp(argv[1], "com") == 0x00)
        {
            int retval = task_create("task1", 100, 1024, COM_TASK, NULL);
            if (retval < 0)
            {
                printf("unable to create COM task\n");
                return -1;
            }
        }
        /*else*/ if (strcmp(argv[1], "adcs") == 0x00)
        {
            handshake_MSN(1, data);
        }
        else if (strcmp(argv[1], "cam") == 0x00)
        {
            handshake_MSN(2, data);
        }
        else if (strcmp(argv[1], "epdm") == 0x00)
        {
            // handshake_MSN(3, data);
            Execute_EPDM();
        }
        else
        {
            printf("Incorrect name for subsystem\n Enter <application name> <subsystem name>[com, adcs, cam, epdm] \n");
        }
    }
    else
    {
        printf("Too few arguments\n Enter <application name> <subsystem name>[com, adcs, cam, epdm] \n");
        return -1;
    }
    return 0;
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

    printf("Going to receiver mode...\n");
    send_beacon_data();
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
        gpio_write(GPIO_BURNER_EN, 0); // Antenna deployment here
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
        send_data_uart(EPDM_UART, RX_DATA_EPDM, BEACON_DATA_SIZE);
    }
    gpio_write(GPIO_MSN3_EN, 0);
    printf("EPDM Mission complete\n");
    return 0;
}

int turn_msn_on_off(uint8_t subsystem, uint8_t state)
{
    switch (subsystem)
    {
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