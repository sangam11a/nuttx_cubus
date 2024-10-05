// #include "com_app_main.h"
// #include "gpio_definitions.h"
// static void testing(){
//     printf("testing the code \n");
// }

// /****************************************************************************
//  * Send beacon data to COM
//  * To be done every 90 seconds
//  ****************************************************************************/
// int send_beacon_data(uint8_t beacon_type)
// {
//   if (COM_BUSY == 1)
//   {
//     return -1;
//   }
//   // COM_BUSY =
//   uint8_t beacon_data[BEACON_DATA_SIZE];
//   switch (beacon_type)
//   {
//   case 0:

//     serialize_beacon_a(beacon_data);

//     beacon_data[1] = 0xb1;
//     beacon_data[2] = 0x51;
//     beacon_data[83] = 0x7e;
//     break;
//   case 1:
//     serialize_beacon_b(beacon_data);
//     beacon_data[1] = 0xb2;
//     beacon_data[2] = 0x51;
//     break;
//   default:
//     printf("wrong case selected\n");
//     return -1;
//     break;
//   }
//   beacon_data[0] = 0x53;
//   beacon_data[83] = 0x7e;

//   beacon_data[84] = '0';
//   int fd = open(COM_UART, O_WRONLY);
//   if (fd < 0)
//   {
//     printf("unable to open: %s\n", COM_UART);
//     return -1;
//   }
//   sleep(2);
//   printf("Turning on  4v dcdc line..\n");
//   gpio_write(GPIO_DCDC_4V_EN, 1);
//   printf("Turning on COM 4V line..\n");
//   gpio_write(GPIO_COM_4V_EN, 1);

//   int ret = write(fd, beacon_data, BEACON_DATA_SIZE);
//   usleep(10000);
//   if (ret < 0)
//   {
//     printf("unable to send data\n");
//     for (int i = 0; i < BEACON_DATA_SIZE; i++)
//     {
//       ret = write(fd, &beacon_data[i], 1);
//       usleep(1000);
//     }
//     if (ret < 0)
//     {
//       printf("Unable to send data through byte method..\n");
//       return -1;
//     }
//   }
//   /*To delete*/
//   if (beacon_status == 0)
//   {
//     printf("\nbeacon 1:\n");
//   }
//   else
//   {
//     printf("\nbeacon 2:\n");
//   }
//   beacon_type = !beacon_type;

//   for (int i = 0; i < 83; i++)
//   {
//     {
//       printf("%d ", beacon_data[i]);
//     }
//   }
//   printf("\n");
//   /*To delete*/
//   int x = 0;
//   while (x < 200000)
//   {
//     x += 200;
//     usleep(200);
//   }

//   // printf("urning off  4v DCDC line..\n");
//   gpio_write(GPIO_DCDC_4V_EN, 0);
//   // printf("Turning off COM 4V line..\n");
//   gpio_write(GPIO_COM_4V_EN, 0);
//   ioctl(fd, TCFLSH, 2);
//   ioctl(fd, TCDRN, 2);
//   printf("TX RX buffer flused\n");
//   close(fd);
//   printf("Turned off COM 4V line..\n");
//   printf("Beacon Type %d sequence complete\n", beacon_type);
//   work_queue(HPWORK, &work_beacon, send_beacon_data, NULL, SEC2TICK(BEACON_DELAY));

//   return 0;
// }