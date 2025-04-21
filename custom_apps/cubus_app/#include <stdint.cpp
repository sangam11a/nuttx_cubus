#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

typedef struct __attribute__ ((__packed__))  _BEACON_A {
uint8_t HEAD; // 1 byte
uint8_t TYPE : 4; // 4-bit field
int TIM_DAY : 12; // 12-bit field
uint8_t TIM_HOUR; // 1 byte

uint16_t BAT_V; // 2 bytes
uint16_t BAT_C; // 2 bytes
int16_t BAT_T; // 2 bytes
int8_t RAW_C; // 1 byte
uint16_t SOL_TOT_V; // 2 bytes
int16_t SOL_TOT_C; // 2 bytes

int8_t BPB_T; // 1 byte
int8_t OBC_T; // 1 byte
int8_t Y1_T; // 1 byte
int8_t Y_T; // 1 byte
int8_t Z1_T; // 1 byte
int8_t Z_T; // 1 byte
int8_t X1_T; // 1 byte
int8_t X_T; // 1 byte

uint8_t SOL_P1_STAT; // 1 byte
uint8_t SOL_P2_STAT; // 1 byte
uint8_t SOL_P3_STAT; // 1 byte
uint8_t SOL_P4_STAT; // 1 byte
uint8_t SOL_P5_STAT; // 1 byte

uint8_t MSN1_STAT; // 1 byte
uint8_t MSN2_STAT; // 1 byte
uint8_t MSN3_STAT; // 1 byte

uint8_t ANT_STAT; // 1 byte
uint8_t KILL1_STAT; // 1 byte
uint8_t KILL2_STAT; // 1 byte
uint8_t UL_STAT; // 1 byte

uint8_t OPER_MODE; // 1 byte
uint64_t RST_RESET_COUNT; // 2 bytes
uint16_t OBC_RESET_COUNT; // 2 bytes
uint64_t LAST_RESET; // 2 bytes
uint16_t CHK_CRC; // 2 bytes

uint8_t ANT_P_T; // 1 byte
} S2S_BEACON_A;

void deserialize_beacon_a(const char *hex_str, S2S_BEACON_A *beacon) {
uint8_t beacon_data[43]; // Beacon A data size is 43 bytes
char *token;
char *str = strdup(hex_str); // Duplicate the string to manipulate it
int i = 0;

// Split the string based on spaces and convert each token to a byte
token = strtok(str, " ");
while (token != NULL && i < 43) {
beacon_data[i++] = (uint8_t)strtol(token, NULL, 16);
token = strtok(NULL, " ");
}

free(str); // Free the duplicated string

// Deserialize the beacon data
beacon->HEAD = beacon_data[0];
// uint8_t TYPE = (beacon_data[3]);
//s2s_beacon_type_a.TYPE << 4 | (((s2s_beacon_type_a.TIM_DAY)<<4) & 0x0f) & 0xff;
uint8_t TYPE = (beacon_data[4] >> 4) & 0x0F;
uint16_t TIM_DAY = (((beacon_data[4] >> 4 & 0x0Fff)) | beacon_data[5]) &0xffff;

// uint16_t TIM_DAY = ((beacon_data[1 + 1] ) | beacon_data[1 + 2]<<4) &0x0fff;
beacon->TIM_HOUR = beacon_data[1 + 4];

beacon->BAT_V = (beacon_data[1 + 5] << 8) | beacon_data[1 + 6];
beacon->BAT_C = (beacon_data[1 + 7] << 8) | beacon_data[1 + 8];
beacon->BAT_T = (beacon_data[1 + 9] << 8) | beacon_data[1 + 10];
beacon->RAW_C = beacon_data[1 + 11];
beacon->SOL_TOT_V = (beacon_data[1 + 12] << 8) | beacon_data[1 + 13];
beacon->SOL_TOT_C = (beacon_data[1 + 14] << 8) | beacon_data[1 + 15];

beacon->ANT_P_T = beacon_data[1 + 16];
beacon->BPB_T = beacon_data[1 + 17];
beacon->OBC_T = beacon_data[1 + 18];
beacon->X_T = beacon_data[1 + 19];
beacon->X1_T = beacon_data[1 + 20];
beacon->Y_T = beacon_data[1 + 21];
beacon->Y1_T = beacon_data[1 + 22];
beacon->Z_T = beacon_data[1 + 23];

// The next byte contains multiple status flags
uint8_t status_byte = beacon_data[1 + 24];
beacon->SOL_P1_STAT = (status_byte >> 7) & 0x01;
beacon->SOL_P2_STAT = (status_byte >> 6) & 0x01;
beacon->SOL_P3_STAT = (status_byte >> 5) & 0x01;
beacon->SOL_P4_STAT = (status_byte >> 4) & 0x01;
beacon->SOL_P5_STAT = (status_byte >> 3) & 0x01;
beacon->MSN1_STAT = (status_byte >> 2) & 0x01;
beacon->MSN2_STAT = (status_byte >> 1) & 0x01;
beacon->MSN2_STAT = (status_byte) & 0x01;

// The next byte contains ANT_STAT and UL_STAT
uint8_t ant_ul_byte = beacon_data[1 + 25];
beacon->UL_STAT = (ant_ul_byte >> 4) & 0x0F;
beacon->ANT_STAT = ant_ul_byte & 0x0F;

beacon->OPER_MODE = beacon_data[1 + 26];
beacon->OBC_RESET_COUNT = (beacon_data[1 + 27] << 8) | beacon_data[1 + 28];
beacon->RST_RESET_COUNT = (beacon_data[1 + 29] << 24) | (beacon_data[1 + 30]<< 16) | (beacon_data[1 + 31]<< 8) | (beacon_data[1 + 32]) &0xff;
beacon->CHK_CRC = (beacon_data[1 + 31] << 8) | beacon_data[1 + 32];

// The remaining fields need to be mapped correctly based on the serialization code
// TODO: Complete the deserialization for all fields
}

int main() {
char input[1024]; // Buffer to hold the input string

printf("Enter the beacon data as hex values separated by spaces: ");
if (fgets(input, sizeof(input), stdin) == NULL) {
printf("Error reading input.\n");
return 1;
}

// Remove newline character if present
input[strcspn(input, "\n")] = '\0';

S2S_BEACON_A beacon;

deserialize_beacon_a(input, &beacon);

// Print the deserialized data
printf("Beacon A Data:\n");
printf("----------------------------\n");
printf("HEAD: 0x%02X\n", beacon.HEAD);
printf("TYPE: %d\n", beacon.TYPE);
printf("TIM_DAY: %d\n", beacon.TIM_DAY);
printf("TIM_HOUR: %d\n", beacon.TIM_HOUR);
printf("BAT_V: %d\n", beacon.BAT_V);
printf("BAT_C: %d\n", beacon.BAT_C);
printf("BAT_T: %d\n", beacon.BAT_T);
printf("RAW_C: %d\n", beacon.RAW_C);
printf("SOL_TOT_V: %d\n", beacon.SOL_TOT_V);
printf("SOL_TOT_C: %d\n", beacon.SOL_TOT_C);
printf("ANT_P_T: %d\n", beacon.ANT_P_T);
printf("BPB_T: %d\n", beacon.BPB_T);
printf("OBC_T: %d\n", beacon.OBC_T);
printf("X_T: %d\n", beacon.X_T);
printf("X1_T: %d\n", beacon.X1_T);
printf("Y_T: %d\n", beacon.Y_T);
printf("Y1_T: %d\n", beacon.Y1_T);
printf("Z_T: %d\n", beacon.Z_T);
printf("SOL_P1_STAT: %d\n", beacon.SOL_P1_STAT);
printf("SOL_P2_STAT: %d\n", beacon.SOL_P2_STAT);
printf("SOL_P3_STAT: %d\n", beacon.SOL_P3_STAT);
printf("SOL_P4_STAT: %d\n", beacon.SOL_P4_STAT);
printf("SOL_P5_STAT: %d\n", beacon.SOL_P5_STAT);
printf("MSN1_STAT: %d\n", beacon.MSN1_STAT);
printf("MSN2_STAT: %d\n", beacon.MSN2_STAT);
printf("MSN3_STAT: %d\n", beacon.MSN3_STAT);
printf("ANT_STAT: %d\n", beacon.ANT_STAT);
printf("UL_STAT: %d\n", beacon.UL_STAT);
printf("OPER_MODE: %d\n", beacon.OPER_MODE);
printf("OBC_RESET_COUNT: %d\n", beacon.OBC_RESET_COUNT);
printf("RST_RESET_COUNT: %d\n", beacon.RST_RESET_COUNT);
printf("CHK_CRC: 0x%04X\n", beacon.CHK_CRC);
printf("----------------------------\n");

printf("\n=== Code Execution Successful ===\n");

return 0;
}