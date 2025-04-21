
void serialize_beacon_a(uint8_t beacon_data[BEACON_DATA_SIZE])
{
  for (int i = 0; i <= BEACON_DATA_SIZE; i++)
  {
    beacon_data[i] = 0x00;
    // }
    // beacon_data[84] = s2s_beacon_type_a.;
    // beacon_data[85] = s2s_beacon_type_a.;
  }
  // uint8_t beacon_data[BEACON_DATA_SIZE];
  beacon_data[0] = s2s_beacon_type_a.HEAD;
  beacon_data[1] = s2s_beacon_type_a.TYPE << 4 & s2s_beacon_type_a.TIM_DAY << 4 & 0xff;
  beacon_data[2] = (uint8_t)s2s_beacon_type_a.TIM_DAY & 0xff;
  beacon_data[4] = s2s_beacon_type_a.TIM_HOUR;

  beacon_data[3] = 0x01;

  beacon_data[1 + 4] = (s2s_beacon_type_a.BAT_V >> 8) & 0Xff;
  beacon_data[1 + 5] = s2s_beacon_type_a.BAT_V & 0xff;
  beacon_data[1 + 6] = (s2s_beacon_type_a.BAT_C >> 8) & 0Xff;
  beacon_data[1 + 7] = (s2s_beacon_type_a.BAT_C) & 0Xff;
  beacon_data[1 + 8] = (s2s_beacon_type_a.BAT_T >> 8) & 0Xff;
  beacon_data[1 + 9] = (s2s_beacon_type_a.BAT_T) & 0Xff;

  beacon_data[1 + 10] = s2s_beacon_type_a.RAW_C;
  beacon_data[1 + 11] = (s2s_beacon_type_a.SOL_TOT_V >> 8) & 0Xff;
  beacon_data[1 + 12] = (s2s_beacon_type_a.SOL_TOT_V) & 0Xff;
  beacon_data[1 + 13] = (s2s_beacon_type_a.SOL_TOT_C >> 8) & 0Xff;
  beacon_data[1 + 14] = (s2s_beacon_type_a.SOL_TOT_C >> 8) & 0Xff;
  beacon_data[1 + 15] = s2s_beacon_type_a.ANT_P_T;
  beacon_data[1 + 16] = s2s_beacon_type_a.BPB_T;
  beacon_data[1 + 17] = s2s_beacon_type_a.OBC_T;
  beacon_data[1 + 18] = s2s_beacon_type_a.X_T;
  beacon_data[1 + 19] = s2s_beacon_type_a.X1_T;
  beacon_data[1 + 20] = s2s_beacon_type_a.Y_T;
  beacon_data[1 + 21] = s2s_beacon_type_a.Y1_T;
  beacon_data[1 + 22] = sat_health.sol_p5_v;

  beacon_data[1 + 23] = s2s_beacon_type_a.SOL_P1_STAT << 7 & s2s_beacon_type_a.SOL_P2_STAT << 6 & s2s_beacon_type_a.SOL_P3_STAT << 5 & s2s_beacon_type_a.SOL_P4_STAT << 4 & s2s_beacon_type_a.MSN1_STAT << 3 & s2s_beacon_type_a.MSN2_STAT << 2 & s2s_beacon_type_a.MSN3_STAT << 1 & 0xff;
  beacon_data[1 + 24] = s2s_beacon_type_a.ANT_STAT << 4 & s2s_beacon_type_a.UL_STAT << 4;
  beacon_data[1 + 25] = s2s_beacon_type_a.OPER_MODE;
  beacon_data[1 + 26] = (s2s_beacon_type_a.OBC_RESET_COUNT >> 8) & 0xff;
  beacon_data[1 + 27] = s2s_beacon_type_a.OBC_RESET_COUNT & 0xff;
  beacon_data[1 + 28] = 0 * s2s_beacon_type_a.RST_RESET_COUNT >> 8 & 0xff; // TODO no reset mcu so no count needed
  beacon_data[1 + 29] = 0 * s2s_beacon_type_a.RST_RESET_COUNT & 0xff;
  // beacon_data[1 + 30] = s2s_beacon_type_a.LAST_RESET;
  beacon_data[1 + 30] = s2s_beacon_type_a.CHK_CRC;
}
// COM_APP

void serialize_beacon_b(uint8_t beacon_data[BEACON_DATA_SIZE])
{
  for (int i = 0; i <= BEACON_DATA_SIZE; i++)
  {
    beacon_data[i] = 0x00;
  }
  beacon_data[0] = s2s_beacon_type_b.HEAD;
  beacon_data[1] = s2s_beacon_type_b.TYPE;
  beacon_data[2] = s2s_beacon_type_b.TIM_DAY;

  beacon_data[4] = s2s_beacon_type_b.SOL_P1_V;
  beacon_data[5] = s2s_beacon_type_b.SOL_P2_V;
  beacon_data[6] = s2s_beacon_type_b.SOL_P3_V;
  beacon_data[7] = s2s_beacon_type_b.SOL_P4_V;
  beacon_data[8] = s2s_beacon_type_b.SOL_P5_V;

  beacon_data[9] = s2s_beacon_type_b.SOL_P1_C;
  beacon_data[10] = s2s_beacon_type_b.SOL_P2_C;
  beacon_data[11] = s2s_beacon_type_b.SOL_P3_C;
  beacon_data[12] = s2s_beacon_type_b.SOL_P4_C;
  beacon_data[13] = s2s_beacon_type_b.SOL_P5_C;

  beacon_data[14] = ((s2s_beacon_type_b.GYRO_X ) >> 8) & 0xFF;
  beacon_data[15] = (s2s_beacon_type_b.GYRO_X ) & 0xFF;
  beacon_data[16] = ((s2s_beacon_type_b.GYRO_Y ) >> 8) & 0xFF;
  beacon_data[17] = (s2s_beacon_type_b.GYRO_Y ) & 0xFF;
  beacon_data[18] = ((s2s_beacon_type_b.GYRO_Z ) >> 8) & 0xFF;
  beacon_data[19] = (s2s_beacon_type_b.GYRO_Z ) & 0xFF;

  beacon_data[20] = ((s2s_beacon_type_b.ACCL_X ) >> 8) & 0xFF;
  beacon_data[21] = (s2s_beacon_type_b.ACCL_X ) & 0xFF;
  beacon_data[22] = ((s2s_beacon_type_b.ACCL_Y ) >> 8) & 0xFF;
  beacon_data[23] = (s2s_beacon_type_b.ACCL_Y ) & 0xFF;
  beacon_data[24] = ((s2s_beacon_type_b.ACCL_Z ) >> 8) & 0xFF;
  beacon_data[25] = (s2s_beacon_type_b.ACCL_Z ) & 0xFF;

  beacon_data[26] = ((s2s_beacon_type_b.MAG_X ) >> 8) & 0xFF;
  beacon_data[27] = (s2s_beacon_type_b.MAG_X ) & 0xFF;
  beacon_data[28] = ((s2s_beacon_type_b.MAG_Y ) >> 8) & 0xFF;
  beacon_data[29] = (s2s_beacon_type_b.MAG_Y ) & 0xFF;
  beacon_data[30] = ((s2s_beacon_type_b.MAG_Z ) >> 8) & 0xFF;
  beacon_data[31] = (s2s_beacon_type_b.MAG_Z ) & 0xFF;

  beacon_data[32] = s2s_beacon_type_b.CHK_CRC;
}