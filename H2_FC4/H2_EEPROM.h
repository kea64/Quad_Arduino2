#ifndef H2_EEPROM_h
#define H2_EEPROM_h

//EEPROM LAYOUT
/*
Number of Bytes Used - 3
0 - ACRO_EN
1 - STAB_EN
47 - DEBUG_EN

*/

//EEPROM MEMORY LOCATIONS
//0-149 Bit/Int Values //150++ Floats
#define ACRO_ 0
#define STAB_ 1
#define DEBUG_ 47
#define ACCEL_CHECK_ 48
#define ACCEL_CALIB_SCHEDULE_ 49
#define MAG_CALIB_SCHEDULE_ 50
#define MAG_CHECK_ 51
#define COMPLI_DELAY_ 52
#define BARO_DELAY_ 53

#define ACCEL_OFFSET_X_ 150
#define ACCEL_OFFSET_Y_ 154
#define ACCEL_OFFSET_Z_ 158
#define MAG_GAIN_X_ 162
#define MAG_GAIN_Y_ 166
#define MAG_GAIN_Z_ 170
#define MAG_OFFSET_X_ 174
#define MAG_OFFSET_Y_ 178
#define MAG_OFFSET_Z_ 182

struct PACKET_BUFFER{
  String inBuffer;
  String buf1;
  String buf2;
  String buf3;
  String buf4;
};

void EEPROMFlush();

int SerialRequest();

void SerialProcess(struct PACKET_BUFFER &packet);

void TXData(int loc, float dat);
void TXData(int loc, int dat);

#endif
