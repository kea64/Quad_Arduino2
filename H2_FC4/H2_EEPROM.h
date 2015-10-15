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
#define ACRO_LOC 0
#define STAB_LOC 1
#define DEBUG_LOC 47

struct EEPROM_DAT{
  bool ACRO1_EN;
  bool STAB_EN;
  int DEBUG_EN;
  
};

struct PACKET_BUFFER{
  String inBuffer;
  String buf1;
  String buf2;
  String buf3;
  String buf4;
};

void read_EEPROM(struct EEPROM_DAT &EE);

void EEPROMFlush();

void SerialProcess(struct PACKET_BUFFER &packet);

void TXData(int loc, float dat);
void TXData(int loc, int dat);

#endif
