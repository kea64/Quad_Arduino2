#include <Arduino.h>
#include <EEPROM.h>
#include "H2_EEPROM.h"
#include "H2_Drone_Values.h"

#define EEPROM_SIZE 2 //EEPROM SIZE IN BYTES

#define ACRO_LOC 0
#define STAB_LOC 1
#define DEBUG_LOC 47

void read_EEPROM(struct EEPROM_DAT &EE){
  EE.ACRO1_EN = EEPROM.read(ACRO_LOC);
  EE.STAB_EN = EEPROM.read(STAB_LOC); 
  EE.DEBUG_EN = EEPROM.read(DEBUG_LOC);
}

void EEPROMFlush(){
  for(int i = 0; i < 4096; i++){
    if(EEPROM.read(i) != 0){
      EEPROM.write(i, 0);
    }
  } 
} 

void SerialProcess(struct PACKET_BUFFER &packet){
  //operate only upon new received byte
  if (Serial.available() > 0){
    int inChar = Serial.read(); //Get that new byte
    
    //Fill in buffer with packet
    if (inChar == 47){
      packet.buf1 = packet.buf2;
      packet.buf2 = packet.buf3;
      packet.buf3 = packet.buf4;
      packet.buf4 = packet.buf5;
      packet.buf5 = packet.inBuffer;
      packet.inBuffer = "";
    } else {
      packet.inBuffer += (char)inChar; //Continue to build string
    }
    
    
    
    //Execute Command Packet
    if (inChar == 13) {
      
      //Verify Correct Preamble and Checksum
      if(packet.buf1.toInt() == FC_ADDRESS && packet.buf5.toInt() == FC_ADDRESS){
        //Check for purpose of transmission
        //1 = Write to EEPROM, 0 = Simple Read from EEPROM
//        if(packet.buf2.toInt() == 1){
//          EEPROM.put(packet.buf3.toInt(), atof(packet.buf4.c_str()));
//        } else if(packet.buf2.toInt() == 0){
//          Serial.print("A: ");
//          Serial.print(packet.buf3);
//          Serial.print(" V: ");
//          float temp = 0.00f;
//          EEPROM.get(packet.buf3.toInt(), temp);
//          Serial.println(temp);
//        }
        float tempFloat = 0.000000f;
        float tempInt = 0;
        switch(packet.buf2.toInt()){
          case 0:
            //Read Float
            EEPROM.get(packet.buf3.toInt(), tempFloat);
            TXData(packet.buf3.toInt(), tempFloat);
            break;
          case 1:
            //Read Int
            //Serial.println(EEPROM.read(packet.buf3.toInt()));
            TXData(packet.buf3.toInt(), EEPROM.read(packet.buf3.toInt()));
            break;
          case 2:
            //Write Integer
            EEPROM.put(packet.buf3.toInt(), atoi(packet.buf4.c_str()));
            break; 
          case 3:
            //Write Float
            EEPROM.put(packet.buf3.toInt(), atof(packet.buf4.c_str())); 
            break;
          default:
            Serial.print("E2");
            break;
        }
      } else {
        packet.buf1 = "";
        packet.buf2 = "";
        packet.buf3 = "";
        packet.buf4 = "";
        packet.buf5 = "";
        Serial.println("E1");
      }
      packet.inBuffer = "";
      
      /*
      Serial.print("Buf1: ");
      Serial.println(packet.buf1);
      Serial.print("Buf2: ");
      Serial.println(packet.buf2);
      Serial.print("Buf3: ");
      Serial.println(packet.buf3);
      Serial.print("Buf4: ");
      Serial.println(packet.buf4);
      Serial.print("Buf5: ");
      Serial.println(packet.buf5);
      */
    }
    
  }
}

void TXData(int loc, float dat){
  Serial.print(FC_ADDRESS);
  Serial.print("/");
  Serial.print(loc);
  Serial.print("/");
  Serial.print(dat);
  Serial.print("/");
  Serial.println(FC_ADDRESS);
}

void TXData(int loc, int dat){
  Serial.print(FC_ADDRESS);
  Serial.print("/");
  Serial.print(loc);
  Serial.print("/");
  Serial.print(dat);
  Serial.print("/");
  Serial.println(FC_ADDRESS);
}
