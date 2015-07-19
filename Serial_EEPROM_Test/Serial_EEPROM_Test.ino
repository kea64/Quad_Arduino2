//Tests EEPROM saving and SERIAL communication on Arduino NANO!

#include <EEPROM.h>

int FC_ADDRESS = 1053;

int LED = 13;
int LEDAddr = 15;
int LEDPeriod = 250;
int PeriodAddr = 16;
String inBuffer = "";
unsigned long LEDold = 0;
boolean LEDStat = 0;

int buf1 = 0;
int buf2 = 0;

struct messageBuffer{
  String buf1;
  String buf2;
  String buf3;
  String buf4;
  String buf5;
};

messageBuffer inBuf = {"", "", ""};

void setup() {
  //EEPROMClear();
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  doSomething();
  LEDold = millis();
}

void loop() {
  
  SerialProcess();
  //SerialProcess2(buf1, buf2);
  doSomething();
  
}

void SerialProcess(){
  //operate only upon new received byte
  if (Serial.available() > 0){
    int inChar = Serial.read(); //Get that new byte
    
    //Fill in buffer with packet
    if (inChar == 47){
      inBuf.buf1 = inBuf.buf2;
      inBuf.buf2 = inBuf.buf3;
      inBuf.buf3 = inBuf.buf4;
      inBuf.buf4 = inBuf.buf5;
      inBuf.buf5 = inBuffer;
      inBuffer = "";
    } else {
      inBuffer += (char)inChar; //Continue to build string
    }
    
    
    
    //Execute Command Packet
    if (inChar == 13) {
      
      //Verify Correct Preamble and Checksum
      if(inBuf.buf1.toInt() == FC_ADDRESS && inBuf.buf5.toInt() == FC_ADDRESS){
        //Check for purpose of transmission
        //1 = Write to EEPROM, 0 = Simple Read from EEPROM
//        if(inBuf.buf2.toInt() == 1){
//          EEPROM.put(inBuf.buf3.toInt(), atof(inBuf.buf4.c_str()));
//        } else if(inBuf.buf2.toInt() == 0){
//          Serial.print("A: ");
//          Serial.print(inBuf.buf3);
//          Serial.print(" V: ");
//          float temp = 0.00f;
//          EEPROM.get(inBuf.buf3.toInt(), temp);
//          Serial.println(temp);
//        }
        float tempFloat = 0.000000f;
        float tempInt = 0;
        switch(inBuf.buf2.toInt()){
          case 0:
            //Read Float
            EEPROM.get(inBuf.buf3.toInt(), tempFloat);
            TXData(inBuf.buf3.toInt(), tempFloat);
            break;
          case 1:
            //Read Int
            //Serial.println(EEPROM.read(inBuf.buf3.toInt()));
            TXData(inBuf.buf3.toInt(), EEPROM.read(inBuf.buf3.toInt()));
            break;
          case 2:
            //Write Integer
            EEPROM.put(inBuf.buf3.toInt(), atoi(inBuf.buf4.c_str()));
            break; 
          case 3:
            //Write Float
            EEPROM.put(inBuf.buf3.toInt(), atof(inBuf.buf4.c_str())); 
            break;
          default:
            Serial.print("E2");
            break;
        }
      } else {
        inBuf.buf1 = "";
        inBuf.buf2 = "";
        inBuf.buf3 = "";
        inBuf.buf4 = "";
        inBuf.buf5 = "";
        Serial.println("E1");
      }
      inBuffer = "";
      
      /*
      Serial.print("Buf1: ");
      Serial.println(inBuf.buf1);
      Serial.print("Buf2: ");
      Serial.println(inBuf.buf2);
      Serial.print("Buf3: ");
      Serial.println(inBuf.buf3);
      Serial.print("Buf4: ");
      Serial.println(inBuf.buf4);
      Serial.print("Buf5: ");
      Serial.println(inBuf.buf5);
      */
    }
    
  }
}


void doSomething(){
  if(EEPROM.read(LEDAddr) > 0){
    if(millis() - LEDold >= EEPROM.read(PeriodAddr)){
      if (LEDStat == 1){
        digitalWrite(LED, LOW);
        LEDStat = 0;
      } else {
        digitalWrite(LED, HIGH);
        LEDStat = 1;
      }
      LEDold = millis();
    }
  } else {
    digitalWrite(LED, LOW);
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

void EEPROMClear(){
  for(int i = 0; i < 4096; i++){
    EEPROM.write(i, 0);
  } 
}

