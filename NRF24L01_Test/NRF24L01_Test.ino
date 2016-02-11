#include <SPI.h>
#include <EEPROM.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "H2_EEPROM.h"

#define TIMEOUT_RX 100
#define SENSOR_PIN 3

RF24 radio(9, 10);

const uint64_t talking_pipes[5] = { 0xF0F0F0F0D2LL, 0xF0F0F0F0C3LL, 0xF0F0F0F0B4LL, 0xF0F0F0F0A5LL, 0xF0F0F0F096LL };
const uint64_t listening_pipes[5] = { 0x3A3A3A3AD2LL, 0x3A3A3A3AC3LL, 0x3A3A3A3AB4LL, 0x3A3A3A3AA5LL, 0x3A3A3A3A96LL };

PACKET_BUFFER packet;

void setup() {
  Serial.begin(115200);
  Serial.println("NRF24L01 Test");

  //Sensor Setup
  pinMode(SENSOR_PIN, INPUT);
  digitalWrite(SENSOR_PIN, LOW);
  
  radio.begin();

  if (EEPROM.read(FC_ADDRESS) == 1){
    radio.openReadingPipe(1,talking_pipes[0]);
    radio.openReadingPipe(2,talking_pipes[1]);
    radio.openReadingPipe(3,talking_pipes[2]);
    radio.openReadingPipe(4,talking_pipes[3]);
    radio.openReadingPipe(5,talking_pipes[4]);
  } else if (EEPROM.read(FC_ADDRESS) >= 2 && EEPROM.read(FC_ADDRESS) <= 6) {
    // Write on our talking pipe
    radio.openWritingPipe(talking_pipes[EEPROM.read(FC_ADDRESS)-2]);
    // Listen on our listening pipe
    radio.openReadingPipe(1,listening_pipes[EEPROM.read(FC_ADDRESS)-2]);
  } else {
    Serial.println("No Role Assigned");
  }

  radio.startListening();
  radio.printDetails();
}

void loop() {
  TX_PROCESS();
  RX_PROCESS();

  SerialProcess(packet);
}

void TX_PROCESS(){
  if (EEPROM.read(FC_ADDRESS) >= 2 && EEPROM.read(FC_ADDRESS) <= 6){
    radio.stopListening();

    uint8_t stuff;
    if (digitalRead(SENSOR_PIN)){
      stuff = 1;
    } else {
      stuff = 0;
    }
    radio.write(&stuff, sizeof(stuff));
    radio.startListening();
    bool timeout = false;
    unsigned long start_rx = millis();
    while (!radio.available() && !timeout){
      if (millis() - start_rx > TIMEOUT_RX){
        timeout = true;
      }
    }
    if (timeout){
      Serial.println("No Response");
    } else {
      uint8_t data;
      radio.read(&data, sizeof(data));
      Serial.println(data);
    }

    delay(1000);
  }
}

void RX_PROCESS(){
  if (EEPROM.read(FC_ADDRESS) == 1){
    uint8_t pipe_num;
    if (radio.available(&pipe_num)){
      uint8_t data;
      bool done = false;
      while(!done){
        done = radio.read(&data, sizeof(data));
        Serial.println(data);
      }

      radio.stopListening();
      radio.openWritingPipe(listening_pipes[pipe_num - 1]);

      uint8_t stuff = EEPROM.read(FC_ADDRESS);
      radio.write( &stuff, sizeof(stuff) );

      radio.startListening();
    }
  }
}

