#include <EEPROM.h>
#define LED 13
#define EEPROM_SIZE 4096

void setup() {
  Serial.begin(115200);
  
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  
  Serial.println("--------------------");
  Serial.println("EEPROM ERASE UTILITY");
  Serial.println("--------------------");
  Serial.println("");
  Serial.println("Written by H2TECH");
  Serial.println("");
  delay(3000);
  Serial.println("START FLASH");
  digitalWrite(LED, HIGH);
  
  for(int i = 0; i < EEPROM_SIZE; i++){
    digitalWrite(LED, HIGH);
    if(EEPROM.read(i) != 0){
      EEPROM.write(i, 0);
      Serial.print("Cleared Loc: ");
      digitalWrite(LED, LOW);
      Serial.println(i);
    }
    
  }
  
  digitalWrite(LED, LOW);
  Serial.println("THE JOB IS DONE. ALL CLEAR");

}

void loop() {
  // put your main code here, to run repeatedly:

}
