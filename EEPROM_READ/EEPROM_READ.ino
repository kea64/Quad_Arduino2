#include <EEPROM.h>
#define LED 13
#define EEPROM_SIZE 4096

void setup() {
  Serial.begin(115200);
  
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  
  Serial.println("--------------------");
  Serial.println("EEPROM DISPLAY UTILITY");
  Serial.println("--------------------");
  Serial.println("");
  Serial.println("Written by H2TECH");
  Serial.println("");
  delay(3000);
  Serial.println("START READ\n\n");
  digitalWrite(LED, HIGH);
  
  int storedCount = 0;
  
  for(int i = 0; i < EEPROM_SIZE; i++){
    digitalWrite(LED, HIGH);
    Serial.print(EEPROM.read(i));
    Serial.print(",");
    
    if(EEPROM.read(i) != 0){
      storedCount++;
    }
    
  }
  
  digitalWrite(LED, LOW);
  Serial.println("\n\nEND READ TRANSMISSION");
  
  Serial.print("Variables Stored: ");
  Serial.print(storedCount);
  Serial.print("/");
  Serial.println(EEPROM_SIZE);

}

void loop() {
  // put your main code here, to run repeatedly:

}
