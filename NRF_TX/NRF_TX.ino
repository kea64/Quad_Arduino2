//TX

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10);

//const byte rxAddr[6] = "00001";
const uint64_t rxAddr = 0xF0F0F0F096LL;

void setup()
{
  radio.begin();
  radio.setRetries(3, 3);
  radio.openWritingPipe(rxAddr);
  
  radio.stopListening();
}

void loop()
{
  const char text[] = "Hello World";
  radio.write(&text, sizeof(text));
  
  delay(1000);
}
