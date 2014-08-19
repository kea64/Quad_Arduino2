#include <Servo.h>
#include <avr/interrupt.h>
#include <avr/io.h>

long incomingByte = 0;   // for incoming serial data
Servo Esc;
Servo frontWheel;
int escPin=9;
int incre = 0;
int startNumber=0;//1= start, 0 = waiting for command to start, itll change when it recieves data
void setup() {
     Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
         // send data only when you receive data:
     Serial.print("Serial Port Connected"); 
     Serial.write(13);
     Serial.write(10);
     Esc.attach(9);
     delay(1000);
     interrupts();
}
void lineReturn()
{
 Serial.write(13);
 Serial.write(10);
 
}

ISR(USART0_RXC_vect){
  incre++;
}

void loop() {
  Serial.print(incre);
  delay(500);
}
