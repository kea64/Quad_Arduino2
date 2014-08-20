#include <SoftwareSerial.h> 
#include <TinyGPS++.h>
#define RXPin 9    
#define TXPin 8
#define GPSBaud 38400
#define ConsoleBaud 115200

// The serial connection to the GPS device

SoftwareSerial ss(RXPin, TXPin);

// The TinyGPS++ object
TinyGPSPlus gps;

void setup()
{
  
  Serial.begin(ConsoleBaud);
  ss.begin(GPSBaud);
  ss.print("$PUBX,41,1,0007,0003,4800,0*13\r\n"); // Change iTead baudrate
  ss.begin(4800); // reset SoftwareSerial baudrate
  ss.flush();
  

  Serial.println("GPS Example 2");
  Serial.println("A simple tracker using TinyGPS++.");
  Serial.println("by Mikal Hart");
  Serial.println();
}

void loop()
{
  // If any characters have arrived from the GPS,
  // send them to the TinyGPS++ object
  while (ss.available() > 0){
    char c = byte(ss.read());
    gps.encode(c);
  }
    
  // Let's display the new location and altitude
  // whenever either of them have been updated.
  if (gps.location.isUpdated() || gps.altitude.isUpdated())
  {
    Serial.print("Location: "); 
    Serial.print(gps.location.lat(), 6);
    Serial.print(",");
    Serial.print(gps.location.lng(), 6);
    Serial.print("  Altitude: ");
    Serial.println(gps.altitude.meters());
  }
}
