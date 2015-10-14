

void setup() {
  // initialize both serial ports:
  Serial.begin(115200);
  Serial2.begin(38400);
}

void loop() {
  // read from port 1, send to port 0:
  if (Serial2.available()) {
    int inByte = Serial2.read();
    Serial.write(inByte); 
  }
  
  // read from port 0, send to port 1:
  if (Serial.available()) {
    int inByte = Serial.read();
    Serial2.write(inByte); 
  }
}
