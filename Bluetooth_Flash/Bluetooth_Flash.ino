void setup() {
  Serial1.begin(115200);
  delay(2000);
  Serial1.print("AT");
  delay(1000);
  Serial1.print("AT+NAMESPYDER15");
  delay(1000);
  Serial1.print("AT+BAUD8");
  delay(1000);
  Serial1.print("AT+PIN4578");
  delay(1000);
}

void loop() {
  

}
