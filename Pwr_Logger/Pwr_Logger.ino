int currRaw;
float currVolt,currAmp,currPwr,totalEnergy,cycle,elapTime;

unsigned long timerNew,timerOld;

#define currentSenseResistor 100
#define loopDelay 10
#define voltThreshold 0.2

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Motor Power Logger");
  timerOld = 0;
  elapTime = 0;
}

void loop() {
  currRaw = analogRead(6);
  currVolt = 2.0*5.0*((currRaw*1.0)/1024.0);
  currAmp = currVolt/currentSenseResistor;
  currPwr = currVolt*currAmp;
  
  
  timerNew = millis();
  cycle = timerNew-timerOld;
  timerOld = timerNew;
  
  if (currVolt > voltThreshold){
    totalEnergy += (currPwr*((cycle*1.0)/1000.0));
    elapTime += ((cycle*1.0)/1000.0);
  
    commPrint();
  }
  delay(loopDelay);
}

void commPrint(){
  Serial.print(currVolt,4);
  Serial.print("\t");
  Serial.print(currAmp,4);
  Serial.print("\t");
  Serial.print(currPwr,4);
  Serial.print("\t");
  Serial.print(totalEnergy,4);
  Serial.print("\t");
  Serial.print(elapTime,4);
  Serial.print("\n");
}
