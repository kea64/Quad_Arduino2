//Sharp IR Distance Sensor Read Code

int IRRaw,IR;
boolean landed_true;
#define IRPin 0
#define IRAlpha 0.8

void setup() {
  Serial.begin(115200);
  landed_true = 0;
  

}

void loop() {
  getIR();
  Serial.print(landed_true);
  Serial.print("  ");
  Serial.println(IR);
  delay(50);
}

void getIR(){
  IRRaw = analogRead(IRPin);
  IR = IRAlpha * IR + (1-IRAlpha) * IRRaw;
  if (IR > 380){
    landed_true = 1;
  } else {
    //landed_true = 0;
  }
}
