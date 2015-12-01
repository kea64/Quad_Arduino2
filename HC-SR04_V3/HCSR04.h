#ifndef HCSR04_h
#define HCSR04_h

class HCSR04{
  public:
    HCSR04(int trigPin, int echoPin, int cutOffTime);
    
    void pingInterrupt();
    
    void ping();
    
    int getDist();  
    
  private:
    int trigPin_;
    int echoPin_;
    int cutOffTime_;
    int pingDist_;
    
    unsigned long startClock_;
};


#endif
