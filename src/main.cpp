#include <Arduino.h>
#define SteeringInput 2
#define ThrottleInput 3
#define FourInput 4
#define Ain1 1
#define Ain2 5
#define A_PWM 6
#define Bin2 7
#define Bin1 8
#define B_PWM 9
#define L_PWM 10
#define R_PWM 11

volatile unsigned long startTimmingThrottle;
volatile unsigned long startTimmingSteering;
unsigned long startTimmingFour;
unsigned long four;
volatile unsigned long Steering;
volatile unsigned long Throttle;
unsigned long past;
unsigned long highest;
unsigned long lowest;
bool pinhighSteering;
bool pinhighThrottle;
bool pinhighFour;
unsigned long currentTime;



void ReadPWM();
void getthrottle();
void getsteering();

void setup()
{
  
  pinMode(SteeringInput, INPUT);
  pinMode(ThrottleInput, INPUT);
  pinMode(FourInput, INPUT);
  pinMode(Ain1, OUTPUT);
  pinMode(Ain2, OUTPUT);
  pinMode(A_PWM, OUTPUT);
  pinMode(Bin2, OUTPUT);
  pinMode(Bin1, OUTPUT);
  pinMode(B_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinhighSteering = false;
  pinhighThrottle = false;
  pinhighFour = false;
  analogWrite(A_PWM,0);
  analogWrite(B_PWM,0);
  analogWrite(L_PWM,0);
  analogWrite(R_PWM,0);
  attachInterrupt(digitalPinToInterrupt(3),getthrottle,CHANGE);
  attachInterrupt(digitalPinToInterrupt(2),getsteering,CHANGE);

  
  delay(1000);

}
void loop()
{

 ReadPWM();
  
  

  
 
  if (Throttle >= 1600)
  {
   
  if (Throttle > 2200){
    Throttle = 1500;
   }
    digitalWrite(Ain1, HIGH);
    digitalWrite(Ain2, LOW);
    if (four > 1700)
    {
      digitalWrite(Bin1, HIGH);
      digitalWrite(Bin2, LOW);
      analogWrite(B_PWM, map(Throttle, 1600, 2000, 0, 255));
    }
    else
    {
      digitalWrite(Bin1, HIGH);
      digitalWrite(Bin2, HIGH);
      analogWrite(B_PWM, 0);
    }
    analogWrite(A_PWM, map(Throttle, 1600, 2000, 0, 255));
 
  }
  
  if (Throttle <= 1400)
  {
      if(Throttle < 800){
    Throttle = 1500;
   }
    
    digitalWrite(Ain1, LOW);
    digitalWrite(Ain2, HIGH);
    if (four > 1700)
    {
      digitalWrite(Bin1, LOW);
      digitalWrite(Bin2, HIGH);
      analogWrite(B_PWM, map(Throttle, 1400, 1000, 0, 255));
    }
    else
    {
      digitalWrite(Bin1, HIGH);
      digitalWrite(Bin2, HIGH);
      analogWrite(B_PWM, 0);
    }
   
 
   
    analogWrite(A_PWM, map(Throttle, 1400, 1000, 0, 255));
  }
  
  if (Steering >= 1600)
  
  {
   if (Steering > 2200){
    Steering = 1500;
  }
   
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, map(Steering, 1600, 2000, 0, 255));
  }
  if (Steering <= 1400)
  
  {
    if (Steering < 800){
    Steering = 1500;
  }
    
    analogWrite(L_PWM, 0);
    analogWrite(R_PWM, map(Steering, 1400, 1000, 0, 255));
  }
  if (Throttle >= 1400 && Throttle <= 1600){
    analogWrite(A_PWM,0);
    analogWrite(B_PWM,0);
    digitalWrite(Ain1,0);
    digitalWrite(Ain2,0);
    digitalWrite(Bin1,0);
    digitalWrite(Bin2,0);
  }
  if (Steering >= 1400 && Steering <= 1600){
    analogWrite(L_PWM,0);
    analogWrite(R_PWM,0);
  }
 
}

void getthrottle(){
if(digitalRead(ThrottleInput) == HIGH){
  startTimmingThrottle = micros();

}else{
Throttle = micros()-startTimmingThrottle;

}


}

void getsteering(){
  if(digitalRead(SteeringInput) == HIGH){
    startTimmingSteering = micros();
  }else{
    Steering = micros()-startTimmingSteering;
  }
}
void ReadPWM()
{
 
  unsigned long currentTime = micros();
 if (digitalRead(FourInput) == HIGH && pinhighFour == false ){
  startTimmingFour = currentTime;
  pinhighFour = true;
 }
  if (digitalRead(FourInput) == LOW && pinhighFour == true ){
    four = currentTime - startTimmingFour;
    pinhighFour = false;
   
    

 
 }


}

