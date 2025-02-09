#include <Arduino.h>
#define SteeringInput 2
#define ThrottleInput 3
#define ForwardInput 4
#define FourInput 0
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
unsigned long startTimmingForward;
unsigned long startTimmingFour;
unsigned long Forward;
volatile unsigned long Steering;
volatile unsigned long Throttle;
unsigned long past;
unsigned long highest;
unsigned long lowest;
bool pinhighSteering;
bool pinhighThrottle;
bool pinhighForward;
bool pinhighFour;
unsigned long Four;
unsigned long currentTime;



void ReadPWM();
void getthrottle();
void getsteering();
void ForwardCommand(bool);
void ReverseCommand(bool);

void setup()
{
  
  pinMode(SteeringInput, INPUT);
  pinMode(ThrottleInput, INPUT);
  pinMode(ForwardInput, INPUT);
  pinMode(FourInput,INPUT);
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
  pinhighForward = false;
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
  
  

  
 
  if (Throttle >= 1600 && Forward > 1700)
  {
   
  if (Throttle > 2300){
    Throttle = 1500;
   }
    
    if (Four > 1700)
    {
      ForwardCommand(true);
      analogWrite(B_PWM, map(Throttle, 1600, 2000, 0, 255));
    }
    else
    {
      ForwardCommand(false);
      analogWrite(B_PWM, 0);
    }
    analogWrite(A_PWM, map(Throttle, 1600, 2000, 0, 255));
 
  }
  
    if (Throttle >= 1600 && Forward < 1400)
  {
   
  if (Throttle > 2300){
    Throttle = 1500;
   }
    
    if (Four > 1700)
    {
      ReverseCommand(true);
      analogWrite(B_PWM, map(Throttle, 1600, 2000, 0, 255));
    }
    else
    {
      ReverseCommand(false);
      analogWrite(B_PWM, 0);
    }
    analogWrite(A_PWM, map(Throttle, 1600, 2000, 0, 255));
 
  }
  
  
  
  
  if (Throttle <= 1400){
   
    analogWrite(A_PWM,0);
    analogWrite(B_PWM,0);
   
    
    
    digitalWrite(Ain1,LOW);
    digitalWrite(Ain2,LOW);
    digitalWrite(Bin1,HIGH);
    digitalWrite(Bin2,HIGH);
   
 
   
    
  }
    if (Throttle <= 1200){
   
    analogWrite(A_PWM,0);
    analogWrite(B_PWM,0);
   
    
    
    digitalWrite(Ain1,LOW);
    digitalWrite(Ain2,LOW);
    digitalWrite(Bin1,LOW);
    digitalWrite(Bin2,LOW);
   
 
   
    
  }
  if (Throttle > 1400 && Throttle < 1600 ){
     digitalWrite(Ain1,HIGH);
    digitalWrite(Ain2,HIGH);
    digitalWrite(Bin1,HIGH);
    digitalWrite(Bin2,HIGH);

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
 if (digitalRead(ForwardInput) == HIGH && pinhighForward == false ){
  startTimmingForward = currentTime;
  pinhighForward = true;
 }
  if (digitalRead(ForwardInput) == LOW && pinhighForward == true ){
    Forward = currentTime - startTimmingForward;
    pinhighForward = false;
   

 
 }
if(digitalRead(FourInput) == HIGH && pinhighFour == false){
startTimmingFour = currentTime;
pinhighFour = true;

}
if(digitalRead(FourInput) == LOW && pinhighFour == true){
  Four = currentTime - startTimmingFour;
  pinhighFour = false;
}
}

void ForwardCommand(bool FourWheel){
  digitalWrite(Ain1,HIGH);
  digitalWrite(Ain2,LOW);
  if (FourWheel){
    digitalWrite(Bin1,HIGH);
    digitalWrite(Bin2,LOW);

  }else{
    digitalWrite(Bin1,HIGH);
    digitalWrite(Bin2,HIGH);
  }

}


void ReverseCommand(bool FourWheel){
  digitalWrite(Ain1,LOW);
  digitalWrite(Ain2,HIGH);
  if (FourWheel){
    digitalWrite(Bin1,LOW);
    digitalWrite(Bin2,HIGH);

  }else{
    digitalWrite(Bin1,HIGH);
    digitalWrite(Bin2,HIGH);
  }
}