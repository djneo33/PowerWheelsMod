#include <Arduino.h>
#define SteeringInput 4
#define ThrottleInput 5
#define FourInput 3
#define Ain1 1
#define Ain2 2
#define A_PWM 6
#define Bin2 7
#define Bin1 8
#define B_PWM 9
#define L_PWM 10
#define R_PWM 11

unsigned long startTimmingThrottle;
unsigned long startTimmingSteering;
unsigned long startTimmingFour;
unsigned long four;
unsigned long Steering;
unsigned long Throttle;
unsigned long past;
unsigned long highest;
unsigned long lowest;
bool pinhighSteering;
bool pinhighThrottle;
bool pinhighFour;

void ReadSteering();
void ReadThrottle();
void ReadFour();

void setup()
{
  Serial.begin(9600);
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
  past = micros();
}

void loop()
{

  ReadSteering();
  ReadThrottle();
  ReadFour();
  Serial.println(Throttle);
  if (Throttle >= 1600)
  {
    digitalWrite(Ain1, HIGH);
    digitalWrite(Ain2, LOW);
    if (four >= 1700)
    {
      digitalWrite(Bin1, HIGH);
      digitalWrite(Bin2, LOW);
      analogWrite(B_PWM, map(Throttle, 1600, 2200, 0, 255));
    }
    else
    {
      digitalWrite(Bin1, HIGH);
      digitalWrite(Bin2, HIGH);
      analogWrite(B_PWM, 0);
    }
    analogWrite(A_PWM, map(Throttle, 1600, 2200, 0, 255));
  }
  if (Throttle <= 1400)
  {
    digitalWrite(Ain1, LOW);
    digitalWrite(Ain2, HIGH);
    if (four >= 1700)
    {
      digitalWrite(Bin1, LOW);
      digitalWrite(Bin2, HIGH);
      analogWrite(B_PWM, map(Throttle, 1400, 990, 0, 255));
    }
    else
    {
      digitalWrite(Bin1, HIGH);
      digitalWrite(Bin2, HIGH);
      analogWrite(B_PWM, 0);
    }
    analogWrite(A_PWM, map(Throttle, 1400, 990, 0, 255));
  }
  if (Steering >= 1600)
  {
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, map(Steering, 1600, 2200, 0, 255));
  }
  if (Steering <= 1400)
  {
    analogWrite(L_PWM, 0);
    analogWrite(R_PWM, map(Steering, 1400, 990, 0, 255));
  }
  if (Throttle >= 1400 && Throttle <= 1600){
    analogWrite(A_PWM,0);
    analogWrite(B_PWM,0);
  }
  if (Steering >= 1400 && Steering <= 1600){
    analogWrite(L_PWM,0);
    analogWrite(R_PWM,0);
  }

}

void ReadSteering()
{

  if (digitalRead(SteeringInput) == HIGH && pinhighSteering == false)
  {
    startTimmingSteering = micros();
    pinhighSteering = true;
  }
  if (digitalRead(SteeringInput) == LOW && pinhighSteering == true)
  {
    Steering = micros() - startTimmingSteering;
    pinhighSteering = false;
  }
}

void ReadThrottle()
{
  if (digitalRead(ThrottleInput) == HIGH && pinhighThrottle == false)
  {
    startTimmingThrottle = micros();
    pinhighThrottle = true;
  }
  if (digitalRead(ThrottleInput) == LOW && pinhighThrottle == true)
  {
    Throttle = micros() - startTimmingThrottle;
    pinhighThrottle = false;
  }
}

void ReadFour()
{
  if (digitalRead(FourInput) == HIGH && pinhighFour == false)
  {
    startTimmingFour = micros();
    pinhighFour = true;
  }
  if (digitalRead(FourInput) == LOW && pinhighFour == true)
  {
    four = micros() - startTimmingFour;
    pinhighFour = false;
  }
}
