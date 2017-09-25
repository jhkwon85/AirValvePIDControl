#include <PWM.h>

int32_t PWMFrequency = 20; //PWM frequency (in Hz)
int StrainGuage;
int CurrentForce;
int Error = 0;
int Error_derevative = 0;
int Error_integral = 0;
int DesiredForce;
int ControlFoce;
int Kp = 1; // propotional gain
int Kd = 0; // derivative gain
int Ki = 0; // integral gain

char buf[20];

void setup() {
  InitTimersSafe(); //Timer1(PIN9,10), Timer2(PIN11,3) Initialize
  bool success = SetPinFrequencySafe(9, PWMFrequency);
  //bool success = SetPinFrequencySafe(11, PWMFrequency);
  if(success) {
    pinMode(9, OUTPUT);
    digitalWrite(9, HIGH);    
  }
}

void loop() {
  pwmWrite(9, 0); 
  delay(5000);
  pwmWrite(9, 0); 
  delay(5000);
  /*
  analogeRead(A0);
  StrainGuage = analogeRead(A0);
  CurrentForce = StrainGuage; 
  Error = CurrentForce - DesiredForce; 
  Error_derevative = (CurrentForce - CurentForce_before) 
  ControlForce = Kp*Error + kd*Error_derevative + Ki*Error_integral; //PID Controller

  //Command Limit Setting
  if(ControlForce > 255) ControlForce=255;
  if(ControlForce < -255) ControlForce =-255;

  //Air Input Valve Control
  if(ControlForce >= 0)
  {
      pwmWrite(9, ControlForce); //air input, 0~255
  }
  
  //Air Vent Valve Control
  if(ControlForce < 0)
  {
    pwmWrite(10, -ControlForce); //air vent, 0~255
  }
 
  CurrentForce_before = CurrentForce;
  Error_integral = Error_integral + Error;

  sprintf(buf, "A,%d, B,%d, C,%d, D,%d,,", StrainGuage, CurrentForce, Error, ControlForce);
  delay(10);
  */

  
}
