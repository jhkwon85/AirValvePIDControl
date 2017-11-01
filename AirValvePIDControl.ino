#include <PWM.h>
enum CONTROLMODE {NO_CONTROL, P_CONTROL, PD_CONTROL, PID_CONTROL,};

/////  USER INPUT ///////
int32_t PWMFrequency = 20; //PWM frequency [Hz]
int duty = 50; //Duty Cytle Input from User (0~100%)
float desiredForce = 30; //Newton [N]
float coefX1 = 0.4723;  // Strainguage coefficient, Can be changed(if necessary)
float coefX0 = -296.16; // Strainguage coefficient, Can be changed(if necessary)
CONTROLMODE controlMode = P_CONTROL; // CONTROL MODE SETTING
float Kp = 1; // propotional gain
float Kd = 0.01; // derivative gain
float Ki = 0.01; // integral gain
/////////////////////////

int strainGuage;
float currentForce; //Calibrated Current force
float error = 0;
float previousError = 0;
float errorDerevative = 0;
float errorIntegral = 0;
float command; // PID control command

const long interval = 50; // controller cycle [ms]
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;

char buf[20]; // serial print buf

void setup() {
  //PWM Init.
  InitTimersSafe(); //Timer1(PIN9,10), Timer2(PIN11,3) Initialize
  SetPinFrequencySafe(9, PWMFrequency);
  
  // Valve status Init.
  pwmWrite(9, 0); // Air-In Valve(PIN9) Close(0)
  pwmWrite(10, 255); // Air-Out Valve(PIN0) Fully Open(255) 
  delay(1000);
  
  // Serial Comm. Begin
  Serial.begin(9600);
}

void loop() {
  unsigned long currentMillis = millis();
  
  if(currentMillis - previousMillis >= interval) { 
    ReadCurrentForce();
    PIDController(controlMode);
    ValveControl();
    
    sprintf(buf, "A,%d, B,%d, C,%d, D,%d,,", strainGuage, currentForce, error, command);
    Serial.println(buf);
  }
}

//Functions
void ReadCurrentForce()
{
  strainGuage = analogRead(A0);
  currentForce = (coefX1 * strainGuage) +  coefX0;
  if(currentForce < 0) currentForce = 0;
}

void PIDController(CONTROLMODE controlMode)
{
  switch (controlMode) 
  {
      case NO_CONTROL: command = 0;
      case P_CONTROL: 
      {
          error = currentForce - desiredForce;
          command = Kp * error; //P
      }
      case PD_CONTROL: 
      {
          error = currentForce - desiredForce;
          errorDerevative = error - previousError;
          command = (Kp * error) + (Kd * errorDerevative); //PD
          previousError = error;
      }
      case PID_CONTROL: 
      {
          error = currentForce - desiredForce;
          errorDerevative = error - previousError;
          errorIntegral = errorIntegral + error;
          command = (Kp * error) + (Kd * errorDerevative) + (Ki * errorIntegral); //PID
          previousError = error;
      }
  }
}

void ValveControl()
{
  //Command Limit Setting
  if(command > 255) command = 255;
  if(command < -255) command = -255;
    
  //Air Input Valve Control
  if(command >= 0)
  {
    pwmWrite(9, command); //air input, 0~255
    pwmWrite(10, 0); //air vent, 0~255
  }
    
  //Air Vent Valve Control
  if(command < 0)
  {
    pwmWrite(9, 0); //air input, 0~255
    pwmWrite(10, -command); //air vent, 0~255
  }
}

