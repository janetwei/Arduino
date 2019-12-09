#include <Encoder.h>

// pin2, pin3 are interrupt pin
Encoder Enc1(2,4);
Encoder Enc2(3,7);

long newPosition1;
long oldPosition1  = -999;
long newPosition2;
long oldPosition2  = -999;

//Define the Pins
//Motor 1
int pinAIN1 = 9; //Direction 
int pinAIN2 = 8; //Direction 
int pinPWMA = 5; //Speed

//Motor 2
int pinBIN1 = 11; //Direction
int pinBIN2 = 12; //Direction
int pinPWMB = 6; //Speed

//Standby
int pinSTBY = 10;

//Constants to help remember the parameters
static boolean turnCW = 0;  //for motorDrive function
static boolean turnCCW = 1; //for motorDrive function
static boolean motor1 = 0;  //for motorDrive, motorStop, motorBrake functions
static boolean motor2 = 1;  //for motorDrive, motorStop, motorBrake functions

// Set motor variable
int cpr = 12;
int gearratio = 211; 

//Motor variables
int voltageFeed;
int voltageFeed2;
const bool turnDirection = true;  //for motorDrive function
const bool turnDirection2 = true;

//PID control
int targetCount = 0;
int targetCount2 = 0;
int curCount = 0;
int curCount2 = 0;
long integral;
long integral2;
const int KiActivationErrorRange = 100;
const int KiRange2 = 10;
const float KiRange2Factor = 2;
const int KdActivationErrorRange = 100;
//tuning guidelines: (Voltage feed = Kp*(error) + Ki*error.usPassed + Kd*encoderSpeed)
//Kp: error approx 100(small step / 10 motor rounds) -> 10,000(big step) -> around 2.5E-1
//Ki: error approx 1.0E3, total us in transition approx 1.0E6 -> 2.5E-7
//Kd: approx 1.0E-1 encoder counts/us -> 2.5E+3
//const float Kp = 2.5e-1;
//const float Ki = 5e-7;
//const float Kd = -5e3; // negative here to compensate for different speeds when approaching target


////////////////////////////////////////////////////////////////////////////////////////////////////////
//TODO: Adjust the variable
const float Kp = ;
const float Ki = ;
const float Kd = ; // negative here to compensate for different speeds when approaching target

////////////////////////////////////////////////////////////////////////////////////////////////////////

int encoderValue = 2532;
int maximalVoltage = 255;

//global variables
unsigned long usPassed;
unsigned long usPassed2;
float encoderSpeed = 0;
float encoderSpeed2 = 0;


//Encoder Variables
const unsigned short acceptableCountError = 0;
const unsigned short activationCountError = 1;
bool motorActive;
bool motor2Active;
bool encoder1OutA_CurState, encoder1OutB_CurState, encoder1OutA_PrevState, encoder1OutB_PrevState;
bool encoder2OutA_CurState, encoder2OutB_CurState, encoder2OutA_PrevState, encoder2OutB_PrevState;

void setup()
{
  Serial.begin(9600);
  
  //Set the PIN Modes
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinAIN2, OUTPUT);

  pinMode(pinPWMB, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);

  pinMode(pinSTBY, OUTPUT);

}

void loop()
{ 
  read_encoder();
  getusPassed();
  getEncoderSpeed();
  getVoltageFeedFromPID();
  updateMotor();
}

void serialEvent() {
  char inst;
  inst = Serial.read();
    
  if (inst == 'm'){
    float num = Serial.parseFloat();
    motor_rotate(num);
  }
}

void motor_rotate(float bal)
{
  //Serial.println(bal);
  targetCount = bal * encoderValue;
  Serial.print("targetCount");
  Serial.println(targetCount);
  integral = 0; //reset integral in PID control
  motorActive = true;
}


void getVoltageFeedFromPID()
{
  int error;
  float I, D;

  error = targetCount - newPosition1;
  // Integral
  if (abs(error) < KiActivationErrorRange) {
    integral += error * usPassed;
    if (abs(error) < KiRange2) {
      I = Ki * KiRange2Factor * integral;
    } else {
      I = Ki * integral;
    }
  } else {
    integral = 0;
    I = 0;
  }
  //Derivative
  if (abs(error) < KdActivationErrorRange) {
    D = Kd * encoderSpeed;
  } else {
    D = 0;
  }

  //Derive driving voltage
  if (abs(targetCount - newPosition1) > activationCountError) {
    motorActive = true;
  }
  if (abs(targetCount - newPosition1) > acceptableCountError && motorActive) { // after motor has reached acceptableCountError, the error needs to excceed activationCountError to start the motor again.
    voltageFeed = Kp * (error) + I + D;

    if (voltageFeed > 0) {
      if (voltageFeed > maximalVoltage) {
        voltageFeed = maximalVoltage;
      }
      //motorDrive(motor1, turnDirection, voltageFeed);
    } else {
      if (voltageFeed < -maximalVoltage) {
        voltageFeed = -maximalVoltage;
      }
      //motorDrive(motor1, !turnDirection, -voltageFeed);
    }
  } else {
    integral = 0;
    voltageFeed = 0;
  }

  /////////////////
  int error2;
  float I2, D2;

  error2 = targetCount2 - newPosition2;
  // Integral
  if (abs(error2) < KiActivationErrorRange) {
    integral2 += error2 * usPassed2;
    if (abs(error2) < KiRange2) {
      I2 = Ki * KiRange2Factor * integral2;
    } else {
      I2 = Ki * integral2;
    }
  } else {
    integral2 = 0;
    I2 = 0;
  }
  //Derivative
  if (abs(error2) < KdActivationErrorRange) {
    D2 = Kd * encoderSpeed2;
  } else {
    D2 = 0;
  }

  //Derive driving voltage
  if (abs(targetCount2 - newPosition2) > activationCountError) {
    motor2Active = true;
  }
  if (abs(targetCount2 - newPosition2) > acceptableCountError && motor2Active) { // after motor has reached acceptableCountError, the error needs to excceed activationCountError to start the motor again.
    voltageFeed2 = Kp * (error2) + I2 + D2;
    
    if (voltageFeed2 > 0) {
      if (voltageFeed2 > maximalVoltage) {
        voltageFeed2 = maximalVoltage;
      }
      //motorDrive(motor1, turnDirection, voltageFeed);
    } else {
      if (voltageFeed2 < -maximalVoltage) {
        voltageFeed2 = -maximalVoltage;
      }
      //motorDrive(motor1, !turnDirection, -voltageFeed);
    }
  } else {
    integral2 = 0;
    voltageFeed2 = 0;
  }
}

void updateMotor() {
  if (voltageFeed > 0) {
    motorDrive(motor1, turnDirection, voltageFeed);
  } else if (voltageFeed < 0) {
    motorDrive(motor1, !turnDirection, -voltageFeed);
  }
  else  {
    motorActive = false;
    motorStop(motor1);
    motorsStandby();
  }
}

void getusPassed() {
  //Serial.println("getusPassed");
  static unsigned long prevTime = 0;
  unsigned long curTime;
  static unsigned long prevTime2 = 0;
  unsigned long curTime2;

  curTime = micros();
  usPassed = curTime - prevTime;
  prevTime = curTime;

  curTime2 = micros();
  usPassed2 = curTime2 - prevTime2;
  prevTime2 = curTime2;
}


void getEncoderSpeed() {
  static unsigned long usPassedBetweenEncoderReadings = 0;
  static int prevCount = 0;
  float newSpeed;
  static unsigned long usPassed2BetweenEncoderReadings = 0;
  static int prevCount2 = 0;
  float newSpeed2;

  usPassedBetweenEncoderReadings += usPassed;
  if (usPassed > 1000) {
    newSpeed = (float)(curCount - prevCount) * 0.001;
    prevCount = curCount;
    usPassedBetweenEncoderReadings -= 1000;
    encoderSpeed = encoderSpeed * 0.8 + (newSpeed) * 0.2;
  }

  usPassed2BetweenEncoderReadings += usPassed2;
  if (usPassed2 > 1000) {
    newSpeed2 = (float)(curCount2 - prevCount2) * 0.001;
    prevCount2 = curCount2;
    usPassed2BetweenEncoderReadings -= 1000;
    encoderSpeed2 = encoderSpeed2 * 0.8 + (newSpeed2) * 0.2;
  }

}

void motorDrive(boolean motorNumber, boolean motorDirection, int motorSpeed)
{
  /*
  This Drives a specified motor, in a specific direction, at a specified speed:
    - motorNumber: motor1 or motor2 ---> Motor 1 or Motor 2
    - motorDirection: turnCW or turnCCW ---> clockwise or counter-clockwise
    - motorSpeed: 0 to 255 ---> 0 = stop / 255 = fast
  */

  boolean pinIn1;  //Relates to AIN1 or BIN1 (depending on the motor number specified)

 
  //Specify the Direction to turn the motor
  //Clockwise: AIN1/BIN1 = HIGH and AIN2/BIN2 = LOW
  //Counter-Clockwise: AIN1/BIN1 = LOW and AIN2/BIN2 = HIGH
  if (motorDirection == turnCW){
    pinIn1 = HIGH;
  }
  else{
    pinIn1 = LOW;
  }

  //Select the motor to turn, and set the direction and the speed
  if(motorNumber == motor1)
  {
    digitalWrite(pinAIN1, pinIn1);
    digitalWrite(pinAIN2, !pinIn1);  //This is the opposite of the AIN1
    analogWrite(pinPWMA, motorSpeed);
  }
  else
  {
    digitalWrite(pinBIN1, !pinIn1);
    digitalWrite(pinBIN2, pinIn1);  //This is the opposite of the BIN1
    analogWrite(pinPWMB, motorSpeed);
  }
   
 

  //Finally , make sure STBY is disabled - pull it HIGH
  digitalWrite(pinSTBY, HIGH);

}

void motorBrake(boolean motorNumber)
{
/*
This "Short Brake"s the specified motor, by setting speed to zero
*/

  if (motorNumber == motor1){
    analogWrite(pinPWMA, 0);
  }
  else{
    analogWrite(pinPWMB, 0);
  }
}


void motorStop(boolean motorNumber)
{
  /*
  This stops the specified motor by setting both IN pins to LOW
  */
  if (motorNumber == motor1) {
    digitalWrite(pinAIN1, LOW);
    digitalWrite(pinAIN2, LOW);
  }
  else
  {
    digitalWrite(pinBIN1, LOW);
    digitalWrite(pinBIN2, LOW);
  } 
}


void motorsStandby()
{
  /*
  This puts the motors into Standby Mode
  */
  digitalWrite(pinSTBY, LOW);
}


void read_encoder() {
  newPosition1 = Enc1.read();
  if (newPosition1 != oldPosition1) {
    oldPosition1 = newPosition1;
    Serial.println(newPosition1);
  }
}
