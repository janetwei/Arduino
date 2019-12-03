#include <Encoder.h>

// pin2, pin3 are interrupt pin
Encoder Enc1(2,4);
Encoder Enc2(3,7);

long newPosition1;
long oldPosition1  = -999;

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

  if (Serial.available() > 0){
    char inst;
    inst = Serial.read();
    Serial.print("Instruction: ");
    Serial.println(inst);

    if(inst == 'c'){
      motor_rotate(-1);
      delay(1000);
    }
    else if(inst == 'd'){
      motor_rotate(1);
      delay(1000);
    }
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



void motor_rotate(float loopnumber) {
  //int rotationspeed;//, loopnumber = 3;
  long current = oldPosition1;
  long target = current + (loopnumber * cpr * gearratio);
  if (loopnumber >= 0){
    motorDrive(motor1, turnCCW, 255);
    while (oldPosition1 < target) {
      read_encoder();
    }
    motorBrake(motor1);
  }
  else{
    motorDrive(motor1, turnCW, 255);
    while (oldPosition1 > target) {
      read_encoder();
    }
    motorBrake(motor1);
  }
}



void read_encoder() {
  newPosition1 = Enc1.read();
  if (newPosition1 != oldPosition1) {
    oldPosition1 = newPosition1;
    Serial.println(newPosition1);
  }
}
