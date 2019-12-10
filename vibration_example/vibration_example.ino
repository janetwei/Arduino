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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(pinPWMA, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinAIN2, OUTPUT);

  pinMode(pinPWMB, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);

  pinMode(pinSTBY, OUTPUT);
}

// put your main code here, to run repeatedly:
void loop() {
  digitalWrite(pinSTBY, HIGH);
  
  if (Serial.available() > 0) {
    char inst;
    inst = Serial.read();

    if (inst == '1'){
      digitalWrite(pinAIN1, HIGH);
      digitalWrite(pinAIN2, LOW);
      analogWrite(pinPWMA, 255);
    }
    else if (inst == '2'){
      digitalWrite(pinBIN1, HIGH);
      digitalWrite(pinBIN2, LOW);
      analogWrite(pinPWMB, 255);
    }
  }
  digitalWrite(pinSTBY, LOW);
}
