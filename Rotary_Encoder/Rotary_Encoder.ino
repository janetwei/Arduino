int PinA = 8 , PinB = 9;   // pushbutton connected to digital pin 7
int A = 0, B = 0;     // variable to store the read value
int recounter = 0;
int reA, reB, rePA, rePB;
void setup()
{
  Serial.begin(9600);
  pinMode(PinA, INPUT);
  pinMode(PinB, INPUT);
  rePA = digitalRead(PinA);
  rePB = digitalRead(PinB);
}

void loop()
{
  reA = digitalRead(PinA); // Reads the "current" state of the outputA
  reB = digitalRead(PinB);
  
  
  // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (reA != rePA || reB != rePB) {
    // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
    if(reA != rePA && reB != rePB){ // rotate too fast, miss 1 change
      recounter += 2;
    }
    else if (reA == rePB) {
      recounter ++;
    } else {
      recounter --;
    }
    Serial.print("A: ");
    Serial.print(reA);
    Serial.print(", ");
    Serial.print("B: ");
    Serial.println(reB);
    Serial.print("Position: ");
    Serial.println(recounter);
  }
  
  rePA = reA; // Updates the previous state of the outputA with the current state
  rePB = reB;
}
