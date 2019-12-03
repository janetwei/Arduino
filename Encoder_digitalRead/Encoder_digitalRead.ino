int PinA = 8, PinB = 9;   // pushbutton connected to digital pin 7
int A = 0, B = 0;     // variable to store the read value

void setup()
{
  Serial.begin(9600);
  pinMode(PinA, INPUT);      // sets the digital pin A as input
  pinMode(PinB, INPUT);      // sets the digital pin B as input
}

void loop()
{
  A = digitalRead(PinA);   // read the input pin
  Serial.print("A: ");
  Serial.print(A);
  Serial.print(", ");
  B = digitalRead(PinB);   // read the input pin
  Serial.print("B: ");
  Serial.println(B);
}
