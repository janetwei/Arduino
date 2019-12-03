#include <Encoder.h>

Encoder Enc1(2,3);

long Encodervalue;
int PastEncodervalue = -999;
void setup()
{
  Serial.begin(9600); 
}

void loop()
{
  Encodervalue = Enc1.read();
  if(Encodervalue != PastEncodervalue){
    Serial.println(Encodervalue);
    PastEncodervalue = Encodervalue;
  }
}
