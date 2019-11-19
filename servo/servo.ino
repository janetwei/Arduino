#include <Servo.h>

Servo myservo; // 建立Servo物件，控制伺服馬達

void setup() 
{ 
  myservo.attach(9); // 連接數位腳位9，伺服馬達的訊號線
} 

void loop() 
{
  myservo.write(180);
  delay(500);
  myservo.write(0);
  delay(500);
}
