#include <Servo.h>//
Servo esc1;
Servo esc2;
Servo esc3; 
Servo esc4;
void setup()
{
  esc1.attach(1); //Specify the esc signal pin,Here as D8
  esc1.writeMicroseconds(1000); //initialize the signal to 1000
  esc2.attach(2); //Specify the esc signal pin,Here as D8
  esc2.writeMicroseconds(1000); //initialize the signal to 1000
  esc3.attach(3); //Specify the esc signal pin,Here as D8
  esc3.writeMicroseconds(1000); //initialize the signal to 1000
  esc4.attach(4); //Specify the esc signal pin,Here as D8
  esc4.writeMicroseconds(1000); //initialize the signal to 1000
  Serial.begin(9600);
}
void loop()
{
  int val; //Creating a variable val
  val= analogRead(A0);
  val= map(val, 0, 1023,1000,2000); //Since ESCs take 1000us to 2000us, but potentiometer inputs are 0 to 1023, need to map
  esc1.writeMicroseconds(val); //using val as the signal to esc
  esc2.writeMicroseconds(val);
  esc3.writeMicroseconds(val);
  esc4.writeMicroseconds(val);
  Serial.println(val);
}
