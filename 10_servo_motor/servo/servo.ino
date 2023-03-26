/***********************************************************
File name: 09_servo.ino
Description:   The servo motor are rotated to 15 degrees, 30 
               degrees, 45 degrees, 60 degrees, 75 degrees, 
               90 degrees, 75 degrees, 60 degrees, 45 degrees,
               30 degrees, 15 degrees, 0 degrees, and then from 
               0 degrees to 180 degrees and from 180 degrees to
               0 degrees.
Website: www.quadstore.in
***********************************************************/

#include <Servo.h>
Servo myservo;//create servo object to control a servo

void setup()
{
  myservo.attach(6);//attachs the servo on pin 9 to servo object
  myservo.write(0);//back to 0 degrees 
  delay(1000);//wait for a second
}

void loop()
{  
  for(int num=0;num<=180;num++)
  {
     myservo.write(num);//back to 'num' degrees(0 to 180)
     delay(10);//control servo speed
  }
  for(int num=180;num>=0;num--)
  {
     myservo.write(num);//back to 'num' degrees(180 to 0)
     delay(10);//control servo speed 
  }
}
