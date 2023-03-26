/***********************************************************
Description: PWM control the LED gradually from dark to 
             brighter, then from brighter to dark
Name: Vinay Sitaram Kanase
***********************************************************/
int ledpin1 = 11; //definition digital 11 pins as pin to control the LED
int ledpin2 = 9; //definition digital 11 pins as pin to control the LED
int ledpin3 = 3; //definition digital 11 pins as pin to control the LED

void setup ()
{
  pinMode(ledpin1, OUTPUT); //Set digital 11 port mode, the OUTPUT for the output
  pinMode(ledpin2, OUTPUT); //Set digital 11 port mode, the OUTPUT for the output
  pinMode(ledpin3, OUTPUT); //Set digital 11 port mode, the OUTPUT for the output
}
 
void loop()
{
   for (int a = 0; a <= 255; a++)  //Loop, PWM control of LED brightness increase
   {
     analogWrite(ledpin1, a);   //PWM output value a (0~255)
     analogWrite(ledpin2, a);   //PWM output value a (0~255)
     analogWrite(ledpin3, a);   //PWM output value a (0~255)
     delay(15);                //The duration of the current brightness level. 15ms           
   }
   for (int a = 255; a >= 0; a--)  //Loop, PWM control of LED brightness Reduced
   {
     analogWrite(ledpin1, a);   //PWM output value a (255~0)
     analogWrite(ledpin2, a);   //PWM output value a (255~0)
     analogWrite(ledpin3, a);   //PWM output value a (255~0)
     delay(15);                //The duration of the current brightness level. 15ms 
   }
   delay(100);                //100ms delay
}
