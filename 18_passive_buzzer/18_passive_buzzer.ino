int buzzer = 9; // set the buzzer control digital IO pin 

void setup()
{
  pinMode(buzzer, OUTPUT); // set pin 9 as output
} 
void loop() {  
  for (int i = 0; i < 80; i++) {  // make a sound   
    digitalWrite(buzzer, HIGH); // send high signal to buzzer    
    delay(1); // delay 1ms   
    digitalWrite(buzzer, LOW); // send low signal to buzzer   
    delay(1);  
  }  
  delay(50);  
  for (int j = 0; j < 100; j++) { //make another sound   
    digitalWrite(buzzer, HIGH);   
    delay(2); // delay 2ms   
    digitalWrite(buzzer, LOW);   
    delay(2);  
  }  
  delay(100); 
} 
