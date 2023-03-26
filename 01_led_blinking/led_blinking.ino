int pin1 = 13;

void setup()
{
  pinMode(pin1, OUTPUT);
}

void loop()
{
  digitalWrite(pin1, HIGH);
  delay(5000); 
  digitalWrite(pin1, LOW);
  delay(2000); 
}
