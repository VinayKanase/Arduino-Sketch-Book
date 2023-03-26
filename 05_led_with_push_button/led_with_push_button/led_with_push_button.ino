int ledPin = 13;
int pushBtnPin = 5;
void setup()
{
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(pushBtnPin, INPUT);
}

void loop()
{
 int btnState = digitalRead(pushBtnPin);
 Serial.println(btnState);
 if(btnState == HIGH)
    digitalWrite(ledPin, HIGH);
  else
    digitalWrite(ledPin, LOW);
}
