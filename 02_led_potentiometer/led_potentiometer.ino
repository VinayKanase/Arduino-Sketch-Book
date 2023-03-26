
int ledPin = 4;
int ptm = A0;
int ptVal;
long ledVal;

void setup()
{
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
}

void loop()
{
  ptVal = analogRead(ptm);
  ledVal = 0.249 * ptVal;
  Serial.println(ptVal, ledVal);
  analogWrite(ledPin, ledVal);
}
