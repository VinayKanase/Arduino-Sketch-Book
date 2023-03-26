
int delay1 = 3;
int delay2 = 2;
int delay3 = 5;

int pin1 = 7;
int pin2 = 2;
int pin3 = 3;
void setup() {
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(pin1, HIGH);
  delay(delay1*1000);
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, HIGH);
  delay(delay2*1000);
  digitalWrite(pin2, LOW);
  digitalWrite(pin3, HIGH);
  delay(delay3*1000);
  digitalWrite(pin3, LOW);
}
