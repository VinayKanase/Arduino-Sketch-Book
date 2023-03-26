
int a = 13;
int b = 2;
int c = 3;
int d = 4;
int e = 5;
int f = 6;
int g = 7;
int dp = 8;
int d4 = 9;
int d3 = 10;
int d2 = 11;
int d1 = 12;
int btn1 = A0;

void setDigit(int x) // Defined setDigit (x), whose role is to open the port dx
{
  digitalWrite(d1, LOW);
  digitalWrite(d2, LOW);
  digitalWrite(d3, LOW);
  digitalWrite(d4, LOW);
  switch (x)
  {
  case 1:
    digitalWrite(d1, HIGH);
    break;
  case 2:
    digitalWrite(d2, HIGH);
    break;
  case 3:
    digitalWrite(d3, HIGH);
    break;
  default:
    digitalWrite(d4, HIGH);
    break;
  }
}

void setNumber(int x) // Defined setNumber (x), whose role is to display digital x
{
  switch (x)
  {
  case 1:
    one();
    break;
  case 2:
    two();
    break;
  case 3:
    three();
    break;
  case 4:
    four();
    break;
  case 5:
    five();
    break;
  case 6:
    six();
    break;
  case 7:
    seven();
    break;
  case 8:
    eight();
    break;
  case 9:
    nine();
    break;
  default:
    zero();
    break;
  }
}

void dispDec() // Decimal point setting Open
{
  digitalWrite(dp, LOW);
}

void clearLEDs() // Clear screen
{
  digitalWrite(a, HIGH);
  digitalWrite(b, HIGH);
  digitalWrite(c, HIGH);
  digitalWrite(d, HIGH);
  digitalWrite(e, HIGH);
  digitalWrite(f, HIGH);
  digitalWrite(g, HIGH);
  digitalWrite(dp, HIGH);
}

void zero() // Define those figures 0 cathode pin switch
{
  digitalWrite(a, LOW);
  digitalWrite(b, LOW);
  digitalWrite(c, LOW);
  digitalWrite(d, LOW);
  digitalWrite(e, LOW);
  digitalWrite(f, LOW);
  digitalWrite(g, HIGH);
}

void one() // Define those figures 1 cathode pin switch
{
  digitalWrite(a, HIGH);
  digitalWrite(b, LOW);
  digitalWrite(c, LOW);
  digitalWrite(d, HIGH);
  digitalWrite(e, HIGH);
  digitalWrite(f, HIGH);
  digitalWrite(g, HIGH);
}

void two() // Define those figures 2 cathode pin switch
{
  digitalWrite(a, LOW);
  digitalWrite(b, LOW);
  digitalWrite(c, HIGH);
  digitalWrite(d, LOW);
  digitalWrite(e, LOW);
  digitalWrite(f, HIGH);
  digitalWrite(g, LOW);
}

void three() // Define those figures 3 cathode pin switch
{
  digitalWrite(a, LOW);
  digitalWrite(b, LOW);
  digitalWrite(c, LOW);
  digitalWrite(d, LOW);
  digitalWrite(e, HIGH);
  digitalWrite(f, HIGH);
  digitalWrite(g, LOW);
}

void four() // Define those figures 4 cathode pin switch
{
  digitalWrite(a, HIGH);
  digitalWrite(b, LOW);
  digitalWrite(c, LOW);
  digitalWrite(d, HIGH);
  digitalWrite(e, HIGH);
  digitalWrite(f, LOW);
  digitalWrite(g, LOW);
}

void five() // Define those figures 5 cathode pin switch
{
  digitalWrite(a, LOW);
  digitalWrite(b, HIGH);
  digitalWrite(c, LOW);
  digitalWrite(d, LOW);
  digitalWrite(e, HIGH);
  digitalWrite(f, LOW);
  digitalWrite(g, LOW);
}

void six() // Define those figures 6 cathode pin switch
{
  digitalWrite(a, LOW);
  digitalWrite(b, HIGH);
  digitalWrite(c, LOW);
  digitalWrite(d, LOW);
  digitalWrite(e, LOW);
  digitalWrite(f, LOW);
  digitalWrite(g, LOW);
}

void seven() // Define those figures 7 cathode pin switch
{
  digitalWrite(a, LOW);
  digitalWrite(b, LOW);
  digitalWrite(c, LOW);
  digitalWrite(d, HIGH);
  digitalWrite(e, HIGH);
  digitalWrite(f, HIGH);
  digitalWrite(g, HIGH);
}

void eight() // Define those figures 8 cathode pin switch
{
  digitalWrite(a, LOW);
  digitalWrite(b, LOW);
  digitalWrite(c, LOW);
  digitalWrite(d, LOW);
  digitalWrite(e, LOW);
  digitalWrite(f, LOW);
  digitalWrite(g, LOW);
}

void nine() // Define those figures 9 cathode pin switch
{
  digitalWrite(a, LOW);
  digitalWrite(b, LOW);
  digitalWrite(c, LOW);
  digitalWrite(d, LOW);
  digitalWrite(e, HIGH);
  digitalWrite(f, LOW);
  digitalWrite(g, LOW);
}

int min = 0, seconds = 0, reset = 2, stop = 1; // time and control varibles
long int currentMillis = 0, lastMillis = 0; // variabels for millis()
int D1, D2, D3, D4; // Digits for Display

void setup()
{
  // put your setup code here, to run once:
  pinMode(d1, OUTPUT);
  pinMode(d2, OUTPUT);
  pinMode(d3, OUTPUT);
  pinMode(d4, OUTPUT);
  pinMode(a, OUTPUT);
  pinMode(b, OUTPUT);
  pinMode(c, OUTPUT);
  pinMode(d, OUTPUT);
  pinMode(e, OUTPUT);
  pinMode(f, OUTPUT);
  pinMode(g, OUTPUT);
  pinMode(dp, OUTPUT);

  pinMode(btn1, INPUT);
}

void loop()
{
  if (digitalRead(btn1) == LOW && reset == 2)
  {
    lastMillis = millis();
    stop = 0;
    reset = 0;
    delay(800);
  } 
  else if (digitalRead(btn1) == LOW && reset == 1)
  {
    stop = 1;
    currentMillis = 0;
    reset = 2;
    delay(800);
  }
  else if (digitalRead(btn1) == LOW && reset == 0)
  {
    stop = 1;
    reset = 1;
    delay(800);
  }
  if (stop != 1)
    currentMillis = millis() - lastMillis;

  seconds = currentMillis / 1000;
  min = currentMillis / 60000;
  seconds = seconds - (min * 60);

  D1 = min / 10 % 10;
  D2 = min % 10;
  D3 = seconds / 10 % 10;
  D4 = seconds % 10;

  for (int m = 0; m < 80; m++)
  {
    clearLEDs();   // Turn off all LED lights
    setDigit(1);   // Selection of a digital display
    setNumber(D1); // Display digital d1
    delayMicroseconds(200);

    clearLEDs();   // Turn off all LED lights
    setDigit(2);   // Select the first two digital display
    setNumber(D2); // Display digital d2
    dispDec();
    delayMicroseconds(200);

    clearLEDs(); // Turn off all LED lights
    setDigit(3); // Select the first three digital display
    // dispDec(3);          //Decimal display
    setNumber(D3); // Display digital d3
    delayMicroseconds(200);

    clearLEDs();   // Turn off all LED lights
    setDigit(4);   // Select the first four digital display
    setNumber(D4); // Display digital d4
    delayMicroseconds(200);
  }
}
