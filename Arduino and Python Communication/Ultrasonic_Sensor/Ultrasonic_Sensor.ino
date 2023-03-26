#include <NewPing.h>

#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     13  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 100 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

int distance;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup() {
  Serial.begin(9600); // Open serial monitor at 9600 baud to communicate with python 
}

int getDistance(){
  distance = sonar.ping_cm();
  if(distance == 0){
    distance = 50;
  }

  return distance;
}
void loop() {
  delay(50);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  getDistance(); // Send ping, get distance in cm and print result (0 = outside set distance range)
  if((distance > 30) && (distance < 40)){
    delay(50);
    if((distance > 35      ) && (distance < 50))
      Serial.println("Play/Pause");
  }
  getDistance();
  if(distance >= 15 && distance <= 25){
        Serial.println("Volume Up");
  }
  getDistance();
  if(distance >= 2 && distance <= 12){
    Serial.println("Volume Down");
  }
  delay(50);    
}
