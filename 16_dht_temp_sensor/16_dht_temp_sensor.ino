#include <dht.h>

#define dht_apin 9
dht DHT;

void setup() {
  Serial.begin(9600);
  Serial.println("Ready!!!");

}

void loop() {
  DHT.read11(dht_apin);
    Serial.print("Current humidity = ");
    Serial.print(DHT.humidity);
    Serial.print("%  ");
    Serial.print("temperature = ");
    Serial.print(DHT.temperature); 
    Serial.println("C  ");
    delay(2000);
}
