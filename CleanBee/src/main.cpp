#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

//Code based on DHT library example

// put function declarations here:
void getTemperature();
void getHumidity();

#define DHTPIN 18     // Digital pin connected to the DHT sensor 

#define DHTTYPE    DHT22     // DHT 22 (AM2302)

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS = 1000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  dht.begin();
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(delayMS);
  getTemperature();
  getHumidity();
}

// put function definitions here:
void getTemperature() {
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));

    if (event.temperature > 25.5){
      digitalWrite(22, HIGH);
      digitalWrite(23, LOW);
    } else{
      digitalWrite(22, LOW);
      digitalWrite(23, HIGH);
    }
  }
}

void getHumidity() {
  sensors_event_t event;
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }

}