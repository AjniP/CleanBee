#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <I2S.h>
#include <ArduinoJson.h>

//Code based on DHT and json library examples

// put function declarations here:
float getTemperature();
float getHumidity();
void createAlert(float temperature, float humidity);
void sendData();
bool getMeasurements();

#define DHTPIN          18        // Digital pin connected to the DHT sensor 
#define REDPIN          22
#define GREENPIN        23

#define DHTTYPE         DHT22     // DHT 22 (AM2302)


const float TEMPERATURE_HOT_LIMIT_INCREASE = 27.0; // If goes above this create alert and turn on red LED
const float TEMPERATURE_HOT_LIMIT_DECREASE = 26.0; // If goes below this create event and turn on green LED

const float TEMPERATURE_COLD_LIMIT_DECREASE = 25.0; // If goes BELOW this create alert and turn on red LED
const float TEMPERATURE_COLD_LIMIT_INCRESE = 25.5; // If goes ABOVE this create event and turn on green LED

const float HUMIDITY_HIGH_LIMIT_INCREASE = 70.0; // If goes above this create alert and turn on red LED
const float HUMIDITY_HIGH_LIMIT_DECREASE = 66.0; // If goes BELOW this create EVENT and turn on GREEN LED

const float HUMIDITY_LOW_LIMIT_DECREASE = 40.0; // If goes BELOW this create alert and turn on red LED
const float HUMIDITY_LOW_LIMIT_INCREASE = 44.0; // If goes ABOVE this create EVENT and turn on GREEN LED

const int delayMS = 60; 

bool TEMP_HIGH = false;
bool TEMP_LOW = false;

DHT_Unified dht(DHTPIN, DHTTYPE);

struct SensorData {
  unsigned long timestamp_ms;
  float temperature;
  float humidity;
};

const int dataSize = 1000; // 8000 max
SensorData dataPeriod[dataSize];
int dataIndex = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  dht.begin();
  pinMode(GREENPIN, OUTPUT);
  pinMode(REDPIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(delayMS);

  if (dataIndex >= dataSize) {
    while(true){
      if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim(); 
        if (command == "SEND") {
          sendData(); 
          break;
        }
      }
    }
    dataIndex = 0; 
  }
  
  getMeasurements();

  float temperature = getTemperature();
  float humidity = getHumidity();

  if (TEMP_HIGH){
    if (temperature < TEMPERATURE_HOT_LIMIT_DECREASE){
      digitalWrite(REDPIN, LOW);
      digitalWrite(GREENPIN, HIGH); // turn green led on
    }
  } else if (TEMP_LOW){
    if (temperature < TEMPERATURE_COLD_LIMIT_INCRESE){
      digitalWrite(REDPIN, LOW);
      digitalWrite(GREENPIN, HIGH); // turn green led on
    }
  } else if (temperature > TEMPERATURE_HOT_LIMIT_INCREASE || temperature < TEMPERATURE_COLD_LIMIT_DECREASE){
    digitalWrite(REDPIN, HIGH); // turn red led on
    digitalWrite(GREENPIN, LOW);
  } else{
    digitalWrite(REDPIN, LOW);
    digitalWrite(GREENPIN, HIGH); // turn green led on
  }
}

// put function definitions here:
float getTemperature() {
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  float temperature = event.temperature;

  if (isnan(temperature)) {
    Serial.println(F("Error reading temperature!"));
    return -1;
  }
  else {
    return temperature;
  }
}

bool getMeasurements() {
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  float temperature = event.temperature;
  float humidity = event.relative_humidity;

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println(F("Error reading temperature!"));
    return -1;
  }
  else {
      dataPeriod[dataIndex].timestamp_ms = millis();
      dataPeriod[dataIndex].temperature = temperature;
      dataPeriod[dataIndex].humidity = humidity;
      dataIndex++;

    return true;
  }
  return false;
}

float getHumidity() {
  sensors_event_t event;
  dht.humidity().getEvent(&event);
  float humidity = event.relative_humidity;

  if (isnan(humidity)) {
    Serial.println(F("Error reading humidity!"));
    return -1; 
  }
  else {
    //Serial.print(F("Humidity: "));
    //Serial.print(humidity);
    //Serial.println(F("%"));
    return humidity;
  }
}

void createAlert(float temperature, float humidity){

  if (temperature > TEMPERATURE_HOT_LIMIT_INCREASE){
    TEMP_HIGH = true;
  } else if (temperature < TEMPERATURE_COLD_LIMIT_DECREASE){
    TEMP_LOW = true;
  } else {
    TEMP_HIGH = true;
    TEMP_LOW = true;
    Serial.print("Temperature within range: ");
    Serial.print(temperature);
    Serial.println(F("°C!"));
    return;
  }

  Serial.print("Temperature out of range: ");
  Serial.print(temperature);
  Serial.println(F("°C!"));
}

void sendData(){

  JsonDocument doc; // Adjust size as needed
  JsonArray readings = doc["readings"].add<JsonArray>();
  //Serial.println("Events: ");

  for (int i = 0; i < dataIndex; i++){
    JsonObject reading = readings.add<JsonObject>();
    reading["timestamp_ms"] = dataPeriod[i].timestamp_ms;
    reading["temperature"] = dataPeriod[i].temperature;
    reading["humidity"] = dataPeriod[i].humidity;

    /*if (temp > TEMPERATURE_HOT_LIMIT_INCREASE){
      Serial.println("(hot)");
    } else if (temp < TEMPERATURE_COLD_LIMIT_DECREASE){
      Serial.println("(cold)");
    } else {
      Serial.println("(ok)");
    }*/
  
  }
  serializeJson(doc, Serial);

  Serial.println();
}
