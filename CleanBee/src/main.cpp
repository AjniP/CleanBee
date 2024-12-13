#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <ArduinoJson.h>

//Code based on DHT and json library examples

float getTemperature();
float getHumidity();
void sendData();
bool getMeasurements();

#define DHTPIN          18        // Digital pin connected to the DHT sensor 
#define REDPIN          22
#define GREENPIN        23

#define DHTTYPE         DHT22     // DHT 22 (AM2302)


const float TEMPERATURE_HOT_LIMIT_INCREASE = 36.0; // If goes ABOVE this set TEMP_HIGH flag true and turn on red LED
const float TEMPERATURE_HOT_LIMIT_DECREASE = 35.0; // If goes BELOW this set TEMP_HIGH flag false and turn off red LED

const float TEMPERATURE_COLD_LIMIT_DECREASE = 33.0; // If goes BELOW this set TEMP_LOW true and turn on red LED
const float TEMPERATURE_COLD_LIMIT_INCREASE = 34.0; // If goes ABOVE this set TEMP_LOW false and turn off red LED

const float HUMIDITY_HIGH_LIMIT_INCREASE = 95.0; // If goes ABOVE this turn on red LED
const float HUMIDITY_HIGH_LIMIT_DECREASE = 94.0; // If goes BELOW this turn off red LED

const float HUMIDITY_LOW_LIMIT_DECREASE = 90.0; // If goes BELOW this turn on red LED
const float HUMIDITY_LOW_LIMIT_INCREASE = 91.0; // If goes ABOVE this turn off red LED

float delayMS; 

bool TEMP_HIGH = false;
bool TEMP_LOW = false;

DHT_Unified dht(DHTPIN, DHTTYPE);

struct SensorData {
  unsigned long timestamp_ms;
  float temperature;
  float humidity;
};

const float measurementLength = 5; // in minutes
const int dataSize = 175; // 8000 max
SensorData dataPeriod[dataSize];
int dataIndex = 0;

void setup() {
  Serial.begin(115200);
  dht.begin();
  pinMode(GREENPIN, OUTPUT);
  pinMode(REDPIN, OUTPUT);

  delayMS = (measurementLength*60)/dataSize; //automatically determine the measurement interval
}

void loop() {

  delay(delayMS);

  if (dataIndex >= dataSize) { // if dataPeriod array is full stop measurements and wait for command to send data
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
  
  // saves measurements to dataPeriod
  getMeasurements();

  float temperature = getTemperature();
  float humidity = getHumidity();

  if (TEMP_HIGH){
    if (temperature < TEMPERATURE_HOT_LIMIT_DECREASE){
      digitalWrite(REDPIN, LOW);
      TEMP_HIGH = false;
    }
  } else if (TEMP_LOW){ 
    if (temperature < TEMPERATURE_COLD_LIMIT_INCREASE){
      digitalWrite(REDPIN, LOW);
      TEMP_LOW = false;
    }
  } else if (temperature > TEMPERATURE_HOT_LIMIT_INCREASE){
    digitalWrite(REDPIN, HIGH); // turn red led on
    TEMP_HIGH = true;
  } else if(temperature < TEMPERATURE_COLD_LIMIT_DECREASE){
    digitalWrite(REDPIN, HIGH); // turn red led on
    TEMP_LOW = true;
  } else {
    digitalWrite(REDPIN, LOW); // turn red led off
    TEMP_LOW = false;
    TEMP_HIGH = false;
  }

}

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
  dht.humidity().getEvent(&event);
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
    return humidity;
  }
}


void sendData(){

  JsonDocument doc;
  JsonArray readings = doc["readings"].add<JsonArray>();

  for (int i = 0; i < dataIndex; i++){
    JsonObject reading = readings.add<JsonObject>();
    reading["timestamp_ms"] = dataPeriod[i].timestamp_ms;
    reading["temperature"] = dataPeriod[i].temperature;
    reading["humidity"] = dataPeriod[i].humidity;
  }

  serializeJson(doc, Serial);

  Serial.println();
}
