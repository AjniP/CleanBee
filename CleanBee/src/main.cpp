#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <I2S.h>

//Code based on DHT library examples

// put function declarations here:
float getTemperature();
float getHumidity();
void createAlert(float temperature);
void sendData();

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

const int delayMS = 3000;

bool TEMP_HIGH = false;
bool TEMP_LOW = false;

const int count = 20;
int arrayIndex = 0;

float alertTemp[count][2];

DHT_Unified dht(DHTPIN, DHTTYPE);

const int startTime = millis();

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
  float temperature = getTemperature();
  float humidity = getHumidity();

  if (Serial.available() > 0){
    char s = Serial.read();
    if (s == 'T'){
      sendData();
    }
  }

  if (TEMP_HIGH){
    if (temperature < TEMPERATURE_HOT_LIMIT_DECREASE){
      createAlert(temperature);
      digitalWrite(REDPIN, LOW);
      digitalWrite(GREENPIN, HIGH); // turn green led on
    }
  } else if (TEMP_LOW){
    if (temperature < TEMPERATURE_COLD_LIMIT_INCRESE){
      createAlert(temperature);
      digitalWrite(REDPIN, LOW);
      digitalWrite(GREENPIN, HIGH); // turn green led on
    }
  } else if (temperature > TEMPERATURE_HOT_LIMIT_INCREASE || temperature < TEMPERATURE_COLD_LIMIT_DECREASE){
    createAlert(temperature);
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
    //Serial.print(F("Temperature: "));
    //Serial.print(temperature);
    //Serial.println(F("°C"));

    return temperature;
  }
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

void createAlert(float temperature){

  alertTemp[arrayIndex][0] = millis();
  alertTemp[arrayIndex][2] = temperature;

  arrayIndex++;

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
  Serial.println("Events: ");
  for (int i = 0; i < count; i++){
    float time = alertTemp[i][0] - startTime;
    float temp = alertTemp[i][1];
    Serial.print("Time: ");
    Serial.print(time);
    Serial.print(", Temp: ");
    Serial.print(temp);

    if (temp > TEMPERATURE_HOT_LIMIT_INCREASE){
      Serial.println("(hot)");
    } else if (temp < TEMPERATURE_COLD_LIMIT_DECREASE){
      Serial.println("(cold)");
    } else {
      Serial.println("(ok)");
    }
  }

  Serial.print("Current temperature: ");
  Serial.println(getTemperature());

  Serial.print("Current humidity: ");
  Serial.println(getHumidity());
}
