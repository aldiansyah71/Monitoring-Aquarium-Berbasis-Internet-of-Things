#include <EEPROM.h>
#include "GravityTDS.h"
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
#include <HTTPClient.h>

#define BLYNK_TEMPLATE_ID "TMPL6Hh1k2peW"
#define BLYNK_TEMPLATE_NAME "Monitoring Aquarium"
#define BLYNK_AUTH_TOKEN "5vwTgWP-Z2srdBrARXVJHMeFGxDkyklf"

// Wi-Fi Configuration
const char ssid[] = "SKRIPSII"; 
const char pass[] = "12345678"; 

// Webhook URL dari Apps Script
const char* googleScriptURL = "https://script.google.com/macros/s/AKfycbxR7Wx5yTuHZuJxo2EYdj55O3hu4ggA9OYDy6pM1XBczF1SAmJyt8usT2hRjuNZkvL-ZA/exec";

// Variabel Global untuk Status Perangkat
int relayPompa1INState = 0;
int buzzerState = 0;
int servoState = 0;
int servo2State = 0; 
int relayPompa2INState = 0;
int relayPompa2OUTState = 0;
int heaterState = 0;

// TDS Sensor Configuration
#define TdsSensorPin 35
GravityTDS gravityTds;
float tdsTemperature = 25, tdsValue = 0;

// DS18B20 Configuration
#define ONE_WIRE_BUS 23
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress sensorsuhuAddress;

// YF-S201 Water Flow Sensor Configuration
#define FLOW_SENSOR_PIN 25
volatile int pulseCount = 0;
float flowRate = 0.0;
unsigned long oldTime = 0;
const float calibrationFactor = 4.5;

#define TRIG_PIN 18
#define ECHO_PIN 19
#define RELAY_POMPA_AQUARIUM2_IN 15
#define RELAY_POMPA_AQUARIUM2_OUT 26
#define RELAY_POMPA_AQUARIUM1_IN 27
#define RELAY_HEATER 5
#define BUZZER_PIN 13
#define SERVO360PIN 14    
#define SERVO2PIN 33       
Servo servo360;
Servo servo2;

// Interrupt function for YF-S201 pulse counting
void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  Blynk.syncVirtual(V9);
  Blynk.syncVirtual(V0);

  // Initialize Wi-Fi
  WiFi.begin(ssid, pass);
  Serial.print("Menghubungkan ke Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi tersambung!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Initialize Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Initialize TDS Sensor
  EEPROM.begin(512);
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(3.3);
  gravityTds.setAdcRange(4096);
  gravityTds.begin();

  // Initialize DS18B20
  sensors.begin();
  if (sensors.getDeviceCount() < 1) {
    Serial.println("Sensor DS18B20 tidak terdeteksi! Pastikan sensor terhubung.");
    while (true);
  }
  sensors.getAddress(sensorsuhuAddress, 0);

  // Initialize YF-S201
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING);

  // Initialize HC-SR04
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize Relays
  pinMode(RELAY_POMPA_AQUARIUM2_IN, OUTPUT);
  pinMode(RELAY_POMPA_AQUARIUM2_OUT, OUTPUT);
  pinMode(RELAY_POMPA_AQUARIUM1_IN, OUTPUT);
  pinMode(RELAY_HEATER, OUTPUT);
  digitalWrite(RELAY_POMPA_AQUARIUM2_IN, HIGH);
  digitalWrite(RELAY_POMPA_AQUARIUM2_OUT, HIGH);
  digitalWrite(RELAY_POMPA_AQUARIUM1_IN, HIGH);
  digitalWrite(RELAY_HEATER, HIGH);

  // Initialize Buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  // Initialize Servos
  servo360.attach(SERVO360PIN);
  servo360.write(90);
  delay(1000);
  servo360.detach();
  servo2.attach(SERVO2PIN);
  servo2.write(90);
  delay(1000);
  servo2.detach();
}
void sendToGoogleSheet(float tds, float flowRate, float temperature, float distance) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(googleScriptURL);
    String postData = "tds=" + String(round(tds)) +
                      "&flowRate=" + String(round(flowRate)) +
                      "&relayPompa1IN=" + String(relayPompa1INState == 1 ? "ON" : "OFF") +
                      "&buzzerState=" + String(buzzerState == 1 ? "ON" : "OFF") +
                      "&servoState=" + String(servoState == 1 ? "ON" : "OFF") +
                      "&servo2State=" + String(servo2State == 1 ? "ON" : "OFF") + 
                      "&temperature=" + String(temperature) +
                      "&distance=" + String(round(distance)) + 
                      "&relayPompa2IN=" + String(relayPompa2INState == 1 ? "ON" : "OFF") +
                      "&relayPompa2OUT=" + String(relayPompa2OUTState == 1 ? "ON" : "OFF") +
                      "&heaterState=" + String(heaterState == 1 ? "ON" : "OFF");
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    int httpResponseCode = http.POST(postData);
    if (httpResponseCode > 0) {
      Serial.println("Data terkirim ke Google Sheet!");
    } else {
      Serial.print("Error saat mengirim data: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("WiFi tidak terhubung, data tidak dikirim!");
  }
}

void loop() {
  Blynk.run();
  sensors.requestTemperatures();
  float temperature = sensors.getTempC(sensorsuhuAddress);
  updateFlowRate();
  updateTDSValue();
  float distance = readUltrasonicDistance();
  int relayPompa1INState = digitalRead(RELAY_POMPA_AQUARIUM1_IN);
  int buzzerState = digitalRead(BUZZER_PIN);
  int relayPompa2INState = digitalRead(RELAY_POMPA_AQUARIUM2_IN);
  int relayPompa2OUTState = digitalRead(RELAY_POMPA_AQUARIUM2_OUT);
  int heaterState = digitalRead(RELAY_HEATER);

  // Send data to Google Sheets
  sendToGoogleSheet(tdsValue, flowRate, temperature, distance);

  int roundedDistance = round(distance);
  Blynk.virtualWrite(V3, roundedDistance);
  Serial.print("Jarak: ");
  Serial.print(roundedDistance);
  Serial.println(" cm");
  // Display DS18B20 temperature
  Serial.print("Suhu air : ");
  Serial.print(temperature);
  Serial.println("\u00b0C");

  delay(5000);
}
// Blynk control for Relays
BLYNK_WRITE(V5) {
  relayPompa2INState = param.asInt();
  digitalWrite(RELAY_POMPA_AQUARIUM2_IN, !relayPompa2INState);
  Serial.print("Pompa Aquarium 2 IN: ");
  Serial.println(relayPompa2INState ? "ON" : "OFF");
}
BLYNK_WRITE(V6) {
  relayPompa2OUTState = param.asInt();
  digitalWrite(RELAY_POMPA_AQUARIUM2_OUT, !relayPompa2OUTState);
  Serial.print("Pompa Aquarium 2 OUT: ");
  Serial.println(relayPompa2OUTState ? "ON" : "OFF");
}
BLYNK_WRITE(V7) {
  relayPompa1INState = param.asInt();
  digitalWrite(RELAY_POMPA_AQUARIUM1_IN, !relayPompa1INState);
  Serial.print("Pompa Aquarium 1 IN: ");
  Serial.println(relayPompa1INState ? "ON" : "OFF");
}
BLYNK_WRITE(V8) {
  heaterState = param.asInt();
  digitalWrite(RELAY_HEATER, !heaterState);
  Serial.print("Heater: ");
  Serial.println(heaterState ? "ON" : "OFF");
}
//servo1,servo2,buzzer
BLYNK_WRITE(V9) {
  int switchState = param.asInt();  
  if (switchState == 1) {
    servo360.attach(SERVO360PIN); 
    servo360.write(90);           
    Serial.println("Servo1 bergerak");
    delay(2000);                  
    servo360.write(180);          
    delay(2000);                  // Tunggu untuk berhenti sempurna
    servo360.detach();            // Lepas sinyal servo
    servoState = 1;               // Tandai bahwa servo aktif
    Serial.println("Servo1 berhenti");
  } else {
    servoState = 0;               // Tandai bahwa servo tidak aktif
  }
}
BLYNK_WRITE(V10) {
  buzzerState = param.asInt();
  if (buzzerState == 1) {
    tone(BUZZER_PIN, 4000); // Continuous 4000 Hz tone
  } else {
    noTone(BUZZER_PIN);
  }
}
BLYNK_WRITE(V0) {
  int switchState = param.asInt();  
  if (switchState == 1) {
    servo2.attach(SERVO2PIN); 
    servo2.write(90);           
    Serial.println("Servo2 bergerak");
    delay(2000);                  // Gerakkan servo selama 2 detik
    servo2.write(180);          // Berhentikan servo
    delay(2000);                  // Tunggu untuk berhenti sempurna
    servo2.detach();            // Lepas sinyal servo
    servo2State = 1;               
    Serial.println("Servo2 berhenti");
  } else {
    servo2State = 0;               
  }
}
// update sensor arus
void updateFlowRate() {
  unsigned long currentTime = millis();
  if (currentTime - oldTime >= 1000) {
    float flowLiters = (pulseCount / calibrationFactor);
    flowRate = flowLiters * 60;
    int roundedFlowRate = round(flowRate); 
    pulseCount = 0;
    oldTime = currentTime;
    Blynk.virtualWrite(V2, roundedFlowRate);
    Serial.print("Flow Rate: ");
    Serial.print(roundedFlowRate);
    Serial.println(" LPM");
  }
}
// Function to update and send TDS value
void updateTDSValue() {
  gravityTds.setTemperature(tdsTemperature);
  gravityTds.update();
  tdsValue = gravityTds.getTdsValue();
  int tdsValueInt = (int)tdsValue;
  Blynk.virtualWrite(V4, tdsValueInt);
  Serial.print("TDS: ");
  Serial.print(tdsValueInt);
  Serial.println(" ppm");
}
// Function to read HC-SR04 distance
float readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.034 / 2;
}
