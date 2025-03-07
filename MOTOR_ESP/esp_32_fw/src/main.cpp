/**
 *  ESP32 PROJECT
 * 
 * @file main.cpp
 * @authors Yehan Edirisinghe, Deshan Edirisinghe
 * 
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <WiFi.h>

#define TX_PIN 17
#define RX_PIN 16
#define BAUDRATE_ARDUINO 115200
#define BAUDRATE_ESP32 115200

#define USE_WIFI 0 // turn off wifi for testing
// #define USE_WIFI 1  // turn on wifi

HardwareSerial arduino_serial(2);

JsonDocument doc;
WiFiClient client;
extern const char* ssid;
extern const char* password;


void setup() {
  // Begin serial communication to pc
  Serial.begin(BAUDRATE_ESP32);
  // Begin serial communication to arduino
  arduino_serial.begin(BAUDRATE_ARDUINO, SERIAL_8N1, RX_PIN, TX_PIN);

  #if USE_WIFI == 1
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println();
  Serial.println("Connected to the WiFi network");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // client connection
  #endif

  // attach interrupts for pin D5 D19
  attachInterrupt(digitalPinToInterrupt(5), [] {Serial.println("D5 Interrupt rising");}, RISING);
  // attachInterrupt(digitalPinToInterrupt(19), [] {Serial.println("D19 Interrupt");}, RISING);
}

int i=0;

void loop() {

  // doc["m1"] = 1;
  // doc["d1"] = 1;
  // doc["s1"] = i;

  // doc["m2"] = 1;
  // doc["d2"] = 1;
  // doc["s2"] = i;

  // i = (i + 30) % 255;

  // serializeJson(doc, arduino_serial);

  // serializeJson(doc, Serial);
  // Serial.println();

  // doc.clear();

  // delay(300);
}