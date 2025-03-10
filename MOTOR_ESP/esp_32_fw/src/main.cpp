/**
 *  ESP32 PROJECT
 * 
 * @file main.cpp
 * @authors Yehan Edirisinghe, Deshan Edirisinghe
 * 
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <PIDController.h>
#include <WiFi.h>
#include <vector>
#include <string>

#include "encoders.h"

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

volatile long int expected = 7000;

double kp=0.8;
double ki=0.081;
double kd=0.7;

PIDController pid;


void pid_func(void *pvParameters) {
  while(1){

    double val = -pid.compute((double)m1_count);

    if(val > 0){
      doc["d1"] = 1;
    }else{
      doc["d1"] = -1;
    }

    doc["s1"] = abs(val);
    serializeJson(doc, arduino_serial);

    Serial.printf("Count: %d, pid:%lf\n", m1_count, val);
    delay(100);
    
  }
}

// void print_counters(void *pvParameters) {
//   while(1){
//     Serial.println("Counters: " + String(m1_count)+ " " + String(m2_count));
//     delay(200);
//   }
// }

void setup() {

  pinMode(M1_C1_PIN, INPUT_PULLUP);
  pinMode(M1_C2_PIN, INPUT_PULLUP);

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
  attachInterrupt(digitalPinToInterrupt(M1_C1_PIN), interrupt_m1_c1, RISING);
  attachInterrupt(digitalPinToInterrupt(M1_C2_PIN), interrupt_m1_c2, RISING);

  attachInterrupt(digitalPinToInterrupt(M2_C1_PIN), interrupt_m2_c1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_C2_PIN), interrupt_m2_c2, RISING);

  doc["d1"] = 0;
  doc["s1"] = 0;
  doc["d2"] = 0;
  doc["s2"] = 0;
  serializeJson(doc, arduino_serial);

  pid.begin();
  pid.setpoint(7000);
  pid.tune(kp, ki, kd);
  pid.limit(-255, 255);

  // pid task
  auto b = xTaskCreate(pid_func, "pid_func", 10000, NULL, 1, NULL);
  if(b == pdPASS){
    Serial.println("PID Task created successfully");
  }else{
    Serial.println("PID Task creation failed");
  }

  // auto a = xTaskCreate(print_counters, "print_counters", 10000, NULL, 1, NULL);
  // if(a == pdPASS){
  //   Serial.println("Print Counters Task created successfully");
  // }else{
  //   Serial.println("Print Counters Task creation failed");
  // }

  m1_count = 0;
}

void loop() {
  if(arduino_serial.available()){
    String data = arduino_serial.readString();
    Serial.println(data);
  }
}