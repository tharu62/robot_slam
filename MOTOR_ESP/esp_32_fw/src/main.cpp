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

long int expected = 7000;

float kp=0.1;
float ki=0.01;
float kd=0.01;

float proportional=0.0;
float integral=0.0;
int derivative=0.0;
float error=0.0;
float last_error=0.0;

void pid_func(void *pvParameters) {
  while(1){
    error = (float)(expected - m1_count);
    
    proportional = kp * error;
    // integral += ki * error;
    derivative = kd * (error - last_error);
    
    float pid = proportional + integral + derivative;
    last_error = error;
    
    doc["d1"] = (pid > 0) - (pid < 0); // get sign of pid
    doc["s1"] = abs((int)pid);
    serializeJson(doc, arduino_serial);
    
    Serial.println("Counter: " + String(m1_count));
    Serial.printf("Error: %f, p:%f, i:%f, d:%f",error, proportional, integral, derivative);
    Serial.printf("PID: %f\n",doc["s1"].as<float>());
    
    delay(200);
  }
}

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

  // pid task
  auto b = xTaskCreate(pid_func, "pid_func", 10000, NULL, 1, NULL);
  if(b == pdPASS){
    Serial.println("PID Task created successfully");
  }else{
    Serial.println("PID Task creation failed");
  }

  m1_count = 0;
}

void loop() {
  if(arduino_serial.available()){
    String data = arduino_serial.readString();
    Serial.println(data);
  }
}