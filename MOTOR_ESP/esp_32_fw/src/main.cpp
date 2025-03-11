/**
 *  ESP32 PROJECT
 * 
 * @file main.cpp
 * @authors Yehan Edirisinghe, Deshan Edirisinghe
 * 
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <vector>
#include <string>

#include "customPID.h"
#include "encoders.h"

#define COUNT_PER_BIG_ROTATION 5940
#define COUNT_PER_SMALL_ROTATION 22


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

PID<double, double> pid(kp, ki, kd, 5940);

void pid_timer_callback(TimerHandle_t xTimer){

  double speed = pid.compute(m1_count, 0.1);
  Serial.printf("speed: %lf, count: %lf, error: %lf\n", speed, m1_count, pid.getError());

  doc["d1"] = -sign(speed);
  doc["s1"] = (int)abs(speed);
  doc["d2"] = 0;
  doc["s2"] = 0;
  serializeJson(doc, arduino_serial);

}

TimerHandle_t velocity_timer = xTimerCreate("velocity_timer", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, interrupt_v1);
TimerHandle_t velocity_timer2 = xTimerCreate("velocity_timer2", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, interrupt_v2);

TimerHandle_t pid_timer = xTimerCreate("pid_timer", pdMS_TO_TICKS(100), pdTRUE, (void *)0, pid_timer_callback);

void setup() {

  pinMode(M1_C1_PIN, INPUT_PULLUP);
  pinMode(M1_C2_PIN, INPUT_PULLUP);

  // Begin serial communication to pc
  Serial.begin(BAUDRATE_ESP32);
  // Begin serial communication to arduino
  arduino_serial.begin(BAUDRATE_ARDUINO, SERIAL_8N1, RX_PIN, TX_PIN);

  pid.setLimit(255);
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
  
  m1_count = 0;
  m2_count = 0;
  m1_count_old = 0;
  m2_count_old = 0;
  v1_count = 0;
  v2_count = 0;
  
  // xTimerStart(velocity_timer, 0);
  // xTimerStart(velocity_timer2, 0);
  xTimerStart(pid_timer, 2);
}

void loop() {

  if(m1_count >= (10*COUNT_PER_BIG_ROTATION) - 340){
    doc["d1"] = 0;
    doc["s1"] = 0;
    doc["d2"] = 0;
    doc["s2"] = 0;
    serializeJson(doc, arduino_serial);
  }

  if(arduino_serial.available()){
    String data = arduino_serial.readString();
    Serial.println(data);
  }

  if(Serial.available()){

    String data = Serial.readString();

    if(data == "1"){

      xTimerStop(pid_timer, 0);

      doc["d1"] = 0;
      doc["s1"] = 0;
      doc["d2"] = 0;
      doc["s2"] = 0;

      serializeJson(doc, arduino_serial);
    }
  }
}