/**
 *  ESP32 PROJECT
 * 
 * @file main.cpp
 * @authors Yehan Edirisinghe
 * 
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <vector>
#include <string>

#include "customPID.h"
#include "encoders.h"

#define TX_PIN 17
#define RX_PIN 16
#define BAUDRATE_ARDUINO 115200
#define BAUDRATE_ESP32   115200

HardwareSerial arduino_serial(2);

JsonDocument input_doc;
JsonDocument arduino_doc;
JsonDocument output_doc;

void printCounters(TimerHandle_t xTimer){

  output_doc["r1"] = (int)((float)v1_count * 10000/ (float)COUNT_PER_BIG_ROTATION);
  output_doc["r2"] = (int)((float)v2_count * 10000/ (float)COUNT_PER_BIG_ROTATION);
  output_doc["dt"] = (long int)dt.count();

  serializeJson(output_doc, Serial);
  Serial.println();
  // Serial.printf("v1: %ld, v2: %ld, dt1: %ld, dt2: %ld\n", v1_count, v2_count, (long int)dt1.count(), (long int)dt2.count());
}

TimerHandle_t velocity_timer = xTimerCreate("velocity_timer", pdMS_TO_TICKS(50), pdTRUE, (void *)1, interrupt_v);
TimerHandle_t print_counter_timer = xTimerCreate("print_counter_timer", pdMS_TO_TICKS(100), pdTRUE, (void *)0, printCounters);

void setup() {

  pinMode(M1_C1_PIN, INPUT);
  pinMode(M1_C2_PIN, INPUT);

  // Begin serial communication to pc
  Serial.begin(BAUDRATE_ESP32);
  // Begin serial communication to arduino
  arduino_serial.begin(BAUDRATE_ARDUINO, SERIAL_8N1, RX_PIN, TX_PIN);

  if(velocity_timer == NULL || print_counter_timer == NULL){
    while(1){
      Serial.println("Timer creation failed");
      delay(1000);
    }
  }

  // attach interrupts for pin D5 D19
  attachInterrupt(digitalPinToInterrupt(M1_C1_PIN), interrupt_m1_c1, RISING);
  attachInterrupt(digitalPinToInterrupt(M1_C2_PIN), interrupt_m1_c2, RISING);
  
  attachInterrupt(digitalPinToInterrupt(M2_C1_PIN), interrupt_m2_c1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_C2_PIN), interrupt_m2_c2, RISING);
  
  arduino_doc["d1"] = 0;
  arduino_doc["s1"] = 0;
  arduino_doc["d2"] = 0;
  arduino_doc["s2"] = 0;
  serializeJson(arduino_doc, arduino_serial);
  
  m1_count = 0;
  m2_count = 0;
  m1_count_old = 0;
  m2_count_old = 0;
  v1_count = 0;
  v2_count = 0;
  
  xTimerStart(velocity_timer, 50);
  xTimerStart(print_counter_timer, 0);
}

void loop() {

  if(arduino_serial.available()){
    String data = arduino_serial.readString();
    Serial.print("ARDUINO:");
    Serial.println(data);
  }

  if(Serial.available()){

    String data = Serial.readString();
    
    DeserializationError error = deserializeJson(input_doc, data);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      return;
    }

    if(input_doc.containsKey("v1") && input_doc.containsKey("v2")){
      
      arduino_doc["v1"] = input_doc["v1"];
      arduino_doc["v2"] = input_doc["v2"];

    }
  }
}