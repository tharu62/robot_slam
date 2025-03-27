/**
 *  ARDUINO PROJECT
 * 
 * @file main.cpp
 * @authors Yehan Edirisighe
 * 
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <stdint.h>
#include <string.h>
#include "AFMotor.h"
#include <string.h>

#define BAUDRATE 115200
#define MAX_BUFFER_SIZE 256

AF_DCMotor M1(1);
AF_DCMotor M2(2);
JsonDocument doc;


String j_str;
char buffer[MAX_BUFFER_SIZE];
int buffer_index = 0;
char temp;
int mot;
int dir;
int speed;


void setup() {
  Serial.begin(BAUDRATE);

  M1.setSpeed(255);
  M1.run(RELEASE);
  M2.setSpeed(255);
  M2.run(RELEASE);

  for(int i=0; i<MAX_BUFFER_SIZE; ++i){
    buffer[i] = '\0';
  }
  
  Serial.println("Arduino is ready");
}

void loop() {
  if(Serial.available()){
    
    size_t len = Serial.readBytesUntil('}', buffer, MAX_BUFFER_SIZE);
    buffer[len] = '}';

    if(len >0){
    
      j_str = String(buffer);

      auto err = deserializeJson(doc, j_str);
      if(err){

        Serial.println("Error parsing JSON: "+String(err.c_str()));
        Serial.println(j_str);

      }else{

        if(doc.containsKey("v1"))
        {
          speed = doc["v1"].as<int>();

          M1.setSpeed((uint8_t)abs(speed));

          if(speed > 0){
            M1.run(FORWARD);
          }else if(speed < 0){
            M1.run(BACKWARD);
          }else if(speed == 0){
            M1.run(RELEASE);
          }
        }
        
        if(doc.containsKey("v2"))
        {
          speed = doc["v2"].as<int>();

          M2.setSpeed((uint8_t)abs(speed));

          if(speed > 0){
            M2.run(FORWARD);
          }else if(speed < 0){
            M2.run(BACKWARD);
          }else if(speed == 0){
            M2.run(RELEASE);
          }
        }
      }
      
      for(int i=0; i<MAX_BUFFER_SIZE; ++i){
        buffer[i] = '\0';
      }
    }
  }
}