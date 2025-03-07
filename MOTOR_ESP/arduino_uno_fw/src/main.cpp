/**
 *  ARDUINO PROJECT
 * 
 * @file main.cpp
 * @authors Yehan Edirisighe
 * 
 */

#include <Arduino.h>
#include <ArduinoJson.h>
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

        if(doc.containsKey("d1") && doc.containsKey("s1") )
        {
          dir = doc["d1"];
          speed = doc["s1"];

          // Serial.println("d1: "+String(dir)+" s1: "+String(speed));

          M1.setSpeed(speed);
          if(dir == 1){
            M1.run(FORWARD);
          }else if(dir == -1){
            M1.run(BACKWARD);
          }
        }
        
        if(doc.containsKey("d2") && doc.containsKey("s2") )
        {
          dir = doc["d2"];
          speed = doc["s2"];

          // Serial.println("d2: "+String(dir)+" s2: "+String(speed));

          M2.setSpeed(speed);
          if(dir == 1){
            M2.run(FORWARD);
          }else if(dir == -1){
            M2.run(BACKWARD);
          }
        }
      }
      
      clean_buffer_:
      for(int i=0; i<MAX_BUFFER_SIZE; ++i){
        buffer[i] = '\0';
      }
    }
  }
}