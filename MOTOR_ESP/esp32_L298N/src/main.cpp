/**
 * @file main.cpp
 * @author Yehan Edirisinghe
 * @brief This code is for controlling a L298N motor driver with an ESP32.
 * @version 0.1
 * 
 * @details
 * This code is for controlling a L298N motor driver with an ESP32.
 * The motor driver is connected to the ESP32 as follows:
*/

/* ----------------------------------------- */
// PWM pin for motor A and motor B
#define ENA GPIO_NUM_14
#define ENB GPIO_NUM_32

// Directional pins for motor A
#define IN1 GPIO_NUM_27
#define IN2 GPIO_NUM_26

// Directional pins for motor B
#define IN3 GPIO_NUM_25
#define IN4 GPIO_NUM_33
/* ----------------------------------------- */

#include <Arduino.h>
#include <string>
#include <ArduinoJson.h>
#include <MotorDriver.hpp>
#include "driver/gpio.h"

int d1,d2,s1,s2;
MotorDriver motor_driver(ENA, IN1, IN2, IN3, IN4, ENB);
JsonDocument input_message;
char input_buffer[256];

/**
 * * @brief Clear the buffer by setting all bytes to ```'\0'```
 * * @param buffer Buffer to clear
 * * @param size Size of the buffer
 */
void clearBuffer(char* buffer, int size){
  for(int i=0; i<size; ++i)
    buffer[i] = '\0';
}

void setup(){
  // Initialize the serial port
  Serial.begin(115200);
  Serial.println("Motor Driver Test");
  Serial.println("Initializing...");

  // Initialize the GPIO pins for the motor driver
  motor_driver.setup();

  Serial.println("Motor driver initialized");
  Serial.println("Waiting for commands...");
  input_message.clear();
}

void loop(){

  if(Serial.available() > 0){
    
    clearBuffer(input_buffer, sizeof(input_buffer));
    int message_size = Serial.readBytesUntil('\n', input_buffer, sizeof(input_buffer));
    input_buffer[message_size] = '\0';
    auto err = deserializeJson(input_message, input_buffer);

    if(err){
      Serial.print("Error parsing message: ");
      Serial.println(err.c_str());
      return;
    }

    Serial.print("Received message: ");
    serializeJson(input_message, Serial);
    Serial.println();
    
    if(input_message.containsKey("d1")){
      d1 = input_message["d1"];
      Serial.print("d1: ");
      Serial.println(d1);

      if(d1 == 0){
        motor_driver.M1setDirection(0, 0);
      }else if(d1 == 1){
        motor_driver.M1setDirection(1, 0);
      }else if(d1 == -1){
        motor_driver.M1setDirection(-1, 0);
      }else{
        Serial.println("Invalid value for d1. Must be 0, 1 or -1.");
      }
    }

    if(input_message.containsKey("d2")){
      d2 = input_message["d2"];
      Serial.print("d2: ");
      Serial.println(d2);

      if(d2 == 0){
        motor_driver.M2setDirection(0, 0);
      }else if(d2 == 1){
        motor_driver.M2setDirection(1, 0);
      }else if(d2 == -1){
        motor_driver.M2setDirection(-1, 0);
      }else{
        Serial.println("Invalid value for d2. Must be 0, 1 or -1.");
      }
    }

    if(input_message.containsKey("s1")){
      s1 = input_message["s1"];
      Serial.print("s1: ");
      Serial.println(s1);
      motor_driver.writeSpeed(s1, s2);
    }

    if(input_message.containsKey("s2")){
      s2 = input_message["s2"];
      Serial.print("s2: ");
      Serial.println(s2);
      motor_driver.writeSpeed(s1, s2);
    }

    if(input_message.containsKey("stop")){
      motor_driver.stop();
      Serial.println("Motors stopped.");
    }
  }
}