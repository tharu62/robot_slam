// specifications for USART data trasmission
#define WIFI_PORT 80
#define RXPIN 3
#define TXPIN 18
#define LIDAR_BAUDRATE 115200 //transmission speed for lidar
#define BAUDRATE 460800 //transmission speed for pc serial connection

// specifications for data transmission
// DON'T CHANGE THE VALUES
#define SERIAL_ONLY 0
#define WIFI_ONLY 1
#define WIFI_AND_SERIAL 2

#define PACKET_SIZE 90
#define BUFFER_SIZE 22
#define SCAN_SIZE 360


// choose transmission type
#define TRANSMISSION_TYPE SERIAL_ONLY
// #define TRANSMISSION_TYPE WIFI_ONLY
// #define TRANSMISSION_TYPE WIFI_AND_SERIAL

#include <Arduino.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <fstream>
#include <string.h>
#include "lidar.h"

#define START_BIT 0xFA

// USART 1
HardwareSerial Lidar(1);

// reading variables
int packet_index = 0;
int buffer_index = 0;
unsigned char temp,raw_data_curr,raw_data_old;
unsigned char packet[BUFFER_SIZE];

// JSON variables
JsonDocument jdoc;

JsonArray data = jdoc["data"].to<JsonArray>();
JsonArray strength = jdoc["str"].to<JsonArray>();

void setup(){
  // initialise serial comunication
  Serial.begin(BAUDRATE);
  Serial.println();

  // ensure TXPIN is only for receiving data
  pinMode(TXPIN, INPUT);
  Lidar.begin(LIDAR_BAUDRATE, SERIAL_8N1, RXPIN, TXPIN);

}


void loop() {

  if(Lidar.read(&temp, 1) > 0){
    
    raw_data_old = raw_data_curr;
    raw_data_curr = temp;

    if(raw_data_old == START_BIT && raw_data_curr <= 0xf9 && raw_data_curr >= 0xa0){


      if(verify_packet_checksum(packet)){
        
        jdoc["ang"] = angle(packet);
        // jdoc["rpm"] = rpm(packet);
        data.add(dist_mm(packet+4));
        data.add(dist_mm(packet+8));
        data.add(dist_mm(packet+12));
        data.add(dist_mm(packet+16));

        strength.add(signal_strength(packet+6));
        strength.add(signal_strength(packet+10));
        strength.add(signal_strength(packet+14));
        strength.add(signal_strength(packet+18));

        serializeJson(jdoc, Serial);
        Serial.println();

        data.clear();
        strength.clear();

        // dump_packet(packet);
      }

      packet[0] = START_BIT;
      packet[1] = raw_data_curr;
      buffer_index = 2;

      return;
    }

    if(buffer_index > BUFFER_SIZE){
      buffer_index = 0;
      return;
    }

    packet[buffer_index] = raw_data_curr;
    buffer_index++;
  
  }
}