// specifications for UART data trasmission
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

// choose transmission type
// #define TRANSMISSION_TYPE SERIAL_ONLY
#define TRANSMISSION_TYPE WIFI_ONLY
// #define TRANSMISSION_TYPE WIFI_AND_SERIAL

#include <Arduino.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <fstream>
#include <string.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "lidar.h"

#include <chrono>


// USART 1
HardwareSerial Lidar(1);

// UDP server
WiFiUDP udp;

int status = WL_IDLE_STATUS;
uint8_t buffer[30] = {0};
uint8_t raw_data=0;
uint8_t temp=0;
uint8_t raw_data_old=0;
int count = 0;
JsonDocument doc;

int packet_counter = 0;
int packet[PACKET_SIZE][5]= {0};
uint16_t remote_port = 80;
IPAddress remote_ip;

extern const char* ssid;
extern const char* password;

void setup(){
  // initialise serial comunication
  Serial.begin(BAUDRATE);
  Serial.println("");

  #if TRANSMISSION_TYPE == WIFI_ONLY || TRANSMISSION_TYPE == WIFI_AND_SERIAL
  // initialise wifi connection
  Serial.println("Connecting to Wifi...");
  // Serial.printf("SSID: %s\n",ssid);
  // Serial.printf("Password: %s\n",password);
  WiFi.begin(ssid, password);                         
  Serial.println("Establishing connection to WiFi with SSID: " + *(ssid));    
  while (WiFi.status() != WL_CONNECTED) {             
    delay(500);
    Serial.print(".");
  }
  Serial.print("Connected to network with IP address: ");
  Serial.println(WiFi.localIP());

  // UDP server
  udp.begin(WIFI_PORT);
  
  while(udp.available() == 0){
    udp.parsePacket();
    delay(100);
  }
  remote_ip = udp.remoteIP();
  remote_port = udp.remotePort();

  Serial.print("Remote IP: ");
  Serial.println(remote_ip.toString());
  Serial.printf("Remote Port: %d\n", remote_port);

  #endif

  // ensure TXPIN is only for receiving data
  pinMode(TXPIN, INPUT);
  Lidar.begin(LIDAR_BAUDRATE, SERIAL_8N1, RXPIN, TXPIN);


}


void loop(){
  
  udp.beginPacket(remote_ip, remote_port);

  udp.write('a');

  udp.endPacket();
}

#if 0

void loop() {

  if(Lidar.read(&temp, 1)){
    
    raw_data_old = raw_data;
    raw_data = temp;
    
    // if end of packet is reached
    if(raw_data_old == 0XFA && raw_data <= 0XF9 && raw_data >= 0XA0){

      doc["ang"] = angle(buffer);
      doc["d1"] = dist_mm(buffer+4);
      doc["d2"] = dist_mm(buffer+8);
      doc["d3"] = dist_mm(buffer+12);
      doc["d4"] = dist_mm(buffer+16);

      #if TRANSMISSION_TYPE == SERIAL_ONLY || TRANSMISSION_TYPE == WIFI_AND_SERIAL
      serializeJson(doc, Serial);
      Serial.println();
      #endif
      #if TRANSMISSION_TYPE == WIFI_ONLY || TRANSMISSION_TYPE == WIFI_AND_SERIAL
      serializeJson(doc, server);
      server.println();
      #endif

      buffer[0] = 0XFA;
      buffer[1] = raw_data;

      count = 2;
      return;
    }

    if(count <= 22){
      buffer[count] = raw_data;
      count++;
    }
  }

}

#endif