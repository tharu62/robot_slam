// specifications for UART data trasmission
#define WIFI_PORT 80
#define RXPIN 3
#define TXPIN 18
#define LIDAR_BAUDRATE 115200 //transmission speed for lidar
#define BAUDRATE 300000 //transmission speed for pc serial connection

#define NO_WIFI_TRANS 0

#include <Arduino.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <fstream>
#include <string.h>
#include <WiFi.h>
#include "lidar.h"



// USART 1
HardwareSerial Lidar(1);
WiFiServer server(WIFI_PORT);
WiFiClient client;
int status = WL_IDLE_STATUS;
uint8_t buffer[30] = {0};
uint8_t raw_data=0;
uint8_t temp=0;
uint8_t raw_data_old=0;
int available_raw_data = 0;
int count = 0;
JsonDocument doc;

extern const char* ssid;
extern const char* password;

void setup(){
  // initialise serial comunication
  Serial.begin(BAUDRATE);
  
  Serial.println("");

  // initialise wifi connection
  Serial.println("Connecting to Wifi...");
  Serial.printf("SSID: %s\n",ssid);
  Serial.printf("Password: %s\n",password);
  
  WiFi.begin(ssid, password);                         
  Serial.println("Establishing connection to WiFi with SSID: " + *(ssid));    
  while (WiFi.status() != WL_CONNECTED) {             
    delay(500);
    Serial.print(".");
  }
  Serial.print("Connected to network with IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();
  
  // ensure TXPIN is only for receiving data
  pinMode(TXPIN, INPUT);
  Lidar.begin(LIDAR_BAUDRATE, SERIAL_8N1, RXPIN, TXPIN);

  // client connection
  Serial.println("Waiting for client connection...");
  client = server.available();
  while (!client) {
    client = server.available();
  }
  Serial.println("Client connected");  
}

void loop() {
  if(client){
    if(Lidar.read(&temp, 1)){
  
      raw_data_old = raw_data;
      raw_data = temp;
      
      // if end of packet is reached
      if(raw_data_old == 0XFA && raw_data <= 0XF9 && raw_data >= 0XA0){
        
        // print_data(buffer);
        
        doc["ang"] = angle(buffer);
        doc["d1"] = dist_mm(buffer);
        doc["d2"] = dist_mm(buffer+4);
        doc["d3"] = dist_mm(buffer+8);
        doc["d4"] = dist_mm(buffer+12);
  
        #if NO_WIFI_TRANS
          serializeJson(doc, Serial);
          Serial.println();
        #else
          serializeJson(doc, client);
          client.println();
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
  }else{
    client = server.available();
  }
}