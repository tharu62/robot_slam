// specifications for UART data trasmission
#define WIFI_PORT 80
#define RXPIN 3
#define TXPIN 18
#define LIDAR_BAUDRATE 115200 //transmission speed for lidar
#define BAUDRATE 300000 //transmission speed for pc serial connection

// specifications for data transmission
// DON'T CHANGE THE VALUES
#define SERIAL_ONLY 0
#define WIFI_ONLY 1
#define WIFI_AND_SERIAL 2

// choose transmission type
#define TRANSMISSION_TYPE SERIAL_ONLY
// #define TRANSMISSION_TYPE WIFI_ONLY
// #define TRANSMISSION_TYPE WIFI_AND_SERIAL

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
int count = 0;
JsonDocument doc;

int packet_counter = 0;
int packet[360][5] = {0};

extern const char* ssid;
extern const char* password;

void setup(){
  // initialise serial comunication
  Serial.begin(BAUDRATE);
  
  Serial.println("");

  #if TRANSMISSION_TYPE == WIFI_ONLY || TRANSMISSION_TYPE == WIFI_AND_SERIAL
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
  #endif

  // ensure TXPIN is only for receiving data
  pinMode(TXPIN, INPUT);
  Lidar.begin(LIDAR_BAUDRATE, SERIAL_8N1, RXPIN, TXPIN);

  #if TRANSMISSION_TYPE == WIFI_ONLY || TRANSMISSION_TYPE == WIFI_AND_SERIAL
  // client connection
  Serial.println("Waiting for client connection...");
  client = server.available();
  while (!client) {
    client = server.available();
  }
  Serial.println("Client connected");  
  #endif

}

void loop() {

  #if TRANSMISSION_TYPE == WIFI_ONLY || TRANSMISSION_TYPE == WIFI_AND_SERIAL
  if(client){
  #endif
  
    if(Lidar.read(&temp, 1)){
    
        raw_data_old = raw_data;
        raw_data = temp;
        
        // if end of packet is reached
        if(raw_data_old == 0XFA && raw_data <= 0XF9 && raw_data >= 0XA0){

          if(packet_counter == 360){
            for(int i=0; i<360; i++){
              doc["ang"] = packet[i][0];
              doc["d1"] = packet[i][1];
              doc["d2"] = packet[i][2];
              doc["d3"] = packet[i][3];
              doc["d4"] = packet[i][4];

              #if TRANSMISSION_TYPE == SERIAL_ONLY
                serializeJson(doc, Serial);
                Serial.println();
              #elif TRANSMISSION_TYPE == WIFI_ONLY
                serializeJson(doc, client); 
                client.println();
              #elif TRANSMISSION_TYPE == WIFI_AND_SERIAL
                serializeJson(doc, Serial);
                Serial.println();
                serializeJson(doc, client);
                client.println();
              #endif
            }
            packet_counter = 0;
          }

          packet[packet_counter][0] = angle(buffer);
          packet[packet_counter][1] = dist_mm(buffer);
          packet[packet_counter][2] = dist_mm(buffer+4);
          packet[packet_counter][3] = dist_mm(buffer+8);
          packet[packet_counter][4] = dist_mm(buffer+12);
          packet_counter++;

          // print_data(buffer);
          
          // doc["ang"] = angle(buffer);
          // doc["d1"] = dist_mm(buffer);
          // doc["d2"] = dist_mm(buffer+4);
          // doc["d3"] = dist_mm(buffer+8);
          // doc["d4"] = dist_mm(buffer+12);
    
          // #if TRANSMISSION_TYPE == SERIAL_ONLY
          //   serializeJson(doc, Serial);
          //   Serial.println();
          // #elif TRANSMISSION_TYPE == WIFI_ONLY
          //   serializeJson(doc, client); 
          //   client.println();
          // #elif TRANSMISSION_TYPE == WIFI_AND_SERIAL
          //   serializeJson(doc, Serial);
          //   Serial.println();
          //   serializeJson(doc, client);
          //   client.println();
          // #endif
          
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
    
  #if TRANSMISSION_TYPE == WIFI_ONLY || TRANSMISSION_TYPE == WIFI_AND_SERIAL
  }else{
    client = server.available();
  }
  #endif
}