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

#define PACKET_SIZE 90

// choose transmission type
// #define TRANSMISSION_TYPE SERIAL_ONLY
// #define TRANSMISSION_TYPE WIFI_ONLY
#define TRANSMISSION_TYPE WIFI_AND_SERIAL

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

// JsonArray ang = doc["ang"].to<JsonArray>();
JsonArray d1 = doc["data"].to<JsonArray>();


int packet_counter = 0;
int packet[PACKET_SIZE][5]= {0};

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

        // ang.add(angle(buffer));
        d1.add(dist_mm(buffer+4));
        // d1.add(dist_mm(buffer+8));
        // d1.add(dist_mm(buffer+12));
        // d1.add(dist_mm(buffer+16));

        packet_counter++;
        
        if(packet_counter == PACKET_SIZE){
          
          #if TRANSMISSION_TYPE == SERIAL_ONLY || TRANSMISSION_TYPE == WIFI_AND_SERIAL
          serializeJson(doc, Serial);
          Serial.println();
          #endif
          #if TRANSMISSION_TYPE == WIFI_ONLY || TRANSMISSION_TYPE == WIFI_AND_SERIAL
          serializeJson(doc, client);
          client.println();
          #endif
          
          // ang.clear();
          d1.clear();
          packet_counter = 0;
        }

        if(raw_data == 0xA0){

          buffer[0] = 0XFA;
          buffer[1] = raw_data;
          count = 2;
          packet_counter = 0;
          
          return;
        }

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