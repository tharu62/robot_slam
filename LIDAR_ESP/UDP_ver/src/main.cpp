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
#define BUFFER_SIZE 22

// choose transmission type
#define TRANSMISSION_TYPE SERIAL_ONLY
// #define TRANSMISSION_TYPE WIFI_ONLY
// #define TRANSMISSION_TYPE WIFI_AND_SERIAL

#include <Arduino.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <string.h>
#include "lidar.h"

#include <WiFi.h>
#include <WiFiUdp.h>

#define START_BIT 0xFA

// USART 1
HardwareSerial Lidar(1);

// UDP connection
WiFiUDP udp;
IPAddress remote_ip;
uint16_t remote_port;

std::string data;

extern const char* ssid;
extern const char* password;

// reading variables
int packet_index = 0;
int buffer_index = 0;
unsigned char temp,raw_data_curr,raw_data_old;
unsigned char packet[BUFFER_SIZE];


// JSON variables
// JsonDocument jdoc;
// JsonArray data = jdoc["data"].to<JsonArray>();
// JsonArray strength = jdoc["str"].to<JsonArray>();

void setup(){
  // initialise serial comunication
  Serial.begin(BAUDRATE);
  Serial.println();

  Serial.println("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi at IP: " + WiFi.localIP().toString());

  // initialise UDP connection
  udp.begin(WIFI_PORT);
  while(udp.available() == 0){
    udp.parsePacket();
    delay(100);
  }

  remote_ip = udp.remoteIP();
  remote_port = udp.remotePort();
  Serial.println("Connected to remote IP: " + remote_ip.toString() + " at port: " + remote_port);

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
        
        // *packet* contiene i dati del lidar
        // angolo = angle(packet)
        // distanza_1 = dist_mm(packet+4)
        // distanza_2 = dist_mm(packet+8)
        // distanza_3 = dist_mm(packet+12)
        // distanza_4 = dist_mm(packet+16)

        for(int i=0; i<4; ++i){
          if(invalid_data_flag(packet+4*i) || strength_warning_flag(packet+4*i)){
            continue;
          }
          data = std::string("a") + std::to_string(4*angle(packet)+i) + std::string("d") + std::to_string(dist_mm(packet+4*i)) + std::string("\n");

          Serial.println(data.c_str());
          udp.beginPacket(remote_ip, remote_port);
          udp.write((unsigned char*)data.c_str(), data.length());
          udp.endPacket();
        }
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