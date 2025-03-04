#include <HardwareSerial.h>
#include <vector>

// specifications for UART data trasmission
#define RXPIN 3
#define TXPIN 18
#define BAUDRATE 115200 //transmission speed
#include <WiFi.h>                                                                              
#include <ArduinoJson.h>
#include <string.h>                             

const char* ssid = "Rete Da Pesca";
const char* password = "Portineri@1963";

WiFiServer wifiServer(80);

// using UART1
HardwareSerial Lidar(1);

// data acquisition variables
uint8_t read_byte;
uint8_t read_byte_old;

// data input buffer
std::vector<unsigned char> data_buffer;

//ESP32Time rtc(3600);  // offset in seconds GMT+1
typedef std::vector<unsigned char>::const_iterator buffer_pointer;

float dist(buffer_pointer it){
  //if(is_dist_valid(it) == false) return -1;
  return *it | (( *std::next(it,1) & 0x3F) << 8);
}

void setup(){
  // initialise serial comunication
  Serial.begin(BAUDRATE);

  //setup server and wifi connection                            
  WiFi.begin(ssid, password);                         
  Serial.println("Establishing connection to WiFi with SSID: " + String(ssid));    
  while (WiFi.status() != WL_CONNECTED) {             
    delay(500);
    Serial.print(".");
  }
  Serial.print("Connected to network with IP address: ");
  Serial.println(WiFi.localIP());                     
  wifiServer.begin();      
  
  // ensure TXPIN is only for receiving data
  pinMode(TXPIN, INPUT);
  Lidar.begin(BAUDRATE, SERIAL_8N1, RXPIN, TXPIN);
  //rtc.setTime(22,32,0,1,3,2025);
}

void loop() {
  
  String data;
  WiFiClient client = wifiServer.available();
  for(int i = 0; i < sizeof(data); i++){
    data[i] = '0';
  }

  if(client.connected()){

    //Serial.println("Connection with new client established, Client IP address: ");
    //Serial.println(client.remoteIP());

    while (client.connected()) {

      while(Lidar.available()){
        
        read_byte = Lidar.read();
          
        if(read_byte != -1 && read_byte_old == 0xfa && read_byte == 0xa0){

          data_buffer.pop_back(); // remove last 0xFA
          buffer_pointer it;

          for(it=data_buffer.begin(); it!=data_buffer.end(); it++){

            if(*it == 0xfa && *(std::next(it,1)) <= 0xf9 && *(std::next(it,1)) >= 0xa0 ){
              // I valori letti dal lidar sono sempre inoltrati al client appena disponibili.
              String jsonString = "";                           // create a JSON string for sending data to the client
              StaticJsonDocument<200> doc;                      // create a JSON container
              JsonObject object = doc.to<JsonObject>();         // create a JSON Object
              // object["cmd"] = 1;                                // write data into the JSON object
              // object["angle"] = 4*(*std::next(it,1) - 0xa0)+i;
              // object["rpm"] = float(*std::next(it,2) | ((*std::next(it,3)<<8))) / 64.f;
              // object["dist"] = dist(std::next(it,4+4*i));                         // write data into the JSON object
              object["angle"] = 4*(*std::next(it,1) - 0xa0);
              object["rpm"] = float(*std::next(it,2) | ((*std::next(it,3)<<8))) / 64.f;
              object["dist"] = dist(std::next(it,8));       // write data into the JSON object
              serializeJson(doc, jsonString);                   // convert JSON object to string
              //Serial.println(jsonString);                     // print JSON string to console for debug purposes (you can comment this out)
              client.println(jsonString);

              // for(int i = 0; i < 4; i++){
              //   String jsonString = "";                           // create a JSON string for sending data to the client
              //   StaticJsonDocument<200> doc;                      // create a JSON container
              //   JsonObject object = doc.to<JsonObject>();         // create a JSON Object
              //   object["cmd"] = 1;                                // write data into the JSON object
              //   object["angle"] = 4*(*std::next(it,1) - 0xa0)+i;
              //   object["rpm"] = float(*std::next(it,2) | ((*std::next(it,3)<<8))) / 64.f;
              //   object["dist"] = dist(std::next(it,4+4*i));                         // write data into the JSON object
              //   serializeJson(doc, jsonString);                   // convert JSON object to string
              //   //Serial.println(jsonString);                     // print JSON string to console for debug purposes (you can comment this out)
              //   client.println(jsonString);
              // }
            }
          }
          data_buffer.clear();
          data_buffer.push_back(0xfa); //re-add last 0xFA
        }
        data_buffer.push_back(read_byte);
        read_byte_old = read_byte;
      }
    }
  } else {
    Serial.println("No Client connected...");
  }
  client.stop();
  delay(500);
}


// Il server riceve solo due tipi di messaggio: <lidar_on> , <lidar_off> .
//char c;
//data = client.readStringUntil('\0'); // received the server's answer
// if (data != "\0")
// { 
//   // Interpreto comando del client...
//   //for(int i = 0; data[i] != '\0'; i++){
//     //Serial.printf("%c" , data[i]);
//   //}
//   //Serial.printf("\n");
//   //Serial.printf("Client : %c\n" , data[30]);
//   client.flush();
//   data = "\0";
// }