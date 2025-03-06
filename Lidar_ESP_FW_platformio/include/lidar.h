#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <iostream>

void dump_packet(unsigned char* packet){

    for(int i=0; i<22; ++i){
        Serial.print(packet[i],HEX);
        Serial.print(" ");
    }
    Serial.println();

}

int angle(unsigned char *packet) { // 22 bytes in the packet
    return packet[1] - 0xA0; // 16 bits for the angle
}

float rpm(unsigned char *packet) { // 22 bytes in the packet
    return float(packet[2] | ((packet[3]<<8))) / 64.f;
}

bool verify_packet_checksum(unsigned char *packet) { // 22 bytes in the packet
    int checksum32 = 0;
    for (int i=0; i<10; i++)
    checksum32 = (checksum32<<1) + packet[2*i] + (packet[2*i+1]<<8);
    return packet[20]+(packet[21]<<8) == (((checksum32 & 0x7FFF) + (checksum32 >> 15)) & 0x7FFF);
}

int count_errors(unsigned char *buf) { // 1980 bytes in the buffer (90 packets)
    int nb_err = 0;
    for (int i=0; i<90; i++) {
        nb_err += !verify_packet_checksum(buf+i*22);
    }
    return nb_err;
}

// no return/max range/too low of reflectivity
bool invalid_data_flag(unsigned char *data) { // 4 bytes in the data buffer
    return (data[1] & 0x80) >> 7;
}

// object too close, possible poor reading due to proximity; kicks in at < 0.6m
bool strength_warning_flag(unsigned char *data) { // 4 bytes in the data buffer
    return (data[1] & 0x40) >> 6;
}

int dist_mm(unsigned char *data) { // 4 bytes in the data buffer
    return data[0] | (( data[1] & 0x3F) << 8); // 14 bits for the distance
}

int signal_strength(unsigned char *data) { // 4 bytes in the data buffer
    return data[2] | (data[3] << 8); // 16 bits for the signal strength
}

void print_all_data(unsigned char *buf) {
    for (int p=0; p<90; p++) { // for all 90 packets
        Serial.printf("#rpm: %f\n", rpm(buf + p*22));
        // std::cerr << "#rpm: " << rpm(buf + p*22) << std::endl;
        for (int i=0; i<4; i++) { // process 4 chunks per packet
            unsigned char *data = buf + p*22 + 4 + i*4; // current chunk pointer
            if (!invalid_data_flag(data)) {
                int angle_degrees = p*4+i;
                Serial.printf("angle: %d\tdistance: %d\n", angle_degrees, dist_mm(data));
                // std::cerr << "angle: " << angle_degrees << "\tdistance: " << dist_mm(data) << std::endl;
            }
        }
    }
}

void print_data(uint8_t *buffer){
    Serial.printf("%d,%f,%d,%d,%d,%d\n",
            angle(buffer),
            rpm(buffer),
            dist_mm(buffer+4),
            dist_mm(buffer+8),
            dist_mm(buffer+12),
            dist_mm(buffer+16)
            );
    Serial.print("\n");
}