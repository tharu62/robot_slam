#include "encoders.h"

volatile long int m1_count = 0;
volatile long int m2_count = 0;
volatile long int m1_count_old = 0;
volatile long int m2_count_old = 0;

volatile long int v1_count = 0;
volatile long int v2_count = 0;

void interrupt_m1_c1(){
    if(digitalRead(M1_C2_PIN) == LOW){
        m1_count++;
    }else{
        m1_count--;
    }
}


void interrupt_m1_c2(){
    if(digitalRead(M1_C1_PIN) == LOW){
        m1_count--;
    }else{
        m1_count++;
    }
}


void interrupt_m2_c1(){
    if(digitalRead(M2_C2_PIN) == LOW){
        m2_count++;
    }else{
        m2_count--;
    }
}


void interrupt_m2_c2(){
    if(digitalRead(M2_C1_PIN) == LOW){
        m2_count--;
    }else{
        m2_count++;
    }
}

void interrupt_v1(TimerHandle_t xTimer){
    v1_count = m1_count_old - m1_count;
    Serial.printf("C_old: %ld, C_new: %ld, V1: %ld\n", m1_count_old, m1_count, v1_count);
    m1_count_old = m1_count;
}


void interrupt_v2(TimerHandle_t xTimer){
    v2_count = m2_count_old - m2_count;
    Serial.printf("C_old: %ld, C_new: %ld, V2: %ld\n", m2_count_old, m2_count, v2_count);
    m2_count_old = m2_count;
}