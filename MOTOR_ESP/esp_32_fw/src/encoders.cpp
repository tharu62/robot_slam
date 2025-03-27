#include "encoders.h"


volatile long int m1_count = 0;
volatile long int m2_count = 0;
volatile long int m1_count_old = 0;
volatile long int m2_count_old = 0;

volatile long int v1_count = 0;
volatile long int v2_count = 0;

std::chrono::time_point<std::chrono::system_clock> t_old = std::chrono::system_clock::now();
std::chrono::milliseconds dt = std::chrono::milliseconds(0);

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

void interrupt_v(TimerHandle_t xTimer){
    auto now = std::chrono::system_clock::now();
    dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_old);
    t_old = std::chrono::system_clock::now();
    v2_count = m2_count_old - m2_count;
    m2_count_old = m2_count;
    
    v1_count = m1_count_old - m1_count;
    m1_count_old = m1_count;
}