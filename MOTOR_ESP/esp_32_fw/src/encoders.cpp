#include "encoders.h"

volatile long int m1_count = 0;
volatile long int m2_count = 0;

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