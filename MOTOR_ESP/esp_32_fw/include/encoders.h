#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>

#define M1_C1_PIN 12
#define M1_C2_PIN 13
#define M2_C1_PIN 14
#define M2_C2_PIN 27

extern volatile long int m1_count;
extern volatile long int m2_count;

extern volatile long int m1_count_old;
extern volatile long int m2_count_old;

extern volatile long int v1_count;
extern volatile long int v2_count;


void interrupt_m1_c1();
void interrupt_m1_c2();

void interrupt_m2_c1();
void interrupt_m2_c2();

void interrupt_v1(TimerHandle_t xTimer);
void interrupt_v2(TimerHandle_t xTimer);

#endif // ENCODERS_H