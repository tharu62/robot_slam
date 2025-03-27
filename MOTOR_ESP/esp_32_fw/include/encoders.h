#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>
#include <chrono>

#define M1_C1_PIN 13
#define M1_C2_PIN 12
#define M2_C1_PIN 14
#define M2_C2_PIN 27

#define COUNT_PER_BIG_ROTATION 5940
#define COUNT_PER_SMALL_ROTATION 22

extern volatile long int m1_count;
extern volatile long int m2_count;

extern volatile long int m1_count_old;
extern volatile long int m2_count_old;

extern volatile long int v1_count;
extern volatile long int v2_count;

extern std::chrono::time_point<std::chrono::system_clock> t_old;
extern std::chrono::milliseconds dt;

void interrupt_m1_c1();
void interrupt_m1_c2();

void interrupt_m2_c1();
void interrupt_m2_c2();

void interrupt_v(TimerHandle_t xTimer);

#endif // ENCODERS_H