#include <AFMotor.h>

AF_DCMotor M1(1);

void setup() {
  
  Serial.begin(9600);
  M1.setSpeed( 60 );

}

void loop() {
  
  motor_controll();

}

void motor_controll(){
  
  M1.run(RELEASE);

  delay(2000);

  M1.run(FORWARD);

  delay(2000); 

  M1.run(RELEASE);

  delay(2000); 

  M1.run(BACKWARD);

  delay(2000);

}
