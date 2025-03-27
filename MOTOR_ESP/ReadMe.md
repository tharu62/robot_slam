# Motor Controller Communication Protocol

The Motor Controller is composed of a single ESP32-Devkit-1 connected to one L298N Motor Driver Module and two planetary geared motors with encoders.

## The Messages

The messages protocol used is the JSON standard: 
```javascript
{"key":value}
```

**"d1"** and **"d2"** signal the direction: 
    1 = FORWARD
    -1 = BACKWARD
    0 = FREE STOP

**"s1"** and **"s2"** signal the duty cycle of the PWM signal for the motors a.k.a. the speed of the motor.

## Wired Communication Protocol with Raspberry Pi

to be defined...