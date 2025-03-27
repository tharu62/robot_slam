#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include <Arduino.h>
#include "driver/gpio.h"

#define MOTOR_DRIVER_DEBUG

/**
 * * @brief Debugging macros
 * * @note These macros are used for debugging purposes. If MOTOR_DRIVER_DEBUG is defined, the macros will print to the serial port.
 */
#ifdef MOTOR_DRIVER_DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

// Enum for motor driver error codes
enum MotorDriverError
{
    MOTOR_DRIVER_ERROR = -1,
    MOTOR_DRIVER_OK = 0,
    MOTOR_DRIVER_NOT_INIT = 1,
    MOTOR_DRIVER_INIT = 2,
    MOTOR_DRIVER_BUSY = 3,
    MOTOR_DRIVER_TIMEOUT = 4,
    MOTOR_DRIVER_INVALID_PARAM = 5
};

/**
 * This class is for controlling a L298N motor driver with an ESP32.
 */
class MotorDriver
{

public:
    /**
     * @brief Constructor to initialize the motor driver pins and speed
     * @param ena PWM pin for motor A
     * @param in1 Directional pin for motor A
     * @param in2 Directional pin for motor A
     * @param in3 Directional pin for motor B
     * @param in4 Directional pin for motor B
     * @param enb PWM pin for motor B
     */
    MotorDriver(gpio_num_t ena, gpio_num_t in1, gpio_num_t in2, gpio_num_t in3, gpio_num_t in4, gpio_num_t enb) 
            : ena(ena), in1(in1), in2(in2), in3(in3), in4(in4), enb(enb) {}

    /**
     * @brief Setup the motor driver pins and PWM frequency
     * @param ena_freq PWM frequency for motor A (default: 5000Hz)
     * @param enb_freq PWM frequency for motor B (default: 5000Hz)
     * @return MotorDriverError error code
     * @note This function sets up the GPIO pins for the motor driver and initializes the PWM frequency.
     * @example
     * ```cpp
     * MotorDriver motor_driver(ENA, IN1, IN2, IN3, IN4, ENB);
     * motor_driver.setup(5000, 5000);
     * ```
     */
    MotorDriverError setup(uint32_t ena_freq = 5000UL, uint32_t enb_freq = 5000UL);

    /**
     * * @brief Set the direction of motor A
     * * * @param dir Direction of motor A (1 = FORWARD, -1 = BACKWARD, 0 = STOP)
     * * * @param duty_cycle PWM duty cycle value for motor A (0-255)
     * * * @return MotorDriverError error code
     * 
     */
    MotorDriverError setM1Direction(int dir, uint32_t duty_cycle);

    /**
     * * @brief Set the direction of motor B
     * * * @param dir Direction of motor B (1 = FORWARD, -1 = BACKWARD, 0 = STOP)
     * * * @param duty_cycle PWM duty cycle value for motor B (0-255)
     * 
     * * * @return MotorDriverError error code
     */
    MotorDriverError setM2Direction(int dir, uint32_t duty_cycle);

    /**
     * * @brief Set the speed of motor A
     * * @param duty_cycle PWM duty cycle value for motor A (0-255)
     */
    void setM1Speed(uint32_t duty_cycle);

    /**
     * * @brief Set the speed of motor B
     * * @param duty_cycle PWM duty cycle value for motor B (0-255)
     */
    void setM2Speed(uint32_t duty_cycle);

    /**
     * * @brief Set the values for the motor driver
     * * @param in1 Directional pin for motor A
     * * @param in2 Directional pin for motor A
     * * @param in3 Directional pin for motor B
     * * @param in4 Directional pin for motor B
     * * @param ena_pwm PWM value for motor A
     * * @param enb_pwm PWM value for motor B
     * 
     * * @return MotorDriverError error code
     * 
     * * @note This function sets the GPIO levels for the motor driver and the PWM values for the motors.
     * *    The values are set as follows:
     *
     * *    - Motor A FORWARD: in1 = HIGH, in2 = LOW\n
     *
     * *    - Motor A BACKWARD: in1 = LOW, in2 = HIGH
     *
     * *    - Motor B FORWARD: in3 = HIGH, in4 = LOW
     *
     * *    - Motor B BACKWARD: in3 = LOW, in4 = HIGH
     */
    MotorDriverError setValues(uint32_t in1, uint32_t in2, uint32_t in3, uint32_t in4, uint32_t ena_pwm, uint32_t enb_pwm);

    /**
     * * @brief Stop the motors by setting all GPIO levels to LOW and PWM to 0
     * * @return MotorDriverError error code
     * * @note This function only removes power from the motors, it does not force stop them.
     */
    MotorDriverError stop();

    /**
     * * @brief Force stop the motors by setting all GPIO levels to LOW and PWM to 0
     * * @return MotorDriverError error code
     * * @note This function sets all GPIO levels to LOW and PWM to 0.
     */
    MotorDriverError forceStop();

    /**
     * * @brief Write the speed to the motor driver
     * * @param duty_a PWM duty cycle value for motor A
     * * @param duty_b PWM duty cycle value for motor B
     * * @return MotorDriverError error code
     */
    void writeSpeed(uint32_t duty_a, uint32_t duty_b);

    /**
     * * @brief Destructor to free the motor driver pins and PWM channels
     * * @note This function frees the GPIO pins and PWM channels used by the motor driver.
     */
    ~MotorDriver();

private:
    // PWM pin for motor A and motor B
    gpio_num_t ena;
    gpio_num_t enb;

    // Directional pins for motor A
    gpio_num_t in1;
    gpio_num_t in2;
    // Directional pins for motor B
    gpio_num_t in3;
    gpio_num_t in4;
};

#endif // MOTOR_DRIVER_H