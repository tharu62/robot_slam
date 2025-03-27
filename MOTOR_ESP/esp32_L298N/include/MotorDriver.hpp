#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include <stdint.h>
#include <Arduino.h>
#include "driver/gpio.h"

#define DEBUG

#ifdef DEBUG
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
    MotorDriver(gpio_num_t ena, gpio_num_t in1, gpio_num_t in2, gpio_num_t in3, gpio_num_t in4, gpio_num_t enb) : ena(ena), in1(in1), in2(in2), in3(in3), in4(in4), enb(enb) {}

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
    MotorDriverError setup(uint32_t ena_freq = 5000UL, uint32_t enb_freq = 5000UL)
    {

        DEBUG_PRINTLN("Setting up Motor Driver...");

        // setup GPIO pins for motor driver
        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT_OD;
        io_conf.pin_bit_mask = (1ULL << this->ena) | (1ULL << this->in1) | (1ULL << this->in2) | (1ULL << this->in3) | (1ULL << this->in4) | (1ULL << this->enb);

        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        auto err = gpio_config(&io_conf);

        if (err != ESP_OK)
        {
            DEBUG_PRINT("GPIO config failed: ");
            DEBUG_PRINTLN(esp_err_to_name(err));
            return MOTOR_DRIVER_ERROR;
        }
        // gpio set ended

        // set Motors to stop initially
        setValues(0, 0, 0, 0, 0, 0);

        // setup PWM for ENA and ENB
        ledcSetup(0, ena_freq, 8);
        ledcSetup(1, enb_freq, 8);

        ledcAttachPin((uint8_t)this->ena, 0);
        ledcAttachPin((uint8_t)this->enb, 1);

        ledcWrite(0, 0);
        ledcWrite(1, 0);
        // ledc set ended
        DEBUG_PRINTLN("Motor Driver setup complete");
    }

    /**
     * * @brief Set the direction of motor A
     * * * @param dir Direction of motor A (1 = FORWARD, -1 = BACKWARD, 0 = STOP)
     * * * @param duty_cycle PWM duty cycle value for motor A (0-255)
     * * * @return MotorDriverError error code
     * 
     */
    MotorDriverError M1setDirection(int dir, uint32_t duty_cycle)
    {
        esp_err_t err = 0;
        switch (dir)
        {
        case 1: // FORWARD
            err = gpio_set_level(this->in1, 1);
            if (err != ESP_OK)
            {
                DEBUG_PRINT("GPIO set level failed: ");
                DEBUG_PRINTLN(esp_err_to_name(err));
                return MOTOR_DRIVER_ERROR;
            }
            err = gpio_set_level(this->in2, 0);
            if (err != ESP_OK)
            {
                DEBUG_PRINT("GPIO set level failed: ");
                DEBUG_PRINTLN(esp_err_to_name(err));
                return MOTOR_DRIVER_ERROR;
            }
            break;

        case -1: // BACKWARD
            err = gpio_set_level(this->in1, 0);
            if (err != ESP_OK)
            {
                DEBUG_PRINT("GPIO set level failed: ");
                DEBUG_PRINTLN(esp_err_to_name(err));
                return MOTOR_DRIVER_ERROR;
            }
            err = gpio_set_level(this->in2, 1);
            if (err != ESP_OK)
            {
                DEBUG_PRINT("GPIO set level failed: ");
                DEBUG_PRINTLN(esp_err_to_name(err));
                return MOTOR_DRIVER_ERROR;
            }
            break;

        case 0: // STOP
            esp_err_t err = gpio_set_level(this->in1, 0);
            if (err != ESP_OK)
            {
                DEBUG_PRINT("GPIO set level failed: ");
                DEBUG_PRINTLN(esp_err_to_name(err));
                return MOTOR_DRIVER_ERROR;
            }
            err = gpio_set_level(this->in2, 0);
            if (err != ESP_OK)
            {
                DEBUG_PRINT("GPIO set level failed: ");
                DEBUG_PRINTLN(esp_err_to_name(err));
                return MOTOR_DRIVER_ERROR;
            }
            return MOTOR_DRIVER_OK;
            break;

        default:
            DEBUG_PRINTLN("Invalid direction value. Use 1 for FORWARD, -1 for BACKWARD, 0 for STOP.");
            return MOTOR_DRIVER_INVALID_PARAM;
        }
    }

    /**
     * * @brief Set the direction of motor B
     * * * @param dir Direction of motor B (1 = FORWARD, -1 = BACKWARD, 0 = STOP)
     * * * @param duty_cycle PWM duty cycle value for motor B (0-255)
     * * * @return MotorDriverError error code
     * 
     */
    MotorDriverError M2setDirection(int dir, uint32_t duty_cycle)
    {
        esp_err_t err = 0;
        switch (dir)
        {
        case 1: // FORWARD
            err = gpio_set_level(this->in3, 1);
            if (err != ESP_OK)
            {
                DEBUG_PRINT("GPIO set level failed: ");
                DEBUG_PRINTLN(esp_err_to_name(err));
                return MOTOR_DRIVER_ERROR;
            }
            err = gpio_set_level(this->in4, 0);
            if (err != ESP_OK)
            {
                DEBUG_PRINT("GPIO set level failed: ");
                DEBUG_PRINTLN(esp_err_to_name(err));
                return MOTOR_DRIVER_ERROR;
            }
            break;

        case -1: // BACKWARD
            err = gpio_set_level(this->in3, 0);
            if (err != ESP_OK)
            {
                DEBUG_PRINT("GPIO set level failed: ");
                DEBUG_PRINTLN(esp_err_to_name(err));
                return MOTOR_DRIVER_ERROR;
            }
            err = gpio_set_level(this->in4, 1);
            if (err != ESP_OK)
            {
                DEBUG_PRINT("GPIO set level failed: ");
                DEBUG_PRINTLN(esp_err_to_name(err));
                return MOTOR_DRIVER_ERROR;
            }
            break;

        case 0: // STOP
            esp_err_t err = gpio_set_level(this->in3, 0);
            if (err != ESP_OK)
            {
                DEBUG_PRINT("GPIO set level failed: ");
                DEBUG_PRINTLN(esp_err_to_name(err));
                return MOTOR_DRIVER_ERROR;
            }
            err = gpio_set_level(this->in4, 0);
            if (err != ESP_OK)
            {
                DEBUG_PRINT("GPIO set level failed: ");
                DEBUG_PRINTLN(esp_err_to_name(err));
                return MOTOR_DRIVER_ERROR;
            }
            return MOTOR_DRIVER_OK;
            break;
            
        default:
            DEBUG_PRINTLN("Invalid direction value. Use 1 for FORWARD, -1 for BACKWARD, 0 for STOP.");
            return MOTOR_DRIVER_INVALID_PARAM;
        }
    }

    MotorDriverError M2setDirection(uint32_t in3, uint32_t in4)
    {
        // set GPIO levels for IN3 and IN4
        // 0 = LOW, 1 = HIGH
        esp_err_t err = 0;
        err = gpio_set_level(this->in3, in3);
        if (err != ESP_OK)
        {
            DEBUG_PRINT("GPIO set level failed: ");
            DEBUG_PRINTLN(esp_err_to_name(err));
        }
        err = gpio_set_level(this->in4, in4);
        if (err != ESP_OK)
        {
            DEBUG_PRINT("GPIO set level failed: ");
            DEBUG_PRINTLN(esp_err_to_name(err));
        }
    }

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
    MotorDriverError setValues(uint32_t in1, uint32_t in2, uint32_t in3, uint32_t in4, uint32_t ena_pwm, uint32_t enb_pwm)
    {

        // set GPIO levels for IN1, IN2, IN3, IN4
        // 0 = LOW, 1 = HIGH
        esp_err_t err = 0;
        err = gpio_set_level(this->in1, in1);
        if (err != ESP_OK)
        {
            DEBUG_PRINT("GPIO set level failed: ");
            DEBUG_PRINTLN(esp_err_to_name(err));
        }
        err = gpio_set_level(this->in2, in2);
        if (err != ESP_OK)
        {
            DEBUG_PRINT("GPIO set level failed: ");
            DEBUG_PRINTLN(esp_err_to_name(err));
        }
        err = gpio_set_level(this->in3, in3);
        if (err != ESP_OK)
        {
            DEBUG_PRINT("GPIO set level failed: ");
            DEBUG_PRINTLN(esp_err_to_name(err));
        }
        err = gpio_set_level(this->in4, in4);
        if (err != ESP_OK)
        {
            DEBUG_PRINT("GPIO set level failed: ");
            DEBUG_PRINTLN(esp_err_to_name(err));
        }

        // set PWM values for ENA and ENB
        ledcWrite(0, ena_pwm);
        ledcWrite(1, enb_pwm);

        return MOTOR_DRIVER_OK;
    }

    /**
     * * @brief Stop the motors by setting all GPIO levels to LOW and PWM to 0
     * * @return MotorDriverError error code
     * * @note This function only removes power from the motors, it does not force stop them.
     */
    MotorDriverError stop()
    {
        // free stop the motors by setting all GPIO levels to LOW and PWM to 0
        return setValues(0, 0, 0, 0, 0, 0);
    }

    /**
     * * @brief Force stop the motors by setting all GPIO levels to LOW and PWM to 0
     * * @return MotorDriverError error code
     * * @note This function sets all GPIO levels to LOW and PWM to 0.
     */
    MotorDriverError forceStop()
    {
        // force stop the motors by setting all GPIO levels to LOW and PWM to 0
        return setValues(0, 0, 0, 0, 0, 0);
    }

    /**
     * * @brief Write the speed to the motor driver
     * * @param duty_a PWM duty cycle value for motor A
     * * @param duty_b PWM duty cycle value for motor B
     * * @return MotorDriverError error code
     */
    void writeSpeed(uint32_t duty_a, uint32_t duty_b)
    {
        // write the speed to the motor driver
        ledcWrite(0, duty_a);
        ledcWrite(1, duty_b);
    }

    /**
     * * @brief Destructor to free the motor driver pins and PWM channels
     * * @note This function frees the GPIO pins and PWM channels used by the motor driver.
     */
    ~MotorDriver()
    {

        DEBUG_PRINTLN("Freeing Motor Driver...");

        // stop the motors
        stop();
        // free the PWM channels
        ledcDetachPin((uint8_t)this->ena);
        ledcDetachPin((uint8_t)this->enb);

        // free the GPIO pins
        gpio_reset_pin(this->ena);
        gpio_reset_pin(this->in1);
        gpio_reset_pin(this->in2);
        gpio_reset_pin(this->in3);
        gpio_reset_pin(this->in4);
        gpio_reset_pin(this->enb);
    }

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

#endif // MOTOR_DRIVER_HPP