#include "MotorDriver.hpp"


MotorDriverError MotorDriver::setup(uint32_t ena_freq = 5000UL, uint32_t enb_freq = 5000UL)
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


MotorDriverError MotorDriver::setM1Direction(int dir, uint32_t duty_cycle)
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


MotorDriverError MotorDriver::setM2Direction(int dir, uint32_t duty_cycle)
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

void setM1Speed(uint32_t duty_cycle){
    // set the speed of motor A
    ledcWrite(0, duty_cycle);
}


void setM2Speed(uint32_t duty_cycle){
    // set the speed of motor B
    ledcWrite(1, duty_cycle);
}


MotorDriverError MotorDriver::setValues(uint32_t in1, uint32_t in2, uint32_t in3, uint32_t in4, uint32_t ena_pwm, uint32_t enb_pwm)
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


MotorDriverError MotorDriver::stop()
{
    // free stop the motors by setting all GPIO levels to LOW and PWM to 0
    return setValues(0, 0, 0, 0, 0, 0);
}


MotorDriverError MotorDriver::forceStop()
{
    // force stop the motors by setting all GPIO levels to LOW and PWM to 0
    return setValues(0, 0, 0, 0, 0, 0);
}


void MotorDriver::writeSpeed(uint32_t duty_a, uint32_t duty_b)
{
    // write the speed to the motor driver
    ledcWrite(0, duty_a);
    ledcWrite(1, duty_b);
}


MotorDriver::~MotorDriver()
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