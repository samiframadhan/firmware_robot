#include <Arduino.h>
#include "Encoder.h"
#include <QuickPID.h>
#define FORWARD 0
#define BACKWARD 1

struct motor_configs {
                             // Referensi ke motor driver brushlessnya
    int pin_direction   = 0; // VR
    int pin_encoder     = 0; // Signal
    int pin_enable      = 0; // EL atau Enable
    int pin_pwm         = 0; // Z/F
    int pwm_freq        = 0;
    bool reversed       = false;
    uint ppr            = 0;
    // float K_P, K_I, K_D;
};

class Motor
{
private:
    int64_t count = 0;
    float set_point = 0.0, input = 0.0, output = 0.0;
    motor_configs configs;
    int enable_pin = 0;
    QuickPID motor_pid;
    int pwm_channel = 0;
    int getpindir();
    int getpinpwm();
    uint8_t getpinpwm_channel();
    int64_t last_pulse = 0;
    uint64_t last_millis = 0;
    
public:
    Encoder motor_encoder;
    static uint8_t channel;
    uint32_t freq = 800;
    uint8_t res = 8;
    void set_pinpwm(int pinpwm);
    void config(motor_configs);
    float get_rpm();

    int64_t get_encoder_clear();
    int64_t get_encoder();
    bool set_pwm(int pwm_val);
    int absolute(int value);
    void auto_speed();

    Motor(/* args */);
    ~Motor();
};
