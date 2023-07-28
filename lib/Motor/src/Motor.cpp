#include "Motor.h"

uint8_t Motor::channel = 0;

static char *TAG = "Motor";

Motor::Motor() : motor_pid(&set_point, &input, &output)
{
    pwm_channel = channel;
    channel++;
}

Motor::~Motor(){
    motor_encoder.detach();
    ledcDetachPin(configs.pin_pwm);
}

void Motor::config(motor_configs conf){
    configs = conf;
    motor_encoder.attach_single_edge(configs.pin_encoder, configs.pin_direction);
    pinMode(configs.pin_direction, OUTPUT);
    pinMode(configs.pin_enable, OUTPUT);
    // ESP_LOGI(TAG, "Pin Direction: %d, Pin Enable: %d", configs.pin_direction, configs.pin_enable);
    set_pinpwm(configs.pin_pwm);
    if(configs.pin_pwm != 0){
        ledcAttachPin(configs.pin_pwm, this->pwm_channel);
        ledcSetup(this->pwm_channel, configs.pwm_freq, 8);
    }
    digitalWrite(configs.pin_enable, HIGH);
    // motor_pid.SetTunings(conf.K_P, conf.K_I, conf.K_D);
}

void Motor::set_pinpwm(int pinpwm){
    configs.pin_pwm = pinpwm;
    ledcAttachPin(configs.pin_pwm, this->pwm_channel);
    // ledcSetup(motor_pwm_channel, freq, res);
    ledcWrite(this->pwm_channel, 0);
}

bool Motor::set_pwm(int pwm){
    if((configs.pin_enable == 0) || (configs.pin_direction == 0) || (configs.pin_encoder == 0)){
        ESP_LOGE(TAG, "Pins not yet configured correctly!");
        return 0;
    }

    if(pwm > 250){ //Capped to 250 for stability
        pwm = 250;
    }

    if(pwm > 0){
        ledcWrite(this->pwm_channel, pwm);
        // ESP_LOGI(TAG, "PWM Channel invoked: %d, Pin Enable: %d is %d", this->pwm_channel, configs.pin_enable, 1);
        ESP_LOGI(TAG, "PWM Val: %d", pwm);
        if(configs.reversed){
            digitalWrite(configs.pin_direction, HIGH);
            ESP_LOGI(TAG, "Direction: should be 1 currently %d", digitalRead(configs.pin_direction));
            // ESP_LOGI(TAG, "Pin Direction: %d is %d", configs.pin_direction, BACKWARD);
        } else {
            digitalWrite(configs.pin_direction, LOW);
            ESP_LOGI(TAG, "Direction: should be 0 currently %d", digitalRead(configs.pin_direction));
            // ESP_LOGI(TAG, "Pin Direction: %d is %d", configs.pin_direction, FORWARD);
        }
        return 1;
    } else {
        ledcWrite(this->pwm_channel, abs(pwm));
        ESP_LOGI(TAG, "PWM Val: %d", pwm);
        if(configs.reversed){
            digitalWrite(configs.pin_direction, LOW);
            ESP_LOGI(TAG, "Direction: should be 0 currently %d", digitalRead(configs.pin_direction));
            // ESP_LOGI(TAG, "Pin Direction: %d is %d", configs.pin_direction, FORWARD);
        } else {
            digitalWrite(configs.pin_direction, HIGH);
            ESP_LOGI(TAG, "Direction: should be 1 currently %d", digitalRead(configs.pin_direction));
            // ESP_LOGI(TAG, "Pin Direction: %d is %d", configs.pin_direction, BACKWARD);
        }
        return 1;
    }
}

int Motor::getpindir(){
    return configs.pin_direction;
}

int Motor::getpinpwm(){
    return configs.pin_pwm;
}

uint8_t Motor::getpinpwm_channel(){
    return this->pwm_channel;
}

int64_t Motor::get_encoder_clear(){
    if(configs.pin_encoder == 0){
        ESP_LOGE(TAG, "Encoder haven't configured yet");
        return 0;
    }
    count = motor_encoder.read_and_clear();
    return count;
}

int64_t Motor::get_encoder(){
    if(configs.pin_encoder == 0){
        ESP_LOGE(TAG, "Encoder haven't configured yet");
        return 0;
    }
    count = motor_encoder.read();
    return count;
}

float Motor::get_rpm(){
    uint32_t current_duration = millis() - last_millis;
    input = (get_encoder_clear())/configs.ppr/current_duration;
    last_millis = millis();
    return input;
}

void Motor::auto_speed(){
    motor_pid.Compute();
    set_pwm(output);
}

int Motor::absolute(int value){
    if(value>0){
        return value;
    } else return -value;
}