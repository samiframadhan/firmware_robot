#include <Arduino.h>
#include "Motor.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "motordata.pb.h"

Motor motor_kiri;
Motor motor_kanan;
int count = 200;

static char* TAG = "Debugging";

motor_configs left_motor;
motor_configs right_motor;

void setup() {
  
  Serial.begin(115200);

  while (!Serial.available())
  {
    delay(500);
    
  }
  

  // put your setup code here, to run once:
  left_motor.pin_direction  = 13;   // VR
  left_motor.pin_enable     = 4;    // EN/EL
  left_motor.pin_pwm        = 12;   // Z/F
  left_motor.pin_encoder    = 5;    // Signal
  left_motor.pwm_freq       = 1000;
  left_motor.reversed       = true;
  left_motor.ppr            = 10;

  right_motor.pin_direction = 19;
  right_motor.pin_enable    = 14;
  right_motor.pin_pwm       = 18;
  right_motor.pin_encoder   = 15;
  right_motor.pwm_freq      = 1000;
  right_motor.reversed      = true;
  right_motor.ppr           = 10;

  motor_kiri.config(left_motor);
  motor_kanan.config(right_motor);
  // motor_kanan.set_pindir(15, true);
  // motor_kanan.set_pinpwm(13);
  // motor_kanan.set_enable(12);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint32_t last_millis = millis();

  for (size_t i = 0; i < 100; i++)
  {
    // ESP_LOGE(TAG, "Encoder: %d", motor_kiri.get_encoder());
    if(i > 50){
      motor_kiri.set_pwm(200 - (i*4));
      motor_kanan.set_pwm(200 - (i*4));
    } else {
      motor_kiri.set_pwm(i*4);
      motor_kanan.set_pwm(i*4);
    }
    // int temp1 = millis() - last_millis;
    float temp = (float)motor_kiri.get_encoder_clear() / 0.5;
    float temp2 = (float)motor_kanan.get_encoder_clear() / 0.5;
    // last_millis = millis();
    ESP_LOGE(TAG, "Kiri temp: %.1f", temp);
    ESP_LOGE(TAG, "Kanan temp: %.1f", temp);
    delay(500);
  } 
}