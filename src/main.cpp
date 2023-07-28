#include <Arduino.h>
#include "Motor.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "motordata.pb.h"

Motor motor_kiri;
Motor motor_kanan;
int count = 200;
uint8_t buffer[128];
size_t message_length;
bool status;

static char* TAG = "Debugging";

motor_configs left_motor;
motor_configs right_motor;

void setup() {
  // put your setup code here, to run once:
  left_motor.pin_direction  = 13;   // VR
  left_motor.pin_enable     = 4;    // EN/EL
  left_motor.pin_pwm        = 12;   // Z/F
  left_motor.pin_encoder    = 5;    // Signal
  left_motor.pwm_freq       = 1000;
  left_motor.reversed       = true;
  left_motor.ppr            = 10;

  right_motor.pin_direction = 19;   // VR
  right_motor.pin_enable    = 14;   // EN/EL
  right_motor.pin_pwm       = 18;   // Z/F
  right_motor.pin_encoder   = 15;   // Signal
  right_motor.pwm_freq      = 1000;
  right_motor.reversed      = true;
  right_motor.ppr           = 10;

  motorData message = motorData_init_zero;

  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  message.direction = true;
  message.speed = 1000;

  status = pb_encode(&stream, motorData_fields, &message);
  message_length = stream.bytes_written;
  
  motor_kiri.config(left_motor);
  motor_kanan.config(right_motor);
  // motor_kanan.set_pindir(15, true);
  // motor_kanan.set_pinpwm(13);
  // motor_kanan.set_enable(12);

  motorData motormsg = motorData_init_zero;
  pb_istream_t s = pb_istream_from_buffer(buffer, message_length);
  status = pb_decode(&s, motorData_fields, &motormsg);

  Serial.begin(115200);
  if(status){
    Serial.print("Berhasil!");
    
  }
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