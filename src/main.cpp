#include <Arduino.h>
#include "Motor.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "motordata.pb.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* ssid = "UpdateESP32";
const char* password = "UpdateUnpad2023";

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
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    ESP_LOGE(TAG, "Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      ESP_LOGE(TAG, "Start updating %s", type);
    })
    .onEnd([]() {
      ESP_LOGE(TAG, "End");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      ESP_LOGE(TAG ,"Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      ESP_LOGE(TAG, "Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) ESP_LOGE(TAG, "Auth Failed");
      else if (error == OTA_BEGIN_ERROR) ESP_LOGE(TAG, "Begin Failed");
      else if (error == OTA_CONNECT_ERROR) ESP_LOGE(TAG, "Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) ESP_LOGE(TAG, "Receive Failed");
      else if (error == OTA_END_ERROR) ESP_LOGE(TAG, "End Failed");
    });

  ArduinoOTA.begin();
  
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
  
  if(status){
    ESP_LOGE(TAG, "Status: speed = %d", motormsg.speed);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  uint32_t last_millis = millis();
  ArduinoOTA.handle();
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