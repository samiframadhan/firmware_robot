#include "header.h"

const char* ssid = "Nintendo";
const char* password = "papahbaik";

ros::NodeHandle nh;
Motor motor_kiri;
Motor motor_kanan;
int count = 200;
uint8_t buffer[128];
size_t message_length;
bool status;
float left_speed;
float right_speed;
long last_mill;

void cmdVelCb(const geometry_msgs::Twist& cmd_vel);

ros::Subscriber <geometry_msgs::Twist> cmdVel("cmd_vel", cmdVelCb);
static char* TAG = "Debugging";

motor_configs left_motor;
motor_configs right_motor;

void setup() {
  Serial.begin(115200);
  
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
  
  motor_kiri.config(left_motor);
  motor_kanan.config(right_motor);
  // motor_kanan.set_pindir(15, true);
  // motor_kanan.set_pinpwm(13);
  // motor_kanan.set_enable(12);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(cmdVel);
  if(status){
    ESP_LOGI(TAG, "Status: speed = %d", motormsg.speed);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  // uint32_t last_millis = millis();

  nh.spinOnce();
  if(last_mill - millis() < 1000){
    motor_kanan.set_pwm(right_speed);
    motor_kiri.set_pwm(left_speed);
  }
  
  // for (size_t i = 0; i < 100; i++)
  // {
  //   // ESP_LOGE(TAG, "Encoder: %d", motor_kiri.get_encoder());
  //   if(i > 50){
  //     motor_kiri.set_pwm(200 - (i*4));
  //     motor_kanan.set_pwm(200 - (i*4));
  //   } else {
  //     motor_kiri.set_pwm(i*4);
  //     motor_kanan.set_pwm(i*4);
  //   }
  //   float temp = (float)motor_kiri.get_encoder_clear() / 0.5;
  //   float temp2 = (float)motor_kanan.get_encoder_clear() / 0.5;
  //   // ESP_LOGI(TAG, "Kiri temp: %.1f", temp);
  //   // ESP_LOGI(TAG, "Kanan temp: %.1f", temp);

  //   delay(500);
  // } 
}

void cmdVelCb(const geometry_msgs::Twist& data){
  left_speed = data.linear.x - data.angular.z * (0.2 / 2);
  right_speed = data.linear.x + data.angular.z * (0.2 / 2);
}