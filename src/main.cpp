#include "header.h"


// Start: IMU Variables

uint8_t imu_interrupt_status;
uint8_t imu_dev_status;
uint16_t imu_packet_size;
uint16_t imu_fifo_count;
uint8_t imu_fifo_buffer [64];

float imu_offsets[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //Acc X, Y, Z, Gyro X, Y, Z
float acc_scale = 0.00059855;   // measured in m/s^2 is 9.80665/16384 
float gyro_scale = 0.000133231; // measured in rad/s is 0,017453293/131
float acc_stddev = 0.033; // Dari sini https://robotics.stackexchange.com/questions/8860/angle-random-walk-vs-rate-noise-density-mpu6050
float gyro_stddev = 0.033;
float orientation_stddev = 0.0;

float wheelbase = 0.2;

Quaternion imuQ;
VectorInt16 aa;
VectorInt16 gyro;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;

sensor_msgs::Imu imu_ros_msg;
rovit_navsys::debug motor_msg;
float motor_data_array[7];

MPU6050 imu;

bool imu_setup();
void imu_update();
void set_imu_offset();

// End: IMU Variables

ros::NodeHandle nh;

ros::Publisher imu_ros_pub("imu/data_raw", &imu_ros_msg);
ros::Publisher motor_pub("motor", &motor_msg);

Motor motor_kiri;
Motor motor_kanan;
int count = 200;
uint8_t buffer[128];
size_t message_length;
bool imu_ready;
float left_speed;
float right_speed;
long last_mill;
long last_mill2;
long last_mill3;
int test_pwm = 0;
bool up = false;

void cmdVelCb(const geometry_msgs::Twist& cmd_vel);

ros::Subscriber <geometry_msgs::Twist> cmdVel("cmd_vel", cmdVelCb);
static char* TAG = "Debugging";

motor_configs left_motor;
motor_configs right_motor;

void send_motor();

void setup() {
  // Serial.begin(115200);
  
  // put your setup code here, to run once:
  left_motor.pin_direction  = 12;   // Z/F
  left_motor.pin_enable     = 4;    // EN/EL
  left_motor.pin_pwm        = 13;   // VR
  left_motor.pin_encoder    = 5;    // Signal
  left_motor.pwm_freq       = 1000;
  left_motor.reversed       = true;
  left_motor.ppr            = 47;
  left_motor.K_P            = 1.0;
  left_motor.periodUs       = 10000;
  // left_motor.K_I            = -3.39;

  right_motor.pin_direction = 18;   // Z/F
  right_motor.pin_enable    = 14;   // EN/EL
  right_motor.pin_pwm       = 19;   // VR
  right_motor.pin_encoder   = 15;   // Signal
  right_motor.pwm_freq      = 1000;
  right_motor.reversed      = false;
  right_motor.ppr           = 47;
  right_motor.K_P           = 1.0;
  right_motor.periodUs      = 10000;
  
  motor_kiri.config(left_motor);
  motor_kanan.config(right_motor);
  motor_kiri.change_sp(0.0);
  motor_kanan.change_sp(0.0);
  // motor_kanan.set_pindir(15, true);
  // motor_kanan.set_pinpwm(13);
  // motor_kanan.set_enable(12);

  // Start: Initialize ROS

  nh.getHardware()->setBaud(230400);
  nh.initNode();
  nh.subscribe(cmdVel);
  nh.advertise(motor_pub);
  nh.advertise(imu_ros_pub);

  last_mill = millis();
  last_mill2 = millis();

  // End: Initialize ROS

  // Start: Initialize IMU

  Wire.begin();
  Wire.setClock(400000);
  imu_ready = false;

  //imu_setup();

  // End: Initialize IMU
}

void loop() {
  // put your main code here, to run repeatedly:
  // uint32_t last_millis = millis();
  nh.spinOnce();

  if(millis() - last_mill > 100){
    // send_motor();
    
    last_mill = millis();
  }

  if(millis() - last_mill2 > 10){
    motor_kanan.auto_speed();
    motor_kiri.auto_speed();
    send_motor();
    
    last_mill2 = millis();
  }

  // if(!imu_ready){
  //   if(millis() - last_mill3 > 5000){
  //     if(imu_setup()) nh.loginfo("IMU Setup success!");
  //     else nh.loginfo("Retrying imu setup....");
  //     last_mill3 = millis();
  //     nh.spinOnce();
  //   }
  // }
  // else{
  //   if(millis() - last_mill3 > 10){
  //     imu_update();
  //     last_mill3 = millis();
  //   }
  // }
  
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

void send_motor(){
  motor_msg.header.stamp = nh.now();
  motor_msg.leftSpeed = motor_kiri.get_rpm();
  motor_msg.rightSpeed = motor_kanan.get_rpm();
  motor_data_array[0] = motor_kanan.get_pid_input();
  motor_data_array[1] = motor_kanan.get_pid_output();
  motor_data_array[2] = motor_kanan.get_pwm();
  motor_data_array[3] = motor_kiri.get_pid_input();
  motor_data_array[4] = motor_kiri.get_pid_output();
  motor_data_array[5] = motor_kiri.get_pwm();
  motor_msg.data_length = 6;
  motor_msg.data = motor_data_array;
  motor_pub.publish(&motor_msg);
  nh.spinOnce();
}

void cmdVelCb(const geometry_msgs::Twist& data){
  // rpm = 60/(2pi x r) x v
  // rpm = 60/(2pi x 0.08) x v
  // rpm = 119.366207319

  // differential drive:
  // 
  left_speed = data.linear.x - data.angular.z * (wheelbase / 2);  // satuan m/s
  left_speed = left_speed * 119.366207319;                        // satuan rpm
  motor_kiri.change_sp(left_speed);
  right_speed = data.linear.x + data.angular.z * (wheelbase / 2); // satuan m/s
  right_speed = right_speed * 119.366207319;                      // satuan rpm
  motor_kanan.change_sp(right_speed);
  
  // Start - Debug
  
  // String val(left_speed);
  // String res("Kiri = " + val);
  // nh.loginfo(val.c_str());
  // String val_r(right_speed);
  // String res2("Kanan = "+ val_r);
  // nh.loginfo(res2.c_str());

  // End - Debug
}

// Start: IMU Functions

bool imu_setup(){
  long store_millis;
  store_millis = millis();

  String id(imu.getDeviceID());
  id += " ID IMU";
  nh.loginfo(id.c_str());
  nh.spinOnce();

  if (imu.getDeviceID() != 52)
  {
    nh.spinOnce();
    return false;
  }
  imu.initialize();
  imu_dev_status = imu.dmpInitialize();
  
  String imuStat(imu_dev_status);
  imuStat += " imu DMP status";
  nh.loginfo(imuStat.c_str());
  nh.spinOnce();
  
  if(imu_dev_status != 0) return false;
  else{
    set_imu_offset();
    imu.CalibrateAccel(6);
    nh.loginfo("Pass accel calibration");
    imu.CalibrateGyro(6);
    nh.loginfo("Pass gyro calibration");
    imu.setDMPEnabled(true);
    nh.loginfo("Pass enable DMP");
    imu_ready = true;
    return true;
  }
}

void set_imu_offset(){
  imu.setXGyroOffset(imu_offsets[0]);
  imu.setYGyroOffset(imu_offsets[1]);
  imu.setZGyroOffset(imu_offsets[2]);
  imu.setXAccelOffset(imu_offsets[3]);
  imu.setYAccelOffset(imu_offsets[4]);
  imu.setZAccelOffset(imu_offsets[5]);
}

void imu_update(){
  if(imu.dmpGetCurrentFIFOPacket(imu_fifo_buffer) == 1){
    imu.dmpGetQuaternion(&imuQ, imu_fifo_buffer);
    imu.dmpGetGravity(&gravity, &imuQ);
    imu.dmpGetGyro(&gyro, imu_fifo_buffer);
    imu.dmpGetAccel(&aa, imu_fifo_buffer);
    imu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    imu_ros_msg.header.stamp = nh.now();
    imu_ros_msg.orientation.w = imuQ.w;
    imu_ros_msg.orientation.x = imuQ.x;
    imu_ros_msg.orientation.y = imuQ.y;
    imu_ros_msg.orientation.z = imuQ.z;
    imu_ros_msg.angular_velocity.x = gyro.x * gyro_scale;
    imu_ros_msg.angular_velocity.y = gyro.y * gyro_scale;
    imu_ros_msg.angular_velocity.z = gyro.z * gyro_scale;
    imu_ros_msg.linear_acceleration.x = aaReal.x * acc_scale;
    imu_ros_msg.linear_acceleration.y = aaReal.y * acc_scale;
    imu_ros_msg.linear_acceleration.z = aaReal.z * acc_scale;
    imu_ros_msg.linear_acceleration_covariance[0] = acc_stddev;
    imu_ros_msg.linear_acceleration_covariance[4] = acc_stddev;
    imu_ros_msg.linear_acceleration_covariance[8] = acc_stddev;
    imu_ros_msg.angular_velocity_covariance[0] = gyro_stddev;
    imu_ros_msg.angular_velocity_covariance[4] = gyro_stddev;
    imu_ros_msg.angular_velocity_covariance[8] = gyro_stddev;
    imu_ros_msg.orientation_covariance[0] = orientation_stddev;
    imu_ros_msg.orientation_covariance[4] = orientation_stddev;
    imu_ros_msg.orientation_covariance[8] = orientation_stddev;
    imu_ros_pub.publish(&imu_ros_msg);
    nh.spinOnce();
  }
  else{
    nh.loginfo("No imu fifo buffer...");
    nh.spinOnce();
  }
}

// End: IMU Functions