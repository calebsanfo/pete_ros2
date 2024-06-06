#include <SPI.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// Constants for your robot
const float WHEEL_DIAMETER = 0.202; // Example: 20.2 cm
const float WHEEL_DISTANCE = 0.8; // Example: 80 cm
const float TICKS_PER_REV = 4000.0;

// Slave Select pins for encoders 1 and 2
const int slaveSelectEnc1 = 7;
const int slaveSelectEnc2 = 8;

// Encoder counts
signed long encoder1count = 0;
signed long encoder2count = 0;

// Variables for odometry calculation
float last_encoder1count = 0;
float last_encoder2count = 0;
float x = 0.0, y = 0.0, theta = 0.0;

// ROS 2 publisher setup
rcl_publisher_t publisher;
geometry_msgs__msg__Twist odom_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define TIMER_TIMEOUT 1 // 1000Hz

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    //digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void initEncoders() {
  pinMode(slaveSelectEnc1, OUTPUT);
  pinMode(slaveSelectEnc2, OUTPUT);
  digitalWrite(slaveSelectEnc1, HIGH);
  digitalWrite(slaveSelectEnc2, HIGH);
  SPI.begin();

  // Initialize encoder 1
  digitalWrite(slaveSelectEnc1, LOW);
  SPI.transfer(0x88);
  SPI.transfer(0x03);
  digitalWrite(slaveSelectEnc1, HIGH);

  // Initialize encoder 2
  digitalWrite(slaveSelectEnc2, LOW);
  SPI.transfer(0x88);
  SPI.transfer(0x03);
  digitalWrite(slaveSelectEnc2, HIGH);
}

long readEncoder(int encoder) {
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;

  if (encoder == 1) {
    digitalWrite(slaveSelectEnc1, LOW);
    SPI.transfer(0x60);
    count_1 = SPI.transfer(0x00);
    count_2 = SPI.transfer(0x00);
    count_3 = SPI.transfer(0x00);
    count_4 = SPI.transfer(0x00);
    digitalWrite(slaveSelectEnc1, HIGH);
  } else if (encoder == 2) {
    digitalWrite(slaveSelectEnc2, LOW);
    SPI.transfer(0x60);
    count_1 = SPI.transfer(0x00);
    count_2 = SPI.transfer(0x00);
    count_3 = SPI.transfer(0x00);
    count_4 = SPI.transfer(0x00);
    digitalWrite(slaveSelectEnc2, HIGH);
  }

  count_value = (count_1 << 8) + count_2;
  count_value = (count_value << 8) + count_3;
  count_value = (count_value << 8) + count_4;
  return count_value;
}

void clearEncoderCount() {
  // Clear encoder 1
  digitalWrite(slaveSelectEnc1, LOW);
  SPI.transfer(0x98);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  digitalWrite(slaveSelectEnc1, HIGH);

  delayMicroseconds(100);

  digitalWrite(slaveSelectEnc1, LOW);
  SPI.transfer(0xE0);
  digitalWrite(slaveSelectEnc1, HIGH);

  // Clear encoder 2
  digitalWrite(slaveSelectEnc2, LOW);
  SPI.transfer(0x98);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  digitalWrite(slaveSelectEnc2, HIGH);

  delayMicroseconds(100);

  digitalWrite(slaveSelectEnc2, LOW);
  SPI.transfer(0xE0);
  digitalWrite(slaveSelectEnc2, HIGH);
}

void calculateOdometry() {
  float delta_left = (encoder1count - last_encoder1count) * (PI * WHEEL_DIAMETER) / TICKS_PER_REV;
  float delta_right = (encoder2count - last_encoder2count) * (PI * WHEEL_DIAMETER) / TICKS_PER_REV;
  float delta_center = (delta_left + delta_right) / 2.0;
  float delta_theta = (delta_right - delta_left) / WHEEL_DISTANCE;

  x += delta_center * cos(theta);
  y += delta_center * sin(theta);
  theta += delta_theta;

  last_encoder1count = encoder1count;
  last_encoder2count = encoder2count;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    encoder1count = readEncoder(1);
    encoder2count = readEncoder(2);
    calculateOdometry();

    odom_msg.linear.x = x;
    odom_msg.linear.y = y;
    odom_msg.angular.z = theta;

    RCSOFTCHECK(rcl_publish(&publisher, &odom_msg, NULL));
  }
}

void setup() {
  initEncoders();
  clearEncoderCount();

  set_microros_transports();

  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "encoder_node", "", &support));
  RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "wheel_odometry"));

  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(TIMER_TIMEOUT), timer_callback));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  delay(1); // 1000Hz loop
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}

