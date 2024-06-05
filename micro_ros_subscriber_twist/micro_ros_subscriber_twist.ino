#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <PWMServo.h>

#include <geometry_msgs/msg/twist.h>
#include <rcl_interfaces/msg/log.h>
#include <std_msgs/msg/int32.h>


rcl_subscription_t subscriber;
geometry_msgs__msg__Twist twist_msg;

rcl_publisher_t publisher;
rcl_publisher_t log_publisher;
std_msgs__msg__Int32 int32_msg;
rcl_interfaces__msg__Log log_msg;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

PWMServo right_motor;
PWMServo left_motor;


double distance_between_wheels = 3.0;
int right_motor_speed = 92;
int left_motor_speed = 92;
double linear_x_goal;
double angular_z_goal;

#define HWSERIAL Serial1
#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void log_to_rosout(const char* message) {
  log_msg.level = RCUTILS_LOG_SEVERITY_INFO;
  log_msg.name.data = const_cast<char*>("micro_ros_arduino_node");
  log_msg.name.size = strlen(log_msg.name.data);
  log_msg.name.capacity = log_msg.name.size + 1;

  log_msg.msg.data = const_cast<char*>(message);
  log_msg.msg.size = strlen(log_msg.msg.data);
  log_msg.msg.capacity = log_msg.msg.size + 1;

  log_msg.file.data = const_cast<char*>("");
  log_msg.file.size = strlen(log_msg.file.data);
  log_msg.file.capacity = log_msg.file.size + 1;

  log_msg.function.data = const_cast<char*>("");
  log_msg.function.size = strlen(log_msg.function.data);
  log_msg.function.capacity = log_msg.function.size + 1;

  log_msg.line = 0;

  RCSOFTCHECK(rcl_publish(&log_publisher, &log_msg, NULL));
}


//twist message cb
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // if velocity in x direction is 0 turn off LED, if 1 turn on LED
  digitalWrite(LED_PIN, (msg->linear.x == 0) ? LOW : HIGH);
  // HWSERIAL.print("Linear X: ");
  // HWSERIAL.print(msg->linear.x);
  // HWSERIAL.print(" Angular Z: ");
  // HWSERIAL.println(msg->angular.z);
  linear_x_goal = msg->linear.x;
  angular_z_goal = msg->angular.z;

  double right_wheel_speed = convert_to_wheel_speed_right(linear_x_goal, angular_z_goal);
  double left_wheel_speed = convert_to_wheel_speed_left(linear_x_goal, angular_z_goal);

  double right_pwm = convert_to_pwm(right_wheel_speed);
  double left_pwm = convert_to_pwm(left_wheel_speed);

  right_motor.write(right_pwm); // Assume right_motor is defined
  left_motor.write(left_pwm);   // Assume left_motor is defined

  char log_message[100];
  snprintf(log_message, sizeof(log_message), "Right PWM: %f, Left PWM: %f", right_pwm, left_pwm);
  log_to_rosout(log_message);
}

void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  HWSERIAL.begin(115200); // Configure debug serial
  
  delay(2000);

  allocator = rcl_get_default_allocator();

   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create log publisher
  RCCHECK(rclc_publisher_init_default(
    &log_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
    "/rosout"));


  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &subscription_callback, ON_NEW_DATA));

  right_motor.attach(23);
  left_motor.attach(22);
  right_motor.write(right_motor_speed);
  left_motor.write(left_motor_speed);

  int32_msg.data = 0;

  log_to_rosout("Node setup complete");

}

void loop() {
  delay(10);
  
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}



double convert_to_wheel_speed_right(double linear, double angular)
{
    return (2*linear - angular*distance_between_wheels)/2;
}

double convert_to_wheel_speed_left(double linear, double angular)
{
    return (2*linear + angular*distance_between_wheels)/2;
}

double convert_to_pwm(double m_per_s)
{
    return modifiedMap(m_per_s, -1.8, 1.8, 2, 182);
}

double modifiedMap(double x, double in_min, double in_max, double out_min, double out_max)
{
 double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
 temp = (int) (4*temp + .5);
 return (double) temp/4;
}
