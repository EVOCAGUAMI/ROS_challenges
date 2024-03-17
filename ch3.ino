#include <micro_ros_arduino.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/Float32.h>

#include <driver/adc.h>
#include <driver/ledc.h>

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define POT_PIN 36
rcl_timer_t timer_1;

rcl_publisher_t pot_publisher;
std_msgs_msg_Int32 pot_msg;
rcl_publisher_t voltage_publisher;
std_msgs_msg_Float32 voltage_msg;
rcl_timer_t timer_2;
#define ADC_RESOLUTION 12
#define ADC_MAX_VALUE (1 << ADC_RESOLUTION)
#define VOLTAGE_MAX 3.3

#define PWM_PIN 15
rcl_subscription_t pwm_subscriber;
std_msgs_msg_Float32  pwm_msg;

#define PWM_CHANNEL LEDC_CHANNEL_0
#define PWM_RESOLUTION 8
#define PWM_FREQUENCY 5000

rclc_executor_t executor;
rclc_executor_t executor_sub;

#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_1_callback(rcl_timer_t * timer_1, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  uint32_t adc_value = adc1_get_raw(ADC1_CHANNEL_0);
  pot_msg.data = adc_value;

}

void timer_2_callback(rcl_timer_t * timer_2, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  float voltage = static_cast<float>(pot_msg.data) * VOLTAGE_MAX / ADC_MAX_VALUE;
  voltage_msg.data = voltage;

  RCSOFTCHECK(rcl_publish(&pot_publisher, &pot_msg, NULL));
  RCSOFTCHECK(rcl_publish(&voltage_publisher, &voltage_msg, NULL));
}

void subscription_callback(const void * msgin) {
  const std_msgs_msgFloat32 * msg = (const std_msgsmsg_Float32 *)msgin;
  float duty_cycle = msg->data;

  uint32_t pwm_value = static_cast<uint32_t>(duty_cycle * (1 << PWM_RESOLUTION) / 100);

  ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL, pwm_value);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL);

  analogWrite(PWM_PIN, pwm_value);

}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &pot_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_esp32/raw_pot"));

  RCCHECK(rclc_publisher_init_default(
      &voltage_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "micro_ros_esp32/voltage"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &pwm_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "micro_ros_esp32/pwm_duty_cycle"));

  // create timer,
  const unsigned int timer_1_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &timer_1,
    &support,
    RCL_MS_TO_NS(timer_1_timeout),
    timer_1_callback));

  const unsigned int timer_2_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer_2,
    &support,
    RCL_MS_TO_NS(timer_2_timeout),
    timer_2_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_1));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_2));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &pwm_subscriber, &pwm_msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}
