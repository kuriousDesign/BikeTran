#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <Arduino.h>


unsigned long lastUpdateTime_us = micros();

int32_t lastScanTime_us[50];
int32_t idx = 0;

hw_timer_t *My_timer = NULL;
void IRAM_ATTR onTimer(){
    unsigned long timeNow_us = micros();
    lastScanTime_us[idx] = int32_t(timeNow_us - lastUpdateTime_us);
    lastUpdateTime_us = timeNow_us;
    idx++;
    if(idx>=50){
      idx = 0;
    }
}

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_timer_t timer_slow;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}




void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  unsigned long timeNow_us = micros();

  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    lastScanTime_us[idx] = int32_t(timeNow_us - lastUpdateTime_us);
    //RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    lastUpdateTime_us = timeNow_us;
    idx++;
    if(idx>=50){
      idx = 0;
    }
  }
}

void timer_slow_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    int32_t sum = 0;
    for (uint8_t i = 0; i < 50; i++)
    {
      if( lastScanTime_us[i] > sum){
        sum = lastScanTime_us[i];
      }
    }
    msg.data = sum;
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
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
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_node_publisher"));

  // create timer,
  const unsigned int timer_timeout_us = 1000; //1000us
  /*
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_US_TO_NS(timer_timeout_us),
    timer_callback));
*/

  RCCHECK(rclc_timer_init_default(
    &timer_slow,
    &support,
    RCL_MS_TO_NS(50),
    timer_slow_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  //RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_slow));

  My_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(My_timer, &onTimer, true);
    timerAlarmWrite(My_timer, 1000, true);
    timerAlarmEnable(My_timer);

  msg.data = 0;
}

void loop() {
  //delayMicroseconds(10);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, 10));
}
