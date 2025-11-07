#pragma once
#include "vaccum_firmware.hpp"

float target_left_vel = 0.0f;
float target_right_vel = 0.0f;

float left_arm_joint_pos_error = 0.0f;
float left_middle_joint_pos_error = 0.0f;
float right_arm_joint_pos_error = 0.0f;
float right_middle_joint_pos_error = 0.0f;
SemaphoreHandle_t vel_mutex;
float arm_target_pos = 0.0f;
SemaphoreHandle_t arm_state_mutex;
// Encoder counters
volatile int64_t enc_left_count= 0;
volatile int64_t enc_right_count= 0;
SemaphoreHandle_t enc_mutex;
espp::As5600 *g_as5600 = nullptr;
espp::I2c *g_i2c = nullptr; // must outlive g_as5600
SemaphoreHandle_t i2c_mutex;


rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t timer;


// micro-ROS publishers and subscribers
rcl_publisher_t angle_pub;
rcl_publisher_t rpm_pub;
rcl_subscription_t arm_state_sub;
rcl_subscription_t cmd_vel_sub;

// message storage
std_msgs__msg__Float32 angle_msg;
std_msgs__msg__Float32 rpm_msg;
geometry_msgs__msg__Twist cmd_vel_msg;
control_msgs__msg__JointTrajectoryControllerState arm_state_msg;

const char *TAG = "arm_drive_node";
