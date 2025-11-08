#pragma once
#include "vaccum_firmware.hpp"

#define MAX_SPEED 0.675 // m/s

float target_left_vel = 0.0f;
float target_right_vel = 0.0f;

float left_arm_joint_pos_error = 0.0f;
float left_middle_joint_pos_error = 0.0f;
float right_arm_joint_pos_error = 0.0f;
float right_middle_joint_pos_error = 0.0f;
SemaphoreHandle_t vel_mutex;
SemaphoreHandle_t arm_state_mutex;
// Encoder counters
volatile int64_t enc_left_1_count= 0;
volatile int64_t enc_right_1_count= 0;
volatile int64_t enc_left_2_count= 0;
volatile int64_t enc_right_2_count= 0;

float current_speed1 = 0;
float current_speed2 = 0;
float current_speed3 = 0;
float current_speed4 = 0;

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
rcl_publisher_t encoder_counts_pub;


// rcl_publisher_t left_wheel_1_dir_pub;
// rcl_publisher_t right_wheel_1_dir_pub;
// rcl_publisher_t left_wheel_2_dir_pub;
// rcl_publisher_t right_wheel_2_dir_pub;

// rcl_publisher_t left_arm_dir_pub;
// rcl_publisher_t right_arm_dir_pub;

rcl_subscription_t arm_state_sub;
rcl_subscription_t cmd_vel_sub;

// message storage
std_msgs__msg__Float32 angle_msg;
std_msgs__msg__Float32 rpm_msg;
geometry_msgs__msg__TwistStamped cmd_vel_msg;
control_msgs__msg__JointTrajectoryControllerState arm_state_msg;

std_msgs__msg__Bool left_wheel_1_dir_msg;
std_msgs__msg__Bool right_wheel_1_dir_msg;
std_msgs__msg__Bool left_wheel_2_dir_msg;
std_msgs__msg__Bool right_wheel_2_dir_msg;

custom_interfaces__msg__Float32FixedArray encoder_counts_msgs;
// custom_interfaces__msg__Float32FixedArray as5600_msgs;
const char *TAG = "arm_drive_node";
