#pragma once
#include "vaccum_firmware.hpp"

#define MAX_SPEED 0.675 // m/s

float target_left_vel = 0.0f;
float target_right_vel = 0.0f;

float left_arm_joint_pos_error = 0.0f;
float left_middle_joint_pos_error = 0.0f;
float right_arm_joint_pos_error = 0.0f;
float right_middle_joint_pos_error = 0.0f;

// Timestamp for arm state timeout detection (in FreeRTOS ticks)
TickType_t last_arm_state_update_time = 0;

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
// Two AS5600 encoders on separate I2C buses
espp::As5600 *g_as5600_0 = nullptr; // First encoder on I2C_NUM_1
espp::As5600 *g_as5600_1 = nullptr; // Second encoder on I2C_NUM_0

rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

rcl_publisher_t encoder_counts_pub;

rcl_subscription_t arm_state_sub;
rcl_subscription_t cmd_vel_sub;

// message storage
geometry_msgs__msg__TwistStamped cmd_vel_msg;
custom_interfaces__msg__Float32FixedArray8 joint_state_array_msg;
custom_interfaces__msg__Float32FixedArray8 encoder_counts_angel_rpm_msgs;

// Mutex for protecting encoder message access
SemaphoreHandle_t encoder_msg_mutex = NULL;

// custom_interfaces__msg__Float32FixedArray as5600_msgs;
const char *TAG = "arm_drive_node";
