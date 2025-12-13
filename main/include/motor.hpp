#pragma once
// LEDC PWM channels & pins for motors (example)
#define LEFT_ARM_PWM_GPIO 26
#define LEFT_ARM_LEDC_CHANNEL LEDC_CHANNEL_0

#define RIGHT_ARM_PWM_GPIO 27
#define RIGHT_ARM_LEDC_CHANNEL LEDC_CHANNEL_1

#define BASE_LEFT_PWM_GPIO_1 32
#define BASE_LEFT_LEDC_CHANNEL_1 LEDC_CHANNEL_2


#define BASE_RIGHT_PWM_GPIO_1 23
#define BASE_RIGHT_LEDC_CHANNEL_1 LEDC_CHANNEL_3

#define BASE_LEFT_PWM_GPIO_2 33
#define BASE_LEFT_LEDC_CHANNEL_2 LEDC_CHANNEL_4

#define BASE_RIGHT_PWM_GPIO_2 25
#define BASE_RIGHT_LEDC_CHANNEL_2 LEDC_CHANNEL_5

// PID defaults for arm motor (tune these)
#define ARM_KP 1.2f
#define ARM_KI 0.0f
#define ARM_KD 0.01f
#define ARM_MAX_PWM 255

#define DRIVE_KP 1.2f
#define DRIVE_KI 0.0f
#define DRIVE_KD 0.01f
#define DRIVE_MAX_PWM 255
// Drive target velocities (m/s)
extern float target_left_vel;
extern float target_right_vel;
extern SemaphoreHandle_t vel_mutex;

// Arm controller target position (degrees)
extern SemaphoreHandle_t arm_state_mutex;
extern float left_arm_joint_pos_error;
extern float left_middle_joint_pos_error;
extern float right_arm_joint_pos_error;
extern float right_middle_joint_pos_error;


void motors_init();
void set_motor_pwm(uint8_t ledc_channel, float pwm_frac);
void drive_control_task(void *arg);
void arm_control_task(void *arg);
float angle_wrap_delta(float new_deg, float old_deg);

