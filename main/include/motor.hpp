#pragma once
// LEDC PWM channels & pins for motors (example)
#define ARM_PWM_GPIO 18
#define ARM_DIR_A_GPIO 16
#define ARM_DIR_B_GPIO 17
#define ARM_LEDC_CHANNEL LEDC_CHANNEL_0

#define LEFT_PWM_GPIO 27
#define LEFT_DIR_A_GPIO 25
#define LEFT_DIR_B_GPIO 26
#define LEFT_LEDC_CHANNEL LEDC_CHANNEL_1

#define RIGHT_PWM_GPIO 14
#define RIGHT_DIR_A_GPIO 32
#define RIGHT_DIR_B_GPIO 33
#define RIGHT_LEDC_CHANNEL LEDC_CHANNEL_2


// Drive target velocities (m/s)
extern float target_left_vel;
extern float target_right_vel;
extern SemaphoreHandle_t vel_mutex;

// Arm controller target position (degrees)
extern float arm_target_pos;
extern SemaphoreHandle_t arm_target_mutex;


void motors_init(void);
void set_motor_pwm(uint8_t ledc_channel, int gpio_dir_a, int gpio_dir_b, float pwm_frac);
void drive_control_task(void *arg);
void arm_control_task(void *arg);
float angle_wrap_delta(float new_deg, float old_deg);

