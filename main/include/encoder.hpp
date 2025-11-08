#pragma once
// espp includes for AS5600
#include "as5600.hpp"
#include "i2c.hpp"
#include "butterworth_filter.hpp"
#include "task.hpp"

// ========= User configuration (adjust to your board) =========
#define I2C_MASTER_NUM I2C_NUM_1
#define I2C_MASTER_SDA_GPIO GPIO_NUM_21
#define I2C_MASTER_SCL_GPIO GPIO_NUM_22

// Encoder pins (example)
#define ENC_LEFT_1_A_GPIO 12
#define ENC_LEFT_1_B_GPIO 14
#define ENC_RIGHT_1_A_GPIO 13
#define ENC_RIGHT_1_B_GPIO 15
#define ENC_LEFT_2_A_GPIO 16
#define ENC_LEFT_2_B_GPIO 17
#define ENC_RIGHT_2_A_GPIO 18
#define ENC_RIGHT_2_B_GPIO 19

#define TICKS_PER_REV 48.0f // placeholder, set correct value
// Encoder counters
extern volatile int64_t enc_left_1_count;
extern volatile int64_t enc_right_1_count;
extern volatile int64_t enc_left_2_count;
extern volatile int64_t enc_right_2_count;
extern float current_speed1;
extern float current_speed2;
extern float current_speed3;
extern float current_speed4;
extern std_msgs__msg__Int64MultiArray encoder_counts_msgs;
extern std_msgs__msg__Float64MultiArray as5600_msgs;

extern SemaphoreHandle_t enc_mutex;

extern rcl_subscription_t as5600_sample_pub;

extern espp::As5600 *g_as5600;
extern espp::I2c *g_i2c; 

extern SemaphoreHandle_t i2c_mutex;

void IRAM_ATTR enc_left_1_isr_handler(void *arg);
void IRAM_ATTR enc_right_1_isr_handler(void *arg);
void IRAM_ATTR enc_left_2_isr_handler(void *arg);
void IRAM_ATTR enc_right_2_isr_handler(void *arg);

void encoders_init(void);
void encoder_sample_task(void *arg);
void sensor_task(void *arg);

