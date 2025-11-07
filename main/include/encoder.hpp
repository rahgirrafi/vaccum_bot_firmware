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
#define ENC_LEFT_A_GPIO 34
#define ENC_LEFT_B_GPIO 35
#define ENC_RIGHT_A_GPIO 36
#define ENC_RIGHT_B_GPIO 39

// Encoder counters
extern volatile int64_t enc_left_count;
extern volatile int64_t enc_right_count;
extern SemaphoreHandle_t enc_mutex;



// ============ Global state ============

extern espp::As5600 *g_as5600;
extern espp::I2c *g_i2c; // must outlive g_as5600

extern SemaphoreHandle_t i2c_mutex;

void IRAM_ATTR enc_left_isr_handler(void *arg);
void IRAM_ATTR enc_right_isr_handler(void *arg);
void encoders_init(void);
void encoder_sample_task(void *arg);
void sensor_task(void *arg);

