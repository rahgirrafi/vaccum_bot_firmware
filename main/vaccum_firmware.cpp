/*
ESP32 micro-ROS + AS5600 + L298N + N20 encoder example

This file was updated to use the espp As5600 API properly (espp::I2c + espp::As5600)
as shown in your reference example. The As5600 instance runs its own internal
update task (as provided by the espp library) and exposes convenient getters
like get_degrees(), get_rpm(), get_count(), get_radians().

Adjust pins, encoder CPR, PID gains, and micro-ROS transport settings for your
hardware and project setup.
*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <chrono>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#include "sdkconfig.h"

// espp includes for AS5600
#include "as5600.hpp"
#include "i2c.hpp"
#include "butterworth_filter.hpp"
#include "task.hpp"

// micro-ROS / rclc includes
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

static const char *TAG = "arm_drive_node";

using namespace std::chrono_literals;

// ========= User configuration (adjust to your board) =========
#define I2C_MASTER_NUM I2C_NUM_1
#define I2C_MASTER_SDA_GPIO GPIO_NUM_21
#define I2C_MASTER_SCL_GPIO GPIO_NUM_22

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

// Encoder pins (example)
#define ENC_LEFT_A_GPIO 34
#define ENC_LEFT_B_GPIO 35
#define ENC_RIGHT_A_GPIO 36
#define ENC_RIGHT_B_GPIO 39

// Timings
#define SENSOR_PUBLISH_PERIOD_MS 50 // 20 Hz
#define CONTROL_PERIOD_MS 20 // 50 Hz
#define ENCODER_SAMPLE_MS 100

// Robot params
#define WHEEL_RADIUS 0.022f
#define WHEEL_SEPARATION 0.10f

// PID defaults for arm motor (tune these)
#define ARM_KP 1.2f
#define ARM_KI 0.0f
#define ARM_KD 0.01f
#define ARM_MAX_PWM 255

// ============ Global state ============
static espp::As5600 *g_as5600 = nullptr;
static espp::I2c *g_i2c = nullptr; // must outlive g_as5600

static SemaphoreHandle_t i2c_mutex;

// micro-ROS handles
rcl_publisher_t angle_pub;
rcl_publisher_t rpm_pub;
rcl_subscription_t arm_state_sub;
rcl_subscription_t cmd_vel_sub;

rclc_executor_t executor;

// message storage
std_msgs__msg__Float32 angle_msg;
std_msgs__msg__Float32 rpm_msg;

// Drive target velocities (m/s)
static float target_left_vel = 0.0f;
static float target_right_vel = 0.0f;
static SemaphoreHandle_t vel_mutex;

// Arm controller target position (degrees)
static float arm_target_pos = 0.0f;
static SemaphoreHandle_t arm_target_mutex;

// Encoder counters
volatile int64_t enc_left_count = 0;
volatile int64_t enc_right_count = 0;
SemaphoreHandle_t enc_mutex;

// ========= Helper =========
static float angle_wrap_delta(float new_deg, float old_deg){
    float d = new_deg - old_deg;
    if (d > 180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

// ========= PWM / GPIO init =========
static void motors_init(void)
{
    // Configure direction GPIOs
    gpio_set_direction((gpio_num_t)ARM_DIR_A_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)ARM_DIR_B_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)LEFT_DIR_A_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)LEFT_DIR_B_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)RIGHT_DIR_A_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)RIGHT_DIR_B_GPIO, GPIO_MODE_OUTPUT);

    // Configure LEDC PWM timers and channels
    ledc_timer_config_t ledc_timer{};
    ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_timer.timer_num = LEDC_TIMER_0;
    ledc_timer.duty_resolution = LEDC_TIMER_8_BIT; // 8-bit resolution
    ledc_timer.freq_hz = 20000; // 20 kHz
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    // Channel for arm
    ledc_channel_config_t ledc_channel{};
    ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel.channel = ARM_LEDC_CHANNEL;
    ledc_channel.timer_sel = LEDC_TIMER_0;
    ledc_channel.intr_type = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num = ARM_PWM_GPIO;
    ledc_channel.duty = 0;
    ledc_channel.hpoint = 0;
    ledc_channel_config(&ledc_channel);

    // Left
    ledc_channel.channel = LEFT_LEDC_CHANNEL;
    ledc_channel.gpio_num = LEFT_PWM_GPIO;
    ledc_channel_config(&ledc_channel);

    // Right
    ledc_channel.channel = RIGHT_LEDC_CHANNEL;
    ledc_channel.gpio_num = RIGHT_PWM_GPIO;
    ledc_channel_config(&ledc_channel);
}

static void set_motor_pwm(uint8_t ledc_channel, int gpio_dir_a, int gpio_dir_b, float pwm_frac)
{
    // pwm_frac in range -1..1
    if (pwm_frac > 1.0f) pwm_frac = 1.0f;
    if (pwm_frac < -1.0f) pwm_frac = -1.0f;

    int duty = (int)(fabsf(pwm_frac) * 255.0f);
    if (pwm_frac >= 0.0f) {
        gpio_set_level((gpio_num_t)gpio_dir_a, 1);
        gpio_set_level((gpio_num_t)gpio_dir_b, 0);
    } else {
        gpio_set_level((gpio_num_t)gpio_dir_a, 0);
        gpio_set_level((gpio_num_t)gpio_dir_b, 1);
    }
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)ledc_channel, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)ledc_channel);
}

// ========= Encoders (simple quadrature) =========
static inline int read_gpio_level(int gpio) { return gpio_get_level((gpio_num_t)gpio); }

static void IRAM_ATTR enc_left_isr_handler(void *arg)
{
    (void)arg;
    static int lastA = 0;
    int A = read_gpio_level(ENC_LEFT_A_GPIO);
    int B = read_gpio_level(ENC_LEFT_B_GPIO);
    int delta = 0;
    if (A != lastA) {
        delta = (A == B) ? 1 : -1;
        lastA = A;
    }
    if (delta != 0) {
        __atomic_add_fetch(&enc_left_count, delta, __ATOMIC_RELAXED);
    }
}

static void IRAM_ATTR enc_right_isr_handler(void *arg)
{
    (void)arg;
    static int lastA = 0;
    int A = read_gpio_level(ENC_RIGHT_A_GPIO);
    int B = read_gpio_level(ENC_RIGHT_B_GPIO);
    int delta = 0;
    if (A != lastA) {
        delta = (A == B) ? 1 : -1;
        lastA = A;
    }
    if (delta != 0) {
        __atomic_add_fetch(&enc_right_count, delta, __ATOMIC_RELAXED);
    }
}

static void encoders_init(void)
{
    gpio_config_t io_conf{};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<ENC_LEFT_A_GPIO) | (1ULL<<ENC_LEFT_B_GPIO)
                          | (1ULL<<ENC_RIGHT_A_GPIO) | (1ULL<<ENC_RIGHT_B_GPIO);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    // install ISR service
    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)ENC_LEFT_A_GPIO, enc_left_isr_handler, NULL);
    gpio_isr_handler_add((gpio_num_t)ENC_RIGHT_A_GPIO, enc_right_isr_handler, NULL);
}

// ========= micro-ROS callbacks =========
static void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *tw = (const geometry_msgs__msg__Twist *)msgin;
    float linear = (float)tw->linear.x; // m/s
    float angular = (float)tw->angular.z; // rad/s

    float v_left = linear - (angular * WHEEL_SEPARATION * 0.5f);
    float v_right = linear + (angular * WHEEL_SEPARATION * 0.5f);

    if (xSemaphoreTake(vel_mutex, (TickType_t)10) == pdTRUE) {
        target_left_vel = v_left;
        target_right_vel = v_right;
        xSemaphoreGive(vel_mutex);
    }
}

static void arm_state_callback(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *arr = (const std_msgs__msg__Float32MultiArray *)msgin;
    if (arr->data.size > 0) {
        float desired = arr->data.data[0];
        if (xSemaphoreTake(arm_target_mutex, (TickType_t)10) == pdTRUE) {
            arm_target_pos = desired;
            xSemaphoreGive(arm_target_mutex);
        }
    }
}

// ========= Tasks =========
// Sensor task: read As5600 using espp API, publish degrees and rpm
static void sensor_task(void *arg)
{
    (void)arg;
    float last_degrees = 0.0f;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        if (!g_as5600) {
            vTaskDelay(pdMS_TO_TICKS(SENSOR_PUBLISH_PERIOD_MS));
            continue;
        }

        // as5600 runs its own update; just read the cached values
        float degrees = g_as5600->get_degrees();
        float rpm = g_as5600->get_rpm();

        // Optionally compute delta yourself if needed
        float d = angle_wrap_delta(degrees, last_degrees);
        last_degrees = degrees;

        angle_msg.data = degrees;
        rpm_msg.data = rpm;

        rcl_ret_t ret;
        ret = rcl_publish(&angle_pub, &angle_msg, NULL);
        if (ret != RCL_RET_OK) {
            ESP_LOGW(TAG, "angle publish failed: %d", ret);
        }
        ret = rcl_publish(&rpm_pub, &rpm_msg, NULL);
        if (ret != RCL_RET_OK) {
            ESP_LOGW(TAG, "rpm publish failed: %d", ret);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_PUBLISH_PERIOD_MS));
    }
}

// Drive control task
static void drive_control_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const float max_linear_speed = 1.0f; // m/s (tune)

    while (1) {
        float left_v = 0.0f;
        float right_v = 0.0f;
        if (xSemaphoreTake(vel_mutex, (TickType_t)10) == pdTRUE) {
            left_v = target_left_vel;
            right_v = target_right_vel;
            xSemaphoreGive(vel_mutex);
        }

        float left_frac = left_v / max_linear_speed;
        float right_frac = right_v / max_linear_speed;
        if (left_frac > 1.0f) left_frac = 1.0f;
        if (left_frac < -1.0f) left_frac = -1.0f;
        if (right_frac > 1.0f) right_frac = 1.0f;
        if (right_frac < -1.0f) right_frac = -1.0f;

        set_motor_pwm(LEFT_LEDC_CHANNEL, LEFT_DIR_A_GPIO, LEFT_DIR_B_GPIO, left_frac);
        set_motor_pwm(RIGHT_LEDC_CHANNEL, RIGHT_DIR_A_GPIO, RIGHT_DIR_B_GPIO, right_frac);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

// Arm control task: simple PID using As5600 measured degrees
static void arm_control_task(void *arg)
{
    (void)arg;
    float integral = 0.0f;
    float last_error = 0.0f;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        if (!g_as5600) {
            vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
            continue;
        }

        float meas = g_as5600->get_degrees();

        float desired = 0.0f;
        if (xSemaphoreTake(arm_target_mutex, (TickType_t)10) == pdTRUE) {
            desired = arm_target_pos;
            xSemaphoreGive(arm_target_mutex);
        }

        float error = angle_wrap_delta(desired, meas);
        integral += error * (CONTROL_PERIOD_MS / 1000.0f);
        float derivative = (error - last_error) / (CONTROL_PERIOD_MS / 1000.0f);
        float out = ARM_KP * error + ARM_KI * integral + ARM_KD * derivative;
        last_error = error;

        float pwm_frac = out / (float)ARM_MAX_PWM;
        if (pwm_frac > 1.0f) pwm_frac = 1.0f;
        if (pwm_frac < -1.0f) pwm_frac = -1.0f;

        set_motor_pwm(ARM_LEDC_CHANNEL, ARM_DIR_A_GPIO, ARM_DIR_B_GPIO, pwm_frac);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

// Encoder sample & debug task
static void encoder_sample_task(void *arg)
{
    (void)arg;
    int64_t last_left = 0, last_right = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        int64_t left = __atomic_load_n(&enc_left_count, __ATOMIC_RELAXED);
        int64_t right = __atomic_load_n(&enc_right_count, __ATOMIC_RELAXED);

        int64_t dl = left - last_left;
        int64_t dr = right - last_right;
        last_left = left;
        last_right = right;

        float dt = (float)ENCODER_SAMPLE_MS / 1000.0f;
        const float TICKS_PER_REV = 48.0f; // placeholder, set correct value
        float left_rps = (dl / TICKS_PER_REV) / dt;
        float right_rps = (dr / TICKS_PER_REV) / dt;
        float left_mps = left_rps * (2.0f * M_PI * WHEEL_RADIUS);
        float right_mps = right_rps * (2.0f * M_PI * WHEEL_RADIUS);

        ESP_LOGI(TAG, "enc dl=%" PRId64 " dr=%" PRId64 " left_m/s=%.3f right_m/s=%.3f", dl, dr, left_mps, right_mps);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ENCODER_SAMPLE_MS));
    }
}

// ========= micro-ROS init and executor setup =========
static rcl_node_t node;

static void micro_ros_init_and_create_comm(void)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);

    rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "rclc_support_init failed: %d", rc);
    }

    rc = rclc_node_init_default(&node, "esp32_arm_drive_node", "", &support);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "node init failed: %d", rc);
    }

    // publishers
    rcl_publisher_options_t pub_ops = rcl_publisher_get_default_options();
    rc = rcl_publisher_init(&angle_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/arm/angle", &pub_ops);
    if (rc != RCL_RET_OK) ESP_LOGE(TAG, "angle pub init failed: %d", rc);
    rc = rcl_publisher_init(&rpm_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/arm/rpm", &pub_ops);
    if (rc != RCL_RET_OK) ESP_LOGE(TAG, "rpm pub init failed: %d", rc);

    // subscribers
    rcl_subscription_options_t sub_ops = rcl_subscription_get_default_options();
    rc = rcl_subscription_init(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/vaccum_base_controller/cmd_vel", &sub_ops);
    if (rc != RCL_RET_OK) ESP_LOGE(TAG, "cmd_vel sub init failed: %d", rc);

    rc = rcl_subscription_init(&arm_state_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/arm_controller/state", &sub_ops);
    if (rc != RCL_RET_OK) ESP_LOGE(TAG, "arm state sub init failed: %d", rc);

    // executor
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, (void*)&(geometry_msgs__msg__Twist){}, &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &arm_state_sub, (void*)&(std_msgs__msg__Float32MultiArray){}, &arm_state_callback, ON_NEW_DATA);

    // initialize messages
    angle_msg.data = 0.0f;
    rpm_msg.data = 0.0f;
}

static void micro_ros_spin_task(void *arg)
{
    (void)arg;
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ========= app_main =========
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting esp32 ARM+DRIVE node (As5600 updated)");

    // init mutexes
    i2c_mutex = xSemaphoreCreateMutex();
    vel_mutex = xSemaphoreCreateMutex();
    arm_target_mutex = xSemaphoreCreateMutex();
    enc_mutex = xSemaphoreCreateMutex();

    // initialize espp I2C (must match CONFIG_EXAMPLE_* macros in sdkconfig)
    g_i2c = new espp::I2c({
        .port = I2C_MASTER_NUM,
        .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_GPIO,
        .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_GPIO,
    });

    // make the velocity filter for As5600
    static constexpr float filter_cutoff_hz = 4.0f;
    static constexpr float encoder_update_period = 0.01f; // seconds
    static espp::ButterworthFilter<2, espp::BiquadFilterDf2> filter({
        .normalized_cutoff_frequency = 2.0f * filter_cutoff_hz * encoder_update_period});
    auto filter_fn = [&filter](float raw) -> float { return filter.update(raw); };

    // create the As5600 instance (it creates its own task internally)
    g_as5600 = new espp::As5600({
        .write_then_read = std::bind(&espp::I2c::write_read, g_i2c,
                                     std::placeholders::_1, std::placeholders::_2,
                                     std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
        .velocity_filter = filter_fn,
        .update_period = std::chrono::duration<float>(encoder_update_period),
        .log_level = espp::Logger::Verbosity::WARN});

    // init motors & pwm
    motors_init();

    // init encoders
    encoders_init();

    // micro-ROS init
    micro_ros_init_and_create_comm();

    // create tasks
    xTaskCreate(&sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(&drive_control_task, "drive_ctrl", 4096, NULL, 5, NULL);
    xTaskCreate(&arm_control_task, "arm_ctrl", 4096, NULL, 6, NULL);
    xTaskCreate(&encoder_sample_task, "enc_sample", 4096, NULL, 4, NULL);
    xTaskCreate(&micro_ros_spin_task, "micro_ros_spin", 8192, NULL, 3, NULL);

    ESP_LOGI(TAG, "All tasks launched");
}
