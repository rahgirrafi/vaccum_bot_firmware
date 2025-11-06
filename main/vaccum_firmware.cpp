/*
Task:
    Read as5600 sensor:
        - Initialize I2C
        - Read angle data from AS5600
        - Read rpm data from AS5600
        - publish data to micro-ROS topic
    
    Control arm_motor:
        - Motor Driver: L298N
        - Control motor speed and direction based on error from joint_trajectory_controller type controller's  arm_controller/state topic
        - Subscribe to micro-ROS topic for arm_controller/state messages

    Control drive motors:
        - Motor Driver: L298N
        - Motor: N20 DC Motor with Encoder
        - Control motor speed and direction based on Twist messages from micro-ROS topic
        - Subscribe to micro-ROS topic for vaccume_base_controller/cmd_vel messages 
        - Using diif_drive_controller type controller

    
        as5600.hpp is a espp library. Details at: https://components.espressif.com/components/espp/as5600/versions/1.0.30/readme?language=en

        ros2_control param file:
        controller_manager:
  ros__parameters:
    update_rate: 100 # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    vaccum_base_controller:
      type: diff_drive_controller/DiffDriveController

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

      

vaccum_base_controller:
  ros__parameters:
    type: diff_drive_controller/DiffDriveController

    left_wheel_names: ["front_left_wheel_joint", "rear_left_wheel_joint"]
    right_wheel_names: ["front_right_wheel_joint", "rear_right_wheel_joint"]

    wheel_separation: 0.10
    wheels_per_side: 2
    wheel_radius: 0.022

    wheel_separation_multiplier: 1.0
    left_wheel_radius_multiplier: 1.0
    right_wheel_radius_multiplier: 1.0

    publish_rate: 50.0
    odom_frame_id: odom
    base_frame_id: base_link
    pose_covariance_diagonal : [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]

    open_loop: true
    enable_odom_tf: true

    cmd_vel_timeout: 0.5
    # publish_limited_velocity: true
    velocity_rolling_window_size: 10

    # Velocity and acceleration limits
    # Whenever a min_* is unspecified, default to -max_*
    linear.x.has_velocity_limits: false
    linear.x.has_acceleration_limits: false
    linear.x.has_jerk_limits: false
    linear.x.max_velocity: 1.0
    linear.x.min_velocity: -1.0
    linear.x.max_acceleration: 1.0
    linear.x.max_jerk: 0.0
    linear.x.min_jerk: 0.0

    angular.z.has_velocity_limits: false
    angular.z.has_acceleration_limits: false
    angular.z.has_jerk_limits: false
    angular.z.max_velocity: 1.0
    angular.z.min_velocity: -1.0
    angular.z.max_acceleration: 1.0
    angular.z.min_acceleration: -1.0
    angular.z.max_jerk: 0.0
    angular.z.min_jerk: 0.0
    use_stamped_vel: false

arm_controller:
  ros__parameters:
    type: diff_drive_controller/DiffDriveController

    left_wheel_names: ["left_middle_arm_to_base_joint", "right_middle_arm_to_base_joint"]
    right_wheel_names: ["left_arm_joint", "right_arm_joint"]

    wheel_separation: 0.05
    wheels_per_side: 2
    wheel_radius: 0.015

    wheel_separation_multiplier: 1.0
    left_wheel_radius_multiplier: 1.0
    right_wheel_radius_multiplier: 1.0

    publish_rate: 50.0
    odom_frame_id: arm_odom
    base_frame_id: arm_base_link
    pose_covariance_diagonal : [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]

    open_loop: true
    enable_odom_tf: true

    cmd_vel_timeout: 0.5
    # publish_limited_velocity: true
    velocity_rolling_window_size: 10

    # Velocity and acceleration limits
    # Whenever a min_* is unspecified, default to -max_*
    linear.x.has_velocity_limits: false
    linear.x.has_acceleration_limits: false
    linear.x.has_jerk_limits: false
    linear.x.max_velocity: 1.0
    linear.x.min_velocity: -1.0
    linear.x.max_acceleration: 1.0
    linear.x.max_jerk: 0.0
    linear.x.min_jerk: 0.0

    angular.z.has_velocity_limits: false
    angular.z.has_acceleration_limits: false
    angular.z.has_jerk_limits: false
    angular.z.max_velocity: 1.0
    angular.z.min_velocity: -1.0
    angular.z.max_acceleration: 1.0
    angular.z.min_acceleration: -1.0
    angular.z.max_jerk: 0.0
    angular.z.min_jerk: 0.0
    use_stamped_vel: false

arm_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController

    joints:
      - left_arm_joint
      - left_middle_arm_to_base_joint
      - right_arm_joint
      - right_middle_arm_to_base_joint

    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

    state_publish_rate: 50.0
    action_monitor_rate: 20.0

    constraints:
      goal_time: 0.6
      stopped_velocity_tolerance: 0.02
      left_arm_joint:
        trajectory: 0.5
        goal: 0.1
      left_middle_arm_to_base_joint:
        trajectory: 0.5
        goal: 0.1
      right_arm_joint:
        trajectory: 0.5
        goal: 0.1
      right_middle_arm_to_base_joint:
        trajectory: 0.5
        goal: 0.1


*/
/*
ESP32 micro-ROS + AS5600 + L298N + N20 encoder example

Features implemented (best-effort):
 - I2C init (ESP-IDF)
 - AS5600 read (using as5600.hpp from espp)
 - Angle (degrees) and RPM estimate published to micro-ROS topics:
     /arm/angle  (std_msgs/msg/Float32)
     /arm/rpm    (std_msgs/msg/Float32)
 - Subscribes to /arm_controller/state (expects std_msgs/msg/Float32MultiArray)
     - Uses first element as desired joint position (radians or degrees depending on your convention)
     - Computes error = desired - measured and runs a simple PID to control the arm motor via L298N
 - Subscribes to /vaccum_base_controller/cmd_vel (geometry_msgs/msg/Twist)
     - Converts linear.x and angular.z into left and right wheel target velocities for differential drive
     - Controls two DC drive motors via L298N using LEDC PWM + direction pins
 - Encoder ISR counters for wheel encoders (N20). Velocity estimation provided by a periodic task.

NOTES and ADAPTATION:
 - This is a reference implementation. You will very likely need to adapt pins, message types, and micro-ROS init to your project.
 - I used std_msgs/msg/Float32 and std_msgs/msg/Float32MultiArray to keep dependencies small. If you use control_msgs/msg/JointTrajectoryControllerState, change the subscriber type and callback accordingly.
 - Adjust PID gains, wheel parameters, timers, and LEDC channel configs for your hardware.
 - Make sure you have micro-ROS and rclc installed for ESP-IDF and that your project's component CMakeLists.txt links against the necessary libraries.

PIN ASSIGNMENTS (example) - change to your wiring:
 - I2C SDA: GPIO21
 - I2C SCL: GPIO22
 - ARM L298N IN1 (dir A): GPIO16
 - ARM L298N IN2 (dir B): GPIO17
 - ARM L298N ENA (PWM): LEDC channel 0 -> GPIO18
 - LEFT DRIVE IN1: GPIO25
 - LEFT DRIVE IN2: GPIO26
 - LEFT DRIVE ENA PWM: LEDC channel 1 -> GPIO27
 - RIGHT DRIVE IN1: GPIO32
 - RIGHT DRIVE IN2: GPIO33
 - RIGHT DRIVE ENA PWM: LEDC channel 2 -> GPIO14
 - ENCODER LEFT A: GPIO34 (input)
 - ENCODER LEFT B: GPIO35 (input)
 - ENCODER RIGHT A: GPIO36 (input)
 - ENCODER RIGHT B: GPIO39 (input)

Compile-time configuration and further integration notes are inside comments in the file.
*/


#endif
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#include "sdkconfig.h"

// as5600.hpp from espp
#include "as5600.hpp"

// micro-ROS / rclc includes
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

static const char *TAG = "arm_drive_node";

// ========= User configuration (adjust to your board) =========
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000

// AS5600 I2C address default in espp library will be used

// LEDC PWM channels & pins for motors
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

// Encoder pins (example) - using GPIO inputs with interrupts
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
static as5600::AS5600 as5600_sensor; // object from espp as5600.hpp

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

// Arm controller target position (units same as AS5600 output - degrees by default)
static float arm_target_pos = 0.0f;
static SemaphoreHandle_t arm_target_mutex;

// Encoder counters
volatile int64_t enc_left_count = 0;
volatile int64_t enc_right_count = 0;
SemaphoreHandle_t enc_mutex;

// Helper for wrap-aware angle delta
static float angle_wrap_delta(float new_deg, float old_deg){
    float d = new_deg - old_deg;
    if (d > 180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

// ========= I2C init =========
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf{};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// ========= PWM / GPIO init =========
static void motors_init(void)
{
    // Configure direction GPIOs
    gpio_config_t io_conf{};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<ARM_DIR_A_GPIO) | (1ULL<<ARM_DIR_B_GPIO)
                          | (1ULL<<LEFT_DIR_A_GPIO) | (1ULL<<LEFT_DIR_B_GPIO)
                          | (1ULL<<RIGHT_DIR_A_GPIO) | (1ULL<<RIGHT_DIR_B_GPIO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

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

// Write direction + pwm convenience
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
        portENTER_CRITICAL_ISR(&((portMUX_TYPE){0}));
        enc_left_count += delta;
        portEXIT_CRITICAL_ISR(&((portMUX_TYPE){0}));
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
        portENTER_CRITICAL_ISR(&((portMUX_TYPE){0}));
        enc_right_count += delta;
        portEXIT_CRITICAL_ISR(&((portMUX_TYPE){0}));
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

// cmd_vel callback
static void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *tw = (const geometry_msgs__msg__Twist *)msgin;
    float linear = (float)tw->linear.x; // m/s
    float angular = (float)tw->angular.z; // rad/s

    // Convert to left/right wheel speeds (approx)
    float v_left = linear - (angular * WHEEL_SEPARATION * 0.5f);
    float v_right = linear + (angular * WHEEL_SEPARATION * 0.5f);

    if (xSemaphoreTake(vel_mutex, (TickType_t)10) == pdTRUE) {
        target_left_vel = v_left;
        target_right_vel = v_right;
        xSemaphoreGive(vel_mutex);
    }
}

// arm_state callback (we assume Float32MultiArray: first element = desired pos)
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

// Sensor task: read AS5600, compute rpm, publish
static void sensor_task(void *arg)
{
    (void)arg;
    float last_angle_deg = 0.0f;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        // Read angle from AS5600
        float angle_deg = 0.0f;
        {
            xSemaphoreTake(i2c_mutex, portMAX_DELAY);
            // as5600::AS5600 read example from espp: get_angle() returns degrees 0..360
            angle_deg = as5600_sensor.get_angle();
            xSemaphoreGive(i2c_mutex);
        }

        // Compute delta angle and rpm (approx)
        float d = angle_wrap_delta(angle_deg, last_angle_deg);
        last_angle_deg = angle_deg;

        // d degrees per period (SENSOR_PUBLISH_PERIOD_MS)
        float period_s = (float)SENSOR_PUBLISH_PERIOD_MS / 1000.0f;
        float rpm = 0.0f;
        // degrees per second = d / period_s
        float deg_per_sec = d / period_s;
        float rev_per_sec = deg_per_sec / 360.0f;
        rpm = rev_per_sec * 60.0f;

        // Publish angle and rpm
        angle_msg.data = angle_deg;
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

// Drive control task: convert target velocities to pwm and set motor outputs
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

        // Normalize to pwm fraction - simple proportional mapping
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

// Arm control task: simple PID using AS5600 measured angle
static void arm_control_task(void *arg)
{
    (void)arg;
    float last_angle = 0.0f;
    float integral = 0.0f;
    float last_error = 0.0f;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        // read measured angle
        float meas = 0.0f;
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        meas = as5600_sensor.get_angle();
        xSemaphoreGive(i2c_mutex);

        float desired = 0.0f;
        if (xSemaphoreTake(arm_target_mutex, (TickType_t)10) == pdTRUE) {
            desired = arm_target_pos;
            xSemaphoreGive(arm_target_mutex);
        }

        float error = angle_wrap_delta(desired, meas); // desired - meas with wrap handling
        integral += error * (CONTROL_PERIOD_MS / 1000.0f);
        float derivative = (error - last_error) / (CONTROL_PERIOD_MS / 1000.0f);
        float out = ARM_KP * error + ARM_KI * integral + ARM_KD * derivative;
        last_error = error;

        // Map out to pwm fraction
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
        int64_t left, right;
        left = enc_left_count;
        right = enc_right_count;

        int64_t dl = left - last_left;
        int64_t dr = right - last_right;
        last_left = left;
        last_right = right;

        float dt = (float)ENCODER_SAMPLE_MS / 1000.0f;
        // ticks to meters: you must know ticks per revolution
        const float TICKS_PER_REV = 48.0f; // placeholder, set to your encoder CPR
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

    // NOTE: user must set up rmw configuration (transport) separately according to micro-ROS esp-idf guide

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

    // We subscribe to arm_controller/state as Float32MultiArray for simplicity
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
    ESP_LOGI(TAG, "Starting esp32 ARM+DRIVE node");

    // init mutexes
    i2c_mutex = xSemaphoreCreateMutex();
    vel_mutex = xSemaphoreCreateMutex();
    arm_target_mutex = xSemaphoreCreateMutex();
    enc_mutex = xSemaphoreCreateMutex();

    // init i2c
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %d", err);
    }

    // init as5600 object (the constructor in espp may require i2c number and address)
    // Example assuming as5600::AS5600::AS5600(i2c_num)
    as5600_sensor = as5600::AS5600(I2C_MASTER_NUM);

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
