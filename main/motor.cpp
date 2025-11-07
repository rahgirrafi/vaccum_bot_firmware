
#include "vaccum_firmware.hpp"

// ========= Helper =========
float angle_wrap_delta(float new_deg, float old_deg){
    float d = new_deg - old_deg;
    if (d > 180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

// ========= PWM / GPIO init =========
void motors_init(void)
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

void set_motor_pwm(uint8_t ledc_channel, int gpio_dir_a, int gpio_dir_b, float pwm_frac)
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



// Drive control task
void drive_control_task(void *arg)
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
void arm_control_task(void *arg)
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
