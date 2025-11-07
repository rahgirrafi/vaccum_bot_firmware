
#include "vaccum_firmware.hpp"

// ========= Helper =========
float angle_wrap_delta(float new_deg, float old_deg){
    float d = new_deg - old_deg;
    if (d > 180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

// ========= PWM / GPIO init =========
void motors_init(){
   
   
    // Configure LEDC PWM timers and channels
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT, // 13-bit resolution
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000, // 5 kHz
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // Channel for left arm
    ledc_channel_config_t ledc_channel = {
        .gpio_num = LEFT_ARM_PWM_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEFT_ARM_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ledc_channel);
    
    // Channel for right arm
    ledc_channel.channel = RIGHT_ARM_LEDC_CHANNEL;
    ledc_channel.gpio_num = RIGHT_ARM_PWM_GPIO;
    ledc_channel_config(&ledc_channel);


    // Left Base Body 1
    ledc_channel.channel = BASE_LEFT_LEDC_CHANNEL_1;
    ledc_channel.gpio_num = BASE_LEFT_PWM_GPIO_1;
    ledc_channel_config(&ledc_channel);

    // Right Base Body 1
    ledc_channel.channel = BASE_RIGHT_LEDC_CHANNEL_1;
    ledc_channel.gpio_num = BASE_RIGHT_PWM_GPIO_1;
    ledc_channel_config(&ledc_channel);

    // Left Base Body 2
    ledc_channel.channel = BASE_LEFT_LEDC_CHANNEL_2;
    ledc_channel.gpio_num = BASE_LEFT_PWM_GPIO_2;
    ledc_channel_config(&ledc_channel);

    // Right Base Body 2
    ledc_channel.channel = BASE_RIGHT_LEDC_CHANNEL_2;
    ledc_channel.gpio_num = BASE_RIGHT_PWM_GPIO_2;
    ledc_channel_config(&ledc_channel);
}

void set_motor_pwm(uint8_t ledc_channel, float pwm_frac)
{
    // pwm_frac in range -1..1
    if (pwm_frac > 1.0f) pwm_frac = 1.0f;
    if (pwm_frac < -1.0f) pwm_frac = -1.0f;
    //publish the raspberry pi direction control pin msg
    int duty = (int)(fabsf(pwm_frac) * 255.0f);

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
        //apply PID control to matchc the target velocities
        float left_frac = left_v / max_linear_speed;
        float right_frac = right_v / max_linear_speed;
        if (left_frac > 1.0f) left_frac = 1.0f;
        if (left_frac < -1.0f) left_frac = -1.0f;
        if (right_frac > 1.0f) right_frac = 1.0f;
        if (right_frac < -1.0f) right_frac = -1.0f;


        // float left_frac = left_v / max_linear_speed;
        // float right_frac = right_v / max_linear_speed;
        // if (left_frac > 1.0f) left_frac = 1.0f;
        // if (left_frac < -1.0f) left_frac = -1.0f;
        // if (right_frac > 1.0f) right_frac = 1.0f;
        // if (right_frac < -1.0f) right_frac = -1.0f;

        set_motor_pwm(left_channel,  left_frac);
        set_motor_pwm(right_channel,  right_frac);

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
        if (xSemaphoreTake(arm_state_mutex, (TickType_t)10) == pdTRUE) {
            desired = arm_target_pos;
            xSemaphoreGive(arm_state_mutex);
        }

        float error = angle_wrap_delta(desired, meas);
        integral += error * (CONTROL_PERIOD_MS / 1000.0f);
        float derivative = (error - last_error) / (CONTROL_PERIOD_MS / 1000.0f);
        float out = ARM_KP * error + ARM_KI * integral + ARM_KD * derivative;
        last_error = error;

        float pwm_frac = out / (float)ARM_MAX_PWM;
        if (pwm_frac > 1.0f) pwm_frac = 1.0f;
        if (pwm_frac < -1.0f) pwm_frac = -1.0f;

        set_motor_pwm(arm_channel, arm_dir_a, arm_dir_b, pwm_frac);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
