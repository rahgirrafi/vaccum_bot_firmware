
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


float last_error_drive_1 = 0.0f;
float last_error_drive_2 = 0.0f;
float last_error_drive_3 = 0.0f;
float last_error_drive_4 = 0.0f;
// Drive control task
void drive_control_task(void *arg)
{
    ESP_LOGI("DRIVE_CONTROL_TASK", "DRIVE CONTROL TASK  INITIATING");
    
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const float max_linear_speed = 1.0f; // m/s (tune)
    ESP_LOGI("DRIVE_CONTROL_TASK", "DRIVE CONTROL TASK  INITIATED");
    while (1) {
        ESP_LOGI("DRIVE_CONTROL_TASK", "DRIVE CONTROL SUPER LOOP BEGIN");
        float left_v = 0.0f;
        float right_v = 0.0f;
        if (xSemaphoreTake(vel_mutex, (TickType_t)10) == pdTRUE) {
            left_v = target_left_vel;
            right_v = target_right_vel;
            xSemaphoreGive(vel_mutex);
        }
        //current speed vs target speed
        float error1 = current_speed1 - target_left_vel;
        float error2 = current_speed2 - target_right_vel;
        float error3 = current_speed3 - target_left_vel;
        float error4 = current_speed4 - target_right_vel;

        float derivative1 = (error1 - last_error_drive_1) / (CONTROL_PERIOD_MS / 1000.0f);
        float derivative2 = (error1 - last_error_drive_1) / (CONTROL_PERIOD_MS / 1000.0f);
        float derivative3 = (error1 - last_error_drive_1) / (CONTROL_PERIOD_MS / 1000.0f);
        float derivative4 = (error1 - last_error_drive_1) / (CONTROL_PERIOD_MS / 1000.0f);

        float out1 = error1 * DRIVE_KP + derivative1 * DRIVE_KD;
        float out2 = error2 * DRIVE_KP + derivative2 * DRIVE_KD;
        float out3 = error3 * DRIVE_KP + derivative3 * DRIVE_KD;
        float out4 = error4 * DRIVE_KP + derivative4 * DRIVE_KD;

        last_error_drive_1 = error1;
        last_error_drive_2 = error2;
        last_error_drive_3 = error3;
        last_error_drive_4 = error4;

        float left_frac1 = out1 / (float)DRIVE_MAX_PWM;
        float right_frac1 = out2 / (float)DRIVE_MAX_PWM;
        float left_frac2 = out3 / (float)DRIVE_MAX_PWM;
        float right_frac2 = out4 / (float)DRIVE_MAX_PWM;

        if (left_frac1 > 1.0f) left_frac1 = 1.0f;
        if (left_frac1 < -1.0f) left_frac1 = -1.0f;
        if (right_frac1 > 1.0f) right_frac1 = 1.0f;
        if (right_frac1 < -1.0f) right_frac1 = -1.0f;
        if (left_frac2 > 1.0f) left_frac2 = 1.0f;
        if (left_frac2 < -1.0f) left_frac2 = -1.0f;
        if (right_frac2 > 1.0f) right_frac2 = 1.0f;
        if (right_frac2 < -1.0f) right_frac2 = -1.0f;

        set_motor_pwm(BASE_LEFT_LEDC_CHANNEL_1,  left_frac1);
        set_motor_pwm(BASE_RIGHT_LEDC_CHANNEL_1 , right_frac1);
        set_motor_pwm(BASE_LEFT_LEDC_CHANNEL_2,  left_frac2);
        set_motor_pwm(BASE_RIGHT_LEDC_CHANNEL_2,  right_frac2);
        ESP_LOGI("DRIVE_CONTROL_TASK", "DRIVE CONTROL SUPER LOOP END");

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
    float last_error_arm_1 = 0.0f;
    float last_error_arm_2 = 0.0f;
    float last_error_arm_3 = 0.0f;
    float last_error_arm_4 = 0.0f;
// Arm control task: simple PID using As5600 measured degrees
void arm_control_task(void *arg)
{

    ESP_LOGI("ARM_CONTROL_TASK", "ARM CONTROL TASK INITATED");
    motors_init();
    
    (void)arg;
    // float integral1 = 0.0f;
    // float integral2 = 0.0f;
    // float integral3 = 0.0f;
    // float integral4 = 0.0f;
    TickType_t last_wake = xTaskGetTickCount();
    ESP_LOGI("ARM_CONTROL_TASK", "ARM CONTROL TASK INITIATED");

    while (1) {
        ESP_LOGI("ARM_CONTROL_TASK", "ARM CONTROL SUPER LOOP BEGIN");

        float error1 = left_arm_joint_pos_error;
        float error2 =  right_arm_joint_pos_error;

        float derivative1 = (error1 - last_error_arm_1) / (CONTROL_PERIOD_MS / 1000.0f);
        float derivative2 = (error2 - last_error_arm_2) / (CONTROL_PERIOD_MS / 1000.0f);

        float out1 = ARM_KP * error1 + ARM_KD *  derivative1;
        float out2 = ARM_KP * error2 + ARM_KD *  derivative2;

        last_error_arm_1 = error1; 
        last_error_arm_2 = error2 ;
 
        float pwm_frac1 = out1 / (float)ARM_MAX_PWM;
        float pwm_frac2 = out2 / (float)ARM_MAX_PWM;

        if (pwm_frac1 > 1.0f) pwm_frac1 = 1.0f;
        if (pwm_frac1 < -1.0f) pwm_frac1 = -1.0f;
        if (pwm_frac2 > 1.0f) pwm_frac2 = 1.0f;
        if (pwm_frac2 < -1.0f) pwm_frac2 = -1.0f;

        set_motor_pwm(LEFT_ARM_LEDC_CHANNEL, pwm_frac1);
        set_motor_pwm(RIGHT_ARM_LEDC_CHANNEL, pwm_frac2);

        ESP_LOGI("ARM_CONTROL_TASK", "ARM CONTROL SUPER LOOP END");
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
