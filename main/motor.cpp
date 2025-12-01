
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
    ESP_LOGD("MOTOR", "Initializing LEDC PWM for motor control...");
   
    // Configure LEDC PWM timers and channels
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT, // 13-bit resolution
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000, // 5 kHz
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
        ESP_LOGE("MOTOR", "Failed to configure LEDC timer: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGD("MOTOR", "LEDC timer configured successfully");

    // Channel for left middle arm
    ledc_channel_config_t ledc_channel = {
        .gpio_num = LEFT_ARM_PWM_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEFT_ARM_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) {
        ESP_LOGE("MOTOR", "Failed to configure left arm channel: %s", esp_err_to_name(err));
        return;
    }
    
    // Channel for right middle arm
    ledc_channel.channel = RIGHT_ARM_LEDC_CHANNEL;
    ledc_channel.gpio_num = RIGHT_ARM_PWM_GPIO;
    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) {
        ESP_LOGE("MOTOR", "Failed to configure right arm channel: %s", esp_err_to_name(err));
        return;
    }

    // Left Base Body 1
    ledc_channel.channel = BASE_LEFT_LEDC_CHANNEL_1;
    ledc_channel.gpio_num = BASE_LEFT_PWM_GPIO_1;
    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) {
        ESP_LOGE("MOTOR", "Failed to configure left base 1 channel: %s", esp_err_to_name(err));
        return;
    }

    // Right Base Body 1
    ledc_channel.channel = BASE_RIGHT_LEDC_CHANNEL_1;
    ledc_channel.gpio_num = BASE_RIGHT_PWM_GPIO_1;
    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) {
        ESP_LOGE("MOTOR", "Failed to configure right base 1 channel: %s", esp_err_to_name(err));
        return;
    }

    // Left Base Body 2
    ledc_channel.channel = BASE_LEFT_LEDC_CHANNEL_2;
    ledc_channel.gpio_num = BASE_LEFT_PWM_GPIO_2;
    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) {
        ESP_LOGE("MOTOR", "Failed to configure left base 2 channel: %s", esp_err_to_name(err));
        return;
    }

    // Right Base Body 2
    ledc_channel.channel = BASE_RIGHT_LEDC_CHANNEL_2;
    ledc_channel.gpio_num = BASE_RIGHT_PWM_GPIO_2;
    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) {
        ESP_LOGE("MOTOR", "Failed to configure right base 2 channel: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGD("MOTOR", "All LEDC PWM channels configured successfully");
}

void set_motor_pwm(uint8_t ledc_channel, float pwm_frac)
{
    // pwm_frac in range -1..1
    if (pwm_frac > 1.0f) pwm_frac = 1.0f;
    if (pwm_frac < -1.0f) pwm_frac = -1.0f;
    
    // Calculate duty for 13-bit resolution (0-8191)
    // LEDC_TIMER_13_BIT = 2^13 - 1 = 8191
    int duty = (int)(fabsf(pwm_frac) * 8191.0f);

    esp_err_t err1 = ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)ledc_channel, duty);
    esp_err_t err2 = ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)ledc_channel);
    
    if (err1 != ESP_OK) {
        ESP_LOGE("MOTOR", "Failed to set duty for channel %d: %s", ledc_channel, esp_err_to_name(err1));
    }
    if (err2 != ESP_OK) {
        ESP_LOGE("MOTOR", "Failed to update duty for channel %d: %s", ledc_channel, esp_err_to_name(err2));
    }
}


float last_error_drive_1 = 0.0f;
float last_error_drive_2 = 0.0f;
float last_error_drive_3 = 0.0f;
float last_error_drive_4 = 0.0f;
    float last_error_arm_1 = 0.0f;
    float last_error_arm_2 = 0.0f;
    float last_error_arm_3 = 0.0f;
    float last_error_arm_4 = 0.0f;
// Arm control task: simple PID using As5600 measured degrees
// Drive control task
void drive_control_task(void *arg)
{
    ESP_LOGD("DRIVE_CONTROL_TASK", "DRIVE CONTROL TASK  INITIATING");
    
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const float max_linear_speed = 1.0f; // m/s (tune)

    static uint32_t log_counter = 0;

    

    while (1) {
        // if(log_counter % 40 == 0) {
        //     ESP_LOGI("DRIVE_CONTROL_TASK", "DRIVE CONTROL TASK  INITIATED");
        // }
        float left_v = 0.0f;
        float right_v = 0.0f;
        if (xSemaphoreTake(vel_mutex, (TickType_t)10) == pdTRUE) {
            left_v = target_left_vel;
            right_v = target_right_vel;
            xSemaphoreGive(vel_mutex);
        }
        //current speed vs target speed
        float error1 = (current_speed1 - target_left_vel);
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
       
        // ARM CONTROL using AS5600 sensor data
        // Check if arm state data is stale (timeout after 500ms)
        TickType_t current_time = xTaskGetTickCount();
        const TickType_t timeout_ticks = pdMS_TO_TICKS(500); // 500ms timeout
        bool arm_state_timeout = (last_arm_state_update_time == 0) || 
                                 ((current_time - last_arm_state_update_time) > timeout_ticks);
        
        if (arm_state_timeout) {
            // Stop arm motors if no updates received within timeout period
            set_motor_pwm(LEFT_ARM_LEDC_CHANNEL, 0.0f);
            set_motor_pwm(RIGHT_ARM_LEDC_CHANNEL, 0.0f);
            if (log_counter % 40 == 0) {
                ESP_LOGW("DRIVE_CONTROL_TASK", "Arm state timeout - motors stopped (no updates for >500ms)");
            }
            log_counter++;
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
            continue;
        }
        
        error1 = left_middle_joint_pos_error;
        error2 = right_middle_joint_pos_error;

        // Get current speed from AS5600 sensors (RPM)
        float current_left_arm_speed = 0.0f;
        float current_right_arm_speed = 0.0f;
        
        if (g_as5600_0) {
            current_left_arm_speed = g_as5600_0->get_rpm();
        }
        if (g_as5600_1) {
            current_right_arm_speed = g_as5600_1->get_rpm();
        }

        // Target speed is 0.11 RPM
        const float target_arm_speed = 0.11f;
        
        // Calculate speed error (target - current)
        float left_arm_speed_error = target_arm_speed - current_left_arm_speed;
        float right_arm_speed_error = target_arm_speed - current_right_arm_speed;

        left_arm_speed_error = (left_arm_speed_error/target_arm_speed) * ARM_MAX_PWM;;
        right_arm_speed_error = (right_arm_speed_error/target_arm_speed) * ARM_MAX_PWM;

        // PID control for speed
        float left_arm_derivative = (left_arm_speed_error - last_error_arm_1) / (CONTROL_PERIOD_MS / 1000.0f);
        float right_arm_derivative = (right_arm_speed_error - last_error_arm_2) / (CONTROL_PERIOD_MS / 1000.0f);

        float left_arm_output = left_arm_speed_error * ARM_KP + left_arm_derivative * ARM_KD;
        float right_arm_output = right_arm_speed_error * ARM_KP + right_arm_derivative * ARM_KD;

        last_error_arm_1 = left_arm_speed_error;
        last_error_arm_2 = right_arm_speed_error;

        // Normalize to PWM range
        float left_arm_frac = left_arm_output / (float)ARM_MAX_PWM;
        float right_arm_frac = right_arm_output / (float)ARM_MAX_PWM;

        // Clamp to [-1.0, 1.0]
        if (left_arm_frac > 1.0f) left_arm_frac = 1.0f;
        if (left_arm_frac < -1.0f) left_arm_frac = -1.0f;
        if (right_arm_frac > 1.0f) right_arm_frac = 1.0f;
        if (right_arm_frac < -1.0f) right_arm_frac = -1.0f;

        // Determine direction based on position error and apply PWM
        // If error1 > 0: need to move forward (positive direction)
        // If error1 < 0: need to move backward (negative direction)
        float left_arm_direction = (error1 > 0.0f) ? 1.0f : -1.0f;
        float right_arm_direction = (error2 > 0.0f) ? 1.0f : -1.0f;

        // Only apply control if there's significant position error (deadband)
        const float position_deadband = 0.001f; // degrees

        
        
    
        if (fabsf(error1) > position_deadband) {
            set_motor_pwm(LEFT_ARM_LEDC_CHANNEL, left_arm_direction * fabsf(left_arm_frac));
             // Logging every 40 iterations
             if (log_counter % 40 == 0) {
            ESP_LOGI("DRIVE_CONTROL_TASK", "Left Arm - Pos Error: %.2f, Speed: %.2f RPM, PWM: %.2f", 
                     error1, current_left_arm_speed, left_arm_frac);
        
        }
        } else {
            set_motor_pwm(LEFT_ARM_LEDC_CHANNEL, 0.0f);
            ESP_LOGI("DRIVE_CONTROL_TASK", "Left Arm stopped due to position deadband");
        }
        
        if (fabsf(error2) > position_deadband) {
            set_motor_pwm(RIGHT_ARM_LEDC_CHANNEL, right_arm_direction * fabsf(right_arm_frac));
                ESP_LOGI("DRIVE_CONTROL_TASK", "Right Arm - Pos Error: %.2f, Speed: %.2f RPM, PWM: %.2f", 
                     error2, current_right_arm_speed, right_arm_frac);
        } else {
            set_motor_pwm(RIGHT_ARM_LEDC_CHANNEL, 0.0f);
            ESP_LOGI("DRIVE_CONTROL_TASK", "Right Arm stopped due to position deadband");
        }

       

        if(log_counter % 40 == 0) {
            ESP_LOGI("DRIVE_CONTROL_TASK", "DRIVE CONTROL SUPERLOOP ENDED (iteration %lu)", log_counter);
        }
      
        log_counter++;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
