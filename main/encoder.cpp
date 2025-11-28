#include "vaccum_firmware.hpp"

// No longer need multiplexer functions - using two separate I2C buses

inline int read_gpio_level(int gpio) { return gpio_get_level((gpio_num_t)gpio); }


// ISR counters for debugging
static volatile uint32_t isr_count_left1 = 0;
static volatile uint32_t isr_count_right1 = 0;
static volatile uint32_t isr_count_left2 = 0;
static volatile uint32_t isr_count_right2 = 0;

void IRAM_ATTR enc_left_1_isr_handler(void *arg){
    isr_count_left1++;
    // Do NOT use ESP_LOGI in ISR - it will crash!
    if (read_gpio_level(ENC_LEFT_1_B_GPIO)) {
        __atomic_fetch_add(&enc_left_1_count, 1, __ATOMIC_RELAXED);
    } else {
        __atomic_fetch_sub(&enc_left_1_count, 1, __ATOMIC_RELAXED);
    }
}

void IRAM_ATTR enc_right_1_isr_handler(void *arg){
    isr_count_right1++;
    // Do NOT use ESP_LOGI in ISR - it will crash!
    if (read_gpio_level(ENC_RIGHT_1_B_GPIO)) {
        __atomic_fetch_add(&enc_right_1_count, 1, __ATOMIC_RELAXED);
    } else {
        __atomic_fetch_sub(&enc_right_1_count, 1, __ATOMIC_RELAXED);
    }
}

void IRAM_ATTR enc_left_2_isr_handler(void *arg){
    isr_count_left2++;
    // Do NOT use ESP_LOGI in ISR - it will crash!
    if (read_gpio_level(ENC_LEFT_2_B_GPIO)) {
        __atomic_fetch_add(&enc_left_2_count, 1, __ATOMIC_RELAXED);
    } else {
        __atomic_fetch_sub(&enc_left_2_count, 1, __ATOMIC_RELAXED);
    }
}

void IRAM_ATTR enc_right_2_isr_handler(void *arg){
    isr_count_right2++;
    // Do NOT use ESP_LOGI in ISR - it will crash!
    if (read_gpio_level(ENC_RIGHT_2_B_GPIO)) {
        __atomic_fetch_add(&enc_right_2_count, 1, __ATOMIC_RELAXED);
    } else {
        __atomic_fetch_sub(&enc_right_2_count, 1, __ATOMIC_RELAXED);
    }
}

void encoders_init(void)
{
    // Make two separate I2C instances for the two AS5600 sensors
    // I2C_NUM_1 for first AS5600 sensor
    static espp::I2c i2c1({
        .port = I2C_NUM_1,
        .sda_io_num = (gpio_num_t)21,  // Standard I2C SDA pin
        .scl_io_num = (gpio_num_t)22,  // Standard I2C SCL pin
    });

    // I2C_NUM_0 for second AS5600 sensor
    static espp::I2c i2c2({
        .port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)5,   // Alternative SDA pin
        .scl_io_num = (gpio_num_t)4,   // Alternative SCL pin
    });

    // velocity filters for both sensors
    static constexpr float filter_cutoff_hz = 4.0f;
    static constexpr float encoder_update_period1 = 0.01f; // 10ms for sensor 1 (100 Hz)
    static constexpr float encoder_update_period2 = 0.01f; // 10ms for sensor 2 (100 Hz)
    
    static espp::ButterworthFilter<2, espp::BiquadFilterDf2> filter_0(
        {.normalized_cutoff_frequency = 2.0f * filter_cutoff_hz * encoder_update_period1});
    static espp::ButterworthFilter<2, espp::BiquadFilterDf2> filter_1(
        {.normalized_cutoff_frequency = 2.0f * filter_cutoff_hz * encoder_update_period2});
    
    // Filter functions for both sensors
    auto filter_fn_0 = [&filter_0](float raw) -> float { return filter_0.update(raw); };
    auto filter_fn_1 = [&filter_1](float raw) -> float { return filter_1.update(raw); };

    // Create first AS5600 on I2C_NUM_1
    g_as5600_0 = new espp::As5600(
        {.write_then_read =
             std::bind(&espp::I2c::write_read, &i2c1, std::placeholders::_1, std::placeholders::_2,
                       std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
         .velocity_filter = filter_fn_0,
         .update_period = std::chrono::duration<float>(encoder_update_period1),
         .log_level = espp::Logger::Verbosity::WARN});

    // Create second AS5600 on I2C_NUM_0
    g_as5600_1 = new espp::As5600(
        {.write_then_read =
             std::bind(&espp::I2c::write_read, &i2c2, std::placeholders::_1, std::placeholders::_2,
                       std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
         .velocity_filter = filter_fn_1,
         .update_period = std::chrono::duration<float>(encoder_update_period2),
         .log_level = espp::Logger::Verbosity::WARN});

    ESP_LOGI("ENCODER", "Dual AS5600 encoders initialized on separate I2C buses");
    ESP_LOGI("ENCODER", "  AS5600_0: I2C_NUM_1 (SDA=21, SCL=22)");
    ESP_LOGI("ENCODER", "  AS5600_1: I2C_NUM_0 (SDA=5, SCL=4)");

    ESP_LOGI("ENCODER", "Setting up GPIO encoder pins with pull-ups...");
    ESP_LOGI("ENCODER", "GPIO pins configured:");
    ESP_LOGI("ENCODER", "  Left1:  A=GPIO%d, B=GPIO%d", ENC_LEFT_1_A_GPIO, ENC_LEFT_1_B_GPIO);
    ESP_LOGI("ENCODER", "  Right1: A=GPIO%d, B=GPIO%d", ENC_RIGHT_1_A_GPIO, ENC_RIGHT_1_B_GPIO);
    ESP_LOGI("ENCODER", "  Left2:  A=GPIO%d, B=GPIO%d", ENC_LEFT_2_A_GPIO, ENC_LEFT_2_B_GPIO);
    ESP_LOGI("ENCODER", "  Right2: A=GPIO%d, B=GPIO%d", ENC_RIGHT_2_A_GPIO, ENC_RIGHT_2_B_GPIO);

    // Check for potential GPIO conflicts  
    if (ENC_LEFT_1_B_GPIO == 12) {
        ESP_LOGW("ENCODER", "WARNING: GPIO12 has bootstrap function - may interfere with encoder operation");
    }
    if (ENC_RIGHT_1_B_GPIO == 15) {
        ESP_LOGW("ENCODER", "WARNING: GPIO15 has bootstrap function - may interfere with encoder operation");
    }

    // Configure all encoder A pins (with interrupts and pull-ups)
    esp_err_t ret;
    
    ret = gpio_set_direction((gpio_num_t)ENC_LEFT_1_A_GPIO, GPIO_MODE_INPUT);
    if (ret != ESP_OK) ESP_LOGE("ENCODER", "Failed to set direction for GPIO%d: %s", ENC_LEFT_1_A_GPIO, esp_err_to_name(ret));
    gpio_set_pull_mode((gpio_num_t)ENC_LEFT_1_A_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_intr_type((gpio_num_t)ENC_LEFT_1_A_GPIO, GPIO_INTR_POSEDGE);
    
    gpio_set_direction((gpio_num_t)ENC_LEFT_2_A_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)ENC_LEFT_2_A_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_intr_type((gpio_num_t)ENC_LEFT_2_A_GPIO, GPIO_INTR_POSEDGE);

    gpio_set_direction((gpio_num_t)ENC_RIGHT_1_A_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)ENC_RIGHT_1_A_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_intr_type((gpio_num_t)ENC_RIGHT_1_A_GPIO, GPIO_INTR_POSEDGE);

    gpio_set_direction((gpio_num_t)ENC_RIGHT_2_A_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)ENC_RIGHT_2_A_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_intr_type((gpio_num_t)ENC_RIGHT_2_A_GPIO, GPIO_INTR_POSEDGE);

    // Configure all encoder B pins (inputs with pull-ups, no interrupts)
    gpio_set_direction((gpio_num_t)ENC_LEFT_1_B_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)ENC_LEFT_1_B_GPIO, GPIO_PULLUP_ONLY);
    
    gpio_set_direction((gpio_num_t)ENC_LEFT_2_B_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)ENC_LEFT_2_B_GPIO, GPIO_PULLUP_ONLY);
    
    gpio_set_direction((gpio_num_t)ENC_RIGHT_1_B_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)ENC_RIGHT_1_B_GPIO, GPIO_PULLUP_ONLY);
    
    gpio_set_direction((gpio_num_t)ENC_RIGHT_2_B_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)ENC_RIGHT_2_B_GPIO, GPIO_PULLUP_ONLY);

    ESP_LOGI("ENCODER", "GPIO directions and pull-ups configured");

    // Install ISR service with error checking
    ret = gpio_install_isr_service(0);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI("ENCODER", "ISR service already installed");
    } else if (ret != ESP_OK) {
        ESP_LOGE("ENCODER", "Failed to install ISR service: %s", esp_err_to_name(ret));
        return;
    } else {
        ESP_LOGI("ENCODER", "ISR service installed successfully");
    }

    // Add interrupt handlers with error checking
    ret = gpio_isr_handler_add((gpio_num_t)ENC_LEFT_1_A_GPIO, enc_left_1_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE("ENCODER", "Failed to add ISR for ENC_LEFT_1_A (GPIO%d): %s", ENC_LEFT_1_A_GPIO, esp_err_to_name(ret));
    } else {
        ESP_LOGI("ENCODER", "✓ ENC_LEFT_1_A ISR added to GPIO%d", ENC_LEFT_1_A_GPIO);
    }
    
    ret = gpio_isr_handler_add((gpio_num_t)ENC_RIGHT_1_A_GPIO, enc_right_1_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE("ENCODER", "Failed to add ISR for ENC_RIGHT_1_A (GPIO%d): %s", ENC_RIGHT_1_A_GPIO, esp_err_to_name(ret));
    } else {
        ESP_LOGI("ENCODER", "✓ ENC_RIGHT_1_A ISR added to GPIO%d", ENC_RIGHT_1_A_GPIO);
    }
    
    ret = gpio_isr_handler_add((gpio_num_t)ENC_LEFT_2_A_GPIO, enc_left_2_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE("ENCODER", "Failed to add ISR for ENC_LEFT_2_A (GPIO%d): %s", ENC_LEFT_2_A_GPIO, esp_err_to_name(ret));
    } else {
        ESP_LOGI("ENCODER", "✓ ENC_LEFT_2_A ISR added to GPIO%d", ENC_LEFT_2_A_GPIO);
    }
    
    ret = gpio_isr_handler_add((gpio_num_t)ENC_RIGHT_2_A_GPIO, enc_right_2_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE("ENCODER", "Failed to add ISR for ENC_RIGHT_2_A (GPIO%d): %s", ENC_RIGHT_2_A_GPIO, esp_err_to_name(ret));
    } else {
        ESP_LOGI("ENCODER", "✓ ENC_RIGHT_2_A ISR added to GPIO%d", ENC_RIGHT_2_A_GPIO);
    }
    
    // Test initial GPIO states
    ESP_LOGI("ENCODER", "Initial GPIO States:");
    ESP_LOGI("ENCODER", "  L1A=%d L1B=%d R1A=%d R1B=%d", 
            gpio_get_level((gpio_num_t)ENC_LEFT_1_A_GPIO),
            gpio_get_level((gpio_num_t)ENC_LEFT_1_B_GPIO),
            gpio_get_level((gpio_num_t)ENC_RIGHT_1_A_GPIO),
            gpio_get_level((gpio_num_t)ENC_RIGHT_1_B_GPIO));
    ESP_LOGI("ENCODER", "  L2A=%d L2B=%d R2A=%d R2B=%d",
            gpio_get_level((gpio_num_t)ENC_LEFT_2_A_GPIO),
            gpio_get_level((gpio_num_t)ENC_LEFT_2_B_GPIO),
            gpio_get_level((gpio_num_t)ENC_RIGHT_2_A_GPIO),
            gpio_get_level((gpio_num_t)ENC_RIGHT_2_B_GPIO));
    
    ESP_LOGI("ENCODER", "All encoder interrupts configured successfully");
}



// Encoder sample & debug task
void encoder_sample_task(void *arg)
{
    // ESP_LOGI("ENCODER_TASK", "ENCODER SAMPLE TASK SUPER LOOP INITIATING");
    encoders_init();
    (void)arg;
    int64_t last_left1 = 0, last_right1 = 0;
    int64_t last_left2 = 0, last_right2 = 0;
    int log_counter = 0;  // Add missing log counter variable
    TickType_t last_wake = xTaskGetTickCount();
    // ESP_LOGI("ENCODER_TASK", "ENCODER SAMPLE TASK SUPER LOOP INITIATED");
    
    while (1) {
        // if(log_counter % 40 == 0) {
        //     ESP_LOGI("ENCODER_TASK", "ENCODER SAMPLE TASK SUPER LOOP BEGIN");
        // }
        int64_t left1 = (int64_t) __atomic_load_n(&enc_left_1_count, __ATOMIC_RELAXED);
        int64_t right1 = (int64_t) __atomic_load_n(&enc_right_1_count, __ATOMIC_RELAXED);
        int64_t left2 = (int64_t) __atomic_load_n(&enc_left_2_count, __ATOMIC_RELAXED);
        int64_t right2 = (int64_t) __atomic_load_n(&enc_right_2_count, __ATOMIC_RELAXED);
        int64_t dl1 = left1 - last_left1;
        int64_t dr1 = right1 - last_right1;
        int64_t dl2 = left2 - last_left2;
        int64_t dr2 = right2 - last_right2;
        last_left1 = left1;
        last_right1 = right1;
        last_left2 = left2;
        last_right2 = right2;
        
        float dt = (float)ENCODER_SAMPLE_MS / 1000.0f;
        
        float left_rps1 = (dl1 / TICKS_PER_REV) / dt;
        float right_rps1 = (dr1 / TICKS_PER_REV) / dt;
        float left_rps2 = (dl2 / TICKS_PER_REV) / dt;
        float right_rps2 = (dr2 / TICKS_PER_REV) / dt;
        float left_mps1 = left_rps1 * (2.0f * M_PI * WHEEL_RADIUS);
        float right_mps1 = right_rps1 * (2.0f * M_PI * WHEEL_RADIUS);
        float left_mps2 = left_rps2 * (2.0f * M_PI * WHEEL_RADIUS);
        float right_mps2 = right_rps2 * (2.0f * M_PI * WHEEL_RADIUS);
        current_speed1 = left_mps1;
        current_speed2 = right_mps1;
        current_speed3 = left_mps2;
        current_speed4 = right_mps2;


        // Protect encoder message with mutex
        if (xSemaphoreTake(encoder_msg_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            encoder_counts_angel_rpm_msgs.element[0] = left_rps1;
            encoder_counts_angel_rpm_msgs.element[1] = right_rps1;
            encoder_counts_angel_rpm_msgs.element[2] = left_rps2;
            encoder_counts_angel_rpm_msgs.element[3] = right_rps2;
            // encoder_counts_angel_rpm_msgs.element[0] = 0;
            // encoder_counts_angel_rpm_msgs.element[1] = 0;
            // encoder_counts_angel_rpm_msgs.element[2] = 0;
            // encoder_counts_angel_rpm_msgs.element[3] = 0;
            
            xSemaphoreGive(encoder_msg_mutex);
        } else {
            ESP_LOGW("ENCODER_TASK", "Failed to take encoder_msg_mutex for basic data");
        }


        // if(log_counter % 40 == 0) {
        //     ESP_LOGI("SENSOR_TASK", "SENSOR TASK INITIATED");
        // }

        // AS5600 sensor readings from separate I2C buses
        // No need for channel switching - each sensor has its own bus
        
        // Read from first AS5600 on I2C_NUM_1
        float radians_0 = 0.0f, rpm_0 = 0.0f;
        if (g_as5600_0) {
            radians_0 = g_as5600_0->get_radians();
            rpm_0 = g_as5600_0->get_rpm();
        }

        // Read from second AS5600 on I2C_NUM_0
        float radians_1 = 0.0f, rpm_1 = 0.0f;
        if (g_as5600_1) {
            radians_1 = g_as5600_1->get_radians();
            rpm_1 = g_as5600_1->get_rpm();
        }

        // Protect encoder message publishing with mutex
        if (xSemaphoreTake(encoder_msg_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Update encoder message with dual AS5600 data
            encoder_counts_angel_rpm_msgs.element[4] = radians_0;  // First AS5600 degrees
            encoder_counts_angel_rpm_msgs.element[5] = rpm_0;      // First AS5600 RPM
            encoder_counts_angel_rpm_msgs.element[6] = radians_1;  // Second AS5600 degrees  
            encoder_counts_angel_rpm_msgs.element[7] = rpm_1;      // Second AS5600 RPM
            
            // SOLUTION: Just update the message data - let micro_ros_spin_task handle publishing
            // This eliminates the race condition between tasks accessing micro-ROS network stack
            
            xSemaphoreGive(encoder_msg_mutex);
            
            ESP_LOGI("ENCODER_TASK", "Dual encoder data updated (publishing handled by micro_ros_spin_task)");
        } else {
            ESP_LOGW("ENCODER_TASK", "Failed to take encoder_msg_mutex for updating data");
        }
            
            // Throttle encoder data logging to reduce memory pressure
            if(log_counter % 50 == 0) {
                ESP_LOGI("ENCODER", "MOTOR Encoders L1: %lld R1: %lld | L2: %lld R2: %lld", 
                        left1, right1, left2, right2);
                ESP_LOGI("ENCODER", "ISR Counters - L1:%lu R1:%lu | L2:%lu R2:%lu", 
                        isr_count_left1, isr_count_right1, isr_count_left2, isr_count_right2);
                ESP_LOGI("ENCODER", "AS5600 Sensors - Sensor0: %.2f° %.2fRPM | Sensor1: %.2f° %.2fRPM", 
                        radians_0, rpm_0, radians_1, rpm_1);
                        
                // Show GPIO pin states for debugging
                ESP_LOGI("ENCODER", "GPIO States - L1A:%d L1B:%d R1A:%d R1B:%d | L2A:%d L2B:%d R2A:%d R2B:%d",
                        gpio_get_level((gpio_num_t)ENC_LEFT_1_A_GPIO),
                        gpio_get_level((gpio_num_t)ENC_LEFT_1_B_GPIO),
                        gpio_get_level((gpio_num_t)ENC_RIGHT_1_A_GPIO),
                        gpio_get_level((gpio_num_t)ENC_RIGHT_1_B_GPIO),
                        gpio_get_level((gpio_num_t)ENC_LEFT_2_A_GPIO),
                        gpio_get_level((gpio_num_t)ENC_LEFT_2_B_GPIO),
                        gpio_get_level((gpio_num_t)ENC_RIGHT_2_A_GPIO),
                        gpio_get_level((gpio_num_t)ENC_RIGHT_2_B_GPIO));
            }
           
            // if(log_counter % 100 == 0) {
            //     ESP_LOGI("ENCODER_TASK", "ENCODER SAMPLE TASK SUPER LOOP ENDED (iteration %lu)", log_counter);
            // }
        log_counter++;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ENCODER_SAMPLE_MS));
        }
}
