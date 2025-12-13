#include "vaccum_firmware.hpp"

inline int read_gpio_level(int gpio) { return gpio_get_level((gpio_num_t)gpio); }


void IRAM_ATTR enc_left_1_isr_handler(void *arg){

    if (read_gpio_level(ENC_LEFT_1_B_GPIO)) {
        __atomic_fetch_add(&enc_left_1_count, 1, __ATOMIC_RELAXED);
    } else {
        __atomic_fetch_sub(&enc_left_1_count, 1, __ATOMIC_RELAXED);
    }
}

void IRAM_ATTR enc_right_1_isr_handler(void *arg){

    if (read_gpio_level(ENC_RIGHT_1_B_GPIO)) {
        __atomic_fetch_add(&enc_right_1_count, 1, __ATOMIC_RELAXED);
    } else {
        __atomic_fetch_sub(&enc_right_1_count, 1, __ATOMIC_RELAXED);
    }
}

void IRAM_ATTR enc_left_2_isr_handler(void *arg){

    if (read_gpio_level(ENC_LEFT_2_B_GPIO)) {
        __atomic_fetch_add(&enc_left_2_count, 1, __ATOMIC_RELAXED);
    } else {
        __atomic_fetch_sub(&enc_left_2_count, 1, __ATOMIC_RELAXED);
    }
}

void IRAM_ATTR enc_right_2_isr_handler(void *arg){

    if (read_gpio_level(ENC_RIGHT_2_B_GPIO)) {
        __atomic_fetch_add(&enc_right_2_count, 1, __ATOMIC_RELAXED);
    } else {
        __atomic_fetch_sub(&enc_right_2_count, 1, __ATOMIC_RELAXED);
    }
}

void encoders_init(void)
{
    //  // make the I2C that we'll use to communicate
    // static espp::I2c i2c({
    //     .port = I2C_MASTER_NUM,
    //     .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_GPIO,
    //     .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_GPIO,
    // });

    // // velocity filter
    // static constexpr float filter_cutoff_hz = 4.0f;
    // static constexpr float encoder_update_period = 0.01f; // seconds
    // static espp::ButterworthFilter<2, espp::BiquadFilterDf2> filter(
    //     {.normalized_cutoff_frequency = 2.0f * filter_cutoff_hz * encoder_update_period});
    // //disable filtering by simply returning the raw value from this function
    // auto filter_fn = [&filter](float raw) -> float { return filter.update(raw); };

    // // now make the as5600 which decodes the data
    // g_as5600 = new espp::As5600(
    //     {.write_then_read =
    //          std::bind(&espp::I2c::write_read, &i2c, std::placeholders::_1, std::placeholders::_2,
    //                    std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
    //      .velocity_filter = filter_fn,
    //      .update_period = std::chrono::duration<float>(encoder_update_period),
    //      .log_level = espp::Logger::Verbosity::WARN});

    

    gpio_set_direction((gpio_num_t)ENC_LEFT_1_A_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)ENC_LEFT_1_B_GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type((gpio_num_t)ENC_LEFT_1_A_GPIO, GPIO_INTR_POSEDGE);
    
    gpio_set_direction((gpio_num_t)ENC_LEFT_1_B_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)ENC_LEFT_1_B_GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type((gpio_num_t)ENC_LEFT_1_B_GPIO, GPIO_INTR_POSEDGE);

    gpio_set_direction((gpio_num_t)ENC_RIGHT_1_A_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)ENC_RIGHT_1_B_GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type((gpio_num_t)ENC_RIGHT_1_A_GPIO, GPIO_INTR_POSEDGE);

    gpio_set_direction((gpio_num_t)ENC_RIGHT_1_B_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)ENC_RIGHT_1_B_GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type((gpio_num_t)ENC_RIGHT_1_B_GPIO, GPIO_INTR_POSEDGE);

    gpio_set_direction((gpio_num_t)ENC_LEFT_2_A_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)ENC_LEFT_2_B_GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type((gpio_num_t)ENC_LEFT_2_A_GPIO, GPIO_INTR_POSEDGE);

    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)ENC_LEFT_1_A_GPIO, enc_left_1_isr_handler, NULL);
    gpio_isr_handler_add((gpio_num_t)ENC_RIGHT_1_A_GPIO, enc_right_1_isr_handler, NULL);
    gpio_isr_handler_add((gpio_num_t)ENC_LEFT_2_A_GPIO, enc_left_2_isr_handler, NULL);
    gpio_isr_handler_add((gpio_num_t)ENC_RIGHT_2_A_GPIO, enc_right_2_isr_handler, NULL);
}



// Encoder sample & debug task
void encoder_sample_task(void *arg)
{
    (void)arg;
    int64_t last_left1 = 0, last_right1 = 0;
    int64_t last_left2 = 0, last_right2 = 0;
    TickType_t last_wake = xTaskGetTickCount();
    
    while (1) {
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


        encoder_counts_msgs.element[0] = left_rps1;
        encoder_counts_msgs.element[1] = right_rps1;
        encoder_counts_msgs.element[2] = left_rps2;
        encoder_counts_msgs.element[3] = right_rps2;
        
        rcl_publish(&encoder_counts_pub, &encoder_counts_msgs, NULL);
        ESP_LOGI(TAG,
                 "MOTOR Encoders L1: %lld R1: %lld | L2: %lld R2: %lld",
                 left1, right1,
                 left2, right2);

        // static auto start = std::chrono::high_resolution_clock::now();
        // auto now = std::chrono::high_resolution_clock::now();
        // auto seconds = std::chrono::duration<float>(now - start).count();
        // auto count = as5600.get_count();
        // auto radians = as5600.get_radians();
        // auto degrees = as5600.get_degrees();
        // auto rpm = as5600.get_rpm();

        // as5600_msgs.data.data[0] = (float32_t) seconds;
        // as5600_msgs.data.data[1] = (float32_t) count;
        // as5600_msgs.data.data[2] = (float32_t)radians;
        // as5600_msgs.data.data[3] = (float32_t)degrees;
        // as5600_msgs.data.data[4] = (float32_t) rpm;


        // rcl_publish(&as5600_sample_pub, &as5600_msgs, NULL)
        // ESP_LOGI(TAG, "AS5600: Seconds = %f, Count = %f, Radians = %f, Degrees = %f, RPM = %f \n", seconds, count, radians, degrees, rpm )
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ENCODER_SAMPLE_MS));
    }
}

// ========= Tasks =========
// Sensor task: read As5600 using espp API, publish degrees and rpm
void sensor_task(void *arg)
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