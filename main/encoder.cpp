#include "vaccum_firmware.hpp"
// ========= Encoders (simple quadrature) =========
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
    int64_t data[4] = {0,0,0,0};
    encoder_counts_msgs.data.data = data;
    encoder_counts_msgs.data.size = 4;
    encoder_counts_msgs.data.capacity = 4;
    encoder_counts_msgs.layout.dim.size = 0;
    encoder_counts_msgs.layout.dim.capacity = 0;
    while (1) {
        int64_t left1 = (int64_t) __atomic_load_n(&enc_left_1_count, __ATOMIC_RELAXED);
        int64_t right1 = (int64_t) __atomic_load_n(&enc_right_1_count, __ATOMIC_RELAXED);
        int64_t left2 = (int64_t) __atomic_load_n(&enc_left_2_count, __ATOMIC_RELAXED);
        int64_t right2 = (int64_t) __atomic_load_n(&enc_right_2_count, __ATOMIC_RELAXED);

        encoder_counts_msgs.data.data[0] = left1;
        encoder_counts_msgs.data.data[1] = right1;
        encoder_counts_msgs.data.data[2] = left2;
        encoder_counts_msgs.data.data[3] = right2;
        
        rcl_publish(&encoder_counts_pub, &encoder_counts_msgs, NULL);


        ESP_LOGI(TAG,
                 "Encoders L1: %lld R1: %lld | L2: %lld R2: %lld",
                 left1, right1,
                 left2, right2);
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