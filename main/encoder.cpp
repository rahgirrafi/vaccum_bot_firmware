
#include "vaccum_firmware.hpp"
// ========= Encoders (simple quadrature) =========
inline int read_gpio_level(int gpio) { return gpio_get_level((gpio_num_t)gpio); }

void IRAM_ATTR enc_left_isr_handler(void *arg)
{
    (void)arg;
    int lastA = 0;
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

void IRAM_ATTR enc_right_isr_handler(void *arg)
{
    (void)arg;
    int lastA = 0;
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

void encoders_init(void)
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




// Encoder sample & debug task
void encoder_sample_task(void *arg)
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