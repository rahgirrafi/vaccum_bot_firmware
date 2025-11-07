/*
ESP32 micro-ROS + AS5600 + L298N + N20 encoder example

This file was updated to use the espp As5600 API properly (espp::I2c + espp::As5600)
as shown in your reference example. The As5600 instance runs its own internal
update task (as provided by the espp library) and exposes convenient getters
like get_degrees(), get_rpm(), get_count(), get_radians().

*/

#include "vaccum_firmware.hpp"
#include "global_declarations.hpp"

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
    constexpr float filter_cutoff_hz = 4.0f;
    constexpr float encoder_update_period = 0.01f; // seconds
    espp::ButterworthFilter<2, espp::BiquadFilterDf2> filter({
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
