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
    arm_state_mutex = xSemaphoreCreateMutex();
    enc_mutex = xSemaphoreCreateMutex();

    
    // micro-ROS init
    micro_ros_init_and_create_comm();
    // init motors & pwm
    motors_init();

    // init encoders
    encoders_init();

    

    // create tasks
    xTaskCreate(&sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(&encoder_sample_task, "enc_sample", 4096, NULL, 4, NULL);
    xTaskCreate(&drive_control_task, "drive_ctrl", 4096, NULL, 5, NULL);
    xTaskCreate(&arm_control_task, "arm_ctrl", 4096, NULL, 6, NULL);
    xTaskCreate(&micro_ros_spin_task, "micro_ros_spin", 8192, NULL, 3, NULL);

    ESP_LOGI(TAG, "All tasks launched");
}
