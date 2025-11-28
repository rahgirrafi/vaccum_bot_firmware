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
    ESP_ERROR_CHECK(uros_network_interface_initialize());

    // init mutexes (no longer need i2c_mutex - using separate I2C buses)
    vel_mutex = xSemaphoreCreateMutex();
    arm_state_mutex = xSemaphoreCreateMutex();
    enc_mutex = xSemaphoreCreateMutex();
    encoder_msg_mutex = xSemaphoreCreateMutex();

    // Initialize motors/PWM before creating tasks
    motors_init();
    ESP_LOGI(TAG, "Motors PWM initialized");

    // RE-ENABLE MICRO-ROS with ultra-conservative settings
    micro_ros_init_and_create_comm();

    // create tasks with optimized priorities for better responsiveness
    xTaskCreate(encoder_sample_task, "enc_sample", 4096, NULL, 1, NULL);
    xTaskCreate(drive_control_task, "drive_ctrl", 4096, NULL, 3, NULL);  // Increased priority for motor control
    xTaskCreate(micro_ros_spin_task, "micro_ros_spin", 8192, NULL, 2, NULL); // LOWEST priority
    ESP_LOGI(TAG, "All tasks launched with ultra-conservative micro-ROS settings");
}       


