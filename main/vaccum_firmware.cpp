/*
ESP32 micro-ROS + AS5600 + L298N + N20 encoder example

This file was updated to use the espp As5600 API properly (espp::I2c + espp::As5600)
as shown in your reference example. The As5600 instance runs its own internal
update task (as provided by the espp library) and exposes convenient getters
like get_degrees(), get_rpm(), get_count(), get_radians().

*/

#include "vaccum_firmware.hpp"
#include "global_declarations.hpp"

void print_mem(const char *tag) {
    printf("MEM: %s free_heap=%u min_free=%u\n", tag, (unsigned)esp_get_free_heap_size(), (unsigned)esp_get_minimum_free_heap_size());
}
extern "C" void app_main(void)
{

    ESP_LOGI(TAG, "Starting esp32 ARM+DRIVE node (As5600 updated)");
    print_mem("startup-before-network");
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    print_mem("after-network-init");

    // init mutexes
    i2c_mutex = xSemaphoreCreateMutex();
    vel_mutex = xSemaphoreCreateMutex();
    arm_state_mutex = xSemaphoreCreateMutex();
    enc_mutex = xSemaphoreCreateMutex();
    print_mem("after-create-mutexes-and-before-micro-ros-init");

    // micro-ROS init
    micro_ros_init_and_create_comm();
print_mem("after-micro-ros-init");

    // init motors & pwm
    motors_init();

    // init encoders
    encoders_init();

    

    // create tasks
    xTaskCreate(&sensor_task, "sensor_task", 2048, NULL, 5, NULL);
    xTaskCreate(&encoder_sample_task, "enc_sample", 2048, NULL, 4, NULL);
    xTaskCreate(&drive_control_task, "drive_ctrl", 2048, NULL, 5, NULL);
    xTaskCreate(&arm_control_task, "arm_ctrl", 2048, NULL, 6, NULL);
    xTaskCreate(&micro_ros_spin_task, "micro_ros_spin", 4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "All tasks launched");
}


