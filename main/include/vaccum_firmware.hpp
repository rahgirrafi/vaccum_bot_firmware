#ifndef VACUUM_FIRMWARE_HPP
#define VACUUM_FIRMWARE_HPP

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <chrono>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#include "sdkconfig.h"

#include  "motor.hpp"
#include "micro_ros.hpp"
#include "encoder.hpp"
#include "esp_event.h"
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include <uros_network_interfaces.h>
using namespace std::chrono_literals;
extern const char *TAG ;

// Timings
#define SENSOR_PUBLISH_PERIOD_MS 50 // 20 Hz
#define CONTROL_PERIOD_MS 20 // 50 Hz
#define ENCODER_SAMPLE_MS 100

// Robot params
#define WHEEL_RADIUS 0.022f
#define WHEEL_SEPARATION 0.10f

void print_mem(const char *tag);

#endif // VACUUM_FIRMWARE_HPP__