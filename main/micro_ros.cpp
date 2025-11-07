

#include "vaccum_firmware.hpp"
void micro_ros_init_and_create_comm(void)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);

    rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "rclc_support_init failed: %d", rc);
    }

    rc = rclc_node_init_default(&node, "esp32_arm_drive_node", "", &support);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "node init failed: %d", rc);
    }

    // publishers
    rcl_publisher_options_t pub_ops = rcl_publisher_get_default_options();
    rc = rcl_publisher_init(&angle_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/arm/angle", &pub_ops);
    if (rc != RCL_RET_OK) ESP_LOGE(TAG, "angle pub init failed: %d", rc);
    rc = rcl_publisher_init(&rpm_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/arm/rpm", &pub_ops);
    if (rc != RCL_RET_OK) ESP_LOGE(TAG, "rpm pub init failed: %d", rc);

    // subscribers
    rcl_subscription_options_t sub_ops = rcl_subscription_get_default_options();
    rc = rcl_subscription_init(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/vaccum_base_controller/cmd_vel_unstamped", &sub_ops);
    if (rc != RCL_RET_OK) ESP_LOGE(TAG, "cmd_vel sub init failed: %d", rc);

    rc = rcl_subscription_init(&arm_state_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, JointTrajectoryControllerState), "/arm_controller/state", &sub_ops);
    if (rc != RCL_RET_OK) ESP_LOGE(TAG, "arm state sub init failed: %d", rc);



    // executor
    rclc_executor_init(&executor, &support.context, 2, &allocator);

    // initialize messages
    angle_msg.data = 0.0f;
    rpm_msg.data = 0.0f;
}

void micro_ros_spin_task(void *arg)
{
    (void)arg;
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}



void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *tw = (const geometry_msgs__msg__Twist *)msgin;
    float linear = (float)tw->linear.x; // m/s
    float angular = (float)tw->angular.z; // rad/s

    float v_left = linear - (angular * WHEEL_SEPARATION * 0.5f);
    float v_right = linear + (angular * WHEEL_SEPARATION * 0.5f);

    if (xSemaphoreTake(vel_mutex, (TickType_t)10) == pdTRUE) {
        target_left_vel = v_left;
        target_right_vel = v_right;
        xSemaphoreGive(vel_mutex);
    }
}

static void arm_state_callback(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *arr = (const std_msgs__msg__Float32MultiArray *)msgin;
    if (arr->data.size > 0) {
        float desired = arr->data.data[0];
        if (xSemaphoreTake(arm_target_mutex, (TickType_t)10) == pdTRUE) {
            arm_target_pos = desired;
            xSemaphoreGive(arm_target_mutex);
        }
    }
}
