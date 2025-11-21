

#include "vaccum_firmware.hpp"
#include "micro_ros.hpp"


void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__TwistStamped *tws = (const geometry_msgs__msg__TwistStamped *)msgin;
    const geometry_msgs__msg__Twist tw = tws->twist;
    float linear = (float)tw.linear.x; // m/s
    float angular = (float)tw.angular.z; // rad/s
    ESP_LOGI("CMD_VEL_CALLBACK", "Linear Vel: %.3f, Angular Vel: %.3f", linear, angular);

    float v_left = linear - (angular * WHEEL_SEPARATION * 0.5f);
    float v_right = linear + (angular * WHEEL_SEPARATION * 0.5f);
    if(v_left > 0){

    }
    if (xSemaphoreTake(vel_mutex, (TickType_t)10) == pdTRUE) {
        target_left_vel = v_left;
        target_right_vel = v_right;
        xSemaphoreGive(vel_mutex);
    }
}

void arm_state_sub_callback(const void *msgin)
{
    const control_msgs__msg__JointTrajectoryControllerState *arr = (const control_msgs__msg__JointTrajectoryControllerState *)msgin;

    if (arr->joint_names.size > 0) {
        left_arm_joint_pos_error = arr->error.positions.data[0];
        left_middle_joint_pos_error = arr->error.positions.data[1];
        right_arm_joint_pos_error = arr->error.positions.data[2];
        right_middle_joint_pos_error = arr->error.positions.data[3];
    }

    if (xSemaphoreTake(arm_state_mutex, (TickType_t)10) == pdTRUE) {
        arm_state_msg = *arr;
        xSemaphoreGive(arm_state_mutex);
    }

}

void micro_ros_init_and_create_comm(void)
{
    // Initialize message structures using proper micro-ROS initialization
    memset(&cmd_vel_msg, 0, sizeof(cmd_vel_msg));
    memset(&arm_state_msg, 0, sizeof(arm_state_msg));  
    memset(&encoder_counts_angel_rpm_msgs, 0, sizeof(encoder_counts_angel_rpm_msgs));
    
    allocator = rcl_get_default_allocator();
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
    #ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	// RCCHECK(rmw_uros_discover_agent(rmw_options));
    #endif
	// create init_options
	// RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    //write a detailed error log if rclc_support_init_with_options fails
    ESP_LOGI("MICRO_ROS", "About to initialize support with options...");
    rcl_ret_t rc = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    if (rc != RCL_RET_OK) {
        ESP_LOGE("MICRO_ROS", "rclc_support_init_with_options failed with code %d: %s", (int)rc, rcl_get_error_string().str);
        ESP_LOGE("MICRO_ROS", "Make sure micro-ROS agent is running at %s:%s", CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
        ESP_LOGE("MICRO_ROS", "Check WiFi connectivity and network configuration");
        fflush(stdout);
        vTaskDelete(NULL);
    }
    ESP_LOGI("MICRO_ROS", "Support initialization successful!");
    node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "vaccum_base_body_node", "", &support));      


    RCCHECK(rclc_publisher_init_default(
            &encoder_counts_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(custom_interfaces, msg, Float32FixedArray8),
            "/encoder_counts"));


    // subscribers
    RCCHECK(rclc_subscription_init_default(
            &cmd_vel_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
            "/vaccum_base_controller/cmd_vel_out")
    );

    RCCHECK(
        rclc_subscription_init_default(
            &arm_state_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, JointTrajectoryControllerState),
            "/arm_controller/state")
    );
executor = rclc_executor_get_zero_initialized_executor();
    // executor with both cmd_vel and arm_state subscriptions
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &arm_state_sub, &arm_state_msg, &arm_state_sub_callback, ON_NEW_DATA));

    // Print executor info
    ESP_LOGI("MICRO_ROS", "Executor initialized with %d handles", executor.max_handles);


}
void micro_ros_spin_task(void *arg)
{
    ESP_LOGI("MICRO_ROS_TASK", "SPIN TASK INITIATING");
    
    const TickType_t xDelay = pdMS_TO_TICKS(100);
    ESP_LOGI("MICRO_ROS_TASK", "SPIN TASK INITIATED");
    while (1) {
        ESP_LOGI("MICRO_ROS_TASK", "SPIN TASK SUPERLOOP BEGIN");
        
        rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        if (rc != RCL_RET_OK) {
            ESP_LOGW("MICRO_ROS_TASK", "Executor spin failed: %d - Agent may not be available", (int)rc);
        } else {
            ESP_LOGI("MICRO_ROS_TASK", "SPIN TASK SUPERLOOP ENDED");
        }
        
        vTaskDelay(xDelay);
    }
}
