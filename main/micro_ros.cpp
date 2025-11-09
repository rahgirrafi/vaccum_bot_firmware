

#include "vaccum_firmware.hpp"
#include "micro_ros.hpp"


void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__TwistStamped *tws = (const geometry_msgs__msg__TwistStamped *)msgin;
    const geometry_msgs__msg__Twist tw = tws->twist;
    float linear = (float)tw.linear.x; // m/s
    float angular = (float)tw.angular.z; // rad/s

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

static void arm_state_callback(const void *msgin)
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
    // initialize messages
    angle_msg.data = 0.0f;
    rpm_msg.data = 0.0f;
    
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    print_mem("before-getting-init-options");
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    print_mem("after-getting-init-options");
	rcl_init_options_init(&init_options, allocator);
    print_mem("after-initing-init-options");
    #ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
    #endif
	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    print_mem("after-supporting-init-options");
    RCCHECK(rclc_node_init_default(&node, "vaccum_base_body_node", "", &support));
    print_mem("after-supporting-init-options");

    
    RCCHECK(rclc_publisher_init_default(
        &angle_pub, 
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), 
        "/arm/angle"));
    print_mem("after-publisher-angle");
    RCCHECK(rclc_publisher_init_default(
        &rpm_pub, 
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), 
        "/arm/rpm"));
print_mem("after-publisher-rpm");
        rcl_publisher_t encoder_counts_pub;
    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
    qos_profile.depth = 1;
    RCCHECK(rclc_publisher_init(
            &encoder_counts_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(custom_interfaces, msg, Float32FixedArray),
            "/encoder_counts",
            &qos_profile));
print_mem("after-publisher-encoder");

    // subscribers
    RCCHECK(rclc_subscription_init_default(
            &cmd_vel_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
            "/vaccum_base_controller/cmd_vel_out")
    );
print_mem("after-sub-vel");
    RCCHECK(
        rclc_subscription_init_default(
            &arm_state_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, JointTrajectoryControllerState),
            "/arm_controller/state")
    );
print_mem("after-sub-state");
executor = rclc_executor_get_zero_initialized_executor();
    // executor two subscriptions and two publishers
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &arm_state_sub, &arm_state_msg, &arm_state_callback, ON_NEW_DATA));

    // Add executor validation
    if (!executor.handles) {
        ESP_LOGE("MICRO_ROS", "Executor handles not initialized!");
        return;
    }



    // Print executor info
    ESP_LOGI("MICRO_ROS", "Executor initialized with %d handles", executor.max_handles);


}

void micro_ros_spin_task(void *arg)
{
    const TickType_t xDelay = pdMS_TO_TICKS(100);
    int consecutive_timeouts = 0;
    const int MAX_TIMEOUTS = 5;

    while (1) {
        rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        
        if (rc == RCL_RET_TIMEOUT) {
            consecutive_timeouts++;
            ESP_LOGW("MICRO_ROS", "Executor timeout %d/%d", consecutive_timeouts, MAX_TIMEOUTS);
            
            if (consecutive_timeouts >= MAX_TIMEOUTS) {
                ESP_LOGE("MICRO_ROS", "Too many timeouts, attempting reconnection");
                // Check agent connection
                bool agent_available = rmw_uros_ping_agent(100, 3);
                if (!agent_available) {
                    ESP_LOGE("MICRO_ROS", "Agent not responding, reinitializing...");
                    micro_ros_init_and_create_comm();
                }
                consecutive_timeouts = 0;
            }
        } else if (rc != RCL_RET_OK) {
            ESP_LOGW("MICRO_ROS", "Executor spin failed with error: %d", rc);
        } else {
            consecutive_timeouts = 0;  // Reset on successful spin
        }
        
        vTaskDelay(xDelay);
    }
}
