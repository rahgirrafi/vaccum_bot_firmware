

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
    
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
    #ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
    #endif
	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
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
    // executor two subscriptions and two publishers
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &arm_state_sub, &arm_state_msg, &arm_state_callback, ON_NEW_DATA));

    // Print executor info
    ESP_LOGI("MICRO_ROS", "Executor initialized with %d handles", executor.max_handles);


}

void micro_ros_spin_task(void *arg)
{
    ESP_LOGI("MICRO_ROS_TASK", "SPIN TASK INITIATING");
    micro_ros_init_and_create_comm();
    const TickType_t xDelay = pdMS_TO_TICKS(100);
    ESP_LOGI("MICRO_ROS_TASK", "SPIN TASK INITIATED");
    while (1) {
        ESP_LOGI("MICRO_ROS_TASK", "SPIN TASK SUPERLOOP BEGIN");
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        ESP_LOGI("MICRO_ROS_TASK", "SPIN TASK SUPERLOOP ENDED");
        
        vTaskDelay(xDelay);
    }
}
