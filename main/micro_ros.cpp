

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

    // initialize messages
    angle_msg.data = 0.0f;
    rpm_msg.data = 0.0f;
    

    
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "vaccum_base_body_node", "", &support));

    

    // publishers
    rcl_publisher_options_t pub_ops = rcl_publisher_get_default_options();

    RCCHECK(rclc_publisher_init_default(
        &angle_pub, 
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), 
        "/arm/angle"));

    RCCHECK(rclc_publisher_init_default(
        &rpm_pub, 
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), 
        "/arm/rpm"));

    RCCHECK(rclc_publisher_init_default(
            &encoder_counts_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
            "/encoder_counts"));

    // RCCHECK(rclc_publisher_init(
    //         &as5600_sample_pub,
    //         &node,
    //         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    //         "/as5600_counts",
    //         &pub_ops));





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

	
	// RCCHECK(rclc_timer_init_default(
	// 	&timer,
	// 	&support,
	// 	RCL_MS_TO_NS(100),
	// 	timer_callback));



    // executor two subscriptions and two publishers
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    // RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &arm_state_sub, &arm_state_msg, &arm_state_callback, ON_NEW_DATA));


}



void micro_ros_spin_task(void *arg)
{
    (void)arg;
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
// void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
// {
// 	RCLC_UNUSED(last_call_time);
// 	if (timer != NULL) {
// 		rcl_publish(&encoder_counts_pub, &encoder_counts_msgs, NULL);
// 	}
// }
