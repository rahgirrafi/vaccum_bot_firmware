#ifndef MICRO_ROS_HPP
#define MICRO_ROS_HPP
// micro-ROS / rclc includes

#define RCCHECK(fn) { \
  rcl_ret_t temp_rc = (fn); \
  if ((temp_rc) != RCL_RET_OK) { \
    printf("RCCHECK FAILED %s:%d rc=%d -> %s\n", __FILE__, __LINE__, (int)temp_rc, rcl_get_error_string().str); \
    fflush(stdout); \
    vTaskDelete(NULL); \
  } \
}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = (fn); if((temp_rc) != RCL_RET_OK){printf("RCSOFTCHECK %s:%d rc=%d -> %s\n", __FILE__, __LINE__, (int)temp_rc, rcl_get_error_string().str);}}


#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int64_multi_array.h>
#include <custom_interfaces/msg/float32_fixed_array8.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <control_msgs/msg/joint_trajectory_controller_state.h>

extern float left_arm_joint_pos_error;
extern float left_middle_joint_pos_error;
extern float right_arm_joint_pos_error;
extern float right_middle_joint_pos_error;


extern rcl_node_t node;
extern rclc_executor_t executor;
extern rcl_allocator_t allocator;
extern rclc_support_t support;
extern rcl_timer_t timer;


// micro-ROS publishers and subscribers
extern rcl_publisher_t angle_pub;
extern rcl_publisher_t rpm_pub;
extern rcl_publisher_t encoder_counts_pub;
// extern rcl_subscription_t as5600_sample_pub;


extern rcl_subscription_t arm_state_sub;
extern rcl_subscription_t cmd_vel_sub;


// message storage
extern geometry_msgs__msg__TwistStamped cmd_vel_msg;
extern control_msgs__msg__JointTrajectoryControllerState arm_state_msg;
extern custom_interfaces__msg__Float32FixedArray8 encoder_counts_angel_rpm_msgs;
// extern custom_interfaces__msg__Float32FixedArray as5600_msgs;

void micro_ros_init_and_create_comm(void);
void micro_ros_spin_task(void *arg);

//callbacks
void arm_state_sub_callback(const void *msgin);
void cmd_vel_callback(const void *msgin);
// void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
#endif // MICRO_ROS_HPP__

