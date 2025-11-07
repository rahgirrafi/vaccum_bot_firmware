#ifndef MICRO_ROS_HPP
#define MICRO_ROS_HPP
// micro-ROS / rclc includes
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <control_msgs/msg/joint_trajectory_controller_state.h>



extern rcl_node_t node;
extern rclc_executor_t executor;


// micro-ROS publishers and subscribers
extern rcl_publisher_t angle_pub;
extern rcl_publisher_t rpm_pub;
extern rcl_subscription_t arm_state_sub;
extern rcl_subscription_t cmd_vel_sub;



// message storage
extern std_msgs__msg__Float32 angle_msg;
extern std_msgs__msg__Float32 rpm_msg;
extern control_msgs__msg__JointTrajectoryControllerState state_multiarray_msg;

void micro_ros_init_and_create_comm(void);
void micro_ros_spin_task(void *arg);

//callbacks
void arm_state_sub_callback(const void *msgin);
void cmd_vel_callback(const void *msgin);

#endif // MICRO_ROS_HPP__

