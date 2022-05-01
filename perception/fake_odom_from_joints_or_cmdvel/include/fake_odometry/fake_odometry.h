/* Authors: George Popescu */

#ifndef FAKE_ODOMETRY_H_
#define FAKE_ODOMETRY_H_

#include <math.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "fake_odometry.h"

#define LEFT                            0
#define RIGHT                           1

#define MAX_LINEAR_VELOCITY             0.22   // m/s
#define MAX_ANGULAR_VELOCITY            2.84   // rad/s
#define VELOCITY_STEP                   0.01   // m/s
#define VELOCITY_LINEAR_X               0.01   // m/s
#define VELOCITY_ANGULAR_Z              0.1    // rad/s
#define SCALE_VELOCITY_LINEAR_X         1
#define SCALE_VELOCITY_ANGULAR_Z        1

#define DEG2RAD(x)                      (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                      (x * 57.2957795131)  // *180/PI

#define TORQUE_ENABLE                   1       // Value for enabling the torque of motor
#define TORQUE_DISABLE                  0       // Value for disabling the torque of motor

class OdometryFake
{
 public:
  OdometryFake();
  ~OdometryFake();
  bool init();
  bool update();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Parameters
  // (TODO)

  // ROS Time
  ros::Time last_joints_states_time_;
  ros::Time prev_update_time_;
  ros::Time last_cmd_vel_time_;

  // ROS Topic Publishers
  ros::Publisher odom_pub_;

  // ROS Topic Subscribers
  ros::Subscriber joints_states_sub_;
  ros::Subscriber cmd_vel_sub_;

  sensor_msgs::JointState joint_states_;
  nav_msgs::Odometry odom_;
  tf::TransformBroadcaster tf_broadcaster_;

  double joint_states_speed[2];
  double joint_states_timeout;
  double cmd_vel_timeout_;

  float  odom_pose_[3];
  float  odom_vel_[3];
  double pose_cov_[36];

  // std::string joint_states_name_[2];
  std::vector<std::string> joint_states_name_;

  double last_position_[2];
  double last_velocity_[2];
  double wheel_speed_cmd_[2];
  double goal_linear_velocity_;
  double goal_angular_velocity_;

  double wheel_seperation_;
  double turning_radius_;
  double robot_radius_;
  double wheel_radius_;
  int num_joints_;

  // Function prototypes
  void joint_states_callback(const sensor_msgs::JointState joint_states);
  bool update_odometry(ros::Duration diff_time);
  void update_tf(geometry_msgs::TransformStamped& odom_tf);
  void command_velocity_callback(const geometry_msgs::TwistConstPtr cmd_vel_msg);
};

#endif // FAKE_ODOMETRY_H_
