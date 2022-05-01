/* Authors: George Popescu */

#include <fake_odometry/fake_odometry.h>

OdometryFake::OdometryFake()
: nh_priv_("~")
{
  //Init fake odometry node
  bool init_result = init();
  ROS_ASSERT(init_result);
}

OdometryFake::~OdometryFake()
{
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool OdometryFake::init()
{
  // initialize ROS parameter

  std::string robot_model = nh_.param<std::string>("tb3_model", "");

  // if (!robot_model.compare("burger"))
  // {
  //   wheel_seperation_ = 0.160;
  //   turning_radius_   = 0.080;
  //   robot_radius_     = 0.105;
  // }
  // else if (!robot_model.compare("waffle") || !robot_model.compare("waffle_pi"))
  // {
  //   wheel_seperation_ = 0.287;
  //   turning_radius_   = 0.1435;
  //   robot_radius_     = 0.220;
  // }

  // nh_.param("wheel_left_joint_name", joint_states_name_[LEFT],  std::string("wheel_left_joint"));
  // nh_.param("wheel_right_joint_name", joint_states_name_[RIGHT],  std::string("wheel_right_joint"));

  nh_.param<double>("mobile_base_controller/wheel_radius", wheel_radius_, 0.033);
  nh_.param<double>("mobile_base_controller/wheel_separation", wheel_seperation_, 0.287);

  // Get joint names
  nh_.getParam("hardware_interface/joints", joint_states_name_);
  if (joint_states_name_.size() == 0)
  {
    ROS_FATAL_STREAM_NAMED("init","No joints found on parameter server for controller. Did you load the proper yaml file?");
  }
  num_joints_ = joint_states_name_.size();

  nh_.param("joint_states_frame", joint_states_.header.frame_id, std::string("base_footprint"));
  nh_.param("odom_frame", odom_.header.frame_id, std::string("odom"));
  nh_.param("base_frame", odom_.child_frame_id, std::string("base_footprint"));

  // initialize variables
  wheel_speed_cmd_[LEFT]  = 0.0;
  wheel_speed_cmd_[RIGHT] = 0.0;
  goal_linear_velocity_   = 0.0;
  goal_angular_velocity_  = 0.0;
  joint_states_timeout    = 1.0;
  cmd_vel_timeout_    = 1.0;
  last_position_[LEFT]    = 0.0;
  last_position_[RIGHT]   = 0.0;
  last_velocity_[LEFT]    = 0.0;
  last_velocity_[RIGHT]   = 0.0;

  double pcov[36] = { 0.1,   0,   0,   0,   0, 0,
                        0, 0.1,   0,   0,   0, 0,
                        0,   0, 1e6,   0,   0, 0,
                        0,   0,   0, 1e6,   0, 0,
                        0,   0,   0,   0, 1e6, 0,
                        0,   0,   0,   0,   0, 0.2};
  memcpy(&(odom_.pose.covariance),pcov,sizeof(double)*36);
  memcpy(&(odom_.twist.covariance),pcov,sizeof(double)*36);

  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;

  odom_vel_[0] = 0.0;
  odom_vel_[1] = 0.0;
  odom_vel_[2] = 0.0;

  joint_states_.name.push_back(joint_states_name_[LEFT]);
  joint_states_.name.push_back(joint_states_name_[RIGHT]);
  joint_states_.position.resize(2,0.0);
  joint_states_.velocity.resize(2,0.0);
  joint_states_.effort.resize(2,0.0);

  // initialize publishers
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 100);

  // initialize subscribers
  joints_states_sub_  = nh_.subscribe("/gazebo_rover/wheel_speed_read", 100,  &OdometryFake::joint_states_callback, this);
  cmd_vel_sub_  = nh_.subscribe("/cmd_vel", 100,  &OdometryFake::command_velocity_callback, this);

  prev_update_time_ = ros::Time::now();

  return true;
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void OdometryFake::joint_states_callback(const sensor_msgs::JointState joint_states)
{
  last_joints_states_time_ = ros::Time::now();

  joint_states_speed[LEFT]  = joint_states.velocity[LEFT];
  joint_states_speed[RIGHT] = joint_states.velocity[RIGHT];
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void OdometryFake::command_velocity_callback(const geometry_msgs::TwistConstPtr cmd_vel_msg)
{
  last_cmd_vel_time_ = ros::Time::now();

  goal_linear_velocity_  = cmd_vel_msg->linear.x;
  goal_angular_velocity_ = cmd_vel_msg->angular.z;

  wheel_speed_cmd_[LEFT]  = goal_linear_velocity_ - (goal_angular_velocity_ * wheel_seperation_ / 2);
  wheel_speed_cmd_[RIGHT] = goal_linear_velocity_ + (goal_angular_velocity_ * wheel_seperation_ / 2);
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool OdometryFake::update_odometry(ros::Duration diff_time)
{
  double wheel_l, wheel_r; // rotation value of wheel [rad]
  double delta_s, delta_theta;
  double v[2], w[2];

  wheel_l = wheel_r     = 0.0;
  delta_s = delta_theta = 0.0;

  v[LEFT]  = wheel_speed_cmd_[LEFT];
  w[LEFT]  = v[LEFT] / wheel_radius_;  // w = v / r
  v[RIGHT] = wheel_speed_cmd_[RIGHT];
  w[RIGHT] = v[RIGHT] / wheel_radius_;

  last_velocity_[LEFT]  = w[LEFT];
  last_velocity_[RIGHT] = w[RIGHT];

  wheel_l = w[LEFT]  * diff_time.toSec();
  wheel_r = w[RIGHT] * diff_time.toSec();

  if(isnan(wheel_l))
  {
    wheel_l = 0.0;
  }

  if(isnan(wheel_r))
  {
    wheel_r = 0.0;
  }

  last_position_[LEFT]  += wheel_l;
  last_position_[RIGHT] += wheel_r;

  delta_s     = wheel_radius_ * (wheel_r + wheel_l) / 2.0;
  delta_theta = wheel_radius_ * (wheel_r - wheel_l) / wheel_seperation_;

  // compute odometric pose
  odom_pose_[0] += delta_s * cos(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[1] += delta_s * sin(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[2] += delta_theta;

  // compute odometric instantaneouse velocity
  odom_vel_[0] = delta_s / diff_time.toSec();     // v
  odom_vel_[1] = 0.0;
  odom_vel_[2] = delta_theta / diff_time.toSec(); // w

  odom_.pose.pose.position.x = odom_pose_[0];
  odom_.pose.pose.position.y = odom_pose_[1];
  odom_.pose.pose.position.z = 0;
  odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose_[2]);

  // We should update the twist of the odometry
  odom_.twist.twist.linear.x  = odom_vel_[0];
  odom_.twist.twist.angular.z = odom_vel_[2];

  return true;
}

/*******************************************************************************
* Calculate the TF
*******************************************************************************/
void OdometryFake::update_tf(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom_.header;
  odom_tf.child_frame_id = odom_.child_frame_id;
  odom_tf.transform.translation.x = odom_.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_.pose.pose.position.z;
  odom_tf.transform.rotation = odom_.pose.pose.orientation;
}

/*******************************************************************************
* Update function
*******************************************************************************/
bool OdometryFake::update()
{
  ros::Time time_now = ros::Time::now();
  ros::Duration step_time = time_now - prev_update_time_;
  prev_update_time_ = time_now;

  // zero-ing after timeout
  if((time_now - last_cmd_vel_time_).toSec() > cmd_vel_timeout_)
  {
    wheel_speed_cmd_[LEFT]  = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
  }

  // odom
  update_odometry(step_time);
  odom_.header.stamp = time_now;
  odom_pub_.publish(odom_);

  // tf
  geometry_msgs::TransformStamped odom_tf;
  update_tf(odom_tf);
  tf_broadcaster_.sendTransform(odom_tf);

  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "fake_odometry_node");
  OdometryFake odomfake;

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    odomfake.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
