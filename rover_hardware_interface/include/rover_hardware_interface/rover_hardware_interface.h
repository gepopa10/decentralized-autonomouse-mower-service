#ifndef ROS_CONTROL__ROVER_HARDWARE_INTERFACE_H
#define ROS_CONTROL__ROVER_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkState.h>
#include <rover_hardware_interface/rover_hardware.h>
#include <ros/callback_queue.h>
#include <string>
#include <rover_hardware_interface/motor.h>
#include <mavros_msgs/ActuatorControl.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::VelocityJointSoftLimitsHandle;
using joint_limits_interface::VelocityJointSoftLimitsInterface;

#define LEFT                            0
#define RIGHT                           1

namespace rover_hardware_interface
{
	class RoverHardwareInterface: public rover_hardware_interface::RoverHardware
	{
		public:
			RoverHardwareInterface(ros::NodeHandle& nh);
			~RoverHardwareInterface();
			void init();
			void update(const ros::TimerEvent& e);
			void updateJointsFromHardware(ros::Duration elapsed_time); //read
			void writeCommandsToHardware(ros::Duration elapsed_time); //write

		protected:
			ros::NodeHandle nh_;
			ros::Timer non_realtime_loop_;
			ros::Duration control_period_;
			ros::Duration elapsed_time_;

			VelocityJointInterface velocityJointInterface;
			VelocityJointSoftLimitsInterface velocityJointSoftLimitsInterface;

			double loop_hz_;
			double motor_p_gain_;
			double motor_d_gain_;
			double motor_i_gain_;

			boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
			std::string _logInfo;

			std::vector<double> _wheel_velocity;
			std::vector<double> _wheel_velocity_encoder;

		private:
			void JointsStateCallback(const sensor_msgs::JointState& msg);
			double readVelocity(const int& joint_idx) const;
			double angularToLinear(const double &angle_vel) const;
			void getJointVelocities(void);
			double calculatePosition(const int& joint_idx, ros::Duration elapsed_time);
			void LeftEncoderCallback(const std_msgs::Float32& msg);
			void RightEncoderCallback(const std_msgs::Float32& msg);

			ros::Subscriber joints_state_sub_;
			ros::Publisher left_wheel_vel_pub_;
			ros::Publisher right_wheel_vel_pub_;
			ros::Publisher mavros_pub_;
			ros::Subscriber left_encoder_sub_;
			ros::Subscriber right_encoder_sub_;

			double wheel_radius_;
			double wheel_separation_;
			double max_throttle_speed_;
			bool sim_;
			double last_position_[2];
			rovercpp::Motor motors_[2];

			mavros_msgs::ActuatorControl mavros_msg_actuator_;
	};

}

#endif
