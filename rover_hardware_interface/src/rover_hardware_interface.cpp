#include <sstream>
#include <rover_hardware_interface/rover_hardware_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

namespace rover_hardware_interface
{
	RoverHardwareInterface::RoverHardwareInterface
	    (ros::NodeHandle& nh)
		: nh_(nh)
	{
		init();
		controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

		nh_.param<double>("hardware_interface/loop_hz", loop_hz_, 10.0);
		nh_.param<double>("mobile_base_controller/wheel_radius", wheel_radius_, 0.033);
		nh_.param<double>("mobile_base_controller/wheel_separation", wheel_separation_, 0.287);
		nh_.param<double>("hardware_interface/pid/p", motor_p_gain_, 0.1);
		nh_.param<double>("hardware_interface/pid/d", motor_d_gain_, 0.01);
		nh_.param<double>("hardware_interface/pid/i", motor_i_gain_, 0.01);
		nh_.param<double>("hardware_interface/max_throttle_speed", max_throttle_speed_, 3000); //rad/s
		nh_.param<bool>("sim_encoders", sim_, true);
		ROS_DEBUG_STREAM_NAMED("constructor","Using loop freqency of " << loop_hz_ << " hz");
		ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
		non_realtime_loop_ = nh_.createTimer(update_freq, &RoverHardwareInterface::update, this);

		//Suscribe to topic to get velocities in simulation from gazebo plugin
		joints_state_sub_ = nh_.subscribe("gazebo_rover/wheel_speed_read", 1, &RoverHardwareInterface::JointsStateCallback, this);

		//Suscribe to topics to get velocities from encoders provided by encoder_publisher.py
		left_encoder_sub_ = nh_.subscribe("encoder_left_wheel_speed_read", 1, &RoverHardwareInterface::LeftEncoderCallback, this);
		right_encoder_sub_ = nh_.subscribe("encoder_right_wheel_speed_read", 1, &RoverHardwareInterface::RightEncoderCallback, this);

		//Publish cmd topics
		//this is the topic that the libset_speed_joint_plugin.so plugin listen to
		left_wheel_vel_pub_ = nh.advertise<std_msgs::Float32>("gazebo_rover/left_wheel_speed_cmd", 1);
		right_wheel_vel_pub_ = nh.advertise<std_msgs::Float32>("gazebo_rover/right_wheel_speed_cmd", 1);

		last_position_[LEFT]    = 0.0;
		last_position_[RIGHT]   = 0.0;

		rovercpp::Motor motor1(motor_p_gain_,motor_d_gain_,motor_i_gain_);
		rovercpp::Motor motor2(motor_p_gain_,motor_d_gain_,motor_i_gain_);
		motors_[LEFT] = motor1;
		motors_[RIGHT] = motor2;

		//mavros publisher to send cmds to PX4
		mavros_msg_actuator_.header.frame_id = '0';
		mavros_msg_actuator_.group_mix = 0;
		mavros_pub_ = nh.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 1);

		ROS_INFO_NAMED("hardware_interface", "Loaded generic_hardware_interface.");
		if (sim_){
			ROS_INFO_NAMED("hardware_interface", "With simulated of encoders.");
		} else {
			ROS_INFO_NAMED("hardware_interface", "With hardware encoders.");
		}
	}

	RoverHardwareInterface::~RoverHardwareInterface()
	{
	}

	void RoverHardwareInterface::init()
	{
		//joint_mode_ = 3; // ONLY EFFORT FOR NOW
		// Get joint names
		nh_.getParam("hardware_interface/joints", joint_names_);
		if (joint_names_.size() == 0)
		{
		  ROS_FATAL_STREAM_NAMED("init","No joints found on parameter server for controller. Did you load the proper yaml file?");
		}
		num_joints_ = joint_names_.size();

		// Resize vectors
		joint_position_.resize(num_joints_);
		joint_velocity_.resize(num_joints_);
		joint_effort_.resize(num_joints_);
		joint_position_command_.resize(num_joints_);
		joint_velocity_command_.resize(num_joints_);
		joint_effort_command_.resize(num_joints_);

		_wheel_velocity.resize(num_joints_);
		_wheel_velocity_encoder.resize(num_joints_);


		// Initialize controller
		for (int i = 0; i < num_joints_; ++i)
		{

		  ROS_DEBUG_STREAM_NAMED("constructor","Loading joint name: " << joint_names_[i]);

		  // Create joint state interface
			JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
		  joint_state_interface_.registerHandle(jointStateHandle);

		  // Create position joint interface
			JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
			JointLimits limits;
 	   	SoftJointLimits softLimits;
			if (getJointLimits(joint_names_[i], nh_, limits) == false) {
				ROS_ERROR_STREAM("Cannot set joint limits for " << joint_names_[i]);
			} else {
				VelocityJointSoftLimitsHandle jointLimitsHandle(jointVelocityHandle, limits, softLimits);
				velocityJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
			}
		  velocity_joint_interface_.registerHandle(jointVelocityHandle);

		} //end loop joints

		registerInterface(&joint_state_interface_);
		registerInterface(&velocity_joint_interface_);
		registerInterface(&velocityJointSoftLimitsInterface);
	}

	void RoverHardwareInterface::update(const ros::TimerEvent& e)
	{
		_logInfo = "\n";
		_logInfo += "Joint Velocity Command:\n";
		for (int i = 0; i < num_joints_; i++)
		{
			std::ostringstream jointVelocityStr;
			jointVelocityStr << joint_velocity_command_[i];
			_logInfo += "  " + joint_names_[i] + ": " + jointVelocityStr.str() + "\n";
		}

		elapsed_time_ = ros::Duration(e.current_real - e.last_real);

		updateJointsFromHardware(elapsed_time_);
		controller_manager_->update(ros::Time::now(), elapsed_time_);
		writeCommandsToHardware(elapsed_time_);

		//ROS_INFO_STREAM(_logInfo);
	}

	/**
	* Pull latest speed measurements from encoders, and store in joint structure for ros_control
	*/
	void RoverHardwareInterface::updateJointsFromHardware(ros::Duration elapsed_time)
	{
		// get joints velocities from commands
		if (!sim_){
			getJointVelocities();
		}

		_logInfo += "Joint State:\n";
		for (int i = 0; i < num_joints_; i++)
		{

			joint_velocity_[i] = readVelocity(i);
			motors_[i].setSpeed(joint_velocity_[i]); //set the speed of the motor so it uses last know speed for PID
			// std::cout << "joint_velocity_[i] :" << joint_velocity_[i] <<  std::endl;
			joint_position_[i] = calculatePosition(i, elapsed_time); //we need to set this for odom calculation

			std::ostringstream jointVelocityStr;
			jointVelocityStr << joint_velocity_[i];
			_logInfo += "  " + joint_names_[i] + ": " + jointVelocityStr.str() + "\n";

		}
	}

	/**
  * Get latest velocity commands from ros_control via joint structure, and send to PX4
  */
	void RoverHardwareInterface::writeCommandsToHardware(ros::Duration elapsed_time)
	{
		velocityJointSoftLimitsInterface.enforceLimits(elapsed_time);

		_logInfo += "Joint Velocity Command:\n";


		// Get the cmd from ros_control
		double diff_vel_speed_left_raw = joint_velocity_command_[LEFT]; //these are angular
		double diff_vel_speed_right_raw = joint_velocity_command_[RIGHT];

		// we apply pid to these cmds before sending them to gazebo
		double diff_vel_speed_left = motors_[LEFT].controlPID(diff_vel_speed_left_raw);
		double diff_vel_speed_right = motors_[RIGHT].controlPID(diff_vel_speed_right_raw);

		// For simulation in gazebo
		std_msgs::Float32 left_wheel_vel_msg;
		left_wheel_vel_msg.data = diff_vel_speed_left;
		left_wheel_vel_pub_.publish(left_wheel_vel_msg);
		std_msgs::Float32 right_wheel_vel_msg;
		right_wheel_vel_msg.data = diff_vel_speed_right;
		right_wheel_vel_pub_.publish(right_wheel_vel_msg);

		std::ostringstream leftVelocityCmdStr;
		leftVelocityCmdStr << joint_velocity_command_[LEFT];
		_logInfo += "  " + joint_names_[LEFT] + ": " + leftVelocityCmdStr.str() + "\n";
		std::ostringstream rightVelocityCmdStr;
		rightVelocityCmdStr << joint_velocity_command_[RIGHT];
		_logInfo += "  " + joint_names_[RIGHT] + ": " + rightVelocityCmdStr.str() + "\n";

		// double diff_linear_speed_left = angularToLinear(diff_vel_speed_left);
		// double diff_linear_speed_right = angularToLinear(diff_vel_speed_right);

		//Publish through mavros
		double vel_cmd = wheel_radius_*(diff_vel_speed_left + diff_vel_speed_right)/2;
		double yawrate = wheel_radius_*(diff_vel_speed_right - diff_vel_speed_left)/wheel_separation_;
		//we need to remap before sending to mavros because the output should be between -1 and 1, 1 is 3.3V=?rad/s
		mavros_msg_actuator_.controls[0] = yawrate/max_throttle_speed_; //this is the roll channel
		mavros_msg_actuator_.controls[1] = vel_cmd/angularToLinear(max_throttle_speed_); //this is the pitch channel
		mavros_pub_.publish(mavros_msg_actuator_);

	}


	void RoverHardwareInterface::JointsStateCallback(const sensor_msgs::JointState& msg)
	{
		//In simulation, we should not enter here if gazebo doesnt publish any msg (from plugin), so its ok
		// std::cout << "JointsStateCallback msg.velocity[0] :" << msg.velocity[0] <<  std::endl;
		_wheel_velocity[LEFT] = msg.velocity[LEFT];
		_wheel_velocity[RIGHT] = msg.velocity[RIGHT];
	}

	void RoverHardwareInterface::LeftEncoderCallback(const std_msgs::Float32& msg)
	{
		_wheel_velocity_encoder[LEFT] = msg.data;
	}

	void RoverHardwareInterface::RightEncoderCallback(const std_msgs::Float32& msg)
	{
		_wheel_velocity_encoder[RIGHT] = msg.data;
	}

	void RoverHardwareInterface::getJointVelocities(void)
	{
		//take from encoders here
		// _wheel_velocity[LEFT] = joint_velocity_command_[LEFT];
		// _wheel_velocity[RIGHT] = joint_velocity_command_[RIGHT];
		// _wheel_velocity[LEFT] = motors_[LEFT].getSpeed();
		// _wheel_velocity[RIGHT] = motors_[RIGHT].getSpeed();
		_wheel_velocity[LEFT] = _wheel_velocity_encoder[LEFT];
		_wheel_velocity[RIGHT] = _wheel_velocity_encoder[RIGHT];
	}

	double RoverHardwareInterface::readVelocity(const int& joint_idx) const
	{
		return _wheel_velocity[joint_idx];
	}

	double RoverHardwareInterface::calculatePosition(const int& joint_idx, ros::Duration elapsed_time)
	{
		last_position_[joint_idx]  += _wheel_velocity[joint_idx]*elapsed_time.toSec();
		return last_position_[joint_idx];
	}

	/**
	* RobotHW provides velocity command in rad/s, PX4 needs m/s,
	*/
	double RoverHardwareInterface::angularToLinear(const double &angle_vel) const
	{
		return angle_vel * wheel_radius_;
	}

} // namespace rover_hardware_interface
