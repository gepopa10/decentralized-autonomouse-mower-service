[![License](https://img.shields.io/badge/license-HUMANITAS-blue)](https://192.168.0.172/robotics/rover/foundation/posix_pkg/-/blob/development/LICENSE)
[![Contributors](https://img.shields.io/badge/contributors-3-orange)](https://192.168.0.172/robotics/rover/foundation/posix_pkg/-/project_members)

# rover_hardware_interface

To launch: \
`roslaunch rover_hardware_interface rover_controller.launch`

**Simulation**
====

This package can be run with the `turtlebot3_world.launch` by installing in the workspace:
- (https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
- (https://github.com/ROBOTIS-GIT/turtlebot3)
```
cd /workspace/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
```

In simulation works with `set_speed_joint_cpp` plugin.

To add this plugin:

In turtlebot3/turtlebot3_description/urdf/turtle3_waffle.gazebo.xacro \
Comment:
`<plugin name="turtlebot3_waffle_controller" filename="libgazebo_ros_diff_drive.so">`
and add:
```
<gazebo>
  <plugin name="set_speed_joint_plugin" filename="libset_speed_joint_plugin.so">
    <namespace_model>gazebo_rover</namespace_model>
    <activate_pid_control>yes</activate_pid_control>                
    <wheel_kp>0.03</wheel_kp> <!-- needs to be small for turtlebot3 0.02-->
    <wheel_ki>0.000001</wheel_ki>
    <wheel_kd>0.000001</wheel_kd>
  </plugin>
</gazebo>
```
In simulation we can add friction to a joint to test controller:
```
<dynamics damping="0.0" friction="1.0"/>
```

**Hardware**
====

To see how value are read by an encoder connected to gpio and published on a ros topic:
```
roslaunch rover_hardware_interface encoder.launch
```
Params to set:
- encoder_topic : topic where the speed is published
- encoder_read_frequency : desired publishing frequency
- encoder_gpio_A_num : pin for magnet 1
- encoder_gpio_B_num : pin for magnet 2
- motor_N : number of pulses per revolution

Compile motor.cpp for test with real encoders:
```
g++ -I ~/workspace/src/rover_hardware_interface/include motor.cpp
./a.out
```
Run test in rover_hardware_interface:
```
catkin_make run_tests_rover_hardware_interface
```
or:
```
roslaunch rover_hardware_interface motor.test
```
or:
```
rostest --text rover_hardware_interface motor.test
```
> Note: Currently not supported because we use python nodes to publish the encoders data

**Explanations**
====

In /config/controllers.yaml, two controllers are launch:
- base_joint_publisher that publishes joints_states
- mobile_base_controller that is specifically a diff_drive_controller/DiffDriveController which listen to mobile_base_controller/cmd_vel and publishes odometry to /mobile_base_controller/odom

The rover_hardware_interface inherits from `hardware_interface::RobotHW` and we need to provide the velocity of the joint to the interface while it will give use the required commanded velocity to send.
Specifically, in `RoverHardwareInterface::updateJointsFromHardware` the `joint_velocity_` is changed based on the read joint state. `joint_velocity_` is provided to a `JointStateHandle` in rover_hardware_interface.cpp. Then this `JointStateHandle` is then given to a `JointHandle` that will update the `joint_velocity_command_` that we need to give to motors. We send these commands to hardware in `RoverHardwareInterface::writeCommandsToHardware`. These commands are open loop, so we close the loop with a PID controller using the velocity read by the encoder and we output corrected velocity commands using the Motor class.

For the current implementation with PX4, we actually send commands via mavros in `RoverHardwareInterface::writeCommandsToHardware`. Since in mavros the actuator control message expects values between -1 and -1, we need to map the `joint_velocity_command_` that will be in rad/s to this mavros scale. This is done through the max_throttle_speed param. Therefore, we need to find when we send 3.3V to the motors (actuator control message to +1), what is the speed of the motor.

The outputs of `joint_velocity_command_` can also be viewed in rad/s over the topics `gazebo_rover/left_wheel_speed_cmd` and `gazebo_rover/right_wheel_speed_cmd` respectivelly and not just when the package is ran with gazebo.

Also, an odometry topic is provided by the diff_drive_controller but `joint_position_` also need to be provided to the `JointStateHandle`. Since our encoders do not provide position, we calculate the position of the joints using the velocity and the elapsed time.

The launch file `rover_controller.launch` can be run using simulation parameter set to true or false.
- When set to __true__, the encoders node will not run and a gazebo simulation fo a robot like `turtlebot3_world.launch` should be running with the `set_speed_joint_cpp` plugin. This plugin will listen to the topics `gazebo_rover/left_wheel_speed_cmd` and `gazebo_rover/right_wheel_speed_cmd` to control the wheel joints of the simulated robot in gazebo using PID control. It will also output a fake encoder velocity data to the `gazebo_rover/wheel_speed_read` topic that will be used to update the `joint_velocity_` in rover_hardware_interface.cpp.
- When set to __false__, the encoders node will be launched and will provide the speed of the motors. They will publish to the topics `left_encoder_speed_gpio` and `left_encoder_speed_gpio` that the rover_hardware_interface.cpp suscribes to to update `joint_velocity_`.

**Ressources**
====
- [ROS differential drive controller](http://wiki.ros.org/diff_drive_controller)
- [Sample 1 of robot hardware interface](https://github.com/eborghi10/my_ROS_mobile_robot/blob/e04acfd3e7eb4584ba0aab8a969a74d6c30eed34/my_robot_base/include/my_robot_hw_interface.h)
- [Sample 2 of robot hardware interface](https://github.com/husky/husky/blob/kinetic-devel/husky_base/include/husky_base/husky_hardware.h)
- [Explanation of hardware interface and template used for this package](https://medium.com/@slaterobotics/how-to-implement-ros-control-on-a-custom-robot-748b52751f2e)
- [Differential drive controller configuration](https://www.theconstructsim.com/ros-qa-126-how-to-configure-the-differential-drive-ros-controller/)
- [Differential drive controller physical robot](https://answers.ros.org/question/307520/what-are-the-steps-to-use-diff_drive_controller-in-a-physical-robot/)
- [set_speed_joint_cpp plugin template](https://bitbucket.org/theconstructcore/set_speed_joint_cpp/src/master/)

**Encoders**
===
Diverse filters where tested on the raw speed data of a magnetic encoder.

Specifically, a 1D extended kalman filter and the savgol polynomial filter from the python scipy library were put to test.

As the follow plot suggests, the best filtering is obtain with the savgol filtering with a window size of 23 and a polynomial order of zero.

![Alt text](scripts/encoder_data/filters.png?raw=true "Diverse filters on encoder raw data")
