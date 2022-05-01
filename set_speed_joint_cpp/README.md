# set_speed_joint_cpp

This is a plugin that should also be built with the rover_hardware_interface when testing the hardware interface in simulation with gazebo.

It controls the rover joints velocities using PID.

This plugin listens to:
- `/gazebo_rover/left_wheel_speed_cmd`
- `/gazebo_rover/right_wheel_speed_cmd`
and gives joints velocities like a fake encoder in this topic:
- `/gazebo_rover/wheel_speed_read`

To add this plugin:

In turtlebot3/turtlebot3_description/urdf/turtle3_waffle.gazebo.xacro \
Comment:
```
<plugin name="turtlebot3_waffle_controller" filename="libgazebo_ros_diff_drive.so">
```
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
**Ressources**
====
- [set_speed_joint_cpp plugin template](https://bitbucket.org/theconstructcore/set_speed_joint_cpp/src/master/)
