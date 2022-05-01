#include <rover_hardware_interface/rover_hardware_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rover_hardware_interface");
  ros::CallbackQueue ros_queue;

  ros::NodeHandle nh;
  nh.setCallbackQueue(&ros_queue);
  rover_hardware_interface::RoverHardwareInterface rover(nh);

  ros::MultiThreadedSpinner spinner(0);
  spinner.spin(&ros_queue);
  return 0;
}
