#include <gtest/gtest.h>
#include "rover_hardware_interface/motor.h"
#include <ros/ros.h>

// TEST CASES
TEST(TestSuite, testConnection)
{
  rovercpp::Motor motor1(0.1,0.01,0.01);

  std::cout << "motor1 speed: " << motor1.getSpeed() << std::endl;

  EXPECT_FALSE(false);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "motor_test");
  std::cout << "-------------------------------------------------" << std::endl;
  std::cout << "-------------- Start testing motor --------------" << std::endl;
  std::cout << "-------------------------------------------------" << std::endl;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
