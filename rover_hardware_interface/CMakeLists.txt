cmake_minimum_required(VERSION 2.8.3)
project(rover_hardware_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
  roscpp
  control_msgs
  trajectory_msgs
  actionlib
  pluginlib
  transmission_interface
  urdf
  control_toolbox
  joint_limits_interface
  mavros_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    hardware_interface
    controller_manager
    roscpp
    control_msgs
    trajectory_msgs
    pluginlib
    transmission_interface
    urdf
    control_toolbox
    joint_limits_interface
    mavros_msgs
  LIBRARIES
    rover_hardware_interface
    motor
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

add_library(motor src/motor.cpp)
target_link_libraries(motor
	${catkin_LIBRARIES})

add_library(rover_hardware_interface src/rover_hardware_interface.cpp)
target_link_libraries(rover_hardware_interface
  motor
	${catkin_LIBRARIES})

add_executable(rover_hardware_interface_node src/rover_hardware_interface_node.cpp)
add_dependencies(rover_hardware_interface_node rover_hardware_interface)

target_link_libraries(rover_hardware_interface_node
	rover_hardware_interface
	${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_motor test/motor.test test/motortest.cpp)
  target_link_libraries(test_motor ${catkin_LIBRARIES} motor)
endif()

catkin_install_python(PROGRAMS scripts/encoder_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
