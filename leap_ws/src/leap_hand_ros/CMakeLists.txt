cmake_minimum_required(VERSION 3.5)
project(leap_hand_ros)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate services from srv/ folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LeapVelocity.srv"
  "srv/LeapPosition.srv"
  "srv/LeapEffort.srv"
  "srv/LeapPosVelEff.srv"
  DEPENDENCIES std_msgs
)

# Install Python package (the leap_hand/ folder)
ament_python_install_package(leap_hand
  PACKAGE_DIR "leap_hand"
)


# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(
  PROGRAMS leap_hand/leaphand_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME leaphand_node
)

install(
  PROGRAMS leap_hand/tcp_leap_receiver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tcp_leap_receiver
)



ament_package()

