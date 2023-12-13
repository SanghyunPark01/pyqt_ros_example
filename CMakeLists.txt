cmake_minimum_required(VERSION 3.0.2)
project(pyqt_ros_example)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
  DIRECTORY srv
  FILES
  set_img_flag.srv
  drawing_type.srv
  start_calculate_flag.srv
  start_drawing_flag.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pyqt_ros_example
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/main_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/service_test_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)