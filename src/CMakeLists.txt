cmake_minimum_required(VERSION 2.8.3)
project(pidrone_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  diagnostic_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  RC.msg
  Mode.msg
  Battery.msg
  State.msg
  StateGroundTruth.msg
  TagDetection.msg
  TagMap.msg
  AnchorLayout.msg
  PeerState.msg
  FlightReadiness.msg
  MspImu.msg
  IntegratedImu.msg
  EkfVerticalTrace.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(${catkin_INCLUDE_DIRS})

add_subdirectory(native)

catkin_install_python(PROGRAMS
  perception/apriltag_detection_drone
  perception/apriltag_mapping_node
  localization/uwb_localization_node
  localization/fusion_localization_node
  boot/rosbridge_websocket.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
