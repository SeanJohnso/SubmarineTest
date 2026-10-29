# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target ping360_sonar_msgs::ping360_sonar_msgs
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${ping360_sonar_msgs_TARGETS}.
if(ping360_sonar_msgs_TARGETS AND NOT TARGET ping360_sonar_msgs::ping360_sonar_msgs)
  add_library(ping360_sonar_msgs::ping360_sonar_msgs INTERFACE IMPORTED)
  set_target_properties(ping360_sonar_msgs::ping360_sonar_msgs PROPERTIES
    INTERFACE_LINK_LIBRARIES "${ping360_sonar_msgs_TARGETS}")
endif()
