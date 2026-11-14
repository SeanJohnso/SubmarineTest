cmake_minimum_required(VERSION 3.8)
project(jit_task)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(jit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robotx_comm REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(waypoint_listener_node src/waypoint_listener_node.cpp)
ament_target_dependencies(waypoint_listener_node
  rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros jit_msgs)

add_executable(task_node src/task_node.cpp)
target_include_directories(task_node PRIVATE include)
ament_target_dependencies(task_node
  rclcpp std_msgs geometry_msgs sensor_msgs jit_msgs robotx_comm)
# Same pattern as jit_control's local_guided_node.
if(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(task_node yaml-cpp::yaml-cpp)
else()
  target_include_directories(task_node PRIVATE ${YAML_CPP_INCLUDE_DIR})
  target_link_libraries(task_node ${YAML_CPP_LIBRARIES})
endif()

install(TARGETS waypoint_listener_node task_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
