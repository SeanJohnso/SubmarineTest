cmake_minimum_required(VERSION 3.8)
project(estop)

# xcrsf's README examples use C++23 features (structured bindings on
# aggregate return types, etc). If your compiler can't do -std=c++23,
# drop this to 20 and see if it still builds.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# xcrsf is installed system-wide via its own `sudo make install`
# (not a ROS package / not colcon-built), so we locate it manually.
find_library(XCRSF_LIBRARY xcrsf REQUIRED)
find_path(XCRSF_INCLUDE_DIR xcrsf/crossfire.h REQUIRED)

# libgpiod — install with: sudo apt install libgpiod-dev
find_library(GPIOD_LIBRARY gpiod REQUIRED)
find_path(GPIOD_INCLUDE_DIR gpiod.h REQUIRED)

add_executable(crsf_channel_node src/crsf_channel_node.cpp)
target_include_directories(crsf_channel_node PRIVATE ${XCRSF_INCLUDE_DIR})
ament_target_dependencies(crsf_channel_node rclcpp std_msgs)
target_link_libraries(crsf_channel_node ${XCRSF_LIBRARY})

add_executable(gpio_estop_node src/gpio_estop_node.cpp)
target_include_directories(gpio_estop_node PRIVATE ${GPIOD_INCLUDE_DIR})
ament_target_dependencies(gpio_estop_node rclcpp std_msgs)
target_link_libraries(gpio_estop_node ${GPIOD_LIBRARY})

install(TARGETS
  crsf_channel_node
  gpio_estop_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()