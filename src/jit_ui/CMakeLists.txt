cmake_minimum_required(VERSION 3.8)
project(jit_ui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(jit_msgs REQUIRED)
find_package(Threads REQUIRED)

# rpi_ws281x drives the LED panel over PWM+DMA. Installed system-wide via its
# own build (not a ROS package / not colcon-built), so located manually - see
# README.MD for build instructions. Same pattern jit_safety uses for xcrsf.
find_library(WS2811_LIBRARY ws2811 REQUIRED)
# make install puts the headers under include/ws2811/, not include/ directly.
find_path(WS2811_INCLUDE_DIR ws2811.h PATH_SUFFIXES ws2811 REQUIRED)

add_executable(led_driver_node src/led_driver_node.cpp)
target_include_directories(led_driver_node PRIVATE ${WS2811_INCLUDE_DIR})
ament_target_dependencies(led_driver_node rclcpp jit_msgs)
target_link_libraries(led_driver_node ${WS2811_LIBRARY} Threads::Threads)

install(TARGETS led_driver_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
