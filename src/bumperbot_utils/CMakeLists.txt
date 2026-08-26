cmake_minimum_required(VERSION 3.8)
project(bumperbot_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(twist_mux_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

include_directories(include)

add_library(safety_state STATIC src/safety_state.cpp)

add_executable(trajectory_drawer src/trajectory_drawer.cpp)
ament_target_dependencies(trajectory_drawer rclcpp nav_msgs geometry_msgs)

add_executable(safety_stop src/safety_stop.cpp)
ament_target_dependencies(safety_stop rclcpp sensor_msgs std_msgs rclcpp_action twist_mux_msgs visualization_msgs)
target_link_libraries(safety_stop safety_state)

add_executable(joy_watchdog src/joy_watchdog.cpp)
ament_target_dependencies(joy_watchdog rclcpp sensor_msgs std_msgs)

install(PROGRAMS
  ${PROJECT_NAME}/trajectory_drawer.py
  ${PROJECT_NAME}/safety_stop.py
  ${PROJECT_NAME}/find_phantom2.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  trajectory_drawer
  safety_stop
  joy_watchdog
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_safety_state test/test_safety_state.cpp)
  target_link_libraries(test_safety_state safety_state)
endif()

ament_package()
