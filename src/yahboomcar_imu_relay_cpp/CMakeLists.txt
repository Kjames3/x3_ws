cmake_minimum_required(VERSION 3.8)
project(yahboomcar_imu_relay_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(livox_imu_relay_cpp src/livox_imu_relay.cpp)
target_compile_features(livox_imu_relay_cpp PRIVATE cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # Match the separate multiply and add operations in gyro_bias.py on ARM too.
  target_compile_options(livox_imu_relay_cpp PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
endif()
ament_target_dependencies(livox_imu_relay_cpp rclcpp sensor_msgs)

install(TARGETS livox_imu_relay_cpp DESTINATION lib/${PROJECT_NAME})
ament_package()
