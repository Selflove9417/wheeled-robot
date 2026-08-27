cmake_minimum_required(VERSION 3.8)
project(bbot_real)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Include directories
include_directories(include)

# ==================== Library: bbot_real_core ====================
# Shared library for kinematics, PID, CAN interface, motor drivers
add_library(bbot_real_core STATIC
    src/can_interface.cpp
    src/kinematics.cpp
    src/pid.cpp
    src/joint_motor_driver.cpp
    src/wheel_motor_driver.cpp
)

target_include_directories(bbot_real_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bbot_real_core
    rclcpp
    sensor_msgs
    geometry_msgs
    std_msgs
    tf2
)

# ==================== Executables ====================

# PID Balance Controller
add_executable(pid_balance_controller
    src/pid_balance_controller.cpp
)
target_link_libraries(pid_balance_controller bbot_real_core)
ament_target_dependencies(pid_balance_controller
    rclcpp sensor_msgs geometry_msgs std_msgs tf2
)

# LQR Balance Controller
add_executable(lqr_balance_controller
    src/lqr_balance_controller.cpp
)
target_link_libraries(lqr_balance_controller bbot_real_core)
ament_target_dependencies(lqr_balance_controller
    rclcpp sensor_msgs geometry_msgs std_msgs tf2
)

# 关节电机零点标定工具
add_executable(calibrate_motors
    src/calibrate_motors.cpp
    src/can_interface.cpp
    src/joint_motor_driver.cpp
)
target_include_directories(calibrate_motors PRIVATE include)

add_executable(single_leg_test_node 
    src/single_leg_test_node.cpp
)

target_link_libraries(single_leg_test_node bbot_real_core)
ament_target_dependencies(single_leg_test_node
    rclcpp
    sensor_msgs
    geometry_msgs
    std_msgs
    tf2
)

add_executable(dual_leg_test_node 
    src/dual_leg_test_node.cpp
)

target_link_libraries(dual_leg_test_node bbot_real_core)
ament_target_dependencies(dual_leg_test_node
    rclcpp
    sensor_msgs
    geometry_msgs
    std_msgs
    tf2
)

add_executable(wheel_motor_test_node
    src/wheel_motor_test_node.cpp
)
target_link_libraries(wheel_motor_test_node bbot_real_core)
ament_target_dependencies(wheel_motor_test_node
    rclcpp
    std_msgs
)



# ==================== Install ====================
install(TARGETS
    pid_balance_controller
    lqr_balance_controller
    calibrate_motors
    single_leg_test_node
    dual_leg_test_node
    wheel_motor_test_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
