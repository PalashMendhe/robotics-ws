cmake_minimum_required(VERSION 3.20)
project(week1_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorReading.msg"
  "srv/GetSensorReading.srv"
  "action/Calibrate.action"
  DEPENDENCIES std_msgs
)


rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_runtime)

# Executables
add_executable(sensor_publisher src/sensor_publisher.cpp)
target_link_libraries(sensor_publisher
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)
add_executable(sensor_subscriber src/sensor_subscriber.cpp)
target_link_libraries(sensor_subscriber
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)
add_executable(custom_publisher src/custom_publisher.cpp)
target_include_directories(custom_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
target_link_libraries(custom_publisher
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${cpp_typesupport_target}
)
add_executable(sensor_service src/sensor_service.cpp)
target_link_libraries(sensor_service
  rclcpp::rclcpp
  ${cpp_typesupport_target}
)

# ament_package MUST be last
add_executable(calibration_server src/calibration_server.cpp)
target_include_directories(calibration_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
target_link_libraries(calibration_server
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${cpp_typesupport_target}
)
add_dependencies(calibration_server ${PROJECT_NAME}__rosidl_typesupport_cpp)

add_executable(calibration_client src/calibration_client.cpp)
target_include_directories(calibration_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp> 
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(calibration_client
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${action_typesupport_target}
)
add_dependencies(calibration_client ${PROJECT_NAME}__rosidl_typesupport_cpp)
install(TARGETS 
  sensor_publisher 
  sensor_subscriber 
  custom_publisher 
  sensor_service 
  calibration_server 
  calibration_client
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
