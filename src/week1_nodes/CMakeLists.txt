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

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorReading.msg"
  "srv/GetSensorReading.srv"
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
install(TARGETS sensor_publisher
  DESTINATION lib/${PROJECT_NAME})

add_executable(sensor_subscriber src/sensor_subscriber.cpp)
target_link_libraries(sensor_subscriber
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)
install(TARGETS sensor_subscriber
  DESTINATION lib/${PROJECT_NAME})

add_executable(custom_publisher src/custom_publisher.cpp)
target_include_directories(custom_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
target_link_libraries(custom_publisher
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${cpp_typesupport_target}
)
install(TARGETS custom_publisher
  DESTINATION lib/${PROJECT_NAME})
add_executable(sensor_service src/sensor_service.cpp)
target_link_libraries(sensor_service
  rclcpp::rclcpp
  ${cpp_typesupport_target}
)
install(TARGETS sensor_service
  DESTINATION lib/${PROJECT_NAME})
# ament_package MUST be last
ament_package()
