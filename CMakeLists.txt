cmake_minimum_required(VERSION 3.8)
project(grippers)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake         REQUIRED)
find_package(rclcpp              REQUIRED)
find_package(tf2_ros             REQUIRED)
find_package(grippers_interfaces REQUIRED)
find_package(sensor_msgs         REQUIRED)

# --- TCP client library ---
add_subdirectory(lib)

# --- Gripper node executable ---
add_executable(grippers_node
  src/grippers_node.cpp
)

target_link_libraries(grippers_node
  gripper_sdk
  grippers_interfaces::grippers_interfaces__rosidl_typesupport_cpp
)

ament_target_dependencies(grippers_node
  rclcpp
  grippers_interfaces
  sensor_msgs
)

# --- Install ---
install(TARGETS grippers_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()