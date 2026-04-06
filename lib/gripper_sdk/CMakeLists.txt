add_library(gripper_sdk
	src/gripper_sdk.cpp
)

target_include_directories(gripper_sdk PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gripper_sdk
  rclcpp
)

# --- Install ---
install(TARGETS gripper_sdk
  EXPORT export_gripper_sdk
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)