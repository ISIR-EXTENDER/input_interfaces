cmake_minimum_required(VERSION 3.8)
project(hand_joystick_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(signal_processing REQUIRED)

add_library(hand_joystick_control_mapping STATIC
  src/control_mapping.cpp
)

ament_target_dependencies(hand_joystick_control_mapping
  Eigen3
  signal_processing
)

target_include_directories(hand_joystick_control_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(hand_joystick_control_mapping
  Eigen3::Eigen
)

target_compile_features(hand_joystick_control_mapping PUBLIC cxx_std_17)

add_library(hand_joystick_parameter_descriptors STATIC
  src/parameter_descriptors.cpp
)

ament_target_dependencies(hand_joystick_parameter_descriptors
  rcl_interfaces
)

target_include_directories(hand_joystick_parameter_descriptors PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(hand_joystick_parameter_descriptors PUBLIC cxx_std_17)

# Add the executable for the hand joystick interface node
add_executable(hand_joystick_node
  src/main.cpp
  src/hand_joystick_interface.cpp
)

# Package dependencies
ament_target_dependencies(hand_joystick_node
  rclcpp
  rcl_interfaces
  geometry_msgs
  std_msgs
  sensor_msgs
  visualization_msgs
  Eigen3
  signal_processing
)

target_include_directories(hand_joystick_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(hand_joystick_node
  Eigen3::Eigen
  hand_joystick_control_mapping
  hand_joystick_parameter_descriptors
)

target_compile_features(hand_joystick_node PUBLIC c_std_99 cxx_std_17)

# Install targets
install(TARGETS
  hand_joystick_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES LICENSE
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(
    test_control_mapping
    test/test_control_mapping.cpp
  )
  target_link_libraries(
    test_control_mapping
    hand_joystick_control_mapping
  )

  ament_add_gtest(
    test_parameter_descriptors
    test/test_parameter_descriptors.cpp
  )
  ament_target_dependencies(
    test_parameter_descriptors
    rcl_interfaces
  )
  target_link_libraries(
    test_parameter_descriptors
    hand_joystick_parameter_descriptors
  )

  ament_add_pytest_test(
    test_overlay_configuration
    test/test_overlay_configuration.py
  )
  ament_add_pytest_test(
    test_launch_contracts
    test/test_launch_contracts.py
  )
  ament_add_pytest_test(
    test_hand_pipeline
    test/test_hand_pipeline.py
    TIMEOUT 90
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
