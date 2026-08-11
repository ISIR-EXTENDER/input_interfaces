cmake_minimum_required(VERSION 3.10)
project(joystick_mapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(signal_processing REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/joystick_mapper.cpp
)
target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${PROJECT_NAME}_lib
  geometry_msgs
  rclcpp
  sensor_msgs
  signal_processing
  std_msgs
)

add_executable(joystick_mapper_node
  src/main.cpp
)
target_compile_features(joystick_mapper_node PUBLIC cxx_std_17)
target_link_libraries(joystick_mapper_node
  ${PROJECT_NAME}_lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY bringup/config bringup/launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    ${PROJECT_NAME}_lib
    joystick_mapper_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  geometry_msgs
  rclcpp
  sensor_msgs
  signal_processing
  std_msgs
)

ament_package()
