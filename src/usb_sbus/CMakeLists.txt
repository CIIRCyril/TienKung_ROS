cmake_minimum_required(VERSION 3.8)
project(usb_sbus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(${PROJECT_NAME}_node
  src/CerealPort.cpp
  src/usbSbus.cpp
  src/sbus_main.cpp
)

target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
