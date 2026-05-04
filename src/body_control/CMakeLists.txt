cmake_minimum_required(VERSION 3.8)
project(body_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bodyctrl_msgs REQUIRED)
find_package(fast_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

if(UNIX AND NOT APPLE)
  set(OS "linux")
  set(OS_LIBS pthread rt)
endif()

include_directories(
  src/common
  src/plugins/body_control/soem_master
  src/plugins/body_control/soem_master/soem/app
  src/plugins/body_control/soem_master/soem/soem
  src/plugins/body_control/soem_master/soem/osal
  src/plugins/body_control/soem_master/soem/osal/${OS}
  src/plugins/body_control/soem_master/soem/oshw/${OS}
  src/plugins/body_control/devices/rm_imu
  ${EIGEN3_INCLUDE_DIR}
)

include_directories(${PROJECT_SOURCE_DIR}/src/plugins/xsens_imu/xsens/include/xsense)
include_directories(${PROJECT_SOURCE_DIR}/src/plugins/xsens_imu/xsens/include/xsense/xscommon)
include_directories(${PROJECT_SOURCE_DIR}/src/plugins/xsens_imu/xsens/include/xsense/xscontroller)
include_directories(${PROJECT_SOURCE_DIR}/src/plugins/xsens_imu/xsens/include/xsense/xsensdeviceapi)
include_directories(${PROJECT_SOURCE_DIR}/src/plugins/xsens_imu/xsens/include/xsense/xstypes)
include_directories(${PROJECT_SOURCE_DIR}/src/plugins/vn100_imu/vn100/include)

link_directories(${PROJECT_SOURCE_DIR}/src/plugins/xsens_imu/xsens/lib)

add_library(glog_init src/common/glog/GlogInitializer.cpp)
target_link_libraries(glog_init glog)

add_library(bodyctrl_plugin SHARED
  src/plugins/body_control/BodyControlPlugin.cpp
  src/plugins/body_control/devices/motor/MotorDevice.cpp
  src/plugins/body_control/devices/motor/MotorDeviceManager.cpp
  src/plugins/body_control/devices/motor/math_ops.cpp
  src/plugins/body_control/devices/rm_imu/RmImuDevice.cpp
  src/plugins/body_control/devices/rm_imu/ubt_hw/filters/complementary_filter.cpp
  src/plugins/body_control/devices/rm_imu/ubt_hw/filters/lp_filter.cpp
  src/plugins/body_control/devices/rm_imu/ubt_hw/drive/imu/rm_imu.cpp
  src/plugins/body_control/devices/xsens_imu/base/XsensImuDevice.cpp
  src/plugins/body_control/devices/xsens_imu/base/mode1/XsensImuDevice.cpp
  src/plugins/body_control/devices/xsens_imu/base/mode2/XsensImuDevice.cpp
  src/plugins/body_control/devices/xsens_imu/high_rate/XsensImuHRDevice.cpp
  src/plugins/body_control/devices/xsens_imu/high_rate/XsensImuHR.cpp
  src/plugins/body_control/devices/sri_sensor/SriSensorDevice.cpp
  src/plugins/body_control/devices/sri_sensor/SriDeviceManager.cpp
  src/plugins/body_control/devices/power_board/PowerBoardDevice.cpp
  src/plugins/body_control/devices/power_board/PowerMgr.cpp
  src/plugins/body_control/devices/hand/tsinghua/TsinghuaHandDevice.cpp
  src/plugins/body_control/devices/hand/tsinghua/TsinghuaHandDeviceManager.cpp
  src/plugins/body_control/devices/zeroerr_motor/ZeroErrMotorDevice.cpp
  src/plugins/body_control/devices/zeroerr_motor/ZeroErrMotorDevMgr.cpp
  src/plugins/body_control/devices/eyou_motor/EyouMotorDevice.cpp
  src/plugins/body_control/devices/eyou_motor/EyouMotorDevMgr.cpp
)
target_link_libraries(bodyctrl_plugin soem_master soem glog_init yaml-cpp)
ament_target_dependencies(bodyctrl_plugin rclcpp rclcpp_components bodyctrl_msgs fast_ros)
rclcpp_components_register_node(bodyctrl_plugin PLUGIN "body_control::BodyControl" EXECUTABLE body_control_node)

add_library(monitor_plugin SHARED src/plugins/monitor/MonitorPlugin.cpp)
target_link_libraries(monitor_plugin glog_init yaml-cpp)
ament_target_dependencies(monitor_plugin rclcpp rclcpp_components bodyctrl_msgs fast_ros)
rclcpp_components_register_node(monitor_plugin PLUGIN "body_control::MonitorPlugin" EXECUTABLE monitor_node)

add_library(test_plugin SHARED src/plugins/test/TestPlugin.cpp)
ament_target_dependencies(test_plugin rclcpp rclcpp_components bodyctrl_msgs)
rclcpp_components_register_node(test_plugin PLUGIN "body_control::TestPlugin" EXECUTABLE test_plugin_node)

install(TARGETS bodyctrl_plugin test_plugin monitor_plugin glog_init
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/src/plugins/xsens_imu/xsens/lib/
  DESTINATION lib
)

install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

add_subdirectory(src/plugins/body_control/soem_master)

ament_package()
