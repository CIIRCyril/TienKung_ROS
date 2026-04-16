cmake_minimum_required(VERSION 3.8)
project(bodyctrl_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CmdMotorCtrl.msg"
  "msg/CmdSetMotorCurTor.msg"
  "msg/CmdSetMotorDistance.msg"
  "msg/CmdSetMotorPosition.msg"
  "msg/CmdSetMotorSpeed.msg"
  "msg/CmdSetTsHandCtrl.msg"
  "msg/CmdSetTsHandPosition.msg"
  "msg/CmdSetWaistMotorPos.msg"
  "msg/Euler.msg"
  "msg/Exception.msg"
  "msg/ExceptionArray.msg"
  "msg/Imu.msg"
  "msg/MotorCtrl.msg"
  "msg/MotorName.msg"
  "msg/MotorStatus.msg"
  "msg/MotorStatusMsg.msg"
  "msg/MotorStatusMsg2.msg"
  "msg/MotorStatusMsgPlate.msg"
  "msg/NodeState.msg"
  "msg/PowerBoardKeyStatus.msg"
  "msg/PowerStatus.msg"
  "msg/SetMotorCurTor.msg"
  "msg/SetMotorDistance.msg"
  "msg/SetMotorPosition.msg"
  "msg/SetMotorSpeed.msg"
  "msg/SetTsHandCtrl.msg"
  "msg/SetTsHandCtrlItem.msg"
  "msg/SetTsHandPosition.msg"
  "msg/Sri.msg"
  "msg/SriName.msg"
  "msg/TsHandName.msg"
  "msg/TsHandStatus.msg"
  "msg/TsHandStatusMsg.msg"
  "msg/WaistMotorStatus.msg"
  "srv/MotorInit.srv"
  "srv/MotorStart.srv"
  "srv/MotorStop.srv"
  "srv/MotorResetPosition.srv"
  "srv/SetMotorZeroOffset.srv"
  "srv/ResetMotorZeroOffset.srv"
  "srv/XSensImuInit.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
