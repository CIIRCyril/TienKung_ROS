#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <yaml-cpp/yaml.h>
#include <bodyctrl_msgs/msg/motor_status_msg.hpp>
#include <bodyctrl_msgs/msg/motor_name.hpp>
#include <bodyctrl_msgs/msg/imu.hpp>
#include <bodyctrl_msgs/msg/node_state.hpp>
#include <bodyctrl_msgs/msg/ts_hand_status_msg.hpp>
#include <bodyctrl_msgs/msg/cmd_motor_ctrl.hpp>
#include <bodyctrl_msgs/msg/cmd_set_motor_position.hpp>
#include <bodyctrl_msgs/msg/cmd_set_motor_speed.hpp>
#include <bodyctrl_msgs/msg/cmd_set_motor_distance.hpp>
#include <bodyctrl_msgs/msg/cmd_set_ts_hand_position.hpp>
#include <bodyctrl_msgs/msg/power_status.hpp>
#include <bodyctrl_msgs/msg/sri.hpp>
#include <fast_ros/fast_ros.h>

#include <math.h> //fabs

#include <thread>
#include <mutex>
#include <algorithm>
#include <iostream>  
#include <fstream> 

#include "glog/GlogInitializer.h"

#define LOG_F2(LEVEL) LOG(LEVEL) << std::fixed << std::setprecision(2)
#define LOG_IF_F2(LEVEL, CONDITION) LOG_IF(LEVEL, CONDITION) < std::fixed << std::setprecision(2)


enum {
  TYPE_MOTOR_STATUS = 1,
  TYPE_IMU,
  TYPE_IMU_HR,
  TYPE_SET_MOTOR,
  TYPE_MOTOR_CTRL,
  TYPE_SET_MOTOR_POSITION,
  TYPE_SET_MOTOR_DISTANCE,
  TYPE_SET_MOTOR_SPEED,
  TYPE_TSHAND_STATUS,
  TYPE_SET_TSHAND_POSITION,
  TYPE_SRI,
  TYPE_MAX
};

#define MAX_NUM TYPE_MAX

namespace body_control  // The usage of the namespace is a good practice but not mandatory
{

class MonitorPlugin : public rclcpp::Node
{
public:
  explicit MonitorPlugin(const rclcpp::NodeOptions & options)
    : Node("Monitor", rclcpp::NodeOptions(options)
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true))
  {
    INIT_GLOG("./glogs");
    motorNames = {
      {bodyctrl_msgs::msg::MotorName::MOTOR_LEG_LEFT_1, "MOTOR_LEG_LEFT_1"},
      {bodyctrl_msgs::msg::MotorName::MOTOR_LEG_LEFT_2, "MOTOR_LEG_LEFT_2"},
      {bodyctrl_msgs::msg::MotorName::MOTOR_LEG_LEFT_3, "MOTOR_LEG_LEFT_3"},
      {bodyctrl_msgs::msg::MotorName::MOTOR_LEG_LEFT_4, "MOTOR_LEG_LEFT_4"},
      {bodyctrl_msgs::msg::MotorName::MOTOR_LEG_LEFT_5, "MOTOR_LEG_LEFT_5"},
      {bodyctrl_msgs::msg::MotorName::MOTOR_LEG_LEFT_6, "MOTOR_LEG_LEFT_6"},
      {bodyctrl_msgs::msg::MotorName::MOTOR_LEG_RIGHT_1, "MOTOR_LEG_RIGHT_1"},
      {bodyctrl_msgs::msg::MotorName::MOTOR_LEG_RIGHT_2, "MOTOR_LEG_RIGHT_2"},
      {bodyctrl_msgs::msg::MotorName::MOTOR_LEG_RIGHT_3, "MOTOR_LEG_RIGHT_3"},
      {bodyctrl_msgs::msg::MotorName::MOTOR_LEG_RIGHT_4, "MOTOR_LEG_RIGHT_4"},
      {bodyctrl_msgs::msg::MotorName::MOTOR_LEG_RIGHT_5, "MOTOR_LEG_RIGHT_5"},
      {bodyctrl_msgs::msg::MotorName::MOTOR_LEG_RIGHT_6, "MOTOR_LEG_RIGHT_6"},
      {bodyctrl_msgs::msg::MotorName::MOTOR_ARM_LEFT_1, "MOTOR_ARM_LEFT_1"},
      {bodyctrl_msgs::msg::MotorName::MOTOR_ARM_LEFT_2, "MOTOR_ARM_LEFT_2"},
      {bodyctrl_msgs::msg::MotorName::MOTOR_ARM_LEFT_3, "MOTOR_ARM_LEFT_3"},
      {bodyctrl_msgs::msg::MotorName::MOTOR_ARM_RIGHT_1, "MOTOR_ARM_RIGHT_1"},
      {bodyctrl_msgs::msg::MotorName::MOTOR_ARM_RIGHT_2, "MOTOR_ARM_RIGHT_2"},
      {bodyctrl_msgs::msg::MotorName::MOTOR_ARM_RIGHT_3, "MOTOR_ARM_RIGHT_3"},
      {bodyctrl_msgs::msg::MotorName::MOTOR_HEAD_TOP, "MOTOR_HEAD_1"},
      {bodyctrl_msgs::msg::MotorName::MOTOR_HEAD_LEFT, "MOTOR_HEAD_2"},
      {bodyctrl_msgs::msg::MotorName::MOTOR_HEAD_RIGHT, "MOTOR_HEAD_3"},
      {bodyctrl_msgs::msg::MotorName::MOTOR_ARM_LEFT_4, "MOTOR_ARM_LEFT_4"},
      {bodyctrl_msgs::msg::MotorName::MOTOR_ARM_RIGHT_4, "MOTOR_ARM_RIGHT_4"},
    };
    // Defer onInit
    init_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(0),
      [this]() {
        init_timer_->cancel();
        onInit();
      });
  }

private:

  void LoadMotors() {
    this->declare_parameter("motor_setting_file", std::string(""));
    auto motor_file = this->get_parameter("motor_setting_file").as_string();
    if (motor_file.empty()) {
      LOG(WARNING) << "no motor setting file.";
      return;
    }
    try {
      YAML::Node yaml = YAML::LoadFile(motor_file);
      if (yaml["motors_mapping"]) {
        auto mappings_list = yaml["motors_mapping"];
        LOG(INFO) << "Mappings loaded:";
        for (size_t i = 0; i < mappings_list.size(); ++i) {
          auto motor_mapping = mappings_list[i];
          if (motor_mapping.size() == 5 || motor_mapping.size() == 6) {
            namesAvailable.push_back(motor_mapping[0].as<int>());
          } else {
            LOG(ERROR) << "Malformed motor mapping at index " << i;
          }
        }
      }
    } catch (const std::exception& e) {
      LOG(ERROR) << "Failed to load motor YAML: " << e.what();
    }
  }

  void onInit()
  {
    LoadMotors();
    subNodeState = this->create_subscription<bodyctrl_msgs::msg::NodeState>(
      "/BodyControl/node_state", 1000,
      std::bind(&MonitorPlugin::OnNodeState, this, std::placeholders::_1));
  }

  void OnNodeState(const bodyctrl_msgs::msg::NodeState::ConstSharedPtr msg)
  {
    static bool started = false;
    if (msg->state == bodyctrl_msgs::msg::NodeState::NODE_STATE_RUNNING && !started) {
      LOG(INFO) << "Start Monitor.";
      started = true;
      std::thread([this](){
        
        auto fnh = fast_ros::NodeHandle(shared_from_this());
        // sub sensors
        sub[TYPE_MOTOR_STATUS] = fnh.subscribe("/BodyControl/motor_state", 1000, &MonitorPlugin::OnMotorStatus, this, fast_ros::NATIVE_ROS);
        sub[TYPE_IMU] = fnh.subscribe("/BodyControl/imu", 1000, &MonitorPlugin::OnImu, this, fast_ros::NATIVE_ROS);
        sub[TYPE_IMU_HR] = fnh.subscribe("/BodyControl/imu_hr", 1000, &MonitorPlugin::OnImuHr, this, fast_ros::NATIVE_ROS);
        sub[TYPE_TSHAND_STATUS] = fnh.subscribe("/BodyControl/tshand_status", 1000, &MonitorPlugin::OnTshandStatus, this, fast_ros::NATIVE_ROS);

        // sub motion control
        sub[TYPE_MOTOR_CTRL] = fnh.subscribe("/BodyControl/motor_ctrl", 10, &MonitorPlugin::OnCmdMotorCtrlMsg, this, fast_ros::NATIVE_ROS);
        sub[TYPE_SET_MOTOR_POSITION] = fnh.subscribe("/BodyControl/set_motor_position", 10, &MonitorPlugin::OnCmdSetMotorPosition, this, fast_ros::NATIVE_ROS);
        sub[TYPE_SET_MOTOR_DISTANCE] = fnh.subscribe("/BodyControl/set_motor_distance", 10, &MonitorPlugin::OnCmdSetMotorDistance, this, fast_ros::NATIVE_ROS);
        sub[TYPE_SET_MOTOR_SPEED] = fnh.subscribe("/BodyControl/set_motor_speed", 10, &MonitorPlugin::OnCmdSetMotorSpeed, this, fast_ros::NATIVE_ROS);

        // tshand
        sub[TYPE_SET_TSHAND_POSITION] = fnh.subscribe("/BodyControl/set_tshand_position", 10, &MonitorPlugin::OnCmdSetTshandPosition, this, fast_ros::NATIVE_ROS);

        // sri
        sub[TYPE_SRI] = fnh.subscribe("/BodyControl/sri", 10, &MonitorPlugin::OnSriMsg, this, fast_ros::NATIVE_ROS);

        // powerStatus
        subPowerStatus = this->create_subscription<bodyctrl_msgs::msg::PowerStatus>(
          "/BodyControl/power_status", 10,
          std::bind(&MonitorPlugin::OnPowerStatus, this, std::placeholders::_1));

        rclcpp::Rate rate(0.2);
        // rclcpp::Rate rate(1);
        timeLast = this->get_clock()->now();

        // reset record
        for (auto i = 0; i < MAX_NUM; ++i) {
          count[i] = 0;
          lanSec[i] = 0.0;
          motorMaxTemperature[i] = 0;
          motorMaxCurrent[i] = 0;
          motorTotalCurrent[i] = 0;
        }

        while (rclcpp::ok()) {
          
          rate.sleep();

          for (int i = 0; i < MAX_NUM; ++i) {
            mtx[i].lock();
          }

          auto now = this->get_clock()->now();
          auto dur = now - timeLast;
          double hz[MAX_NUM] = {0.0};
          double lanPs[MAX_NUM] = {0.0};
          for (auto i = 0; i < MAX_NUM; ++i) {
            hz[i] = count[i] / dur.seconds();
            lanPs[i] = (count[i] == 0 ? 0 : lanSec[i] / count[i]);
          }

          std::unordered_map<int, double> motorLoss;
          for (auto name : namesAvailable) {
            motorLoss[name] = (count[TYPE_MOTOR_STATUS] - countAnyMotor[name]) * 1.0 / count[TYPE_MOTOR_STATUS];
          }

          // show
          LOG(INFO) << "-------------Monitor Check---------------";
          LOG(INFO) << "------------Rate & Lantency--------------";
          LOG_F2(INFO) << "rate of motor status: " << hz[TYPE_MOTOR_STATUS] << " hz";
          // LOG_F2(INFO) << "rate of tshand status: " << hz[TYPE_TSHAND_STATUS] << " hz";
          if (hz[TYPE_IMU] < 390) {
            LOG_F2(WARNING) << "rate of imu (not same): " << hz[TYPE_IMU]  << " hz";
          }
          else {
            LOG_F2(INFO) << "rate of imu (not same): " << hz[TYPE_IMU]  << " hz";
          }
          // if (hz[TYPE_IMU_HR] < 390) {
          //   LOG_F2(WARNING) << "rate of imu-hr: " << hz[TYPE_IMU_HR] << " hz";
          // }
          // else {
          //   LOG_F2(INFO) << "rate of imu-hr: " << hz[TYPE_IMU_HR] << " hz";
          // }
          // LOG_F2(INFO) << "rate of sri: " << hz[TYPE_SRI] << " hz";
          LOG_F2(INFO) << "rate of control: " << hz[TYPE_SET_MOTOR] << " hz";
          // LOG_F2(INFO) << "rate of tshand control: " << hz[TYPE_SET_TSHAND_POSITION] << " hz";
          LOG_F2(INFO) << "average lantency of motor status: " << lanPs[TYPE_MOTOR_STATUS] * 1000.0 << " ms";
          // LOG_F2(INFO) << "average lantency of tshand status: " << lanPs[TYPE_TSHAND_STATUS] * 1000.0 << " ms";
          LOG_F2(INFO) << "average lantency of imu: " << lanPs[TYPE_IMU] * 1000.0 << " ms";
          // LOG_F2(INFO) << "average lantency of imu-hr: " << lanPs[TYPE_IMU_HR] * 1000.0 << " ms";
          // LOG_F2(INFO) << "average lantency of sri: " << lanPs[TYPE_SRI] * 1000.0 << " ms"; 
          // LOG_F2(INFO) << "average lantency of control: " << lanPs[TYPE_SET_MOTOR] * 1000.0 << " ms"; 
          // LOG_F2(INFO) << "average lantency of tshand control: " << lanPs[TYPE_SET_TSHAND_POSITION] * 1000.0 << " ms"; 
          LOG_F2(INFO) << "---------------Motor Status--------------";
          for (auto name : namesAvailable) {
            auto curAvg = motorTotalCurrent[name] / countAnyMotor[name];

            (motorLoss[name] > 0 ? (LOG_F2(WARNING)) : (LOG_F2(INFO))) 
              << "" << motorNames[name] << " loss rate: " << motorLoss[name] * 100 << "%";

            (motorMaxTemperature[name] > 80 ? LOG_F2(WARNING) : LOG_F2(INFO)) 
              << "" << motorNames[name]<< " highest temperature: " << motorMaxTemperature[name] << " C";

            (motorMaxCurrent[name] > 60 ? LOG_F2(WARNING) : LOG_F2(INFO)) 
              << "" << motorNames[name]<< " highest current: " << motorMaxCurrent[name] << " A";

            // (curAvg > 22 ? LOG_F2(WARNING) : LOG_F2(INFO)) 
            //   << "" << motorNames[name]<< " average current: " << curAvg << " A";
              LOG(INFO) << "--------------------";
              
          }
          LOG(INFO) << "-----------------------------------------";

          // reset record
          for (auto i = 0; i < MAX_NUM; ++i) {
            count[i] = 0;
            lanSec[i] = 0.0;
            motorMaxTemperature[i] = 0;
            motorMaxCurrent[i] = 0;
            motorTotalCurrent[i] = 0;
          }

          timeLast = now;
          
          for (auto name : namesAvailable) {
            countAnyMotor[name] = 0;
          }

          for (int i = 0; i < MAX_NUM; ++i) {
            mtx[i].unlock();
          }
        }
      }).detach();
    }
  }

  void OnMotorStatus(const bodyctrl_msgs::msg::MotorStatusMsg::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> guard(mtx[TYPE_MOTOR_STATUS]);
    ++count[TYPE_MOTOR_STATUS];
    lanSec[TYPE_MOTOR_STATUS] += (rclcpp::Clock().now() - msg->header.stamp).seconds();

    // loss
    for (auto& status : msg->status) {
       countAnyMotor[status.name]++;
       motorMaxTemperature[status.name] = std::max(motorMaxTemperature[status.name], (double)status.temperature);
       motorMaxCurrent[status.name] = std::max(motorMaxCurrent[status.name], (double)status.current);
       motorTotalCurrent[status.name] = motorTotalCurrent[status.name] + status.current;
    }
  }

  void OnImu(const bodyctrl_msgs::msg::Imu::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> guard(mtx[TYPE_IMU]);
    static bodyctrl_msgs::msg::Imu::ConstSharedPtr lastMsg = nullptr;
    if (lastMsg == nullptr) {
      ++count[TYPE_IMU];
      lastMsg = msg;
    }
    else if (lastMsg->euler.pitch != msg->euler.pitch 
            && lastMsg->euler.pitch != msg->euler.pitch
            && lastMsg->euler.yaw != msg->euler.yaw) {
      ++count[TYPE_IMU];
      lastMsg = msg; 
    }
    lanSec[TYPE_IMU] += (rclcpp::Clock().now() - msg->header.stamp).seconds();

    // // record
    // if (true) {
    //   static std::ofstream file("output.txt"); 
    //   if (!file.is_open()) {  
    //       std::cerr << "Unable to open file";  
    //       return;  
    //   }
    //   file << msg->angular_velocity.x << " ";
    //   file << msg->angular_velocity.y << " ";
    //   file << msg->angular_velocity.z << " ";
    //   file << msg->linear_acceleration.x << " ";
    //   file << msg->linear_acceleration.y << " ";
    //   file << msg->linear_acceleration.z << " ";
    //   file << msg->euler.roll << " ";
    //   file << msg->euler.pitch << " ";
    //   file << msg->euler.yaw << " ";
    //   file << std::endl;
    //   file.flush();
    // }
  }

  void OnImuHr(const bodyctrl_msgs::msg::Imu::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> guard(mtx[TYPE_IMU_HR]);
    ++count[TYPE_IMU_HR];
    lanSec[TYPE_IMU_HR] += (rclcpp::Clock().now() - msg->header.stamp).seconds();
  }

  void OnTshandStatus(const bodyctrl_msgs::msg::TsHandStatusMsg::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> guard(mtx[TYPE_TSHAND_STATUS]);
    ++count[TYPE_TSHAND_STATUS];
    lanSec[TYPE_TSHAND_STATUS] += (rclcpp::Clock().now() - msg->header.stamp).seconds();
  }

  void OnCmdMotorCtrlMsg(const bodyctrl_msgs::msg::CmdMotorCtrl::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> guard(mtx[TYPE_SET_MOTOR]);
    ++count[TYPE_SET_MOTOR];
    rclcpp::Time zero(0, 0, RCL_ROS_TIME);
    if (msg->header.stamp != zero) {
      lanSec[TYPE_SET_MOTOR] += (rclcpp::Clock().now() - msg->header.stamp).seconds();
    }

  }

  void OnCmdSetMotorPosition(const bodyctrl_msgs::msg::CmdSetMotorPosition::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> guard(mtx[TYPE_SET_MOTOR]);
    ++count[TYPE_SET_MOTOR];
    rclcpp::Time zero(0, 0, RCL_ROS_TIME);
    if (msg->header.stamp != zero) {
      lanSec[TYPE_SET_MOTOR] += (rclcpp::Clock().now() - msg->header.stamp).seconds();
    }
  }

  void OnCmdSetMotorDistance(const bodyctrl_msgs::msg::CmdSetMotorDistance::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> guard(mtx[TYPE_SET_MOTOR]);
    ++count[TYPE_SET_MOTOR];
    rclcpp::Time zero(0, 0, RCL_ROS_TIME);
    if (msg->header.stamp != zero) {
      lanSec[TYPE_SET_MOTOR] += (rclcpp::Clock().now() - msg->header.stamp).seconds();
    }
  }

  void OnCmdSetMotorSpeed(const bodyctrl_msgs::msg::CmdSetMotorSpeed::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> guard(mtx[TYPE_SET_MOTOR]);
    ++count[TYPE_SET_MOTOR];
    rclcpp::Time zero(0, 0, RCL_ROS_TIME);
    if (msg->header.stamp != zero) {
      lanSec[TYPE_SET_MOTOR] += (rclcpp::Clock().now() - msg->header.stamp).seconds();
    }
  }

  void OnCmdSetTshandPosition(const bodyctrl_msgs::msg::CmdSetTsHandPosition::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> guard(mtx[TYPE_SET_TSHAND_POSITION]);
    ++count[TYPE_SET_TSHAND_POSITION];
    rclcpp::Time zero(0, 0, RCL_ROS_TIME);
    if (msg->header.stamp != zero) {
      lanSec[TYPE_SET_TSHAND_POSITION] += (rclcpp::Clock().now() - msg->header.stamp).seconds();
    }
  }

  void OnSriMsg(const bodyctrl_msgs::msg::Sri::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> guard(mtx[TYPE_SRI]);
    ++count[TYPE_SRI];
    rclcpp::Time zero(0, 0, RCL_ROS_TIME);
    if (msg->header.stamp != zero) {
      lanSec[TYPE_SRI] += (rclcpp::Clock().now() - msg->header.stamp).seconds();
    }
  }

  void OnPowerStatus(const bodyctrl_msgs::msg::PowerStatus::ConstSharedPtr msg) {
    // GET_BATTERY_CUMULATIVE_VOL
    // if (msg->data[7] > 60) {
    //   LOG_F2(WARNING) << "battery cumulative voltage too high! value = " << msg->data[7];
    // }

    // GET_BATTERY_SAMPLING_VOL
    if (msg->battery_voltage > 60) {
      LOG_F2(WARNING) << "battery sampling voltage too high! value = " << msg->battery_voltage;
    }

    // GET_BATTERY_CURRENT
    if (msg->battery_current > 160) {
      LOG_F2(WARNING) << "battery current voltage too high! value = " << msg->battery_current;
    }

    // GET_BATTERY_LEVEL
    if (msg->battery_power < 10) {
      LOG_F2(WARNING) << "battery level voltage too low! value = " << msg->battery_power;
    }
  }

  rclcpp::Subscription<bodyctrl_msgs::msg::NodeState>::SharedPtr subNodeState;
  fast_ros::Subscriber sub[MAX_NUM];
  rclcpp::Subscription<bodyctrl_msgs::msg::PowerStatus>::SharedPtr subPowerStatus;
  std::mutex mtx[MAX_NUM];
  double lanSec[MAX_NUM] = {0.0};
  long count[MAX_NUM] = {0};
  long countAnyMotor[60] = {0};

  rclcpp::Time timeLast{0, 0, RCL_ROS_TIME};
  
  std::unordered_map<int, std::string> motorNames;
  std::vector<int> namesAvailable;
  std::unordered_map<int, double> motorMaxTemperature;
  std::unordered_map<int, double> motorMaxCurrent;
  std::unordered_map<int, double> motorTotalCurrent;

  rclcpp::TimerBase::SharedPtr init_timer_;
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(body_control::MonitorPlugin)
