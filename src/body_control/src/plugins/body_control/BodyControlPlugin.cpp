#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <condition_variable>
#include <thread>
#include <any>
#include <unordered_map>
#include <bodyctrl_msgs/srv/motor_init.hpp>
#include <bodyctrl_msgs/srv/motor_start.hpp>
#include <bodyctrl_msgs/srv/motor_stop.hpp>
#include <bodyctrl_msgs/msg/motor_name.hpp>
#include <bodyctrl_msgs/msg/motor_status_msg.hpp>
#include <bodyctrl_msgs/msg/cmd_motor_ctrl.hpp>
#include <bodyctrl_msgs/msg/cmd_set_motor_position.hpp>
#include <bodyctrl_msgs/msg/cmd_set_motor_speed.hpp>
#include <bodyctrl_msgs/msg/cmd_set_motor_distance.hpp>
#include <bodyctrl_msgs/msg/cmd_set_motor_cur_tor.hpp>
#include <bodyctrl_msgs/msg/cmd_set_ts_hand_position.hpp>
#include <bodyctrl_msgs/msg/ts_hand_status_msg.hpp>
#include <bodyctrl_msgs/msg/ts_hand_name.hpp>
#include <bodyctrl_msgs/msg/cmd_set_ts_hand_ctrl.hpp>
#include <bodyctrl_msgs/msg/imu.hpp>
#include <bodyctrl_msgs/msg/sri.hpp>
#include <bodyctrl_msgs/msg/power_status.hpp>
#include <bodyctrl_msgs/msg/power_board_key_status.hpp>
#include <bodyctrl_msgs/msg/node_state.hpp>
#include <bodyctrl_msgs/msg/motor_status.hpp>
#include <fast_ros/fast_ros.h>

#include <sstream>
#include <math.h> //fabs

#include "util/LockFreeQueue.h"
#include "glog/GlogInitializer.h"

#include "soem_master/SoemMaster.h"
#include "devices/motor/MotorDeviceManager.h"
#include "devices/rm_imu/RmImuDevice.h"
#include "devices/sri_sensor/SriDeviceManager.h"
#include "devices/power_board/PowerBoardDevice.h"
#include "devices/xsens_imu/base/XsensImuDevice.h"
#include "devices/xsens_imu/high_rate/XsensImuHRDevice.h"
#include "devices/hand/tsinghua/TsinghuaHandDeviceManager.h"
#include "devices/zeroerr_motor/ZeroErrMotorDevMgr.h"
#include "devices/eyou_motor/EyouMotorDevMgr.h"



static bool cvt_msg(const PowerMgr& mgr, bodyctrl_msgs::msg::PowerStatus::SharedPtr& status)
{
  // temperature information
  {
    auto obj = mgr.GetPowerStatus<PbIns::GET_WAIST_TEMP>();
    status->waist_temp = obj->value();
    status->waist_temp_max = obj->max();
    status->waist_temp_min = obj->min();
  }
  {
    auto obj = mgr.GetPowerStatus<PbIns::GET_ARM_A_TEMP>();
    status->arm_a_temp = obj->value();
    status->arm_a_temp_max = obj->max();
    status->arm_a_temp_min = obj->min();
  }
  {
    auto obj = mgr.GetPowerStatus<PbIns::GET_ARM_B_TEMP>();
    status->arm_b_temp = obj->value();
    status->arm_b_temp_max = obj->max();
    status->arm_b_temp_min = obj->min();
  }
  {
    auto obj = mgr.GetPowerStatus<PbIns::GET_LEG_A_TEMP>();
    status->leg_a_temp = obj->value();
    status->leg_a_temp_max = obj->max();
    status->leg_a_temp_min = obj->min();
  }
  {
    auto obj = mgr.GetPowerStatus<PbIns::GET_LEG_B_TEMP>();
    status->leg_b_temp = obj->value();
    status->leg_b_temp_max = obj->max();
    status->leg_b_temp_min = obj->min();
  }
  // current information
  {
    auto obj = mgr.GetPowerStatus<PbIns::GET_ARM_A_CURR>();
    status->arm_a_curr = obj->value();
    status->arm_a_curr_max = obj->max();
    status->arm_a_curr_min = obj->min();
  }
  {
    auto obj = mgr.GetPowerStatus<PbIns::GET_ARM_B_CURR>();
    status->arm_b_curr = obj->value();
    status->arm_b_curr_max = obj->max();
    status->arm_b_curr_min = obj->min();
  }
  {
    auto obj = mgr.GetPowerStatus<PbIns::GET_LEG_A_CURR>();
    status->leg_a_curr = obj->value();
    status->leg_a_curr_max = obj->max();
    status->leg_a_curr_min = obj->min();
  }
  {
    auto obj = mgr.GetPowerStatus<PbIns::GET_LEG_B_CURR>();
    status->leg_b_curr = obj->value();
    status->leg_b_curr_max = obj->max();
    status->leg_b_curr_min = obj->min();
  }
  {
    auto obj = mgr.GetPowerStatus<PbIns::GET_WAIST_CURR>();
    status->waist_curr = obj->value();
    status->waist_curr_max = obj->max();
    status->waist_curr_min = obj->min();
  }
  {
    auto obj = mgr.GetPowerStatus<PbIns::GET_HEAD_CURR>();
    status->head_curr = obj->value();
    status->head_curr_max = obj->max();
    status->head_curr_min = obj->min();
  }
  // version information
  {
    auto obj = mgr.GetPowerVersionInfo();
    {
      std::stringstream ss;
      ss << std::hex << obj->hardware_version();
      status->hardware_version = ss.str();
    }
    {
      std::stringstream ss;
      ss << std::hex << obj->software_version();
      status->software_version = ss.str();
    }
  }

  // battery infomation
  {
    auto obj = mgr.GetBatteryInfomation();
    status->battery_voltage = obj->voltage();
    status->battery_current = obj->current();
    status->battery_power = obj->power();
  }

  return true;
}
static bool cvt_msg(const PowerMgr& mgr, bodyctrl_msgs::msg::PowerBoardKeyStatus::SharedPtr& status)
{
  auto obj                     = mgr.GetPowerBoardStatus();
  status->work_time            = obj->work_time();
  status->is_estop.data        = obj->is_estop();
  status->is_remote_estop.data = obj->is_remote_estop();
  status->is_power_on.data     = obj->is_power_on();
  return true;
}


namespace body_control
{

class BodyControl : public rclcpp::Node
{
public:
  explicit BodyControl(const rclcpp::NodeOptions & options)
    : Node("BodyControl", rclcpp::NodeOptions(options)
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true))
  {
    INIT_GLOG("./glogs");
    // Defer onInit to allow shared_from_this() to work
    init_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(0),
      [this]() {
        init_timer_->cancel();
        onInit();
      });
  }

  ~BodyControl() {
    SoemMaster::Instance().Stop();
  }

  void RunPubMsg() {
    LOG(INFO) << "start publisher thread: " << gettid();
 
    std::unique_lock<std::mutex> lck(mtxPubMsgCv);
    while (rclcpp::ok()) 
    {
      while (!msgQueue.empty()) {
        auto msg = msgQueue.pop();
        if (msg.type == CacheMessage::MessageType::MOTOR) {
          pubMotorsState.publish(std::any_cast<bodyctrl_msgs::msg::MotorStatusMsg::SharedPtr>(msg.msg));
          
        }
        else if (msg.type == CacheMessage::MessageType::IMU) {
          pubImu.publish(std::any_cast<bodyctrl_msgs::msg::Imu::SharedPtr>(msg.msg));
        }
        else if (msg.type == CacheMessage::MessageType::IMU_HR) {
          pubXImuHr.publish(std::any_cast<bodyctrl_msgs::msg::Imu::SharedPtr>(msg.msg));
        }
        else if(msg.type == CacheMessage::MessageType::POWER_KEY)
        {
          pubPowerBoardKeyStatus->publish(*std::any_cast<bodyctrl_msgs::msg::PowerBoardKeyStatus::SharedPtr>(msg.msg));
        }
        else if (msg.type == CacheMessage::MessageType::POWER) {
          pubPowerStatus->publish(*std::any_cast<bodyctrl_msgs::msg::PowerStatus::SharedPtr>(msg.msg));
        }
        else if (msg.type == CacheMessage::MessageType::SRI) {
          pubSri->publish(*std::any_cast<bodyctrl_msgs::msg::Sri::SharedPtr>(msg.msg));
        }
        else if (msg.type == CacheMessage::MessageType::HAND_TSINGHUA) {
          pubHandsStatus->publish(*std::any_cast<bodyctrl_msgs::msg::TsHandStatusMsg::SharedPtr>(msg.msg));
        }
        else if (msg.type == CacheMessage::MessageType::ZE_MOTOR) {
          pubZeMotorStatus->publish(*std::any_cast<bodyctrl_msgs::msg::MotorStatusMsg::SharedPtr>(msg.msg));
        }
        else if (msg.type == CacheMessage::MessageType::EYOU_MOTOR) {
          pubEyouMotorStatus->publish(*std::any_cast<bodyctrl_msgs::msg::MotorStatusMsg::SharedPtr>(msg.msg));
        }
      }
      cvPubMsg.wait_for(lck, std::chrono::microseconds(500));
    }
  }

  void LoadEthercatParam(const YAML::Node& yaml) {
    if (yaml["slave_mode"]) {
      std::string logTxt = "slave_mode: ";
      auto slaveMode = yaml["slave_mode"];
      for (size_t i = 0; i < slaveMode.size(); ++i) {
        auto id = slaveMode[i].as<int>();
        vecSlaveMode.push_back((SoemMaster::Mode)id);
        logTxt = logTxt + std::to_string(id) + " ";
      }
      LOG(INFO) << logTxt;
    } else {
      LOG(WARNING) << ("no ethercat param setting.");
    }
  }

  void LoadMotors(const YAML::Node& yaml) {
    auto fnh = fast_ros::NodeHandle(shared_from_this());

    float top_temp_limit = 90.0;
    if (yaml["top_temp_limit"]) {
      top_temp_limit = yaml["top_temp_limit"].as<float>();
    }
    LOG(INFO) << "top_temp_limit: " << top_temp_limit;
    mdm.reset(new MotorDeviceManager);
    mdm->SetOnStatusReady([this, top_temp_limit](bodyctrl_msgs::msg::MotorStatusMsg::SharedPtr msg){
        msg->header.stamp = rclcpp::Clock().now();
        CacheMessage cmsg;
        cmsg.type = CacheMessage::MessageType::MOTOR;
        cmsg.msg = msg;
        msgQueue.push(cmsg);

        if(power)
        {
              if(this->mdm->is_temperature_high(top_temp_limit))
              {
                this->power->enable_beep();
              }
              else
              {
                this->power->disable_beep();
              }
        }

        cvPubMsg.notify_one();
    });
    // get mapping for moters
    if (yaml["motors_mapping"]) {
        auto mappings_list = yaml["motors_mapping"];
        // motor interfaces
        pubMotorsState = fnh.advertise<bodyctrl_msgs::msg::MotorStatusMsg>("motor_state", 1000);
        subCmdMotorCtrl = this->create_subscription<bodyctrl_msgs::msg::CmdMotorCtrl>(
          "motor_ctrl", 10, std::bind(&BodyControl::OnCmdMotorCtrlMsg, this, std::placeholders::_1));
        subCmdSetMotorPosition = this->create_subscription<bodyctrl_msgs::msg::CmdSetMotorPosition>(
          "set_motor_position", 10, std::bind(&BodyControl::OnCmdSetMotorPosition, this, std::placeholders::_1));
        subCmdSetMotorDistance = this->create_subscription<bodyctrl_msgs::msg::CmdSetMotorDistance>(
          "set_motor_distance", 10, std::bind(&BodyControl::OnCmdSetMotorDistance, this, std::placeholders::_1));
        subCmdSetMotorSpeed = this->create_subscription<bodyctrl_msgs::msg::CmdSetMotorSpeed>(
          "set_motor_speed", 10, std::bind(&BodyControl::OnCmdSetMotorSpeed, this, std::placeholders::_1));
        LOG(INFO) << "Mappings loaded:";
        for (size_t i = 0; i < mappings_list.size(); ++i) {
            auto motor_mapping = mappings_list[i];
            if (motor_mapping.size() == 5) {
                LOG(INFO)
                  << " " << motor_mapping[0].as<int>() << ","
                  << " " << motor_mapping[1].as<int>() << ","
                  << " " << motor_mapping[2].as<int>() << ","
                  << " " << motor_mapping[3].as<int>() << ","
                  << " " << motor_mapping[4].as<int>();
                mdm->NewDevice(
                  motor_mapping[0].as<int>(),
                  motor_mapping[1].as<int>(),
                  motor_mapping[2].as<int>(),
                  motor_mapping[3].as<int>(),
                  motor_mapping[4].as<int>()
                );
                bEnableMotors = true;
            } else if (motor_mapping.size() == 6) {
                LOG(INFO)
                  << " " << motor_mapping[0].as<int>() << ","
                  << " " << motor_mapping[1].as<int>() << ","
                  << " " << motor_mapping[2].as<int>() << ","
                  << " " << motor_mapping[3].as<int>() << ","
                  << " " << motor_mapping[4].as<int>() << ","
                  << " " << motor_mapping[5].as<double>();
                mdm->NewDevice(
                  motor_mapping[0].as<int>(),
                  motor_mapping[1].as<int>(),
                  motor_mapping[2].as<int>(),
                  motor_mapping[3].as<int>(),
                  motor_mapping[4].as<int>(),
                  motor_mapping[5].as<double>()
                );
                bEnableMotors = true;
            }
            else {
                LOG(ERROR) << "Malformed motor mapping at index " << i;
            }
        }
    } else {
        LOG(WARNING) << ("no motor setting.");
    }
  }

  void LoadTsinghuaHand(const YAML::Node& yaml) {
    tsHandMgr.reset(new TsinghuaHandDeviceManager);
    tsHandMgr->SetOnStatusReady([this](bodyctrl_msgs::msg::TsHandStatusMsg::SharedPtr msg){
        msg->header.stamp = rclcpp::Clock().now();
        CacheMessage cmsg;
        cmsg.type = CacheMessage::MessageType::HAND_TSINGHUA;
        cmsg.msg = msg;
        msgQueue.push(cmsg);
        cvPubMsg.notify_one();
    });
    // get mapping for moters
    if (yaml["hand_mapping"]) {
        auto mappings_list = yaml["hand_mapping"];
        // tshand interface
        pubHandsStatus = this->create_publisher<bodyctrl_msgs::msg::TsHandStatusMsg>("tshand_status", 1000);
        subCmdTsHandSetPos = this->create_subscription<bodyctrl_msgs::msg::CmdSetTsHandPosition>(
          "set_tshand_position", 10, std::bind(&BodyControl::OnCmdSetSetTsHandPosition, this, std::placeholders::_1));
        subCmdTsHandSetCtrl = this->create_subscription<bodyctrl_msgs::msg::CmdSetTsHandCtrl>(
          "set_tshand_ctrl", 10, std::bind(&BodyControl::OnCmdSetSetTsHandCtrl, this, std::placeholders::_1));
        LOG(INFO) << "hand mappings loaded:";
        for (size_t i = 0; i < mappings_list.size(); ++i) {
            auto motor_mapping = mappings_list[i];
            if (motor_mapping.size() == 4) {
                LOG(INFO)
                  << " " << motor_mapping[0].as<int>() << ","
                  << " " << motor_mapping[1].as<int>() << ","
                  << " " << motor_mapping[2].as<int>() << ","
                  << " " << motor_mapping[3].as<int>();
                tsHandMgr->NewDevice(
                  motor_mapping[0].as<int>(),
                  motor_mapping[1].as<int>(),
                  motor_mapping[2].as<int>(),
                  motor_mapping[3].as<int>()
                );
                bEnableTsHands = true;
            } else {
                LOG(ERROR) << "Malformed tshand mapping at index " << i;
            }
        }
    } else {
        LOG(WARNING) << ("no tshand setting.");
    }
  }

  void LoadRmImu(const YAML::Node& yaml) {
    auto fnh = fast_ros::NodeHandle(shared_from_this());

    if (!yaml["imu_mapping"] || !yaml["imu_mapping"]["slave"]) {
      LOG(WARNING) << ("No imu setting.");
      return;
    }
    int slave = yaml["imu_mapping"]["slave"].as<int>();
    // imu interface
    pubImu = fnh.advertise<bodyctrl_msgs::msg::Imu>("imu", 1000);
    if (!yaml["imu_mapping"]["id1"] || !yaml["imu_mapping"]["id2"] || !yaml["imu_mapping"]["id3"]) {
      LOG(ERROR) << ("Failed to load imu parameter");
      return;
    }
    int id1 = yaml["imu_mapping"]["id1"].as<int>();
    int id2 = yaml["imu_mapping"]["id2"].as<int>();
    int id3 = yaml["imu_mapping"]["id3"].as<int>();
    LOG(INFO) << "Imu slave: " << slave << ", id1: " << id1 << ", id2: " << id2;
    rmImu.reset(new RmImuDevice(shared_from_this(), slave, id1, id1, id2, id2,
      [this](const ubt_hw::ImuData* data) {
        auto msg = std::make_shared<bodyctrl_msgs::msg::Imu>();
        msg->header.stamp = data->stamp;
        msg->orientation.x = data->orientation[0];
        msg->orientation.y = data->orientation[1];
        msg->orientation.z = data->orientation[2];
        msg->orientation.w = data->orientation[3];
        msg->angular_velocity.x = data->angular_vel[0];
        msg->angular_velocity.y = data->angular_vel[1];
        msg->angular_velocity.z = data->angular_vel[2];
        msg->linear_acceleration.x = data->linear_accel[0];
        msg->linear_acceleration.y = data->linear_accel[1];
        msg->linear_acceleration.z = data->linear_accel[2];
        msg->euler.roll = data->rpy[0];
        msg->euler.pitch = data->rpy[1];
        msg->euler.yaw = data->rpy[2];
        msg->orientation_covariance[0] = data->orientation_covariance[0];
        msg->orientation_covariance[1] = data->orientation_covariance[1];
        msg->orientation_covariance[2] = data->orientation_covariance[2];
        msg->angular_velocity_covariance[0] = data->angular_velocity_covariance[0];
        msg->angular_velocity_covariance[1] = data->angular_velocity_covariance[1];
        msg->angular_velocity_covariance[2] = data->angular_velocity_covariance[2];
        msg->linear_acceleration_covariance[0] = data->linear_acceleration_covariance[0];
        msg->linear_acceleration_covariance[1] = data->linear_acceleration_covariance[1];
        msg->linear_acceleration_covariance[2] = data->linear_acceleration_covariance[2];
        CacheMessage amsg;
        amsg.type = CacheMessage::MessageType::IMU;
        amsg.msg = msg;
        msgQueue.push(amsg);
        cvPubMsg.notify_one();
      }
    ));
    SoemMaster::Instance().RegisterDevice(slave, {id1, id2}, rmImu);
    bEnableRmImu = true;
  }

  void LoadXsensImu(const YAML::Node& yaml) {
    auto fnh = fast_ros::NodeHandle(shared_from_this());

    int slave;
    int passages[4] = {0};
    uint16_t ids[4] = {0};

    if (!yaml["xsens_imu_mapping"] || !yaml["xsens_imu_mapping"]["slave"]) {
      LOG(WARNING) << ("No Xsens Imu setting.");
      return;
    }
    slave = yaml["xsens_imu_mapping"]["slave"].as<int>();

    // imu interface
    pubImu = fnh.advertise<bodyctrl_msgs::msg::Imu>("imu", 1000);

    std::string echoInfo = "Xsens Imu slave: ";
    for (int i = 0; i < 4; ++i) {
      std::string name = "id" + std::to_string(i + 1);
      if (!yaml["xsens_imu_mapping"][name]) {
        LOG(ERROR) << "Failed to load Xsens Imu parameter: xsens_imu_mapping/" << name;
        return;
      }
      int value = yaml["xsens_imu_mapping"][name].as<int>();
      ids[i] = passages[i] = value;
      echoInfo = echoInfo + "id" + std::to_string(i + 1) + ": " + std::to_string(ids[i]) + " ";
    }

    LOG(INFO) << echoInfo;

    xImu.reset(new XsensImuDevice(slave, passages, ids,
      [this](ImuData& data) {
        auto msg = std::make_shared<bodyctrl_msgs::msg::Imu>();
        msg->header.stamp = rclcpp::Clock().now();
        msg->orientation.x = data.orientation[0];
        msg->orientation.y = data.orientation[1];
        msg->orientation.z = data.orientation[2];
        msg->orientation.w = data.orientation[3];
        msg->angular_velocity.x = data.angular_vel[0];
        msg->angular_velocity.y = data.angular_vel[1];
        msg->angular_velocity.z = data.angular_vel[2];
        msg->linear_acceleration.x = data.linear_accel[0];
        msg->linear_acceleration.y = data.linear_accel[1];
        msg->linear_acceleration.z = data.linear_accel[2];
        msg->euler.roll = data.rpy[0];
        msg->euler.pitch = data.rpy[1];
        msg->euler.yaw = data.rpy[2];
        msg->orientation_covariance[0] = 0;
        msg->orientation_covariance[1] = 0;
        msg->orientation_covariance[2] = 0;
        msg->angular_velocity_covariance[0] = 0;
        msg->angular_velocity_covariance[1] = 0;
        msg->angular_velocity_covariance[2] = 0;
        msg->linear_acceleration_covariance[0] = 0;
        msg->linear_acceleration_covariance[1] = 0;
        msg->linear_acceleration_covariance[2] = 0;
        CacheMessage amsg;
        amsg.type = CacheMessage::MessageType::IMU;
        amsg.msg = msg;
        msgQueue.push(amsg);
        cvPubMsg.notify_one();
      }
    ));
    SoemMaster::Instance().RegisterDevice(slave, {passages[0], passages[1], passages[2], passages[3]}, xImu);
    bEnableXsensImu = true;
  } 

  void LoadXsensHrImu(const YAML::Node& yaml) {
    auto fnh = fast_ros::NodeHandle(shared_from_this());

    int slave, id1, id2, id3;
    // get mapping for imu
    if (!yaml["xhr_imu_mapping"] || !yaml["xhr_imu_mapping"]["slave"]) {
      LOG(WARNING) << ("No xhr imu setting.");
      return;
    }
    slave = yaml["xhr_imu_mapping"]["slave"].as<int>();
    pubXImuHr = fnh.advertise<bodyctrl_msgs::msg::Imu>("imu_hr", 1000);
    if (!yaml["xhr_imu_mapping"]["id1"] || !yaml["xhr_imu_mapping"]["id2"] || !yaml["xhr_imu_mapping"]["id3"]) {
      LOG(ERROR) << ("Failed to load xhr imu parameter");
      return;
    }
    id1 = yaml["xhr_imu_mapping"]["id1"].as<int>();
    id2 = yaml["xhr_imu_mapping"]["id2"].as<int>();
    id3 = yaml["xhr_imu_mapping"]["id3"].as<int>();
    LOG(INFO) << "XHR Imu slave: " << slave << ", id1: " << id1 << ", id2: " << id2;
    xhrImu.reset(new XsensImuHRDevice(shared_from_this(), slave, id1, id1, id2, id2,
      [this](const ubt_hw::ImuData* data) {
        auto msg = std::make_shared<bodyctrl_msgs::msg::Imu>();
        msg->header.stamp = data->stamp;
        msg->orientation.x = data->orientation[0];
        msg->orientation.y = data->orientation[1];
        msg->orientation.z = data->orientation[2];
        msg->orientation.w = data->orientation[3];
        msg->angular_velocity.x = data->angular_vel[0];
        msg->angular_velocity.y = data->angular_vel[1];
        msg->angular_velocity.z = data->angular_vel[2];
        msg->linear_acceleration.x = data->linear_accel[0];
        msg->linear_acceleration.y = data->linear_accel[1];
        msg->linear_acceleration.z = data->linear_accel[2];
        msg->euler.roll = data->rpy[0];
        msg->euler.pitch = data->rpy[1];
        msg->euler.yaw = data->rpy[2];
        msg->orientation_covariance[0] = data->orientation_covariance[0];
        msg->orientation_covariance[1] = data->orientation_covariance[1];
        msg->orientation_covariance[2] = data->orientation_covariance[2];
        msg->angular_velocity_covariance[0] = data->angular_velocity_covariance[0];
        msg->angular_velocity_covariance[1] = data->angular_velocity_covariance[1];
        msg->angular_velocity_covariance[2] = data->angular_velocity_covariance[2];
        msg->linear_acceleration_covariance[0] = data->linear_acceleration_covariance[0];
        msg->linear_acceleration_covariance[1] = data->linear_acceleration_covariance[1];
        msg->linear_acceleration_covariance[2] = data->linear_acceleration_covariance[2];
        CacheMessage amsg;
        amsg.type = CacheMessage::MessageType::IMU_HR;
        amsg.msg = msg;
        msgQueue.push(amsg);
        cvPubMsg.notify_one();
      }
    ));
    SoemMaster::Instance().RegisterDevice(slave, {id1, id2}, xhrImu);
    bEnableXsensHrImu = true;
  }

  void LoadSriSensor(const YAML::Node& yaml) {
    sriMgr.reset(new SriDeviceManager);
    sriMgr->SetOnStatusReady([this](bodyctrl_msgs::msg::Sri::SharedPtr msg){
        msg->header.stamp = rclcpp::Clock().now();
        CacheMessage cmsg;
        cmsg.type = CacheMessage::MessageType::SRI;
        cmsg.msg = msg;
        msgQueue.push(cmsg);
        cvPubMsg.notify_one();
    });

    int name, slave, id_cmd, passage_cmd, id_data[3], passage_data[3];
    if (yaml["sri_mapping"]) {
        auto mappings_list = yaml["sri_mapping"];
        // sri interface
        pubSri = this->create_publisher<bodyctrl_msgs::msg::Sri>("sri", 1000);
        LOG(INFO) << "sri mappings loaded:";
        for (size_t i = 0; i < mappings_list.size(); ++i) {
            auto one = mappings_list[i];
            if (one.size() == 10) {
              std::string logtxt = "";
              for (int j = 0; j < 10; ++j) {
                logtxt = logtxt +  " " + std::to_string(one[j].as<int>()) + ",";
              }
              LOG(INFO) << logtxt;
              name = one[0].as<int>();
              slave = one[1].as<int>();
              id_cmd = one[2].as<int>();
              passage_cmd = one[3].as<int>();
              for (int j = 0; j < 3; ++j) {
                id_data[j] = one[4 + j * 2].as<int>();
                passage_data[j] = one[4 + j * 2 + 1].as<int>();
              }
              LOG(INFO) << "ids:" << id_data[0] << " " << id_data[1] << " " << id_data[2];
              LOG(INFO) << "ps:" << passage_data[0] << " " << passage_data[1] << " " << passage_data[2];
              sriMgr->NewDevice(name, slave, id_cmd, passage_cmd, id_data, passage_data);
              bEnableSri = true;
            } else {
                LOG(ERROR) << "Malformed sri mapping at index " << i;
            }
        }
    } else {
        LOG(WARNING) << ("no sri setting.");
    }
  }

  void LoadPowerBoard(const YAML::Node& yaml) {
    int slave, passage, id;
    // get mapping for power board
    if (!yaml["power_mapping"] || !yaml["power_mapping"]["slave"]) {
      LOG(WARNING) << ("No PowerBoard setting.");
      return;
    }
    slave = yaml["power_mapping"]["slave"].as<int>();
    if (!yaml["power_mapping"]["passage"]) {
      LOG(ERROR) << ("Failed to load PowerBoard parameter");
      return;
    }
    passage = yaml["power_mapping"]["passage"].as<int>();
    // power manager interface
    pubPowerStatus = this->create_publisher<bodyctrl_msgs::msg::PowerStatus>("power_status", 1000);
    pubPowerBoardKeyStatus = this->create_publisher<bodyctrl_msgs::msg::PowerBoardKeyStatus>("power_board_key_status", 1000);
    if (!yaml["power_mapping"]["id"]) {
      LOG(ERROR) << ("Failed to load PowerBoard parameter");
      return;
    }
    id = yaml["power_mapping"]["id"].as<int>();
    LOG(INFO) << "PowerBoard slave: " << slave << ", passage: " << passage << ", id: " << id;
    power.reset(new PowerBoardDevice(slave, passage, id,
      [this](const PowerMgr& status) {
        auto msg = std::make_shared<bodyctrl_msgs::msg::PowerStatus>();
        auto key_msg = std::make_shared<bodyctrl_msgs::msg::PowerBoardKeyStatus>();
        {
          cvt_msg(status, msg);
          msg->header.stamp = rclcpp::Clock().now();
          CacheMessage amsg;
          amsg.type = CacheMessage::MessageType::POWER;
          amsg.msg = msg;
          msgQueue.push(amsg);
          cvPubMsg.notify_one();
        }
        {
          cvt_msg(status, key_msg);
          key_msg->header.stamp = rclcpp::Clock().now();
          CacheMessage amsg;
          amsg.type = CacheMessage::MessageType::POWER_KEY;
          amsg.msg = key_msg;
          msgQueue.push(amsg);
          cvPubMsg.notify_one();
        }
      }
    ));
    SoemMaster::Instance().RegisterDevice(slave, passage, power);
    bEnablePowerBoard = true;
  }

  void LoadZeroErrorMotors(const YAML::Node& yaml) {
    if (!yaml["ze_motors_mapping"]) {
      LOG(WARNING) << ("No ze motor setting.");
      return;
    }
    auto mappings_list = yaml["ze_motors_mapping"];

    zMgr.reset(new ZeroErrMotorDevMgr());
    zMgr->SetOnStatusReady([this](bodyctrl_msgs::msg::MotorStatusMsg::SharedPtr msg){
        msg->header.stamp = rclcpp::Clock().now();
        CacheMessage cmsg;
        cmsg.type = CacheMessage::MessageType::ZE_MOTOR;
        cmsg.msg = msg;
        msgQueue.push(cmsg);
        cvPubMsg.notify_one();
    });

    pubZeMotorStatus = this->create_publisher<bodyctrl_msgs::msg::MotorStatusMsg>("ze/status", 1000);
    subCmdZeMotorSetFphc = this->create_subscription<bodyctrl_msgs::msg::CmdMotorCtrl>(
      "ze/set_fphc", 10, std::bind(&BodyControl::OnCmdSetZeMotorFphc, this, std::placeholders::_1));
    subCmdZeMotorSetPos = this->create_subscription<bodyctrl_msgs::msg::CmdSetMotorPosition>(
      "ze/set_pos", 10, std::bind(&BodyControl::OnCmdSetZeMotorPosition, this, std::placeholders::_1));

    // # motor_name_id, slave_index, passage_req, passage_resp, motor_id, control_mode, motor_type, kt_default
    LOG(INFO) << "ZE Mappings loaded:";
    for (size_t i = 0; i < mappings_list.size(); ++i) {
        auto motor_mapping = mappings_list[i];
        if (motor_mapping.size() == 9) {
            LOG(INFO)
              << " " << motor_mapping[0].as<int>() << ","
              << " " << motor_mapping[1].as<int>() << ","
              << " " << motor_mapping[2].as<int>() << ","
              << " " << motor_mapping[3].as<int>() << ","
              << " " << motor_mapping[4].as<int>() << ","
              << " " << motor_mapping[5].as<int>() << ","
              << " " << motor_mapping[6].as<int>() << ","
              << " " << (float)motor_mapping[7].as<double>() << ","
              << " " << (float)motor_mapping[8].as<double>();

            zMgr->NewDevice(
              motor_mapping[0].as<int>(),
              motor_mapping[1].as<int>(),
              motor_mapping[2].as<int>(),
              motor_mapping[3].as<int>(),
              motor_mapping[4].as<int>(),
              motor_mapping[5].as<int>(),
              motor_mapping[6].as<int>(),
              (float)motor_mapping[7].as<double>(),
              (float)motor_mapping[8].as<double>()
            );
            bEnableZeMotors = true;
        } else if (motor_mapping.size() == 8) {
            LOG(INFO)
              << " " << motor_mapping[0].as<int>() << ","
              << " " << motor_mapping[1].as<int>() << ","
              << " " << motor_mapping[2].as<int>() << ","
              << " " << motor_mapping[3].as<int>() << ","
              << " " << motor_mapping[4].as<int>() << ","
              << " " << motor_mapping[5].as<int>() << ","
              << " " << motor_mapping[6].as<int>() << ","
              << " " << (float)motor_mapping[7].as<double>();

            zMgr->NewDevice(
              motor_mapping[0].as<int>(),
              motor_mapping[1].as<int>(),
              motor_mapping[2].as<int>(),
              motor_mapping[3].as<int>(),
              motor_mapping[4].as<int>(),
              motor_mapping[5].as<int>(),
              motor_mapping[6].as<int>(),
              (float)motor_mapping[7].as<double>()
            );
            bEnableZeMotors = true;
        }
        else {
            LOG(ERROR) << "Malformed ze-motor mapping at index " << i;
        }
    }
  }

  void LoadEyouMotors(const YAML::Node& yaml) {
    if (!yaml["ey_motors_mapping"]) {
      LOG(WARNING) << ("No ey motor setting.");
      return;
    }
    auto mappings_list = yaml["ey_motors_mapping"];

    eyouMotorMgr.reset(new eyou::EyouMotorDevMgr());
    eyouMotorMgr->SetOnStatusReady([this](bodyctrl_msgs::msg::MotorStatusMsg::SharedPtr msg){
        msg->header.stamp = rclcpp::Clock().now();
        CacheMessage cmsg;
        cmsg.type = CacheMessage::MessageType::EYOU_MOTOR;
        cmsg.msg = msg;
        msgQueue.push(cmsg);
        cvPubMsg.notify_one();
    });

    pubEyouMotorStatus = this->create_publisher<bodyctrl_msgs::msg::MotorStatusMsg>("ey/status", 1000);
    subCmdEyouMotorSetPos = this->create_subscription<bodyctrl_msgs::msg::CmdSetMotorPosition>(
      "ey/set_pos", 10, std::bind(&BodyControl::OnCmdSetEyouMotorPosition, this, std::placeholders::_1));

    // # motor_name_id, slave_index, passage_req, passage_resp, motor_id, control_mode, motor_type, kt_default
    LOG(INFO) << "EY Mappings loaded:";
    for (size_t i = 0; i < mappings_list.size(); ++i) {
        auto motor_mapping = mappings_list[i];
        if (motor_mapping.size() == 5) {
            LOG(INFO)
              << " " << motor_mapping[0].as<int>() << ","
              << " " << motor_mapping[1].as<int>() << ","
              << " " << motor_mapping[2].as<int>() << ","
              << " " << motor_mapping[3].as<int>() << ","
              << " " << motor_mapping[4].as<int>() << ",";

            eyouMotorMgr->NewDevice(
              motor_mapping[0].as<int>(),
              motor_mapping[1].as<int>(),
              motor_mapping[2].as<int>(),
              motor_mapping[3].as<int>(),
              motor_mapping[4].as<int>(),
              (int)eyou::EyouMotorDevice::Mode::POS,
              (int)eyou::EyouMotorDevice::Type::S,
              0.0
            );
            bEnableEyouMotors = true;
        } else {
            LOG(ERROR) << "Malformed ey-motor mapping at index " << i;
        }
    }
  }

  void onInit()
  {
    LOG(INFO) << "BodyControl onInit()";

    // Load YAML config files
    this->declare_parameter("config_file", std::string(""));
    this->declare_parameter("motor_setting_file", std::string(""));
    this->declare_parameter("imu_setting_file", std::string(""));
    this->declare_parameter("power_setting_file", std::string(""));
    this->declare_parameter("ey_motor_setting_file", std::string(""));

    auto config_file = this->get_parameter("config_file").as_string();
    auto motor_setting_file = this->get_parameter("motor_setting_file").as_string();
    auto imu_setting_file = this->get_parameter("imu_setting_file").as_string();
    auto power_setting_file = this->get_parameter("power_setting_file").as_string();
    auto ey_motor_setting_file = this->get_parameter("ey_motor_setting_file").as_string();

    YAML::Node configYaml, motorYaml, imuYaml, powerYaml, eyMotorYaml;
    try {
      if (!config_file.empty()) configYaml = YAML::LoadFile(config_file);
    } catch(const std::exception& e) { LOG(ERROR) << "Failed to load config YAML: " << e.what(); }
    try {
      if (!motor_setting_file.empty()) motorYaml = YAML::LoadFile(motor_setting_file);
    } catch(const std::exception& e) { LOG(ERROR) << "Failed to load motor YAML: " << e.what(); }
    try {
      if (!imu_setting_file.empty()) imuYaml = YAML::LoadFile(imu_setting_file);
    } catch(const std::exception& e) { LOG(ERROR) << "Failed to load imu YAML: " << e.what(); }
    try {
      if (!power_setting_file.empty()) powerYaml = YAML::LoadFile(power_setting_file);
    } catch(const std::exception& e) { LOG(ERROR) << "Failed to load power YAML: " << e.what(); }
    try {
      if (!ey_motor_setting_file.empty()) eyMotorYaml = YAML::LoadFile(ey_motor_setting_file);
    } catch(const std::exception& e) { LOG(ERROR) << "Failed to load ey motor YAML: " << e.what(); }

    LoadEthercatParam(configYaml);
    LoadRmImu(imuYaml);
    LoadXsensImu(imuYaml);
    LoadXsensHrImu(imuYaml);
    LoadMotors(motorYaml);
    LoadSriSensor(motorYaml);
    LoadPowerBoard(powerYaml);
    LoadTsinghuaHand(motorYaml);
    LoadZeroErrorMotors(motorYaml);
    LoadEyouMotors(eyMotorYaml);

    // get default netcard name
    if (configYaml["net_card_name"]) {
      nameOfNet = configYaml["net_card_name"].as<std::string>();
    }

    pubNodeState = this->create_publisher<bodyctrl_msgs::msg::NodeState>("node_state", 1);

    // publish node state
    std::thread([this](){
      while (rclcpp::ok()) {
      rclcpp::Rate r(1);
        auto msg = std::make_shared<bodyctrl_msgs::msg::NodeState>();
        msg->header.stamp = this->get_clock()->now();
        msg->state = nodeState;
        pubNodeState->publish(*msg);
        r.sleep();
      }
    }).detach();

    bool bAutoInit = false;
    if (configYaml["auto_init"]) {
      bAutoInit = configYaml["auto_init"].as<bool>();
    }
    if (bAutoInit) {
      autoInit(configYaml);
    }
  }

  void CheckReady() {
      rclcpp::Rate r(1);
      auto timeStart = this->get_clock()->now();
      while (rclcpp::ok()) {
        auto timeNow = this->get_clock()->now();
        
        // check all devices
        bool ready = true;
        if (bEnableMotors) {
          ready &= mdm->IsReady();
        }
        if (bEnableTsHands) {
          ready &= tsHandMgr->IsReady();
        }
        if (bEnableRmImu) {
          ready &= rmImu->IsReady();
        }
        if (bEnableXsensImu) {
          ready &= xImu->IsReady();
        }
        if (bEnableXsensHrImu) {
          ready &= xhrImu->IsReady();
        }
        if (bEnablePowerBoard) {
          ready &= power->IsReady();
        }
        if (bEnableSri) {
          ready &= sriMgr->IsReady();
        }
        if (bEnableZeMotors) {
          ready &= zMgr->IsReady();
        }
        if (bEnableEyouMotors) {
          ready &= eyouMotorMgr->IsReady();
        }

        if (ready) {
          LOG(INFO) << "All devices ready.";
          nodeState = bodyctrl_msgs::msg::NodeState::NODE_STATE_RUNNING;
          break;
        }

        if ((timeNow - timeStart).seconds() > 3) {

          std::unordered_map<int, std::string> motorNames = {
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
            {bodyctrl_msgs::msg::MotorName::MOTOR_WAIST, "MOTOR_WAIST"},
          };

          std::unordered_map<int, std::string> handNames = {
            {bodyctrl_msgs::msg::TsHandName::TSINGHUA_HAND_LEFT, "TSINGHUA_HAND_LEFT"},
            {bodyctrl_msgs::msg::TsHandName::TSINGHUA_HAND_RIGHT, "TSINGHUA_HAND_RIGHT"},
          };

          if (bEnableMotors) {
            LOG(WARNING) << "motors state:"  << (mdm->IsReady() ? "ok" : "not");
            for (auto& name : mdm->GetNotReadyList()) {
              LOG(WARNING) << ("Not ready motor: %s", motorNames[name].c_str());
            }
          }

          if (bEnableZeMotors) {
            LOG(WARNING) << "ze motors state:"  << (zMgr->IsReady() ? "ok" : "not");
            for (auto& name : zMgr->GetNotReadyList()) {
              LOG(WARNING) << ("Not ready ze motor: %s", motorNames[name].c_str());
            }
          }

          if (bEnableEyouMotors) {
            LOG(WARNING) << "ey motors state:"  << (eyouMotorMgr->IsReady() ? "ok" : "not");
            for (auto& name : eyouMotorMgr->GetNotReadyList()) {
              LOG(WARNING) << ("Not ready ey motor: %s", motorNames[name].c_str());
            }
          }

          if (bEnableTsHands) {
            LOG(WARNING) << "tshand state:"  << (tsHandMgr->IsReady() ? "ok" : "not");
            for (auto& name : tsHandMgr->GetNotReadyList()) {
              LOG(WARNING) << ("Not ready hand: %s", handNames[name].c_str());
            }
          }

          if (bEnableRmImu) {
            LOG(WARNING) << "rm_imu state:"  << (rmImu->IsReady() ? "ok" : "not");
          }
          if (bEnableXsensImu) {
            LOG(WARNING) << "xsens imu state:"  << (xImu->IsReady() ? "ok" : "not");
          }
          if (bEnableXsensHrImu) {
            LOG(WARNING) << "xsens hr imu state:"  << (xhrImu->IsReady() ? "ok" : "not");
          }
          if (bEnablePowerBoard) {
            LOG(WARNING) << "power board state:"  << (power->IsReady() ? "ok" : "not");
          }
          if (bEnableSri) {
            LOG(WARNING) << "sri state:"  << (sriMgr->IsReady() ? "ok" : "not");
          }


          break;
        }
        r.sleep();
      } 
  }

  void WaitAndSendReady() {
    LOG(INFO) << "Wait 3 sec for all devices ready...";
    std::this_thread::sleep_for(std::chrono::seconds(3));
    LOG(INFO) << "Send ready state.";
    nodeState = bodyctrl_msgs::msg::NodeState::NODE_STATE_RUNNING;
  }

  bool autoInit(const YAML::Node& configYaml) {
    std::thread(&BodyControl::RunPubMsg, this).detach();
    std::thread([this, configYaml](){
      LOG(INFO) << "Start to initialize SOEM master.";
      // TO DO: load ext-can slave from parameters
      auto rlt = SoemMaster::Instance().Init(nameOfNet, 1100, vecSlaveMode);
      if (!rlt) {
        LOG(ERROR) << ("SOEM init failed.");
      }
      // get default netcard name
      auto enableCheckReady = false;
      if (configYaml["enable_check_ready"]) {
        enableCheckReady = configYaml["enable_check_ready"].as<bool>();
      }
      if (enableCheckReady) {
        CheckReady();
      }
      else {
        WaitAndSendReady();
      }
    }).detach();
    return true;
  }

  void OnCmdMotorCtrlMsg(bodyctrl_msgs::msg::CmdMotorCtrl::ConstSharedPtr msg)
  {
    if (nodeState != bodyctrl_msgs::msg::NodeState::NODE_STATE_RUNNING) {
      return;
    }
    for (auto& cmd : msg->cmds) {
      try {
        const auto& dev = mdm->GetDevice(cmd.name);
        if (dev->IsReady()) {
          dev->SendMotorCtrlCmd(cmd.kp, cmd.kd, cmd.pos, cmd.spd, cmd.tor);
        }
      } catch(std::exception& e) {}
    }
  }

  void OnCmdSetMotorPosition(bodyctrl_msgs::msg::CmdSetMotorPosition::ConstSharedPtr msg)
  {
    if (nodeState != bodyctrl_msgs::msg::NodeState::NODE_STATE_RUNNING) {
      return;
    }
    for (auto& cmd : msg->cmds) {
      try {
        const auto& dev = mdm->GetDevice(cmd.name);
        if (dev->IsReady()) {
          dev->SetMotorPosition(cmd.pos, cmd.spd, cmd.cur);
        }
      } catch(std::exception& e) {}
    }
  }

  void OnCmdSetMotorDistance(bodyctrl_msgs::msg::CmdSetMotorDistance::ConstSharedPtr msg)
  {
    if (nodeState != bodyctrl_msgs::msg::NodeState::NODE_STATE_RUNNING) {
      return;
    }
    for (auto& cmd : msg->cmds) {
      try {
        const auto& dev = mdm->GetDevice(cmd.name);
        if (dev->IsReady()) {
          dev->SetMotorDistance(cmd.distance, cmd.spd, cmd.cur);
        }
      } catch(std::exception& e) {}
    }
  }

  void OnCmdSetMotorSpeed(bodyctrl_msgs::msg::CmdSetMotorSpeed::ConstSharedPtr msg)
  {
    if (nodeState != bodyctrl_msgs::msg::NodeState::NODE_STATE_RUNNING) {
      return;
    }
    for (auto& cmd : msg->cmds) {
      try {
        const auto& dev = mdm->GetDevice(cmd.name);
        if (dev->IsReady()) {
          dev->SetMotorSpeed(cmd.spd, cmd.cur);
        }
      } catch(std::exception& e) {}
    }
  }

  void OnCmdSetZeMotorFphc(bodyctrl_msgs::msg::CmdMotorCtrl::ConstSharedPtr msg) {
    if (nodeState != bodyctrl_msgs::msg::NodeState::NODE_STATE_RUNNING) {
      return;
    }
    for (auto& cmd : msg->cmds) {
      try {
        const auto& dev = zMgr->GetDevice(cmd.name);
        if (dev->IsReady()) {
          dev->SetFphc(cmd.kp, cmd.kd, cmd.pos, cmd.spd, cmd.tor);
        }
      } catch(std::exception& e) {}
    }
  }

  void OnCmdSetZeMotorPosition(bodyctrl_msgs::msg::CmdSetMotorPosition::ConstSharedPtr msg) {
    if (nodeState != bodyctrl_msgs::msg::NodeState::NODE_STATE_RUNNING) {
      return;
    }
    for (auto& cmd : msg->cmds) {
      try {
        const auto& dev = zMgr->GetDevice(cmd.name);
        if (dev->IsReady()) {
          dev->SetPos(cmd.pos, cmd.spd);
        }
      } catch(std::exception& e) {}
    }
  }

  void OnCmdSetEyouMotorPosition(bodyctrl_msgs::msg::CmdSetMotorPosition::ConstSharedPtr msg) {
    if (nodeState != bodyctrl_msgs::msg::NodeState::NODE_STATE_RUNNING) {
      return;
    }
    for (auto& cmd : msg->cmds) {
      try {
        const auto& dev = eyouMotorMgr->GetDevice(cmd.name);
        if (dev->IsReady()) {
          dev->SetPos(cmd.pos, cmd.spd);
        }
      } catch(std::exception& e) {}
    }
  }

  void OnCmdSetSetTsHandPosition(bodyctrl_msgs::msg::CmdSetTsHandPosition::ConstSharedPtr msg)
  {
    if (nodeState != bodyctrl_msgs::msg::NodeState::NODE_STATE_RUNNING) {
      return;
    }
    for (auto& cmd : msg->cmds) {
      try {
        const auto& dev = tsHandMgr->GetDevice(cmd.name);
        if (dev->IsReady()) {
          FingerSetPositionCmd fcmd;
          fcmd.thumb.rotation.enable = true;
          fcmd.thumb.rotation.angle = cmd.rotation_angle;
          fcmd.thumb.bend.enable = true;
          fcmd.thumb.bend.angle = cmd.bend_angle[0];
          fcmd.fore.bend.enable = true;
          fcmd.fore.bend.angle = cmd.bend_angle[1];
          fcmd.middle.bend.enable = true;
          fcmd.middle.bend.angle = cmd.bend_angle[2];
          fcmd.ring.bend.enable = true;
          fcmd.ring.bend.angle = cmd.bend_angle[3];
          fcmd.little.bend.enable = true;
          fcmd.little.bend.angle = cmd.bend_angle[4];
          dev->SetPos(fcmd);
        }
      } catch(std::exception& e) {}
    }
  }

  void OnCmdSetSetTsHandCtrl(bodyctrl_msgs::msg::CmdSetTsHandCtrl::ConstSharedPtr msg)
  {
    if (nodeState != bodyctrl_msgs::msg::NodeState::NODE_STATE_RUNNING) {
      return;
    }
    for (auto& cmd : msg->cmds) {
      try {
        const auto& dev = tsHandMgr->GetDevice(cmd.name);
        if (dev->IsReady()) {
          FingerMotionCtrlCmd fcmd;
          memset((void*)&fcmd, 0x00, sizeof(FingerMotionCtrlCmd));
          fcmd.thumb.rotation.enable = true;
          fcmd.thumb.rotation.vel = cmd.rotation.vel;
          fcmd.thumb.rotation.startAngle = cmd.rotation.start_angle;
          fcmd.thumb.rotation.maxAngle = cmd.rotation.max_angle;
          fcmd.thumb.bend.enable = true;
          fcmd.thumb.bend.vel = cmd.bend[0].vel;
          fcmd.thumb.bend.startAngle = cmd.bend[0].start_angle;
          fcmd.thumb.bend.maxAngle = cmd.bend[0].max_angle;
          fcmd.thumb.threshold = cmd.threshold[0];
          fcmd.fore.bend.enable = true;
          fcmd.fore.bend.vel = cmd.bend[1].vel;
          fcmd.fore.bend.startAngle = cmd.bend[1].start_angle;
          fcmd.fore.bend.maxAngle = cmd.bend[1].max_angle;
          fcmd.fore.threshold = cmd.threshold[1];
          fcmd.middle.bend.enable = true;
          fcmd.middle.bend.vel = cmd.bend[2].vel;
          fcmd.middle.bend.startAngle = cmd.bend[2].start_angle;
          fcmd.middle.bend.maxAngle = cmd.bend[2].max_angle;
          fcmd.middle.threshold = cmd.threshold[2];
          fcmd.ring.bend.enable = true;
          fcmd.ring.bend.vel = cmd.bend[3].vel;
          fcmd.ring.bend.startAngle = cmd.bend[3].start_angle;
          fcmd.ring.bend.maxAngle = cmd.bend[3].max_angle;
          fcmd.ring.threshold = cmd.threshold[3];
          fcmd.little.bend.enable = true;
          fcmd.little.bend.vel = cmd.bend[4].vel;
          fcmd.little.bend.startAngle = cmd.bend[4].start_angle;
          fcmd.little.bend.maxAngle = cmd.bend[4].max_angle;
          fcmd.little.threshold = cmd.threshold[4];
          dev->SetMotionCtrl(fcmd);
        }
      } catch(std::exception& e) {}
    }
  }

  

  rclcpp::Subscription<bodyctrl_msgs::msg::CmdMotorCtrl>::SharedPtr subCmdMotorCtrl;
  rclcpp::Subscription<bodyctrl_msgs::msg::CmdSetMotorPosition>::SharedPtr subCmdSetMotorPosition;
  rclcpp::Subscription<bodyctrl_msgs::msg::CmdSetMotorDistance>::SharedPtr subCmdSetMotorDistance;
  rclcpp::Subscription<bodyctrl_msgs::msg::CmdSetMotorSpeed>::SharedPtr subCmdSetMotorSpeed;
  rclcpp::Subscription<bodyctrl_msgs::msg::CmdSetMotorCurTor>::SharedPtr subCmdSetMotorCurTor;
  rclcpp::Subscription<bodyctrl_msgs::msg::CmdSetTsHandPosition>::SharedPtr subCmdTsHandSetPos;
  rclcpp::Subscription<bodyctrl_msgs::msg::CmdSetTsHandCtrl>::SharedPtr subCmdTsHandSetCtrl;
  rclcpp::Subscription<bodyctrl_msgs::msg::CmdSetMotorPosition>::SharedPtr subCmdZeMotorSetPos;
  rclcpp::Subscription<bodyctrl_msgs::msg::CmdSetMotorPosition>::SharedPtr subCmdEyouMotorSetPos;
  rclcpp::Subscription<bodyctrl_msgs::msg::CmdMotorCtrl>::SharedPtr subCmdZeMotorSetFphc;
  rclcpp::Publisher<bodyctrl_msgs::msg::NodeState>::SharedPtr pubNodeState;
  fast_ros::Publisher pubMotorsState;
  fast_ros::Publisher pubImu;
  fast_ros::Publisher pubXImuHr;
  rclcpp::Publisher<bodyctrl_msgs::msg::Sri>::SharedPtr pubSri;
  rclcpp::Publisher<bodyctrl_msgs::msg::PowerStatus>::SharedPtr pubPowerStatus;
  rclcpp::Publisher<bodyctrl_msgs::msg::PowerBoardKeyStatus>::SharedPtr pubPowerBoardKeyStatus;
  rclcpp::Publisher<bodyctrl_msgs::msg::TsHandStatusMsg>::SharedPtr pubHandsStatus;
  rclcpp::Publisher<bodyctrl_msgs::msg::MotorStatusMsg>::SharedPtr pubZeMotorStatus;
  rclcpp::Publisher<bodyctrl_msgs::msg::MotorStatusMsg>::SharedPtr pubEyouMotorStatus;

  rclcpp::TimerBase::SharedPtr init_timer_;

  std::string nameOfNet;

  std::mutex mtxPubMsgCv;
  std::condition_variable cvPubMsg;
  LockFreeQueue<bodyctrl_msgs::msg::MotorStatusMsg> msgStatusQueue;
  struct CacheMessage {
    enum class MessageType : int {
      MOTOR,
      IMU,
      IMU_HR,
      POWER,
      POWER_KEY,
      SRI,
      HAND_TSINGHUA,
      ZE_MOTOR,
      EYOU_MOTOR
    } type;
    std::any msg;
  };
  LockFreeQueue<CacheMessage> msgQueue;

  std::unique_ptr<MotorDeviceManager> mdm;
  std::unique_ptr<TsinghuaHandDeviceManager> tsHandMgr;
  std::shared_ptr<RmImuDevice> rmImu;
  std::shared_ptr<XsensImuDevice> xImu;
  std::shared_ptr<XsensImuHRDevice> xhrImu;
  std::shared_ptr<SriDeviceManager> sriMgr;
  std::shared_ptr<PowerBoardDevice> power;
  std::shared_ptr<ZeroErrMotorDevMgr> zMgr;
  std::shared_ptr<eyou::EyouMotorDevMgr> eyouMotorMgr;
  
  uint16_t nodeState = bodyctrl_msgs::msg::NodeState::NODE_STATE_IDLE;

  std::vector<SoemMaster::Mode> vecSlaveMode;

  bool bEnableMotors = false;
  bool bEnableTsHands = false;
  bool bEnableRmImu = false;
  bool bEnableXsensImu = false;
  bool bEnableXsensHrImu = false;
  bool bEnableSri = false;
  bool bEnableZeMotors = false;
  bool bEnableEyouMotors = false;
  bool bEnablePowerBoard = false;
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(body_control::BodyControl)
