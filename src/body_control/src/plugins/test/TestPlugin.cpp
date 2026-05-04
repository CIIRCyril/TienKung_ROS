#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/float64.hpp>
#include <stdio.h>
#include <glog/logging.h>
#include <bodyctrl_msgs/msg/cmd_set_motor_speed.hpp>
#include <bodyctrl_msgs/msg/motor_name.hpp>
#include <bodyctrl_msgs/msg/node_state.hpp>
#include <bodyctrl_msgs/msg/motor_status_msg.hpp>
#include <bodyctrl_msgs/msg/cmd_set_motor_position.hpp>

#include <math.h> //fabs

#include <thread>
#include <mutex>
#include <cmath>

#include "PreciseRate.h"
#include "HighPrecisionRate.h"
#include "util/SinTime.h"

namespace body_control  // The usage of the namespace is a good practice but not mandatory
{

class TestPlugin : public rclcpp::Node
{
public:
  explicit TestPlugin(const rclcpp::NodeOptions & options)
    : Node("TestPlugin", options)
  {
    onInit();
  }

private:
  void onInit()
  {
    pubSetMotorSpeed = this->create_publisher<bodyctrl_msgs::msg::CmdSetMotorSpeed>("/BodyControl/set_motor_speed", 1000);
    pubSetHeadPos = this->create_publisher<bodyctrl_msgs::msg::CmdSetMotorPosition>("/BodyControl/ey/set_pos", 1000);
    pubSetWaistPos = this->create_publisher<bodyctrl_msgs::msg::CmdSetMotorPosition>("/BodyControl/ze/set_pos", 1000);

    subNodeState = this->create_subscription<bodyctrl_msgs::msg::NodeState>(
      "/BodyControl/node_state", 1000, std::bind(&TestPlugin::OnNodeState, this, std::placeholders::_1));
    subLegStatus = this->create_subscription<bodyctrl_msgs::msg::MotorStatusMsg>(
      "/BodyControl/motor_state", 1000, std::bind(&TestPlugin::OnLegStatus, this, std::placeholders::_1));
    subHeadStatus = this->create_subscription<bodyctrl_msgs::msg::MotorStatusMsg>(
      "/BodyControl/ey/status", 1000, std::bind(&TestPlugin::OnHeadStatus, this, std::placeholders::_1));
    subWaistStatus = this->create_subscription<bodyctrl_msgs::msg::MotorStatusMsg>(
      "/BodyControl/ze/status", 1000, std::bind(&TestPlugin::OnWaistStatus, this, std::placeholders::_1));
  }

  void OnNodeState(bodyctrl_msgs::msg::NodeState::ConstSharedPtr msg)
  {
    if (msg->state == bodyctrl_msgs::msg::NodeState::NODE_STATE_RUNNING) {
      std::thread([this]() {
        bool ready = true;
        for (auto i = 0; i < 12; ++i) {
          ready = ready && inited_leg[i];
        }
        if (ready) {
          if (!module_started[0]) {
            module_started[0] = true;
            TestLegs();
          }
        }
      }).detach();

      std::thread([this]() {
        bool ready = true;
        for (auto i = 0; i < 3; ++i) {
          ready = ready && inited_head[i];
        }
        if (ready) {
          if (!module_started[1]) {
            module_started[1] = true;
            TestHead();
          }
        }
      }).detach();

      std::thread([this]() {
        if (inited_waist) {
          if (!module_started[2]) {
            module_started[2] = true;
            TestWaist();
          }
        }
        
      }).detach();
    }
  }

  void OnLegStatus(bodyctrl_msgs::msg::MotorStatusMsg::ConstSharedPtr msg)
  {
    for (auto& data : msg->status) {
      auto i = data.name - 1;
      if (!inited_leg[i]) {
        start_pos_leg[i] = data.pos;
        inited_leg[i] = true;
      }
    }
  }

  void OnHeadStatus(bodyctrl_msgs::msg::MotorStatusMsg::ConstSharedPtr msg)
  {
    for (auto& data : msg->status) {
      auto i = data.name - bodyctrl_msgs::msg::MotorName::MOTOR_HEAD_TOP;
      if (!inited_head[i]) {
        start_pos_head[i] = data.pos;
        inited_head[i] = true;
      }
    }
  }

  void OnWaistStatus(bodyctrl_msgs::msg::MotorStatusMsg::ConstSharedPtr msg)
  {
    for (auto& data : msg->status) {
      if (!inited_waist) {
        start_pos_waist = data.pos;
        inited_waist = true;
        LOG(INFO) << "start_pos_waist:" << start_pos_waist;
      }
    }
  }

  void TestLegs() {
      rclcpp::Rate rate(400);
      long count = 0;
      double spd_base = 1.5;
      double spd;
      double cur = 2.0; // A
      SinTime st;
      st.init(spd_base);
      while (rclcpp::ok()) {
        // 正弦变速
        spd = st.update(spd_base);

        bodyctrl_msgs::msg::CmdSetMotorSpeed msg;

        // LEG-LEFT
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_LEFT_1;
          cmd.spd = spd * 0.5;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_LEFT_2;
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_LEFT_3;
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_LEFT_4;
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_LEFT_5;
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_LEFT_6;
          cmd.spd = -spd;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        
        // LEG-RIGHT
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_RIGHT_1;
          cmd.spd = -spd * 0.5;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_RIGHT_2;
          cmd.spd = -spd;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_RIGHT_3;
          cmd.spd = -spd;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_RIGHT_4;
          cmd.spd = -spd;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_RIGHT_5;
          cmd.spd = -spd;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_RIGHT_6;
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        
        pubSetMotorSpeed->publish(msg);
        rate.sleep();
      }
  }

  // for 13715 motor
  void TestLegs2() {
      rclcpp::Rate rate(400);
      long count = 0;
      double spd_base = 1.5;
      double spd;
      double cur = 2.0; // A
      SinTime st;
      st.init(spd_base);
      while (rclcpp::ok()) {
        // 正弦变速
        spd = st.update(spd_base);

        bodyctrl_msgs::msg::CmdSetMotorSpeed msg;

        // LEG-LEFT
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_LEFT_1;
          cmd.spd = spd * 0.5;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_LEFT_2;
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_LEFT_3;
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_LEFT_4;
          cmd.spd = -spd;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_LEFT_5;
          cmd.spd = -spd;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_LEFT_6;
          cmd.spd = -spd;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        
        // LEG-RIGHT
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_RIGHT_1;
          cmd.spd = -spd * 0.5;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_RIGHT_2;
          cmd.spd = -spd;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_RIGHT_3;
          cmd.spd = -spd;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_RIGHT_4;
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_RIGHT_5;
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::msg::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::msg::MotorName::MOTOR_LEG_RIGHT_6;
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = rclcpp::Clock().now();
          msg.cmds.push_back(cmd);
        }
        
        pubSetMotorSpeed->publish(msg);
        rate.sleep();
      }
  }

  void TestHead() {
    rclcpp::Rate rate(0.1);
    float pos = 0.15;
    float spd = 0.5;
    while (rclcpp::ok()) {
      bodyctrl_msgs::msg::CmdSetMotorPosition msg;

      bodyctrl_msgs::msg::SetMotorPosition cmd1;
      cmd1.name = bodyctrl_msgs::msg::MotorName::MOTOR_HEAD_TOP;
      cmd1.pos = start_pos_head[0] + pos;
      cmd1.spd = spd;
      cmd1.cur = 1.0;

      bodyctrl_msgs::msg::SetMotorPosition cmd2;
      cmd2.name = bodyctrl_msgs::msg::MotorName::MOTOR_HEAD_LEFT;
      cmd2.pos = start_pos_head[1] + pos;
      cmd2.spd = spd;
      cmd2.cur = 1.0;

      bodyctrl_msgs::msg::SetMotorPosition cmd3;
      cmd3.name = bodyctrl_msgs::msg::MotorName::MOTOR_HEAD_RIGHT;
      cmd3.pos = start_pos_head[2] + pos;
      cmd3.spd = spd;
      cmd3.cur = 1.0;

      msg.cmds.push_back(cmd1);
      msg.cmds.push_back(cmd2);
      msg.cmds.push_back(cmd3);

      pubSetHeadPos->publish(msg);
      rate.sleep();

      pos = -pos;
    }
  }

  void TestWaist() {
    rclcpp::Rate rate(0.2);
    float pos = 0.3;
    float spd = 0.3;
    while (rclcpp::ok()) {
      bodyctrl_msgs::msg::CmdSetMotorPosition msg;

      bodyctrl_msgs::msg::SetMotorPosition cmd1;
      cmd1.name = bodyctrl_msgs::msg::MotorName::MOTOR_WAIST;
      cmd1.pos = start_pos_waist + pos;
      cmd1.spd = spd;
      cmd1.cur = 1.0;

      msg.cmds.push_back(cmd1);

      pubSetWaistPos->publish(msg);
      rate.sleep();

      pos = -pos;
    }
  }

  bool inited_waist = false;
  bool inited_head[3] = {0};
  bool inited_leg[12] = {0};
  float start_pos_waist;
  float start_pos_head[3];
  float start_pos_leg[12];

  bool module_started[3] = {0};

  rclcpp::Subscription<bodyctrl_msgs::msg::NodeState>::SharedPtr subNodeState;
  rclcpp::Subscription<bodyctrl_msgs::msg::MotorStatusMsg>::SharedPtr subLegStatus;
  rclcpp::Subscription<bodyctrl_msgs::msg::MotorStatusMsg>::SharedPtr subHeadStatus;
  rclcpp::Subscription<bodyctrl_msgs::msg::MotorStatusMsg>::SharedPtr subWaistStatus;

  rclcpp::Publisher<bodyctrl_msgs::msg::CmdSetMotorSpeed>::SharedPtr pubSetMotorSpeed;
  rclcpp::Publisher<bodyctrl_msgs::msg::CmdSetMotorPosition>::SharedPtr pubSetHeadPos;
  rclcpp::Publisher<bodyctrl_msgs::msg::CmdSetMotorPosition>::SharedPtr pubSetWaistPos;
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(body_control::TestPlugin)
