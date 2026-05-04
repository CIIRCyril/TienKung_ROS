#pragma once

#include <rclcpp/rclcpp.hpp>
#include "ConnectionType.h"

namespace fast_ros {

class Subscriber {

public:
    void shutdown();

private:
    friend class NodeHandle;

    ConnectionType type;
    std::function<void()> funcShutdown;
    rclcpp::SubscriptionBase::SharedPtr subRos;
};

}