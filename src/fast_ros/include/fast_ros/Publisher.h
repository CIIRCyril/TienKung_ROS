#pragma once

#include <rclcpp/rclcpp.hpp>
#include <any>
#include "ConnectionType.h"

namespace fast_ros {

class Publisher {
public:
    Publisher();
    template <typename M>
    void publish(const std::shared_ptr<M>& message) const
    {
        if (type & ConnectionType::FAST_ROS) {
            funcPubFast(message);
        }

        if (type & ConnectionType::NATIVE_ROS) {
            auto typed_pub = std::dynamic_pointer_cast<rclcpp::Publisher<M>>(pubRos);
            if (typed_pub) {
                typed_pub->publish(*message);
            }
        }
    }
private:
    friend class NodeHandle;

    ConnectionType type;
    std::shared_ptr<rclcpp::PublisherBase> pubRos;
    std::function<void(const std::any&)> funcPubFast;
};

}