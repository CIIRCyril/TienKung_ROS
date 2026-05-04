#pragma once

#include <rclcpp/rclcpp.hpp>
#include "Publisher.h"
#include "Subscriber.h"
#include "FastConnection.h"

namespace fast_ros {

class NodeHandle {
public:
    NodeHandle(rclcpp::Node::SharedPtr node);

    template <class M>
    Publisher advertise(const std::string& topic, uint32_t queue_size, ConnectionType type = ConnectionType::ALL, bool latch = false)
    {
        Publisher pub;
        pub.type = type;
        rclcpp::QoS qos(queue_size);
        if (latch) {
            qos.transient_local();
        }
        pub.pubRos = node_->create_publisher<M>(topic, qos);
        auto ns = std::string(node_->get_namespace());
        auto nsTopic = ns + "/" + topic;
        pub.funcPubFast = [nsTopic](const std::any& message) {
            static FastConnection& fc = FastConnection::Instance();
            fc.publish(nsTopic, message);
        };
        return pub;
    }

    template<class M, class T>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(T::*fp)(const std::shared_ptr<M const>&), T* obj,
        ConnectionType type = ConnectionType::FAST_ROS)
    {
        Subscriber sub;
        if (type & ConnectionType::FAST_ROS) {
            auto func = std::bind(fp, obj, std::placeholders::_1);
            auto funcOnMessage = [func](const std::any& message){
                auto rawMessage = std::any_cast<std::shared_ptr<M>>(message);
                func(rawMessage);
            };
            FastConnection::Instance().subscribe(topic, funcOnMessage);
        }

        if (type & ConnectionType::NATIVE_ROS) {
            sub.subRos = node_->create_subscription<M>(topic, rclcpp::QoS(queue_size),
                [obj, fp](const std::shared_ptr<M const>& msg) {
                    (obj->*fp)(msg);
                });
        }

        return sub;
    }

private:
    rclcpp::Node::SharedPtr node_;
};

}