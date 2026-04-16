#include "fast_ros/NodeHandle.h"

namespace fast_ros {

NodeHandle::NodeHandle(rclcpp::Node::SharedPtr node) : node_(node) {
}

}