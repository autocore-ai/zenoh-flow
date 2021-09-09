#include "replay_planner/sub_node.hpp"

MinimalSubscriber::MinimalSubscriber(): Node("test")
{
    subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
}

void MinimalSubscriber::topic_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", std::to_string(msg->header.stamp.sec).c_str());
}