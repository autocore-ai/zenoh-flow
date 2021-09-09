#ifndef SUB_NODE_HPP
#define SUB_NODE_HPP

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
   explicit MinimalSubscriber();
   void topic_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg) const;
private:
   rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_;
};

#endif