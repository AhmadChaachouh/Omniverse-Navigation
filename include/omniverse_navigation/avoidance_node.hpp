#ifndef AVOIDANCE_NODE_HPP
#define AVOIDANCE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

class VFFNode : public rclcpp::Node {
public:
    VFFNode();

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

#endif // VFF_NODE_HPP
