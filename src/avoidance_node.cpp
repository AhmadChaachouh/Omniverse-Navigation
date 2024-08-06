#include "/home/ahmad/Navi_WS/src/omniverse_navigation/include/omniverse_navigation/avoidance_node.hpp"
#include <vector>
#include <cmath>

VFFNode::VFFNode() : Node("vff_node") {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&VFFNode::laser_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void VFFNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float repulsive_force_x = 0.0;
    float repulsive_force_y = 0.0;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float range = msg->ranges[i];
        if (range < msg->range_max && range > msg->range_min) {
            float angle = msg->angle_min + i * msg->angle_increment;
            float force = 0.1 / range; // simple inverse distance weighting
            repulsive_force_x += force * std::cos(angle);
            repulsive_force_y += force * std::sin(angle);
        }
    }

    // Normalize the repulsive forces
    float magnitude = std::sqrt(repulsive_force_x * repulsive_force_x + repulsive_force_y * repulsive_force_y);
    if (magnitude > 0.0) {
        repulsive_force_x /= magnitude;
        repulsive_force_y /= magnitude;
    }

    // Compute the desired velocity
    float linear_x = 0.2 - repulsive_force_x;
    float angular_z = -repulsive_force_y;

    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = linear_x;
    twist_msg.angular.z = angular_z;

    publisher_->publish(twist_msg);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VFFNode>());
    rclcpp::shutdown();
    return 0;
}
