#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <algorithm>
#include <vector>

using std::placeholders::_1;

class CollisionMonitor : public rclcpp::Node
{
public:
    CollisionMonitor() : Node("collision_monitor")
    {
        // Declare safety distance parameters
        this->declare_parameter("slow_safety_distance", 2);
        this->declare_parameter("stop_safety_distance", 1);
        this->declare_parameter("emergency_distance", 0.5);

        // Subscribe to the scan topic to receive LaserScan data
        laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&CollisionMonitor::scanCallback, this, _1));

        // Subscribe to the odometry topic to receive odometry data
        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&CollisionMonitor::odometryCallback, this, _1));

        // Publisher for the modified (safe) velocity commands
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_safe", 10);

        // Print loaded parameters for debugging
        RCLCPP_INFO(this->get_logger(), "Slow Safety Distance: %.2f", this->get_parameter("slow_safety_distance").as_double());
        RCLCPP_INFO(this->get_logger(), "Stop Safety Distance: %.2f", this->get_parameter("stop_safety_distance").as_double());
        RCLCPP_INFO(this->get_logger(), "Emergency Distance: %.2f", this->get_parameter("emergency_distance").as_double());
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        float min_range = *std::min_element(msg->ranges.begin(), msg->ranges.end());
        RCLCPP_INFO(this->get_logger(), "Minimum range from scan: %.2f", min_range);

        double slow_safety_distance = this->get_parameter("slow_safety_distance").as_double();
        double stop_safety_distance = this->get_parameter("stop_safety_distance").as_double();
        double emergency_distance = this->get_parameter("emergency_distance").as_double();

        geometry_msgs::msg::Twist safe_cmd_vel;

        if (min_range < emergency_distance)
        {
            // Emergency stop
            safe_cmd_vel.linear.x = 0.0;
            safe_cmd_vel.angular.z = 0.0;
            RCLCPP_WARN(this->get_logger(), "Emergency stop activated!");
        }
        else if (min_range < stop_safety_distance)
        {
            // Full stop
            safe_cmd_vel.linear.x = 0.0;
            RCLCPP_WARN(this->get_logger(), "Full stop activated!");
        }
        else if (min_range < slow_safety_distance)
        {
            // Slow down
            safe_cmd_vel.linear.x = current_linear_velocity_.x * 0.5;
            safe_cmd_vel.angular.z = current_angular_velocity_.z * 0.5;
            RCLCPP_WARN(this->get_logger(), "Slowing down!");
        }
        else
        {
            // No obstacle detected within safety distances, maintain current velocity
            safe_cmd_vel.linear.x = current_linear_velocity_.x;
            safe_cmd_vel.angular.z = current_angular_velocity_.z;
        }

        cmd_vel_publisher_->publish(safe_cmd_vel);
        RCLCPP_INFO(this->get_logger(), "Published safe_cmd_vel: linear=%.2f angular=%.2f", safe_cmd_vel.linear.x, safe_cmd_vel.angular.z);
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_position_ = msg->pose.pose.position;
        current_orientation_ = msg->pose.pose.orientation;
        current_linear_velocity_ = msg->twist.twist.linear;
        current_angular_velocity_ = msg->twist.twist.angular;

        RCLCPP_INFO(this->get_logger(), "Received odometry: position(%.2f, %.2f, %.2f) orientation(%.2f, %.2f, %.2f, %.2f)",
                    current_position_.x, current_position_.y, current_position_.z,
                    current_orientation_.x, current_orientation_.y, current_orientation_.z, current_orientation_.w);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    geometry_msgs::msg::Point current_position_;
    geometry_msgs::msg::Quaternion current_orientation_;
    geometry_msgs::msg::Vector3 current_linear_velocity_;
    geometry_msgs::msg::Vector3 current_angular_velocity_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CollisionMonitor>());
    rclcpp::shutdown();
    return 0;
}
