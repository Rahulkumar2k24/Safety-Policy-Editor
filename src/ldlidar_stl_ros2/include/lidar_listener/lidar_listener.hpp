#ifndef LIDAR_LISTENER_HPP
#define LIDAR_LISTENER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>
#include <map>

class LiDARListener : public rclcpp::Node {
public:
    struct Point {
        double x;
        double y;
    };

    LiDARListener();

private:
    void frontCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void leftCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void rightCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void velocity_callback2(const geometry_msgs::msg::Twist::SharedPtr msg);
    void safety_turnoff_callback(const std_msgs::msg::String::SharedPtr msg);

    void processScan(const sensor_msgs::msg::LaserScan::SharedPtr msg, const std::string &side);
    bool isPointInPolygon(const std::vector<Point> &polygon, const Point &point);
    Point polarToCartesian(float range, float angle);
    Point rotatePoint(Point p, float angle);

    void loadSafetyConfig();
    void loadZonesFromDB(const std::string &dbPath, const std::string &side);
    void visualizeZone(const std::string &zoneName);
    void visualizeZones();
    void displayZoneData(const std::string &zoneName);
    void slow_down_robot();
    void stop_robot();
    void check_for_status_in_three_lidar();

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_front;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_left;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_right;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safety_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

    std::map<std::string, std::vector<Point>> zones;
    std::map<std::string, std::string> status_map;
    geometry_msgs::msg::Twist current_velocity_;
    geometry_msgs::msg::Twist cmd_vel_velocity_;
    bool slowdown_initiated;
    bool stop_initiated;
    bool isSafetyTurnOff;
    bool pickdrop;
    int global_danger_count;
    int global_warning_count;
};

#endif // LIDAR_LISTENER_HPP
