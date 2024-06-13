#include "lidar_listener/lidar_listener.hpp"
#include "matplotlibcpp.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <sqlite3.h>

namespace plt = matplotlibcpp;

LiDARListener::LiDARListener() : Node("lidar_listener"), slowdown_initiated(false), stop_initiated(false), isSafetyTurnOff(false) {
    subscription_front = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/front_scan", 10, std::bind(&LiDARListener::frontCallback, this, std::placeholders::_1));
    subscription_left = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/l_scan", 10, std::bind(&LiDARListener::leftCallback, this, std::placeholders::_1));
    subscription_right = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/r_scan", 10, std::bind(&LiDARListener::rightCallback, this, std::placeholders::_1));
    safety_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "safety_turnoff", 10, std::bind(&LiDARListener::safety_turnoff_callback, this, std::placeholders::_1));
    velocity_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        "/odometry/filtered", 10, std::bind(&LiDARListener::velocity_callback, this, std::placeholders::_1));
    cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&LiDARListener::velocity_callback2, this, std::placeholders::_1));
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_remapped", 10);

    global_danger_count = 0;
    global_warning_count = 0;
    status_map["front"] = "safe";
    status_map["left"] = "safe";
    status_map["right"] = "safe";
    pickdrop = false;

    loadSafetyConfig();
    visualizeZones(); // Visualize all zones after loading configurations
    displayZoneData("front"); // Display data for each zone
    displayZoneData("left");
    displayZoneData("right");
}

void LiDARListener::loadSafetyConfig() {
    loadZonesFromDB("/home/fbots/ldlidar_ros2_ws/lidar_visualizer/DB_Data/Fork_Side.sqlite", "front");
    loadZonesFromDB("/home/fbots/ldlidar_ros2_ws/lidar_visualizer/DB_Data/Left_Side.sqlite", "left");
    loadZonesFromDB("/home/fbots/ldlidar_ros2_ws/lidar_visualizer/DB_Data/Right_Side.sqlite", "right");
}

void LiDARListener::loadZonesFromDB(const std::string &dbPath, const std::string &side) {
    sqlite3 *db;
    int rc = sqlite3_open(dbPath.c_str(), &db);
    if (rc != SQLITE_OK) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open database: %s", sqlite3_errmsg(db));
        return;
    }

    // Assuming database contains fields: Slow_Safety_Distance_1 to Slow_Safety_Distance_16 and Stop_Safety_Distance_1 to Stop_Safety_Distance_16
    std::string query = "SELECT Slow_Safety_Distance_1, Slow_Safety_Distance_2, Slow_Safety_Distance_3, Slow_Safety_Distance_4, Slow_Safety_Distance_5, Slow_Safety_Distance_6, Slow_Safety_Distance_7, Slow_Safety_Distance_8, Slow_Safety_Distance_9, Slow_Safety_Distance_10, Slow_Safety_Distance_11, Slow_Safety_Distance_12, Slow_Safety_Distance_13, Slow_Safety_Distance_14, Slow_Safety_Distance_15, Slow_Safety_Distance_16, Stop_Safety_Distance_1, Stop_Safety_Distance_2, Stop_Safety_Distance_3, Stop_Safety_Distance_4, Stop_Safety_Distance_5, Stop_Safety_Distance_6, Stop_Safety_Distance_7, Stop_Safety_Distance_8, Stop_Safety_Distance_9, Stop_Safety_Distance_10, Stop_Safety_Distance_11, Stop_Safety_Distance_12, Stop_Safety_Distance_13, Stop_Safety_Distance_14, Stop_Safety_Distance_15, Stop_Safety_Distance_16, Speed FROM lidar_data;";
    
    sqlite3_stmt *stmt;
    rc = sqlite3_prepare_v2(db, query.c_str(), -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to execute query: %s", sqlite3_errmsg(db));
        sqlite3_close(db);
        return;
    }

    std::vector<Point> zone;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        for (int i = 0; i < 16; ++i) {
            Point point;
            point.x = sqlite3_column_double(stmt, i); // Slow_Safety_Distance fields
            point.y = sqlite3_column_double(stmt, i + 16); // Stop_Safety_Distance fields
            double speed = sqlite3_column_double(stmt, 32); // Speed

            // Displaying the data in the terminal
            RCLCPP_INFO(this->get_logger(), "Zone: %s, Point %d: (%f, %f), Speed: %f", side.c_str(), i + 1, point.x, point.y, speed);

            // Filter points based on speed or other criteria
            if (speed > 0.5) { // Adjust this threshold based on your needs
                zone.push_back(point);
            }
        }
    }

    zones[side] = zone;
    sqlite3_finalize(stmt);
    sqlite3_close(db);
}

void LiDARListener::displayZoneData(const std::string &zoneName) {
    auto it = zones.find(zoneName);
    if (it != zones.end()) {
        const auto &zone = it->second;
        RCLCPP_INFO(this->get_logger(), "Displaying zone data for: %s", zoneName.c_str());
        int field_counter = 1;
        for (const auto &point : zone) {
            RCLCPP_INFO(this->get_logger(), "Field %d: (%f, %f)", field_counter++, point.x, point.y);
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Zone '%s' not found!", zoneName.c_str());
    }
}

bool LiDARListener::isPointInPolygon(const std::vector<Point> &polygon, const Point &point) {
    int num_vertices = polygon.size();
    double x = point.x, y = point.y;
    bool inside = false;

    Point p1 = polygon[0], p2;

    for (int i = 1; i <= num_vertices; i++) {
        p2 = polygon[i % num_vertices];
        if (y > std::min(p1.y, p2.y)) {
            if (y <= std::max(p1.y, p2.y)) {
                if (x <= std::max(p1.x, p2.x)) {
                    double x_intersection = (y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
                    if (p1.x == p2.x || x <= x_intersection) {
                        inside = !inside;
                    }
                }
            }
        }
        p1 = p2;
    }

    return inside;
}

LiDARListener::Point LiDARListener::polarToCartesian(float range, float angle) {
    Point p;
    p.x = range * cos(angle);
    p.y = range * sin(angle);
    return p;
}

LiDARListener::Point LiDARListener::rotatePoint(Point p, float angle) {
    float rad = angle * M_PI / 180.0;
    float cosA = cos(rad);
    float sinA = sin(rad);
    return Point{cosA * p.x - sinA * p.y, sinA * p.x + cosA * p.y};
}

void LiDARListener::processScan(const sensor_msgs::msg::LaserScan::SharedPtr msg, const std::string &side) {
    int danger_zone_count = 0;
    int warning_zone_count = 0;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float range = msg->ranges[i];
        if (range < msg->range_min || range > msg->range_max) continue;
        if (range <= 0.1) continue;
        float angle = msg->angle_min + i * msg->angle_increment;
        Point p = polarToCartesian(range, angle);
        if (side == "left") p = rotatePoint(p, -37.0);
        if (side == "right") p = rotatePoint(p, 45.0);
        if (isPointInPolygon(zones[side], p)) {
            danger_zone_count++;
            global_danger_count++;
            RCLCPP_INFO(this->get_logger(), "Element detected in %s danger zone: range=%f coordinate: x=%f, y=%f", side.c_str(), range, p.x, p.y);
        } else if (isPointInPolygon(zones[side], p)) {
            warning_zone_count++;
            global_warning_count++;
            RCLCPP_INFO(this->get_logger(), "Element detected in %s warning zone: range=%f coordinate: x=%f, y=%f", side.c_str(), range, p.x, p.y);
        }
    }

    if (danger_zone_count >= 5) {
        status_map[side] = "danger";
    } else if (warning_zone_count >= 5) {
        status_map[side] = "warning";
    } else {
        status_map[side] = "safe";
    }

    check_for_status_in_three_lidar();
}

void LiDARListener::frontCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    processScan(msg, "front");
}

void LiDARListener::leftCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    processScan(msg, "left");
}

void LiDARListener::rightCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    processScan(msg, "right");
}

void LiDARListener::velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    current_velocity_ = *msg;
}

void LiDARListener::velocity_callback2(const geometry_msgs::msg::Twist::SharedPtr msg) {
    cmd_vel_velocity_ = *msg;
    RCLCPP_INFO(this->get_logger(), "cmd_vel_velocity_.linear: '%f'", cmd_vel_velocity_.linear.x);
}

void LiDARListener::safety_turnoff_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "unlock") {
        isSafetyTurnOff = true;
    } else if (msg->data == "lock") {
        isSafetyTurnOff = false;
    } else if (msg->data == "pickdrop") {
        pickdrop = false;
    }
    RCLCPP_INFO(this->get_logger(), "cmd_vel_velocity_.linear: %s", msg->data.c_str());
    RCLCPP_INFO(this->get_logger(), "isSafetyTurnOff: %s", isSafetyTurnOff ? "true" : "false");
}

void LiDARListener::slow_down_robot() {
    cmd_vel_velocity_.linear.x *= 0.50;
    cmd_vel_velocity_.angular.z *= 0.50;
    velocity_publisher_->publish(cmd_vel_velocity_);
    RCLCPP_INFO(this->get_logger(), "outgoing cmd_vel: %f", cmd_vel_velocity_.linear.x);
    slowdown_initiated = true;
    global_warning_count = 0;
}

void LiDARListener::stop_robot() {
    cmd_vel_velocity_.linear.x = 0.0;
    cmd_vel_velocity_.angular.z = 0.0;
    velocity_publisher_->publish(cmd_vel_velocity_);
    RCLCPP_INFO(this->get_logger(), "outgoing cmd_vel: %f", cmd_vel_velocity_.linear.x);
    global_danger_count = 0;
}

void LiDARListener::check_for_status_in_three_lidar() {
    if (status_map["front"] == "danger" || status_map["left"] == "danger" || status_map["right"] == "danger") {
        stop_robot();
        status_map["front"] = "safe";
        status_map["left"] = "safe";
        status_map["right"] = "safe";
    } else if (status_map["front"] == "warning" || status_map["left"] == "warning" || status_map["right"] == "warning") {
        if (!slowdown_initiated) {
            slow_down_robot();
        }
        status_map["front"] = "safe";
        status_map["left"] = "safe";
        status_map["right"] = "safe";
    } else {
        velocity_publisher_->publish(cmd_vel_velocity_);
    }
}

void LiDARListener::visualizeZone(const std::string &zoneName) {
    plt::figure_size(800, 600);
    plt::title("LiDAR Zone Visualization: " + zoneName);
    plt::xlabel("X Coordinate");
    plt::ylabel("Y Coordinate");

    auto it = zones.find(zoneName);
    if (it != zones.end()) {
        const auto &zone = it->second;
        std::vector<double> x, y;
        for (const auto &point : zone) {
            x.push_back(point.x);
            y.push_back(point.y);
        }
        // Close the polygon by connecting the last point to the first
        if (!x.empty() && !y.empty()) {
            x.push_back(x.front());
            y.push_back(y.front());
        }

        plt::plot(x, y, "-o");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Zone '%s' not found!", zoneName.c_str());
    }

    if (zoneName == "front") {
        // Define a rectangle for the front zone
        std::vector<double> rect_x = {-0.35, 0.35, 0.35, -0.35, -0.35}; // X coordinates (robot width: 0.7 meters)
        std::vector<double> rect_y = {0.0, 0.0, 1.5, 1.5, 0.0};         // Y coordinates (extend forward up to 1.5 meters)

        plt::plot(rect_x, rect_y, "r-"); // Red rectangle representing the front zone
    } else {
        // Draw the rectangular zone around the robot for left and right zones
        std::vector<double> rect_x = {-0.35, 0.35, 0.35, -0.35, -0.35}; // X coordinates of the rectangle corners
        std::vector<double> rect_y = {-0.7, -0.7, 0.7, 0.7, -0.7};     // Y coordinates of the rectangle corners
        plt::plot(rect_x, rect_y, "r-"); // Red rectangle representing the robot's width
    }

    plt::show();
}

void LiDARListener::visualizeZones() {
    std::vector<std::string> zoneNames = {"front", "left", "right"};

    for (const auto &zoneName : zoneNames) {
        visualizeZone(zoneName);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LiDARListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
