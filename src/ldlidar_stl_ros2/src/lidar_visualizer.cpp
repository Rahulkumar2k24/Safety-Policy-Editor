#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "matplotlibcpp.h"
#include <iostream>

namespace plt = matplotlibcpp;

class LidarVisualizer : public rclcpp::Node
{
public:
  LidarVisualizer() : Node("lidar_visualizer")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&LidarVisualizer::scan_callback, this, std::placeholders::_1));
    plt::ion(); // Interactive mode on
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    angles.clear();
    ranges.clear();
    float angle = msg->angle_min;

    // Print minimum angle and other information
    std::cout << "Min Angle: " << angle << " radians (" << angle * (180.0 / M_PI) << " degrees)" << std::endl;
    std::cout << "Angle Increment: " << msg->angle_increment << " radians" << std::endl;
    std::cout << "Range Min: " << msg->range_min << " meters" << std::endl;
    std::cout << "Range Max: " << msg->range_max << " meters" << std::endl;

    for (const auto &range : msg->ranges)
    {
      angles.push_back(angle);
      ranges.push_back(range);
      angle += msg->angle_increment;
    }

    // Print maximum angle
    std::cout << "Max Angle: " << angle << " radians (" << angle * (180.0 / M_PI) << " degrees)" << std::endl;

    // Print ranges
    // std::cout << "Ranges: ";
    // for (const auto &range : ranges)
    // {
    //   std::cout << range << " ";
    // }
    // std::cout << std::endl << std::endl; // Separate cycles with a blank line

     plt::clf(); // Clear the figure
    // Convert polar to cartesian coordinates for plotting
    std::vector<double> x, y;
    for (size_t i = 0; i < angles.size(); ++i)
    {
      x.push_back(ranges[i] * cos(angles[i]));
      y.push_back(ranges[i] * sin(angles[i]));
    }

    plt::scatter(x, y, 1.0, {{"c", "r"}}); // Scatter plot
    plt::xlim(-10, 10); // Set x-axis limits
    plt::ylim(-10, 10); // Set y-axis limits
    plt::pause(0.001); // Pause for a brief moment to update the plot
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  std::vector<float> angles;
  std::vector<float> ranges;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
