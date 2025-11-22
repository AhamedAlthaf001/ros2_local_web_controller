#ifndef WEB_TEST_MAP_HANDLER_HPP_
#define WEB_TEST_MAP_HANDLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <functional>
#include <string>

class MapHandler {
public:
    MapHandler(rclcpp::Node* node, std::function<void(std::string)> broadcast_callback);

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    std::function<void(std::string)> broadcast_callback_;
};

#endif // WEB_TEST_MAP_HANDLER_HPP_
