#include "web_test/map_handler.hpp"
#include "crow_all.h" // For JSON serialization if needed, or just manual string building

MapHandler::MapHandler(rclcpp::Node* node, std::function<void(std::string)> broadcast_callback)
    : broadcast_callback_(broadcast_callback)
{
    RCLCPP_INFO(node->get_logger(), "MapHandler initialized. Subscribing to /map...");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", qos,
        std::bind(&MapHandler::map_callback, this, std::placeholders::_1));
    RCLCPP_INFO(node->get_logger(), "Subscribed to /map with Transient Local reliability.");
}

void MapHandler::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("map_handler"), "Received map: %d x %d", msg->info.width, msg->info.height);
    crow::json::wvalue x;
    x["type"] = "map";
    x["width"] = msg->info.width;
    x["height"] = msg->info.height;
    x["resolution"] = msg->info.resolution;
    x["origin"]["position"]["x"] = msg->info.origin.position.x;
    x["origin"]["position"]["y"] = msg->info.origin.position.y;
    
    // Convert data to a list
    std::vector<int> data;
    data.reserve(msg->data.size());
    for (auto val : msg->data) {
        data.push_back(val);
    }
    x["data"] = std::move(data);

    broadcast_callback_(x.dump());
}
