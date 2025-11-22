#include <rclcpp/rclcpp.hpp>
#include "crow_all.h"
#include <thread>
#include <atomic>
#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <fstream>
#include <sstream>
#include "std_msgs/msg/string.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

class WebServerNode : public rclcpp::Node {
public:
    WebServerNode() : Node("web_server_node")
    {
        RCLCPP_INFO(this->get_logger(), "Web Server Node Started on port 8081");
        
        publisher_ = this->create_publisher<std_msgs::msg::String>("/from_web", 10);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/to_web", 10, std::bind(&WebServerNode::topic_callback, this, std::placeholders::_1));

        app_ = std::make_shared<crow::SimpleApp>();
        setup_routes();
    }

    void start_server()
    {
        server_thread_ = std::thread([this]() {
            app_->signal_clear();
            app_->port(8081).multithreaded().run();
        });
    }

    void stop_server()
    {
        RCLCPP_INFO(this->get_logger(), "Stopping Crow server...");
        app_->stop();

        if (server_thread_.joinable())
            server_thread_.join();
    }

private:

    std::string get_web_path(const std::string& filename) {
        try {
            std::string pkg_dir = ament_index_cpp::get_package_share_directory("web_test");
            return pkg_dir + "/web/" + filename;
        } catch (...) {
            // Fallback to source directory during development
            return "/home/linux/test_ws/src/web_test/web/" + filename;
        }
    }

    void setup_routes()
    {
        CROW_ROUTE((*app_), "/").methods("GET"_method)
        ([this](const crow::request&, crow::response& res) {
            RCLCPP_INFO(this->get_logger(), "User connected");

            std::ifstream t(get_web_path("index.html"));
            if (!t.is_open()) {
                res.code = 500;
                res.write("index.html not found");
                res.end();
                return;
            }
            std::stringstream buffer;
            buffer << t.rdbuf();
            res.write(buffer.str());
            res.end();
        });

        CROW_ROUTE((*app_), "/style.css").methods("GET"_method)
        ([this](const crow::request&, crow::response& res) {
            std::ifstream t(get_web_path("style.css"));
            if (!t.is_open()) {
                res.code = 404;
                res.write("style.css not found");
                res.end();
                return;
            }
            std::stringstream buffer;
            buffer << t.rdbuf();
            res.set_header("Content-Type", "text/css");
            res.write(buffer.str());
            res.end();
        });

        CROW_ROUTE((*app_), "/ping").methods("POST"_method)
        ([this](const crow::request& req) {
            int id = std::stoi(req.body);
            RCLCPP_INFO(this->get_logger(), "User %d: Ping from Browser", id);
            return "ok";
        });

        CROW_ROUTE((*app_), "/ws")
            .websocket()
            .onopen([this](crow::websocket::connection& conn) {
                std::lock_guard<std::mutex> _(mtx_);
                int id = ++next_user_id_;
                users_[&conn] = id;
                RCLCPP_INFO(this->get_logger(), "New WebSocket connection. Assigned User ID: %d", id);
            })
            .onclose([this](crow::websocket::connection& conn, const std::string& reason) {
                std::lock_guard<std::mutex> _(mtx_);
                int id = users_[&conn];
                users_.erase(&conn);
                RCLCPP_INFO(this->get_logger(), "User %d disconnected: %s", id, reason.c_str());
            })
            .onmessage([this](crow::websocket::connection& conn, const std::string& data, bool is_binary) {
                if (is_binary) return;
                
                std::lock_guard<std::mutex> _(mtx_);
                if (users_.find(&conn) == users_.end()) return; // Should not happen
                
                int id = users_[&conn];
                RCLCPP_INFO(this->get_logger(), "Received from User %d: %s", id, data.c_str());
                
                auto msg = std_msgs::msg::String();
                msg.data = "User " + std::to_string(id) + ": " + data;
                publisher_->publish(msg);
            });
    }

    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> _(mtx_);
        for (auto& [conn, id] : users_) {
            conn->send_text(msg->data);
        }
    }

    std::shared_ptr<crow::SimpleApp> app_;
    std::thread server_thread_;
    std::unordered_map<crow::websocket::connection*, int> users_;
    int next_user_id_ = 0;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::mutex mtx_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<WebServerNode>();

    // Start Crow server thread
    node->start_server();

    // ROS executor
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    exec.spin();            // returns after Ctrl+C
    rclcpp::shutdown();     // stops ROS

    // Stop Crow server cleanly
    node->stop_server();

    return 0;
}
