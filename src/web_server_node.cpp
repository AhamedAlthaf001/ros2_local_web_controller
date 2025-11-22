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
#include <cstdlib>
#include <csignal>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <filesystem>
#include <vector>
#include <algorithm>

namespace fs = std::filesystem;

class ProcessManager {
public:
    pid_t start_process(const std::string& command) {
        pid_t pid = fork();
        if (pid == 0) {
            // Child process
            // Setsid to create a new process group, so we can kill the whole tree
            setsid();
            execl("/bin/sh", "sh", "-c", command.c_str(), nullptr);
            _exit(1); // Should not reach here
        }
        return pid;
    }

    void stop_process(pid_t pid) {
        if (pid > 0) {
            // Kill the process group
            kill(-pid, SIGINT);
            // Wait a bit for graceful shutdown
            std::this_thread::sleep_for(std::chrono::seconds(2));
            // Force kill if needed (optional, but good for robustness)
            // kill(-pid, SIGKILL); 
            int status;
            waitpid(pid, &status, WNOHANG);
        }
    }
};

class WebServerNode : public rclcpp::Node {
public:
    WebServerNode() : Node("web_server_node")
    {
        RCLCPP_INFO(this->get_logger(), "Web Server Node Started on port 8081");
        
        // Declare and get parameters
        this->declare_parameter("mapping_launch_cmd", "ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True");
        this->declare_parameter("navigation_launch_cmd", "ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True");
        this->declare_parameter("map_folder", "maps");
        
        mapping_cmd_ = this->get_parameter("mapping_launch_cmd").as_string();
        nav_cmd_ = this->get_parameter("navigation_launch_cmd").as_string();
        map_folder_ = this->get_parameter("map_folder").as_string();

        // Ensure map folder exists
        if (!fs::exists(map_folder_)) {
            fs::create_directories(map_folder_);
        }

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
            
        // Stop any running child process
        if (current_pid_ > 0) {
            process_manager_.stop_process(current_pid_);
        }
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

        CROW_ROUTE((*app_), "/mapping.html").methods("GET"_method)
        ([this](const crow::request&, crow::response& res) {
            std::ifstream t(get_web_path("mapping.html"));
            if (!t.is_open()) {
                res.code = 404;
                res.write("mapping.html not found");
                res.end();
                return;
            }
            std::stringstream buffer;
            buffer << t.rdbuf();
            res.write(buffer.str());
            res.end();
        });

        CROW_ROUTE((*app_), "/nav").methods("GET"_method)
        ([this](const crow::request&, crow::response& res) {
            std::ifstream t(get_web_path("nav.html"));
            if (!t.is_open()) {
                res.code = 404;
                res.write("nav.html not found");
                res.end();
                return;
            }
            std::stringstream buffer;
            buffer << t.rdbuf();
            res.write(buffer.str());
            res.end();
        });

        CROW_ROUTE((*app_), "/nav.html").methods("GET"_method)
        ([this](const crow::request&, crow::response& res) {
            std::ifstream t(get_web_path("nav.html"));
            if (!t.is_open()) {
                res.code = 404;
                res.write("nav.html not found");
                res.end();
                return;
            }
            std::stringstream buffer;
            buffer << t.rdbuf();
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

        // API Endpoints for Process Management
        
        CROW_ROUTE((*app_), "/api/start_mapping").methods("POST"_method)
        ([this](const crow::request&) {
            std::lock_guard<std::mutex> _(process_mtx_);
            if (current_pid_ > 0) {
                process_manager_.stop_process(current_pid_);
            }
            RCLCPP_INFO(this->get_logger(), "Starting Mapping: %s", mapping_cmd_.c_str());
            current_pid_ = process_manager_.start_process(mapping_cmd_);
            return "started";
        });

        CROW_ROUTE((*app_), "/api/start_navigation").methods("POST"_method)
        ([this](const crow::request& req) {
            std::lock_guard<std::mutex> _(process_mtx_);
            auto body = crow::json::load(req.body);
            std::string map_file = "";
            if (body && body.has("map_name")) {
                std::string map_name = body["map_name"].s();
                map_file = "map:=" + (fs::path(map_folder_) / (map_name + ".yaml")).string();
            }
            
            if (current_pid_ > 0) {
                process_manager_.stop_process(current_pid_);
            }
            
            std::string cmd = nav_cmd_;
            if (!map_file.empty()) {
                cmd += " " + map_file;
            }
            
            RCLCPP_INFO(this->get_logger(), "Starting Navigation: %s", cmd.c_str());
            current_pid_ = process_manager_.start_process(cmd);
            return "started";
        });

        CROW_ROUTE((*app_), "/api/save_map").methods("POST"_method)
        ([this](const crow::request& req) {
            auto body = crow::json::load(req.body);
            if (!body || !body.has("map_name")) return crow::response(400, "Missing map_name");
            
            std::string map_name = body["map_name"].s();
            std::string map_path = (fs::path(map_folder_) / map_name).string();
            
            std::string cmd = "ros2 run nav2_map_server map_saver_cli -f " + map_path;
            RCLCPP_INFO(this->get_logger(), "Saving map: %s", cmd.c_str());
            
            int ret = std::system(cmd.c_str());
            if (ret == 0) return crow::response(200, "saved");
            else return crow::response(500, "failed to save");
        });

        CROW_ROUTE((*app_), "/api/list_maps").methods("GET"_method)
        ([this]() {
            crow::json::wvalue x;
            int i = 0;
            if (fs::exists(map_folder_)) {
                for (const auto& entry : fs::directory_iterator(map_folder_)) {
                    if (entry.path().extension() == ".yaml") {
                        x[i++] = entry.path().stem().string();
                    }
                }
            }
            return x;
        });

        CROW_ROUTE((*app_), "/api/reset_mapping").methods("POST"_method)
        ([this]() {
             std::lock_guard<std::mutex> _(process_mtx_);
            if (current_pid_ > 0) {
                process_manager_.stop_process(current_pid_);
            }
            RCLCPP_INFO(this->get_logger(), "Restarting Mapping: %s", mapping_cmd_.c_str());
            current_pid_ = process_manager_.start_process(mapping_cmd_);
            return "reset";
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
    
    ProcessManager process_manager_;
    pid_t current_pid_ = -1;
    std::mutex process_mtx_;
    
    std::string mapping_cmd_;
    std::string nav_cmd_;
    std::string map_folder_;

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
