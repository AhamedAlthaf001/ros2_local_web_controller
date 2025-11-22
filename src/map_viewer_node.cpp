#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "crow_all.h"
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <thread>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Simple .env parser
std::unordered_map<std::string, std::string> load_env(const std::string& filepath) {
    std::unordered_map<std::string, std::string> env;
    std::ifstream file(filepath);
    if (!file.is_open()) {
        return env;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') continue;
        
        auto pos = line.find('=');
        if (pos != std::string::npos) {
            std::string key = line.substr(0, pos);
            std::string value = line.substr(pos + 1);
            env[key] = value;
        }
    }
    return env;
}

class MapViewerNode : public rclcpp::Node {
public:
    MapViewerNode() : Node("map_viewer_node")
    {
        // Declare parameters
        this->declare_parameter("map_topic", "/map");
        this->declare_parameter("cmd_vel_topic", "/cmd_vel");
        this->declare_parameter("scan_topic", "/scan");
        this->declare_parameter("plan_topic", "/plan");
        this->declare_parameter("goal_topic", "/goal_pose");
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("base_link_frame", "base_link");
        this->declare_parameter("port", 8082);
        this->declare_parameter("env_file_path", "");
        
        // Get parameters
        map_topic_ = this->get_parameter("map_topic").as_string();
        cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
        scan_topic_ = this->get_parameter("scan_topic").as_string();
        plan_topic_ = this->get_parameter("plan_topic").as_string();
        goal_topic_ = this->get_parameter("goal_topic").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        base_link_frame_ = this->get_parameter("base_link_frame").as_string();
        port_ = this->get_parameter("port").as_int();
        std::string env_path = this->get_parameter("env_file_path").as_string();
        
        RCLCPP_INFO(this->get_logger(), "Map Viewer Node Started");
        RCLCPP_INFO(this->get_logger(), "  Port: %d", port_);
        RCLCPP_INFO(this->get_logger(), "  Map topic: %s", map_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Cmd_vel topic: %s", cmd_vel_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Frames: %s -> %s", map_frame_.c_str(), base_link_frame_.c_str());
        
        // Load .env file
        if (env_path.empty()) {
            // Try package directory
            try {
                std::string pkg_dir = ament_index_cpp::get_package_share_directory("web_test");
                env_path = pkg_dir + "/.env";
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "Could not find package directory, using current directory");
                env_path = ".env";
            }
        }
        
        env_vars_ = load_env(env_path);
        if (env_vars_.count("WEB_USERNAME") && env_vars_.count("WEB_PASSWORD")) {
            RCLCPP_INFO(this->get_logger(), "Loaded credentials from %s", env_path.c_str());
            auth_enabled_ = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "No credentials found in %s - Authentication disabled!", env_path.c_str());
            auth_enabled_ = false;
        }
        
        app_ = std::make_shared<crow::SimpleApp>();
        
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_, qos,
            std::bind(&MapViewerNode::map_callback, this, std::placeholders::_1));
            
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic_, 10);
        
        // Navigation subscriptions and publishers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, 10,
            std::bind(&MapViewerNode::scan_callback, this, std::placeholders::_1));
            
        plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            plan_topic_, 10,
            std::bind(&MapViewerNode::plan_callback, this, std::placeholders::_1));
            
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, 10);
        
        // Subscribe to Nav2 action status
        goal_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
            "/navigate_to_pose/_action/status",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            std::bind(&MapViewerNode::goal_status_callback, this, std::placeholders::_1));
        
        // TF Setup
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Timer for robot pose updates (20Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&MapViewerNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to %s and listening to TF (%s->%s)", 
                    map_topic_.c_str(), map_frame_.c_str(), base_link_frame_.c_str());

        setup_routes();
    }

    void start_server()
    {
        server_thread_ = std::thread([this]() {
            app_->signal_clear();
            app_->port(port_).multithreaded().run();
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
    std::string generate_session_token() {
        static int counter = 0;
        return "session_" + std::to_string(std::time(nullptr)) + "_" + std::to_string(++counter);
    }
    
    bool is_authenticated(const crow::request& req) {
        if (!auth_enabled_) return true;  // No auth required
        
        auto cookies = req.get_header_value("Cookie");
        if (cookies.empty()) return false;
        
        // Parse cookies
        std::string token;
        std::istringstream cookie_stream(cookies);
        std::string cookie;
        while (std::getline(cookie_stream, cookie, ';')) {
            auto eq_pos = cookie.find('=');
            if (eq_pos != std::string::npos) {
                std::string key = cookie.substr(0, eq_pos);
                std::string value = cookie.substr(eq_pos + 1);
                // Trim whitespace
                key.erase(0, key.find_first_not_of(" \t"));
                key.erase(key.find_last_not_of(" \t") + 1);
                
                if (key == "session_token") {
                    token = value;
                    break;
                }
            }
        }
        
        if (token.empty()) return false;
        
        std::lock_guard<std::mutex> lock(mtx_);
        return active_sessions_.count(token) > 0;
    }

    void setup_routes()
    {
        // Root - Login page
        CROW_ROUTE((*app_), "/").methods("GET"_method)
        ([this](const crow::request& req, crow::response& res) {
            if (is_authenticated(req)) {
                // Already logged in, redirect to map
                res.code = 302;
                res.set_header("Location", "/map");
                res.end();
                return;
            }
            
            std::ifstream t(get_web_path("login.html"));
            if (!t.is_open()) {
                res.code = 500;
                res.write("Login page not found");
                res.end();
                return;
            }
            std::stringstream buffer;
            buffer << t.rdbuf();
            res.write(buffer.str());
            res.end();
        });
        
        // Login endpoint
        CROW_ROUTE((*app_), "/login").methods("POST"_method)
        ([this](const crow::request& req) {
            auto body = crow::json::load(req.body);
            if (!body) {
                return crow::response(400, "Invalid JSON");
            }
            
            std::string username = body["username"].s();
            std::string password = body["password"].s();
            
            if (!auth_enabled_ || 
                (username == env_vars_["WEB_USERNAME"] && password == env_vars_["WEB_PASSWORD"])) {
                // Valid credentials
                std::string token = generate_session_token();
                {
                    std::lock_guard<std::mutex> lock(mtx_);
                    active_sessions_.insert(token);
                }
                
                crow::response res(200);
                res.set_header("Set-Cookie", "session_token=" + token + "; Path=/; HttpOnly");
                crow::json::wvalue result;
                result["success"] = true;
                res.write(result.dump());
                return res;
            } else {
                return crow::response(401, "{\"success\": false, \"error\": \"Invalid credentials\"}");
            }
        });
        
        // Map page - redirect to teleop
        CROW_ROUTE((*app_), "/map").methods("GET"_method)
        ([this](const crow::request& req, crow::response& res) {
            res.code = 302;
            res.set_header("Location", "/teleop");
            res.end();
        });
        
        // Teleoperation page
        CROW_ROUTE((*app_), "/teleop").methods("GET"_method)
        ([this](const crow::request& req, crow::response& res) {
            if (!is_authenticated(req)) {
                res.code = 302;
                res.set_header("Location", "/");
                res.end();
                return;
            }
            
            std::ifstream t(get_web_path("teleop.html"));
            if (!t.is_open()) {
                res.code = 500;
                res.write("Teleop page not found");
                res.end();
                return;
            }
            std::stringstream buffer;
            buffer << t.rdbuf();
            res.write(buffer.str());
            res.end();
        });
        
        // Navigation page
        CROW_ROUTE((*app_), "/nav").methods("GET"_method)
        ([this](const crow::request& req, crow::response& res) {
            if (!is_authenticated(req)) {
                res.code = 302;
                res.set_header("Location", "/");
                res.end();
                return;
            }
            
            std::ifstream t(get_web_path("nav.html"));
            if (!t.is_open()) {
                res.code = 500;
                res.write("Navigation page not found");
                res.end();
                return;
            }
            std::stringstream buffer;
            buffer << t.rdbuf();
            res.write(buffer.str());
            res.end();
        });
        
        // Static file serving for CSS
        CROW_ROUTE((*app_), "/style.css").methods("GET"_method)
        ([this](const crow::request& req, crow::response& res) {
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

        CROW_ROUTE((*app_), "/ws")
            .websocket()
            .onopen([this](crow::websocket::connection& conn) {
                // Note: We can't easily check auth here with Crow, so we rely on browser cookies
                std::lock_guard<std::mutex> _(mtx_);
                int id = ++next_user_id_;
                users_[&conn] = id;
                RCLCPP_INFO(this->get_logger(), "New WebSocket connection. Assigned User ID: %d", id);
                
                if (!last_map_json_.empty()) {
                    conn.send_text(last_map_json_);
                }
            })
            .onclose([this](crow::websocket::connection& conn, const std::string& reason) {
                std::lock_guard<std::mutex> _(mtx_);
                int id = users_[&conn];
                users_.erase(&conn);
                RCLCPP_INFO(this->get_logger(), "User %d disconnected: %s", id, reason.c_str());
            })
            .onmessage([this](crow::websocket::connection& /*conn*/, const std::string& data, bool is_binary) {
                if (is_binary) return;
                
                try {
                    auto x = crow::json::load(data);
                    if (!x) return;
                    
                    if (x.has("type") && x["type"].s() == "cmd_vel") {
                        auto msg = geometry_msgs::msg::TwistStamped();
                        msg.header.stamp = this->now();
                        msg.header.frame_id = base_link_frame_;
                        
                        // Robust parsing for linear and angular
                        if (x.has("linear")) {
                            if (x["linear"].t() == crow::json::type::Number)
                                msg.twist.linear.x = x["linear"].d();
                        }
                        
                        if (x.has("angular")) {
                            if (x["angular"].t() == crow::json::type::Number)
                                msg.twist.angular.z = x["angular"].d();
                        }

                        cmd_vel_pub_->publish(msg);
                    } else if (x.has("type") && x["type"].s() == "set_goal") {
                        // Publish navigation goal
                        auto goal_msg = geometry_msgs::msg::PoseStamped();
                        goal_msg.header.stamp = this->now();
                        goal_msg.header.frame_id = map_frame_;
                        
                        if (x.has("x") && x["x"].t() == crow::json::type::Number)
                            goal_msg.pose.position.x = x["x"].d();
                        if (x.has("y") && x["y"].t() == crow::json::type::Number)
                            goal_msg.pose.position.y = x["y"].d();
                        
                        // Set orientation from theta
                        double theta = 0.0;
                        if (x.has("theta") && x["theta"].t() == crow::json::type::Number)
                            theta = x["theta"].d();
                        
                        goal_msg.pose.orientation.z = std::sin(theta / 2.0);
                        goal_msg.pose.orientation.w = std::cos(theta / 2.0);
                        
                        goal_pub_->publish(goal_msg);
                        RCLCPP_INFO(this->get_logger(), "Published goal: x=%.2f, y=%.2f, theta=%.2f", 
                                    goal_msg.pose.position.x, goal_msg.pose.position.y, theta);
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error parsing JSON: %s", e.what());
                }
            });
    }

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received map: %d x %d", msg->info.width, msg->info.height);
        
        crow::json::wvalue x;
        x["type"] = "map";
        x["width"] = msg->info.width;
        x["height"] = msg->info.height;
        x["resolution"] = msg->info.resolution;
        x["origin"]["position"]["x"] = msg->info.origin.position.x;
        x["origin"]["position"]["y"] = msg->info.origin.position.y;
        
        std::vector<int> data;
        data.reserve(msg->data.size());
        for (auto val : msg->data) {
            data.push_back(val);
        }
        x["data"] = std::move(data);

        std::string json_str = x.dump();
        
        {
            std::lock_guard<std::mutex> _(mtx_);
            last_map_json_ = json_str; // Cache the map
            for (auto& [conn, id] : users_) {
                conn->send_text(json_str);
            }
        }
    }
    
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform(map_frame_, base_link_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            return;
        }

        crow::json::wvalue x;
        x["type"] = "pose";
        x["x"] = t.transform.translation.x;
        x["y"] = t.transform.translation.y;
        
        // Convert quaternion to yaw (theta)
        double qx = t.transform.rotation.x;
        double qy = t.transform.rotation.y;
        double qz = t.transform.rotation.z;
        double qw = t.transform.rotation.w;
        double siny_cosp = 2 * (qw * qz + qx * qy);
        double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        x["theta"] = std::atan2(siny_cosp, cosy_cosp);
        
        std::string json_str = x.dump();
        
        std::lock_guard<std::mutex> _(mtx_);
        for (auto& [conn, id] : users_) {
            conn->send_text(json_str);
        }
    }
    
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        crow::json::wvalue x;
        x["type"] = "scan";
        x["angle_min"] = msg->angle_min;
        x["angle_max"] = msg->angle_max;
        x["angle_increment"] = msg->angle_increment;
        x["range_max"] = msg->range_max;
        
        // Convert scan to points in base_link frame, then transform to map frame
        std::vector<std::vector<double>> points;
        points.reserve(msg->ranges.size());
        
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(map_frame_, msg->header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            return; // Can't transform, skip this scan
        }
        
        // Convert quaternion to yaw angle
        double qx = transform.transform.rotation.x;
        double qy = transform.transform.rotation.y;
        double qz = transform.transform.rotation.z;
        double qw = transform.transform.rotation.w;
        double siny_cosp = 2 * (qw * qz + qx * qy);
        double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        double yaw = std::atan2(siny_cosp, cosy_cosp);
        
        double angle = msg->angle_min;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float range = msg->ranges[i];
            
            if (range >= msg->range_min && range <= msg->range_max) {
                // Point in sensor frame
                double x_sensor = range * std::cos(angle);
                double y_sensor = range * std::sin(angle);
                
                // Transform to map frame using proper yaw angle
                double x_map = transform.transform.translation.x + 
                               x_sensor * std::cos(yaw) - 
                               y_sensor * std::sin(yaw);
                double y_map = transform.transform.translation.y + 
                               x_sensor * std::sin(yaw) + 
                               y_sensor * std::cos(yaw);
                
                points.push_back({x_map, y_map});
            }
            
            angle += msg->angle_increment;
        }
        
        x["points"] = std::move(points);
        std::string json_str = x.dump();
        
        std::lock_guard<std::mutex> _(mtx_);
        for (auto& [conn, id] : users_) {
            conn->send_text(json_str);
        }
    }
    
    void plan_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        crow::json::wvalue x;
        x["type"] = "plan";
        
        std::vector<std::vector<double>> path;
        path.reserve(msg->poses.size());
        
        for (const auto& pose_stamped : msg->poses) {
            // Extract yaw from quaternion
            double qz = pose_stamped.pose.orientation.z;
            double qw = pose_stamped.pose.orientation.w;
            double theta = 2.0 * std::atan2(qz, qw);
            
            path.push_back({
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                theta
            });
        }
        
        x["path"] = std::move(path);
        std::string json_str = x.dump();
        
        std::lock_guard<std::mutex> _(mtx_);
        for (auto& [conn, id] : users_) {
            conn->send_text(json_str);
        }
    }
    
    void goal_status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Goal status callback triggered, %zu goals in list", msg->status_list.size());
        
        if (msg->status_list.empty()) return;
        
        // Get the latest goal status
        auto& latest_status = msg->status_list.back();
        
        crow::json::wvalue x;
        x["type"] = "nav_status";
        
        // Map status codes to readable strings
        switch (latest_status.status) {
            case 1: // STATUS_ACCEPTED
                x["status"] = "Goal Accepted";
                x["state"] = "active";
                break;
            case 2: // STATUS_EXECUTING
                x["status"] = "Navigating...";
                x["state"] = "active";
                break;
            case 4: // STATUS_SUCCEEDED
                x["status"] = "Goal Succeeded";
                x["state"] = "success";
                break;
            case 5: // STATUS_CANCELED
                x["status"] = "Goal Canceled";
                x["state"] = "failed";
                break;
            case 6: // STATUS_ABORTED
                x["status"] = "Goal Aborted";
                x["state"] = "failed";
                break;
            default:
                x["status"] = "Unknown";
                x["state"] = "";
                break;
        }
        
        std::string json_str = x.dump();
        
        std::lock_guard<std::mutex> _(mtx_);
        for (auto& [conn, id] : users_) {
            conn->send_text(json_str);
        }
    }
    
    
    std::string get_web_path(const std::string& filename) {
        try {
            std::string pkg_dir = ament_index_cpp::get_package_share_directory("web_test");
            return pkg_dir + "/web/" + filename;
        } catch (...) {
            // Fallback to source directory during development
            return "/home/linux/test_ws/src/web_test/web/" + filename;
        }
    }

    std::shared_ptr<crow::SimpleApp> app_;
    std::thread server_thread_;
    std::unordered_map<crow::websocket::connection*, int> users_;
    std::unordered_set<std::string> active_sessions_;
    int next_user_id_ = 0;
    std::string last_map_json_;
    
    std::unordered_map<std::string, std::string> env_vars_;
    bool auth_enabled_ = false;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr goal_status_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::string map_topic_;
    std::string cmd_vel_topic_;
    std::string scan_topic_;
    std::string plan_topic_;
    std::string goal_topic_;
    std::string map_frame_;
    std::string base_link_frame_;
    int port_;
    
    std::mutex mtx_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapViewerNode>();
    node->start_server();
    rclcpp::spin(node);
    node->stop_server();
    rclcpp::shutdown();
    return 0;
}
