#include "action_msgs/msg/goal_status_array.hpp"
#include "crow_all.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace fs = std::filesystem;

class ProcessManager {
public:
  pid_t start_process(const std::string &command) {
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
      // Kill the process group with SIGINT (Ctrl+C equivalent)
      kill(-pid, SIGINT);

      // Wait for process to exit (up to 10 seconds)
      int status;
      for (int i = 0; i < 100; ++i) {
        pid_t result = waitpid(pid, &status, WNOHANG);
        if (result == pid) {
          return; // Process exited
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      // If still running, force kill
      kill(-pid, SIGKILL);
      waitpid(pid, &status, 0);
    }
  }
};

// Simple .env parser
std::unordered_map<std::string, std::string>
load_env(const std::string &filepath) {
  std::unordered_map<std::string, std::string> env;
  std::ifstream file(filepath);
  if (!file.is_open()) {
    return env;
  }

  std::string line;
  while (std::getline(file, line)) {
    // Skip empty lines and comments
    if (line.empty() || line[0] == '#')
      continue;

    auto pos = line.find('=');
    if (pos != std::string::npos) {
      std::string key = line.substr(0, pos);
      std::string value = line.substr(pos + 1);
      env[key] = value;
    }
  }
  return env;
}

// Process state tracking
enum class ProcessState { STOPPED, RUNNING };

class MapViewerNode : public rclcpp::Node {
public:
  MapViewerNode() : Node("map_viewer_node") {
    // Initialize states
    nav_state_ = ProcessState::STOPPED;
    mapping_state_ = ProcessState::STOPPED;
    emergency_stop_active_ = false;
    // Declare parameters
    this->declare_parameter("map_topic", "/map");
    this->declare_parameter("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter("scan_topic", "/scan");
    this->declare_parameter("battery_state_topic", "/battery_state");
    this->declare_parameter("plan_topic", "/plan");
    this->declare_parameter("goal_topic", "/goal_pose");
    this->declare_parameter("initial_pose_topic", "/initialpose");
    this->declare_parameter("nav_action_client_topic", "/navigate_to_pose");
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("base_link_frame", "base_link");
    this->declare_parameter("tf_prefix", "");
    this->declare_parameter("port", 8082);
    this->declare_parameter("env_file_path", "");
    this->declare_parameter("max_linear_velocity", 0.2);
    this->declare_parameter("max_angular_velocity", 0.2);

    // Launch Commands
    this->declare_parameter("mapping_launch_cmd",
                            "ros2 launch turtlebot3_cartographer "
                            "cartographer.launch.py use_sim_time:=True");
    this->declare_parameter("navigation_launch_cmd",
                            "ros2 launch turtlebot3_navigation2 "
                            "navigation2.launch.py use_sim_time:=True");
    this->declare_parameter("map_folder", "maps");

    // Get parameters
    map_topic_ = this->get_parameter("map_topic").as_string();
    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    battery_state_topic_ =
        this->get_parameter("battery_state_topic").as_string();
    plan_topic_ = this->get_parameter("plan_topic").as_string();
    goal_topic_ = this->get_parameter("goal_topic").as_string();
    initial_pose_topic_ = this->get_parameter("initial_pose_topic").as_string();
    nav_action_client_topic_ =
        this->get_parameter("nav_action_client_topic").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    tf_prefix_ = this->get_parameter("tf_prefix").as_string();
    base_link_frame_ = this->get_parameter("base_link_frame").as_string();
    if (tf_prefix_ != "") {
      base_link_frame_ = tf_prefix_ + "/" + base_link_frame_;
    }
    port_ = this->get_parameter("port").as_int();
    max_linear_velocity_ =
        this->get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ =
        this->get_parameter("max_angular_velocity").as_double();
    std::string env_path = this->get_parameter("env_file_path").as_string();

    mapping_cmd_ = this->get_parameter("mapping_launch_cmd").as_string();
    nav_cmd_ = this->get_parameter("navigation_launch_cmd").as_string();
    map_folder_ = this->get_parameter("map_folder").as_string();

    // Ensure map folder exists
    if (!fs::exists(map_folder_)) {
      fs::create_directories(map_folder_);
    }

    RCLCPP_INFO(this->get_logger(), "Map Viewer Node Started");
    RCLCPP_INFO(this->get_logger(), "  Port: %d", port_);
    RCLCPP_INFO(this->get_logger(), "  Map topic: %s", map_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Cmd_vel topic: %s",
                cmd_vel_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Scan topic: %s", scan_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Battery state topic: %s",
                battery_state_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Plan topic: %s", plan_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Goal topic: %s", goal_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Initial pose topic: %s",
                initial_pose_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Nav action client topic: %s",
                nav_action_client_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Frames: %s -> %s", map_frame_.c_str(),
                base_link_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  TF prefix: %s", tf_prefix_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Map folder: %s", map_folder_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Max linear velocity: %f",
                max_linear_velocity_);
    RCLCPP_INFO(this->get_logger(), "  Max angular velocity: %f",
                max_angular_velocity_);

    // Load .env file
    if (env_path.empty()) {
      // Try package directory
      try {
        std::string pkg_dir =
            ament_index_cpp::get_package_share_directory("web_test");
        env_path = pkg_dir + "/.env";
      } catch (...) {
        RCLCPP_WARN(
            this->get_logger(),
            "Could not find package directory, using current directory");
        env_path = ".env";
      }
    }

    env_vars_ = load_env(env_path);
    if (env_vars_.count("WEB_USERNAME") && env_vars_.count("WEB_PASSWORD")) {
      RCLCPP_INFO(this->get_logger(), "Loaded credentials from %s",
                  env_path.c_str());
      auth_enabled_ = true;
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "No credentials found in %s - Authentication disabled!",
                  env_path.c_str());
      auth_enabled_ = false;
    }

    app_ = std::make_shared<crow::SimpleApp>();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, qos,
        std::bind(&MapViewerNode::map_callback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        cmd_vel_topic_, 10);

    // Navigation subscriptions and publishers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, 10,
        std::bind(&MapViewerNode::scan_callback, this, std::placeholders::_1));

    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        battery_state_topic_, 10,
        std::bind(&MapViewerNode::battery_callback, this,
                  std::placeholders::_1));

    plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        plan_topic_, 10,
        std::bind(&MapViewerNode::plan_callback, this, std::placeholders::_1));

    // Create publishers
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        goal_topic_, 10);
    initial_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            initial_pose_topic_, 10);

    // Create action client for canceling navigation goals
    nav_action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, nav_action_client_topic_);

    // Subscribe to Nav2 action status
    goal_status_sub_ =
        this->create_subscription<action_msgs::msg::GoalStatusArray>(
            nav_action_client_topic_ + "/_action/status",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            std::bind(&MapViewerNode::goal_status_callback, this,
                      std::placeholders::_1));

    // TF Setup
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Timer for robot pose updates (20Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&MapViewerNode::timer_callback, this));

    RCLCPP_INFO(
        this->get_logger(), "Subscribed to %s and listening to TF (%s->%s)",
        map_topic_.c_str(), map_frame_.c_str(), base_link_frame_.c_str());

    setup_routes();

    // Start process monitor thread
    running_ = true;
    monitor_thread_ = std::thread(&MapViewerNode::monitor_process, this);

    // Auto-start navigation if enabled
    bool auto_start_nav = this->declare_parameter("auto_start_nav", true);
    std::string default_map =
        this->declare_parameter("default_map", "map"); // Default map name

    // Try to load last used map
    std::string last_map = load_last_map();
    if (!last_map.empty()) {
      default_map = last_map;
      RCLCPP_INFO(this->get_logger(), "Loaded last used map: %s",
                  default_map.c_str());
    }

    if (auto_start_nav) {
      RCLCPP_INFO(this->get_logger(), "Auto-starting navigation...");
      std::string map_file = "";
      // Check if default map exists
      fs::path map_path =
          fs::path(map_folder_) / default_map / (default_map + ".yaml");
      if (fs::exists(map_path)) {
        map_file = "map:=" + map_path.string();
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "Default map not found at %s, starting nav without map",
                    map_path.c_str());
      }

      std::string cmd = nav_cmd_;
      if (!map_file.empty()) {
        cmd += " " + map_file;
      }

      {
        std::lock_guard<std::mutex> _(process_mtx_);
        RCLCPP_INFO(this->get_logger(), "Executing: %s", cmd.c_str());
        current_pid_ = process_manager_.start_process(cmd);
        nav_state_ = ProcessState::RUNNING;
      }
    }
  }

  void save_last_map(const std::string &map_name) {
    try {
      std::string path =
          ament_index_cpp::get_package_share_directory("web_test") +
          "/.last_map";
      std::ofstream f(path);
      if (f.is_open()) {
        f << map_name;
        f.close();
      }
    } catch (...) {
    }
  }

  std::string load_last_map() {
    try {
      std::string path =
          ament_index_cpp::get_package_share_directory("web_test") +
          "/.last_map";
      std::ifstream f(path);
      if (f.is_open()) {
        std::string map_name;
        std::getline(f, map_name);
        return map_name;
      }
    } catch (...) {
    }
    return "";
  }

  void start_server() {
    server_thread_ = std::thread([this]() {
      app_->signal_clear();
      app_->port(port_).multithreaded().run();
    });
  }

  void stop_server() {
    RCLCPP_INFO(this->get_logger(), "Stopping Crow server...");
    app_->stop();

    running_ = false;
    if (monitor_thread_.joinable()) {
      monitor_thread_.join();
    }

    if (server_thread_.joinable())
      server_thread_.join();

    // Stop any running child process
    std::lock_guard<std::mutex> _(process_mtx_);
    if (current_pid_ > 0) {
      process_manager_.stop_process(current_pid_);
    }
  }

  void monitor_process() {
    while (running_ && rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(process_mtx_);
        if (current_pid_ > 0) {
          int status;
          pid_t result = waitpid(current_pid_, &status, WNOHANG);
          if (result == current_pid_) {
            // Process exited
            RCLCPP_INFO(this->get_logger(), "Child process %d exited",
                        current_pid_);
            current_pid_ = -1;
            nav_state_ = ProcessState::STOPPED;
            mapping_state_ = ProcessState::STOPPED;
          }
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

private:
  std::string generate_session_token() {
    static int counter = 0;
    return "session_" + std::to_string(std::time(nullptr)) + "_" +
           std::to_string(++counter);
  }

  bool is_authenticated(const crow::request &req) {
    if (!auth_enabled_)
      return true; // No auth required

    auto cookies = req.get_header_value("Cookie");
    if (cookies.empty())
      return false;

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

    if (token.empty())
      return false;

    std::lock_guard<std::mutex> lock(mtx_);
    return active_sessions_.count(token) > 0;
  }

  void setup_routes() {
    // Root - Login page
    CROW_ROUTE((*app_), "/")
        .methods("GET"_method)(
            [this](const crow::request &req, crow::response &res) {
              if (is_authenticated(req)) {
                // Already logged in, serve nav.html directly (default)
                std::ifstream t(get_web_path("nav.html"));
                if (!t.is_open()) {
                  res.code = 500;
                  res.write("Navigation page not found");
                  res.end();
                  return;
                }
                std::stringstream buffer;
                buffer << t.rdbuf();
                res.set_header("Content-Type", "text/html");
                res.write(buffer.str());
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
              res.set_header("Content-Type", "text/html");
              res.write(buffer.str());
              res.end();
            });

    // Login endpoint
    CROW_ROUTE((*app_), "/login")
        .methods("POST"_method)([this](const crow::request &req) {
          auto body = crow::json::load(req.body);
          if (!body) {
            return crow::response(400, "Invalid JSON");
          }

          std::string username = body["username"].s();
          std::string password = body["password"].s();

          if (!auth_enabled_ || (username == env_vars_["WEB_USERNAME"] &&
                                 password == env_vars_["WEB_PASSWORD"])) {

            std::lock_guard<std::mutex> lock(mtx_);
            // Check if a controller is already active
            if (controller_connection_ != nullptr) {
              return crow::response(
                  503, "{\"success\": false, \"error\": \"System Busy: Another "
                       "user is controlling the robot.\"}");
            }

            // Valid credentials
            std::string token = generate_session_token();
            active_sessions_.insert(token);

            crow::response res(200);
            res.set_header("Set-Cookie", "session_token=" + token + "; Path=/");
            crow::json::wvalue result;
            result["success"] = true;
            res.write(result.dump());
            return res;
          } else {
            return crow::response(
                401,
                "{\"success\": false, \"error\": \"Invalid credentials\"}");
          }
        });

    // Map page - redirect to mapping.html
    CROW_ROUTE((*app_), "/map")
        .methods("GET"_method)(
            [this](const crow::request &req, crow::response &res) {
              res.code = 302;
              res.set_header("Location", "/mapping.html");
              res.end();
            });

    // Teleoperation page - redirect to mapping.html
    CROW_ROUTE((*app_), "/teleop")
        .methods("GET"_method)(
            [this](const crow::request &req, crow::response &res) {
              res.code = 302;
              res.set_header("Location", "/mapping.html");
              res.end();
            });

    // Navigation page
    CROW_ROUTE((*app_), "/nav")
        .methods("GET"_method)(
            [this](const crow::request &req, crow::response &res) {
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
              res.set_header("Content-Type", "text/html");
              res.write(buffer.str());
              res.end();
            });

    // Static file serving for CSS
    CROW_ROUTE((*app_), "/style.css")
        .methods("GET"_method)(
            [this](const crow::request &req, crow::response &res) {
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

    CROW_ROUTE((*app_), "/mapping.html")
        .methods("GET"_method)(
            [this](const crow::request &req, crow::response &res) {
              if (!is_authenticated(req)) {
                res.code = 302;
                res.set_header("Location", "/");
                res.end();
                return;
              }

              std::ifstream t(get_web_path("mapping.html"));
              if (!t.is_open()) {
                res.code = 404;
                res.write("mapping.html not found");
                res.end();
                return;
              }
              std::stringstream buffer;
              buffer << t.rdbuf();
              res.set_header("Content-Type", "text/html");
              res.write(buffer.str());
              res.end();
            });

    CROW_ROUTE((*app_), "/nav.html")
        .methods("GET"_method)(
            [this](const crow::request &req, crow::response &res) {
              if (!is_authenticated(req)) {
                res.code = 302;
                res.set_header("Location", "/");
                res.end();
                return;
              }

              std::ifstream t(get_web_path("nav.html"));
              if (!t.is_open()) {
                res.code = 404;
                res.write("nav.html not found");
                res.end();
                return;
              }
              std::stringstream buffer;
              buffer << t.rdbuf();
              res.set_header("Content-Type", "text/html");
              res.write(buffer.str());
              res.end();
            });

    // API Endpoints for Process Management

    CROW_ROUTE((*app_), "/api/status").methods("GET"_method)([this]() {
      crow::json::wvalue response;
      response["navigation"] = (nav_state_ == ProcessState::RUNNING);
      response["mapping"] = (mapping_state_ == ProcessState::RUNNING);
      return response;
    });

    CROW_ROUTE((*app_), "/api/start_mapping")
        .methods("POST"_method)([this](const crow::request &) {
          std::lock_guard<std::mutex> _(process_mtx_);

          if (mapping_state_ == ProcessState::RUNNING) {
            return "already_running";
          }

          if (current_pid_ > 0) {
            RCLCPP_INFO(this->get_logger(),
                        "Stopping previous process to start mapping");
            process_manager_.stop_process(current_pid_);
            nav_state_ = ProcessState::STOPPED;
          }

          RCLCPP_INFO(this->get_logger(), "Starting Mapping: %s",
                      mapping_cmd_.c_str());
          current_pid_ = process_manager_.start_process(mapping_cmd_);
          mapping_state_ = ProcessState::RUNNING;
          return "started";
        });

    CROW_ROUTE((*app_), "/api/start_navigation")
        .methods("POST"_method)([this](const crow::request &req) {
          std::lock_guard<std::mutex> _(process_mtx_);

          if (nav_state_ == ProcessState::RUNNING) {
            return "already_running";
          }

          auto body = crow::json::load(req.body);
          std::string map_file = "";
          if (body && body.has("map_name")) {
            std::string map_name = body["map_name"].s();
            save_last_map(map_name);
            // Map is now in a subdirectory: maps/<map_name>/<map_name>.yaml
            map_file = "map:=" +
                       (fs::path(map_folder_) / map_name / (map_name + ".yaml"))
                           .string();
          }

          if (current_pid_ > 0) {
            RCLCPP_INFO(this->get_logger(),
                        "Stopping previous process to start navigation");
            process_manager_.stop_process(current_pid_);
            mapping_state_ = ProcessState::STOPPED;
          }

          std::string cmd = nav_cmd_;
          if (!map_file.empty()) {
            cmd += " " + map_file;
          }

          RCLCPP_INFO(this->get_logger(), "Starting Navigation: %s",
                      cmd.c_str());
          current_pid_ = process_manager_.start_process(cmd);
          nav_state_ = ProcessState::RUNNING;
          return "started";
        });

    CROW_ROUTE((*app_), "/api/stop_navigation")
        .methods("POST"_method)([this]() {
          std::lock_guard<std::mutex> _(process_mtx_);
          if (nav_state_ == ProcessState::RUNNING && current_pid_ > 0) {
            process_manager_.stop_process(current_pid_);
            nav_state_ = ProcessState::STOPPED;
            current_pid_ = -1;
            return "stopped";
          }
          return "not_running";
        });

    CROW_ROUTE((*app_), "/api/stop_mapping").methods("POST"_method)([this]() {
      std::lock_guard<std::mutex> _(process_mtx_);
      if (mapping_state_ == ProcessState::RUNNING && current_pid_ > 0) {
        process_manager_.stop_process(current_pid_);
        mapping_state_ = ProcessState::STOPPED;
        current_pid_ = -1;
        return "stopped";
      }
      return "not_running";
    });

    CROW_ROUTE((*app_), "/api/save_map")
        .methods("POST"_method)([this](const crow::request &req) {
          auto body = crow::json::load(req.body);
          if (!body || !body.has("map_name"))
            return crow::response(400, "Missing map_name");

          std::string map_name = body["map_name"].s();

          // Create subdirectory for the map
          fs::path map_dir = fs::path(map_folder_) / map_name;
          if (!fs::exists(map_dir)) {
            fs::create_directories(map_dir);
          }

          // Save map inside the subdirectory
          std::string map_path = (map_dir / map_name).string();

          std::string cmd =
              "ros2 run nav2_map_server map_saver_cli -f " + map_path;
          RCLCPP_INFO(this->get_logger(), "Saving map: %s", cmd.c_str());

          int ret = std::system(cmd.c_str());
          if (ret == 0)
            return crow::response(200, "saved");
          else
            return crow::response(500, "failed to save");
        });

    CROW_ROUTE((*app_), "/api/list_maps").methods("GET"_method)([this]() {
      RCLCPP_INFO(this->get_logger(), "Listing maps from: %s",
                  map_folder_.c_str());
      crow::json::wvalue x;
      int i = 0;
      if (fs::exists(map_folder_)) {
        for (const auto &entry : fs::directory_iterator(map_folder_)) {
          // Check if it's a directory and contains the yaml file
          if (entry.is_directory()) {
            std::string map_name = entry.path().filename().string();
            fs::path yaml_path = entry.path() / (map_name + ".yaml");
            if (fs::exists(yaml_path)) {
              x[i++] = map_name;
            }
          }
        }
      }
      return x;
    });

    CROW_ROUTE((*app_), "/api/reset_mapping").methods("POST"_method)([this]() {
      std::lock_guard<std::mutex> _(process_mtx_);
      if (current_pid_ > 0) {
        process_manager_.stop_process(current_pid_);
      }
      RCLCPP_INFO(this->get_logger(), "Restarting Mapping: %s",
                  mapping_cmd_.c_str());
      current_pid_ = process_manager_.start_process(mapping_cmd_);
      return "reset";
    });

    CROW_ROUTE((*app_), "/ws")
        .websocket()
        .onopen([&](crow::websocket::connection &conn) {
          std::lock_guard<std::mutex> _(mtx_);
          if (users_.size() >= 1) {
            RCLCPP_WARN(this->get_logger(), "Rejecting second user connection");
            crow::json::wvalue x;
            x["type"] = "error";
            x["message"] = "Too many users";
            conn.send_text(x.dump());
            // Do NOT close connection here to avoid crash. Client will close
            // it.
            return;
          }

          int id = ++next_user_id_;
          users_[&conn] = id;
          RCLCPP_INFO(this->get_logger(),
                      "New websocket connection. User ID: %d. Total: %ld", id,
                      users_.size());
          // Do NOT send map data yet. Wait for auth.
        })
        .onclose(
            [&](crow::websocket::connection &conn, const std::string &reason) {
              std::lock_guard<std::mutex> _(mtx_);
              int id = 0;
              if (users_.count(&conn)) {
                id = users_[&conn];
                users_.erase(&conn);
              }

              if (&conn == controller_connection_) {
                RCLCPP_INFO(this->get_logger(), "Controller disconnected.");
                controller_connection_ = nullptr;
                controller_session_token_ = "";
              }

              RCLCPP_INFO(this->get_logger(),
                          "User %d disconnected: %s. Total: %ld", id,
                          reason.c_str(), users_.size());
            })
        .onmessage([this](crow::websocket::connection &conn,
                          const std::string &data, bool is_binary) {
          if (is_binary)
            return;

          // Ignore messages from rejected connections (not in users_)
          {
            std::lock_guard<std::mutex> lock(mtx_);
            if (!users_.count(&conn)) {
              return;
            }
          }

          try {
            auto x = crow::json::load(data);
            if (!x)
              return;

            if (x.has("type") && x["type"].s() == "auth") {
              std::string token = x["token"].s();
              std::lock_guard<std::mutex> lock(mtx_);

              if (active_sessions_.count(token) || !auth_enabled_) {
                // Valid session
                if (controller_connection_ != nullptr &&
                    controller_connection_ != &conn) {
                  // Close old connection
                  controller_connection_->close(
                      "Connection replaced by new session");
                  RCLCPP_INFO(this->get_logger(),
                              "Replacing old controller connection.");
                }

                controller_connection_ = &conn;
                controller_session_token_ = token;
                RCLCPP_INFO(this->get_logger(),
                            "Controller authenticated and active.");

                // Send initial state
                if (!last_map_json_.empty()) {
                  conn.send_text(last_map_json_);
                }

                // Send max velocity parameters
                crow::json::wvalue config_msg;
                config_msg["type"] = "config";
                config_msg["max_linear_velocity"] = max_linear_velocity_;
                config_msg["max_angular_velocity"] = max_angular_velocity_;
                conn.send_text(config_msg.dump());

                // Send current E-STOP state
                crow::json::wvalue estop_msg;
                estop_msg["type"] = "estop_state";
                estop_msg["active"] = emergency_stop_active_.load();
                conn.send_text(estop_msg.dump());
              } else {
                RCLCPP_WARN(this->get_logger(),
                            "Invalid auth token. Closing connection.");
                crow::json::wvalue x;
                x["type"] = "error";
                x["message"] = "Invalid authentication";
                conn.send_text(x.dump());
                // conn.close("Invalid authentication"); // Causes crash
              }
              return;
            }

            // Enforce control: Only controller_connection_ can send commands
            {
              std::lock_guard<std::mutex> lock(mtx_);
              if (&conn != controller_connection_) {
                // Ignore commands from non-controllers (or unauthenticated)
                RCLCPP_WARN(
                    this->get_logger(),
                    "Ignoring command from non-authenticated connection");
                return;
              }
            }

            if (x.has("type") && x["type"].s() == "cmd_vel") {
              // Check if emergency stop is active
              if (emergency_stop_active_) {
                // Silently ignore cmd_vel when E-STOP is active
                return;
              }

              // Validate required fields and types
              if (!x.has("linear") || !x.has("angular")) {
                RCLCPP_WARN(this->get_logger(),
                            "cmd_vel missing required fields");
                return;
              }

              if (x["linear"].t() != crow::json::type::Number ||
                  x["angular"].t() != crow::json::type::Number) {
                RCLCPP_WARN(this->get_logger(),
                            "cmd_vel fields must be numbers");
                return;
              }

              double linear = x["linear"].d();
              double angular = x["angular"].d();

              // Validate ranges
              if (std::isnan(linear) || std::isinf(linear) ||
                  std::isnan(angular) || std::isinf(angular)) {
                RCLCPP_WARN(this->get_logger(),
                            "cmd_vel contains invalid numeric values");
                return;
              }

              auto msg = geometry_msgs::msg::TwistStamped();
              msg.header.stamp = this->now();
              msg.header.frame_id = base_link_frame_;

              // Clamp to safe limits
              msg.twist.linear.x =
                  std::max(std::min(max_linear_velocity_, linear),
                           -max_linear_velocity_);
              msg.twist.angular.z =
                  std::max(std::min(max_angular_velocity_, angular),
                           -max_angular_velocity_);

              cmd_vel_pub_->publish(msg);
            } else if (x.has("type") && x["type"].s() == "set_goal") {
              // Check if emergency stop is active
              if (emergency_stop_active_) {
                RCLCPP_WARN(this->get_logger(),
                            "Goal rejected - emergency stop is active");
                // Send rejection message to client
                crow::json::wvalue reject_msg;
                reject_msg["type"] = "goal_rejected";
                reject_msg["reason"] = "Emergency stop is active";
                conn.send_text(reject_msg.dump());
                return;
              }

              // Validate required fields
              if (!x.has("x") || !x.has("y") || !x.has("theta")) {
                RCLCPP_WARN(this->get_logger(),
                            "set_goal missing required fields (x, y, theta)");
                return;
              }

              if (x["x"].t() != crow::json::type::Number ||
                  x["y"].t() != crow::json::type::Number ||
                  x["theta"].t() != crow::json::type::Number) {
                RCLCPP_WARN(this->get_logger(),
                            "set_goal fields must be numbers");
                return;
              }

              double goal_x = x["x"].d();
              double goal_y = x["y"].d();
              double theta = x["theta"].d();

              // Validate numeric values
              if (std::isnan(goal_x) || std::isinf(goal_x) ||
                  std::isnan(goal_y) || std::isinf(goal_y) ||
                  std::isnan(theta) || std::isinf(theta)) {
                RCLCPP_WARN(this->get_logger(),
                            "set_goal contains invalid numeric values");
                return;
              }

              RCLCPP_INFO(this->get_logger(),
                          "Received set_goal message from WebSocket");

              auto goal_msg = geometry_msgs::msg::PoseStamped();
              goal_msg.header.stamp = this->now();
              goal_msg.header.frame_id = map_frame_;
              goal_msg.pose.position.x = goal_x;
              goal_msg.pose.position.y = goal_y;
              goal_msg.pose.orientation.z = std::sin(theta / 2.0);
              goal_msg.pose.orientation.w = std::cos(theta / 2.0);

              goal_pub_->publish(goal_msg);
              RCLCPP_INFO(this->get_logger(),
                          "Published goal: x=%.2f, y=%.2f, theta=%.2f", goal_x,
                          goal_y, theta);
            } else if (x.has("type") && x["type"].s() == "set_initial_pose") {
              // Validate required fields
              if (!x.has("x") || !x.has("y") || !x.has("theta")) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "set_initial_pose missing required fields (x, y, theta)");
                return;
              }

              if (x["x"].t() != crow::json::type::Number ||
                  x["y"].t() != crow::json::type::Number ||
                  x["theta"].t() != crow::json::type::Number) {
                RCLCPP_WARN(this->get_logger(),
                            "set_initial_pose fields must be numbers");
                return;
              }

              double pose_x = x["x"].d();
              double pose_y = x["y"].d();
              double theta = x["theta"].d();

              // Validate numeric values
              if (std::isnan(pose_x) || std::isinf(pose_x) ||
                  std::isnan(pose_y) || std::isinf(pose_y) ||
                  std::isnan(theta) || std::isinf(theta)) {
                RCLCPP_WARN(this->get_logger(),
                            "set_initial_pose contains invalid numeric values");
                return;
              }

              RCLCPP_INFO(this->get_logger(),
                          "Received set_initial_pose message from WebSocket");

              auto initial_pose_msg =
                  geometry_msgs::msg::PoseWithCovarianceStamped();
              initial_pose_msg.header.stamp = this->now();
              initial_pose_msg.header.frame_id = map_frame_;
              initial_pose_msg.pose.pose.position.x = pose_x;
              initial_pose_msg.pose.pose.position.y = pose_y;
              initial_pose_msg.pose.pose.orientation.z = std::sin(theta / 2.0);
              initial_pose_msg.pose.pose.orientation.w = std::cos(theta / 2.0);
              initial_pose_pub_->publish(initial_pose_msg);
              RCLCPP_INFO(this->get_logger(),
                          "Published initial pose: x=%.2f, y=%.2f, theta=%.2f",
                          pose_x, pose_y, theta);
            } else if (x.has("type") && x["type"].s() == "emergency_stop") {
              // Toggle emergency stop state
              emergency_stop_active_ = !emergency_stop_active_;

              if (emergency_stop_active_) {
                RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP ACTIVATED!");

                // Publish zero velocity
                auto stop_msg = geometry_msgs::msg::TwistStamped();
                stop_msg.header.stamp = this->now();
                stop_msg.header.frame_id = base_link_frame_;
                stop_msg.twist.linear.x = 0.0;
                stop_msg.twist.linear.y = 0.0;
                stop_msg.twist.linear.z = 0.0;
                stop_msg.twist.angular.x = 0.0;
                stop_msg.twist.angular.y = 0.0;
                stop_msg.twist.angular.z = 0.0;

                // Publish multiple times to ensure it's received
                for (int i = 0; i < 10; i++) {
                  cmd_vel_pub_->publish(stop_msg);
                  std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }

                // Cancel all navigation goals using action client
                if (nav_action_client_) {
                  auto cancel_future =
                      nav_action_client_->async_cancel_all_goals();
                  RCLCPP_WARN(
                      this->get_logger(),
                      "Emergency stop - canceling all navigation goals");
                } else {
                  RCLCPP_WARN(this->get_logger(),
                              "Emergency stop - action client not ready");
                }
              } else {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Emergency stop DEACTIVATED - normal operation resumed");
              }

              // Send E-STOP state to client
              crow::json::wvalue estop_msg;
              estop_msg["type"] = "estop_state";
              estop_msg["active"] = emergency_stop_active_.load();
              conn.send_text(estop_msg.dump());
            }
          } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing JSON: %s",
                         e.what());
          }
        });
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received map: %d x %d", msg->info.width,
                msg->info.height);

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
      for (auto &[conn, id] : users_) {
        conn->send_text(json_str);
      }
    }
  }

  void timer_callback() {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(map_frame_, base_link_frame_,
                                      tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
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
    for (auto &[conn, id] : users_) {
      conn->send_text(json_str);
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
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
      transform = tf_buffer_->lookupTransform(map_frame_, msg->header.frame_id,
                                              tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
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
                       x_sensor * std::cos(yaw) - y_sensor * std::sin(yaw);
        double y_map = transform.transform.translation.y +
                       x_sensor * std::sin(yaw) + y_sensor * std::cos(yaw);

        points.push_back({x_map, y_map});
      }

      angle += msg->angle_increment;
    }

    x["points"] = std::move(points);
    std::string json_str = x.dump();

    std::lock_guard<std::mutex> _(mtx_);
    for (auto &[conn, id] : users_) {
      conn->send_text(json_str);
    }
  }

  void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    crow::json::wvalue x;
    x["type"] = "battery";
    x["percentage"] = msg->percentage * 100.0; // Convert to percentage
    x["voltage"] = msg->voltage;
    x["current"] = msg->current;
    x["charge"] = msg->charge;
    x["capacity"] = msg->capacity;
    x["power_supply_status"] = msg->power_supply_status;
    x["present"] = msg->present;

    std::string json_str = x.dump();

    std::lock_guard<std::mutex> _(mtx_);
    for (auto &[conn, id] : users_) {
      conn->send_text(json_str);
    }
  }

  void plan_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    crow::json::wvalue x;
    x["type"] = "plan";

    std::vector<std::vector<double>> path;
    path.reserve(msg->poses.size());

    for (const auto &pose_stamped : msg->poses) {
      // Extract yaw from quaternion
      double qz = pose_stamped.pose.orientation.z;
      double qw = pose_stamped.pose.orientation.w;
      double theta = 2.0 * std::atan2(qz, qw);

      path.push_back(
          {pose_stamped.pose.position.x, pose_stamped.pose.position.y, theta});
    }

    x["path"] = std::move(path);
    std::string json_str = x.dump();

    std::lock_guard<std::mutex> _(mtx_);
    for (auto &[conn, id] : users_) {
      conn->send_text(json_str);
    }
  }

  void
  goal_status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(),
                "Goal status callback triggered, %zu goals in list",
                msg->status_list.size());

    if (msg->status_list.empty())
      return;

    // Get the latest goal status
    auto &latest_status = msg->status_list.back();

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
    for (auto &[conn, id] : users_) {
      conn->send_text(json_str);
    }
  }

  std::string get_web_path(const std::string &filename) {
    try {
      std::string pkg_dir =
          ament_index_cpp::get_package_share_directory("web_test");
      return pkg_dir + "/web/" + filename;
    } catch (...) {
      // Fallback to source directory during development
      return "/home/linux/test_ws/src/web_test/web/" + filename;
    }
  }

  std::shared_ptr<crow::SimpleApp> app_;
  std::thread server_thread_;
  std::unordered_map<crow::websocket::connection *, int> users_;
  std::unordered_set<std::string> active_sessions_;

  // Single User Control
  std::string controller_session_token_;
  crow::websocket::connection *controller_connection_ = nullptr;

  int next_user_id_ = 0;
  std::string last_map_json_;

  std::unordered_map<std::string, std::string> env_vars_;
  bool auth_enabled_ = false;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr
      goal_status_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_pub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      nav_action_client_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string map_topic_;
  std::string cmd_vel_topic_;
  std::string scan_topic_;
  std::string battery_state_topic_;
  std::string plan_topic_;
  std::string goal_topic_;
  std::string initial_pose_topic_;
  std::string nav_action_client_topic_;
  std::string map_frame_;
  std::string base_link_frame_;
  std::string tf_prefix_;
  int port_;

  double max_linear_velocity_;
  double max_angular_velocity_;

  std::mutex mtx_;

  ProcessManager process_manager_;
  pid_t current_pid_ = -1;
  std::mutex process_mtx_;

  // Process state tracking
  std::atomic<ProcessState> nav_state_;
  std::atomic<ProcessState> mapping_state_;
  std::atomic<bool> running_;
  std::atomic<bool> emergency_stop_active_;
  // std::atomic<int> connected_users_{0}; // Removed in favor of users_.size()
  std::thread monitor_thread_;

  std::string mapping_cmd_;
  std::string nav_cmd_;
  std::string map_folder_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapViewerNode>();
  node->start_server();
  rclcpp::spin(node);
  node->stop_server();
  rclcpp::shutdown();
  return 0;
}
