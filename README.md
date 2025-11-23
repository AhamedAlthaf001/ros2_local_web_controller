# ROS 2 Web Controller

A modern web-based interface for controlling and monitoring ROS 2 robots with real-time visualization, mapping, and navigation capabilities.

## 🎯 Features

### ✅ Implemented
- **Real-time Map Visualization** - OccupancyGrid rendering with pan/zoom
- **Robot Teleoperation** - Keyboard and on-screen joystick control
- **Navigation Interface** - Click-to-goal with orientation setting
- **Mapping Interface** - Real-time SLAM visualization
- **LiDAR Visualization** - Live scan data overlay
- **Path Planning Display** - Shows Nav2 planned paths
- **Multi-Map Support** - Load, save, and switch between saved maps
- **Authentication** - Secure login with session management
- **Single-User Enforcement** - One active controller at a time
- **Emergency Stop** - Spacebar shortcut + button with goal cancellation
- **Keyboard Shortcuts** - Power user navigation (G/P/Esc/+/-/R)
- **WebSocket Auto-Reconnect** - Exponential backoff on connection loss
- **Input Validation** - Comprehensive validation for all messages
- **Confirmation Dialogs** - Prevents accidental stops
- **Dark/Light Theme** - Smooth theme switching

### 🔄 Planned
- JWT Authentication
- Session Expiry & Token Refresh
- Waypoint Following
- Navigation Progress Feedback

## 📋 Prerequisites

- **ROS 2** (Jazzy or newer)
- **Nav2** navigation stack
- **TurtleBot3** packages (for simulation)
- **OpenSSL** development libraries
- **Python 3** with bcrypt

```bash
sudo apt install libssl-dev python3-bcrypt
```

## 🚀 Quick Start

### 1. Build the Package

```bash
cd /path/to/your/workspace
colcon build --packages-select web_test
source install/setup.bash
```

### 2. Configure Credentials

Edit `.env` file in the package directory:
```bash
WEB_USERNAME=admin
WEB_PASSWORD=admin123
```

### 3. Launch Simulation (Optional)

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 4. Start the Web Interface

```bash
ros2 launch web_test web_app.launch.py
```

### 5. Open Browser

Navigate to: `http://localhost:8082`

**Default Credentials**: `admin` / `admin123`

## 🎮 Usage

### Navigation Page (`/nav`)
- **Set Goal**: Click "Set Goal" (or press `G`), then click-and-drag on map
- **Set Initial Pose**: Click "Set Pose" (or press `P`), then click-and-drag
- **Emergency Stop**: Press `Spacebar` or click the red E-STOP button
- **Zoom**: Use `+/-` keys or mouse wheel
- **Reset View**: Press `R`
- **Cancel Mode**: Press `Esc`

### Mapping Page (`/map`)
- **Start Mapping**: Click "Start Mapping"
- **Drive Robot**: Use keyboard (WASD/Arrows) or on-screen joystick
- **Save Map**: Enter name and click "Save Map"
- **Stop Mapping**: Click "Stop Mapping" (with confirmation)
- **Emergency Stop**: Press `Spacebar`

### Teleop Page (`/teleop`)
- **Keyboard Control**: WASD or Arrow keys
- **On-Screen Joystick**: Click and drag the joystick
- **Speed Profile**: Select Low/Medium/High speed

## ⚙️ Configuration

### Parameters (`config/params.yaml`)

```yaml
map_viewer_node:
  ros__parameters:
    port: 8082                          # Web server port
    map_topic: "/map"                   # OccupancyGrid topic
    cmd_vel_topic: "cmd_vel"           # Velocity command topic
    scan_topic: "scan"                  # LaserScan topic
    plan_topic: "plan"                  # Path topic
    goal_topic: "goal_pose"             # Goal topic
    initial_pose_topic: "initialpose"   # Initial pose topic
    map_frame: "map"                    # Map frame ID
    base_link_frame: "base_link"        # Robot base frame
    max_linear_velocity: 0.20           # m/s
    max_angular_velocity: 0.20          # rad/s
    map_folder: "/path/to/maps"         # Map storage directory
```

### Authentication (`.env`)

```bash
WEB_USERNAME=your_username
WEB_PASSWORD=your_password
```

> **Note**: Password hashing support is available. Use `scripts/hash_password.py` to generate hashed passwords.

## 🔒 Security Features

### Password Hashing (Infrastructure Ready)

Generate hashed passwords:
```bash
python3 scripts/hash_password.py
```

Verify passwords:
```bash
python3 scripts/hash_password.py --verify
```

### Session Management
- Secure session tokens
- Single active user enforcement
- Connection replacement on new login

## ⌨️ Keyboard Shortcuts

### Navigation Page
| Key | Action |
|-----|--------|
| `G` | Toggle Goal Mode |
| `P` | Toggle Initial Pose Mode |
| `Esc` | Cancel Mode |
| `Space` | Emergency Stop |
| `+` / `=` | Zoom In |
| `-` / `_` | Zoom Out |
| `R` | Reset View |

### Mapping Page
| Key | Action |
|-----|--------|
| `W` / `↑` | Move Forward |
| `S` / `↓` | Move Backward |
| `A` / `←` | Turn Left |
| `D` / `→` | Turn Right |
| `Space` | Emergency Stop |
| `+` / `-` | Zoom |
| `R` | Reset View |

## 🏗️ Architecture

### Components

```
web_test/
├── src/
│   ├── map_viewer_node.cpp    # Main web server + ROS node
│   └── map_handler.cpp         # Map save/load utilities
├── include/
│   ├── crow_all.h              # Crow web framework
│   └── password_utils.h        # Password hashing utilities
├── web/
│   ├── login.html              # Login page
│   ├── nav.html                # Navigation interface
│   ├── mapping.html            # Mapping interface
│   ├── teleop.html             # Teleoperation interface
│   └── style.css               # Shared styles
├── config/
│   └── params.yaml             # ROS parameters
├── launch/
│   └── web_app.launch.py       # Launch file
└── maps/                       # Saved map storage
```

### Communication

- **HTTP**: Login, map operations, API endpoints
- **WebSocket**: Real-time robot data (map, pose, scan, commands)
- **ROS 2 Topics**: Robot communication
- **ROS 2 Actions**: Navigation goal management

## 🐛 Troubleshooting

### Build Errors

**OpenSSL not found**:
```bash
sudo apt install libssl-dev
```

**nav2_msgs not found**:
```bash
sudo apt install ros-${ROS_DISTRO}-nav2-msgs
```

### Runtime Issues

**"Connection refused" in browser**:
- Check server is running: `ros2 node list | grep map_viewer`
- Verify port 8082 is not in use: `netstat -tuln | grep 8082`

**"Invalid authentication"**:
- Clear browser cookies
- Check `.env` file credentials
- Ensure session token is not expired

**Maps not loading**:
- Verify map folder exists and has correct permissions
- Check ROS parameter `map_folder` path
- Ensure `.pgm` and `.yaml` files exist

**WebSocket disconnects frequently**:
- Check network stability
- Verify auto-reconnect is working (status should show "Reconnecting...")
- Check server logs for errors

### Performance

**Slow map rendering**:
- Reduce map resolution in SLAM configuration
- Check browser hardware acceleration
- Monitor CPU usage

**High latency**:
- Check network if accessing remotely
- Reduce sensor data publish rates
- Use lower quality video streams (if applicable)

## 📁 Map Storage

Maps are saved in the configured `map_folder` directory with the following structure:

```
maps/
├── my_map.pgm      # Occupancy grid image
├── my_map.yaml     # Map metadata
├── office.pgm
└── office.yaml
```

### Map Format (YAML)
```yaml
image: my_map.pgm
resolution: 0.05        # meters/pixel
origin: [-10.0, -10.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
```

## 🔧 Development

### Adding New Features

1. **Backend** (C++): Edit `src/map_viewer_node.cpp`
2. **Frontend** (JS/HTML): Edit files in `web/`
3. **Configuration**: Update `config/params.yaml`
4. **Build**: `colcon build --packages-select web_test`

### WebSocket Message Format

**From Frontend**:
```json
{
  "type": "cmd_vel",
  "linear": 0.5,
  "angular": 0.3
}

{
  "type": "set_goal",
  "x": 2.5,
  "y": 1.0,
  "theta": 0.785
}

{
  "type": "emergency_stop"
}
```

**From Backend**:
```json
{
  "type": "map",
  "width": 384,
  "height": 384,
  "resolution": 0.05,
  "origin": {"x": -10.0, "y": -10.0},
  "data": [...]
}

{
  "type": "robot_pose",
  "x": 1.5,
  "y": 0.8,
  "theta": 0.5
}
```

## 📝 API Endpoints

### HTTP REST API

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Redirect to login or main page |
| `/login` | POST | Authenticate user |
| `/api/status` | GET | Get navigation/mapping status |
| `/api/start_navigation` | POST | Start Nav2  navigation |
| `/api/stop_navigation` | POST | Stop navigation |
| `/api/start_mapping` | POST | Start SLAM mapping |
| `/api/stop_mapping` | POST | Stop mapping |
| `/api/save_map` | POST | Save current map |
| `/api/list_maps` | GET | List available maps |
| `/api/reset_mapping` | POST | Reset mapping session |

### WebSocket
- `/ws` - Real-time bidirectional communication

## 📚 Dependencies

### ROS 2 Packages
- `rclcpp` - ROS 2 C++ client library
- `nav_msgs` - Navigation messages
- `geometry_msgs` - Geometric messages
- `sensor_msgs` - Sensor messages
- `tf2_ros` - Transform library
- `nav2_msgs` - Nav2 action messages
- `rclcpp_action` - Action client library

### C++ Libraries
- **Crow** - Web framework (header-only, included)
- **OpenSSL** - Cryptographic functions
- **nlohmann/json** - JSON parsing (via Crow)

### System
- `python3-bcrypt` - Password hashing utility

## 🤝 Contributing

When contributing new features:
1. Test thoroughly with real robot/simulation
2. Add validation for all user inputs
3. Update documentation
4. Follow existing code style
5. Add keyboard shortcuts where appropriate

## 📄 License

This project is licensed under the **Apache License 2.0** - see the [LICENSE](LICENSE) file for details.

Copyright 2025 Ahamed Althaf

[Specify your license here]

## 🙏 Acknowledgments

- Built with [Crow](https://github.com/CrowCpp/Crow) web framework
- Uses [Nav2](https://navigation.ros.org/) for navigation
- Inspired by modern web robotics interfaces

---

**Version**: 1.0.0  
**ROS 2 Distribution**: Jazzy  
**Maintainer**: [Ahamed Althaf]  
**Last Updated**: 2025-11-23
