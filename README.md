# Space Rover - ROS2 Simulation and Control

A comprehensive ROS2-based simulation and control system for a multi-terrain space rover with support for Mars, Moon, and Earth environments.

## Features

- **Multi-Environment Simulation**: Gazebo simulations for Mars, Moon, Earth forest, and curiosity terrain
- **Advanced URDF Model**: Complete rover kinematic chain with realistic sensor integration
- **Motor Control**: Dual implementation (Python and C++ versions) for motor command parsing and control
- **Localization & Mapping**: EKF-based localization and RTABMAP visual SLAM integration
- **Sensor Suite**: Support for RGB-D camera (ASUS Xtion), LiDAR (Hokuyo), IMU, and GPS (u-blox)
- **Navigation Stack**: Full navigation pipeline with behavior trees
- **Teleoperation**: Keyboard and joystick-based teleoperation support
- **URDF/Xacro**: Modular robot description with support for multiple suspension systems (rocker-bogie, differential, turnbuckle)

## Project Structure

```
ros2_rover/
├── rover_bringup/              # Launch files and configurations for rover startup
├── rover_description/          # Robot URDF/Xacro files and meshes
├── rover_gazebo/               # Gazebo simulation environment and plugins
├── rover_localization/         # Localization and mapping (EKF, RTABMAP)
├── rover_motor_controller/     # Python-based motor control
├── rover_motor_controller_cpp/ # C++-based motor control
├── rover_msgs/                 # Custom ROS2 message definitions
├── rover_navigation/           # Navigation stack and behavior trees
├── rover_teleop/               # Teleoperation nodes
└── rover_service/              # Systemd service files
```

## Installation

### Prerequisites

- **ROS2** (Humble or later)
- **Gazebo** (11+)
- **Python 3.8+**
- **Colcon** build system
- **Dependencies**: 
  ```bash
  sudo apt-get install ros-humble-joint-state-publisher
  sudo apt-get install ros-humble-xacro
  sudo apt-get install ros-humble-nav2-*
  sudo apt-get install ros-humble-slam-toolbox
  ```

### Build Instructions

```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws

# Build all packages
colcon build --symlink-install

# Source the setup file
source install/setup.bash
```

## Usage

### Launch Rover in Different Environments

**Mars Environment:**
```bash
ros2 launch rover_gazebo mars.launch.py
```

**Moon Environment:**
```bash
ros2 launch rover_gazebo moon.launch.py
```

**Forest Terrain:**
```bash
ros2 launch rover_gazebo forest.launch.py
```

**Low Moon (Low Gravity):**
```bash
ros2 launch rover_gazebo low_moon.launch.py
```

### Bring Up Rover Hardware

```bash
ros2 launch rover_bringup rover.launch.py
```

### Launch Teleoperation

**Keyboard Control:**
```bash
ros2 launch rover_teleop joy_teleop.launch.py
```

### Navigation and Localization

**Start Navigation Stack:**
```bash
ros2 launch rover_navigation navigation.launch.py
```

**Start Localization:**
```bash
ros2 launch rover_localization localization.launch.py
```

**Start RTABMAP SLAM:**
```bash
ros2 launch rover_localization rtabmap.launch.py
```

## Package Descriptions

### rover_bringup
- Main launch files for rover initialization
- Hardware sensor configuration (GPS, LiDAR)
- Parameter management for field deployment

### rover_description
- Complete URDF/Xacro robot model
- Multiple suspension system options (rocker-bogie, differential, turnbuckle bogie)
- 3D meshes for visualization
- Sensor attachment points

### rover_gazebo
- Gazebo world definitions for different planets
- Ground truth remapper node
- Motor command parser
- Odometry publisher
- Physics simulation configuration

### rover_motor_controller & rover_motor_controller_cpp
- LX-16A servo motor interface
- Velocity command parsing (`/cmd_vel` to individual motor commands)
- Motor control node implementations in Python and C++

### rover_localization
- Extended Kalman Filter for state estimation
- RGB-D odometry
- RTABMAP visual SLAM integration
- Multi-sensor fusion

### rover_navigation
- Nav2 navigation stack integration
- Behavior tree definitions
- Global and local costmap configuration
- Path planning and obstacle avoidance

### rover_teleop
- Keyboard teleoperation node
- Joystick support via joy package

### rover_msgs
- Custom message definitions for rover-specific communication

## Configuration Files

Key configuration files are located in each package's `config/` directory:

- `rover_description/config/control.yaml` - Control parameters
- `rover_gazebo/config/gazebo.yaml` - Physics simulation settings
- `rover_gazebo/config/odometry.yaml` - Odometry configuration
- `rover_localization/config/ekf.yaml` - EKF filter parameters
- `rover_bringup/config/ublox.yaml` - GPS configuration
- `rover_bringup/config/urg_node_serial.yaml` - LiDAR configuration

## Topics and Services

### Key Topics
- `/cmd_vel` - Velocity commands (input)
- `/odom` - Odometry information
- `/tf` - Transform tree
- `/camera/rgb/image_raw` - RGB camera stream
- `/scan` - LiDAR scan data
- `/imu/data` - IMU data
- `/gps/fix` - GPS position

### Motor Control Topics
- `/motor_commands` - Individual motor target positions/velocities

## Troubleshooting

**Build Issues:**
```bash
# Clean build
colcon build --symlink-install --cmake-clean-cache
```

**Gazebo Simulation Issues:**
- Ensure `GAZEBO_MODEL_PATH` includes the rover models directory
- Check physics engine settings in world files

**Sensor Data Not Publishing:**
- Verify sensor plugins are enabled in launch files
- Check configuration YAML files for correct parameters

## Hardware Integration

The rover can be deployed on actual hardware with:
- LX-16A servo motors
- u-blox GPS receiver
- Hokuyo LiDAR
- ASUS Xtion RGB-D camera
- 9-DOF IMU

Service files are provided in `rover_service/` for systemd integration.

## Contributing

Contributions are welcome! Please ensure:
1. Code follows ROS2 style guidelines
2. All packages build cleanly with `colcon build`
3. Launch files are tested before committing

## License

Please see LICENSE file for details.

## Authors

- **PES1UG23CS448** - Main development

## Acknowledgments

- ROS2 community and documentation
- Gazebo simulation environment
- Nav2 navigation stack
