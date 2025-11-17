# CLAUDE.md - AI Assistant Guide for Mini Pupper ROS 2

This document provides comprehensive guidance for AI assistants working with the Mini Pupper ROS 2 codebase. It explains the project structure, development workflows, conventions, and best practices to follow when making contributions or modifications.

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Repository Structure](#repository-structure)
3. [Package Architecture](#package-architecture)
4. [Development Workflows](#development-workflows)
5. [Code Conventions](#code-conventions)
6. [Testing Guidelines](#testing-guidelines)
7. [Common Development Tasks](#common-development-tasks)
8. [Key File Locations](#key-file-locations)
9. [Troubleshooting](#troubleshooting)

---

## Project Overview

### What is Mini Pupper ROS 2?

Mini Pupper ROS 2 is a comprehensive robotics platform built on **ROS 2 Humble** and **Ubuntu 22.04**, designed for the Mini Pupper quadruped robot. The project supports:

- **Autonomous Navigation**: SLAM mapping and Nav2 path planning
- **Computer Vision Tracking**: YOLO11n-powered person detection and following
- **Multi-Robot Fleet Management**: Coordinated multi-robot operations
- **Choreographed Movement**: Programmable dance sequences with audio sync
- **Simulation**: Gazebo integration for testing without hardware

### Key Technologies

- **ROS 2 Humble**: Robot Operating System 2 (LTS release)
- **Ubuntu 22.04**: Jammy Jellyfish (LTS)
- **Python 3.x**: Primary language for nodes and controllers
- **C++17**: Used for performance-critical nodes (fleet management, simulation)
- **Gazebo**: 3D robot simulator
- **Nav2**: Navigation framework
- **YOLO11n**: Neural network for object detection

### License

Apache 2.0 License - All contributions must be compatible with this license.

---

## Repository Structure

### Root Directory Layout

```
mini_pupper_ros/
├── .github/                      # GitHub Actions CI/CD workflows
│   └── workflows/
│       ├── industrial_ci.yml     # ROS build and test pipeline
│       └── lint.yml              # Code style checking (flake8, cpplint)
├── docs/                         # Documentation
│   └── detailed-setup-guide.md   # Setup, networking, troubleshooting
├── imgs/                         # Images and GIFs for documentation
├── mini_pupper_bringup/          # Main launch configurations
├── mini_pupper_dance/            # Dance choreography system
├── mini_pupper_description/      # URDF robot models
├── mini_pupper_driver/           # Hardware interface layer
├── mini_pupper_fleet/            # Multi-robot coordination (C++)
├── mini_pupper_interfaces/       # Custom ROS messages/services
├── mini_pupper_music/            # Audio playback services
├── mini_pupper_navigation/       # Nav2 integration and configs
├── mini_pupper_recognition/      # Line detection for line-following
├── mini_pupper_simulation/       # Gazebo simulation
├── mini_pupper_slam/             # SLAM with Cartographer/SLAM Toolbox
├── mini_pupper_tracking/         # YOLO-based person tracking
├── stanford_controller/          # Quadruped gait controller (Python)
├── .flake8                       # Python linting configuration
├── .gitignore                    # Git ignore patterns
├── .minipupper.repos             # External ROS dependencies (vcstool)
├── CONTRIBUTING.md               # Contribution guidelines
├── LICENSE                       # Apache 2.0 license text
├── README.md                     # Main project documentation
├── pc_install.sh                 # PC installation script
└── pupper_install.sh             # Robot installation script
```

### ROS 2 Packages Overview

| Package | Type | Purpose | Key Files |
|---------|------|---------|-----------|
| **mini_pupper_bringup** | Launch configs (CMake) | Main robot bringup with configurable hardware | 5 launch files, 2 YAML configs |
| **mini_pupper_driver** | Python (ament_python) | Hardware abstraction (servos, IMU, LiDAR, camera) | 5 nodes |
| **mini_pupper_description** | CMake + Python | URDF/Xacro robot models (two variants) | 2 publishers, mesh files |
| **mini_pupper_interfaces** | CMake (rosidl) | Shared message and service definitions | 7 messages, 3 services |
| **stanford_controller** | Python (ament_python) | Advanced quadruped gait control | 2 nodes, kinematics library |
| **mini_pupper_navigation** | Launch configs (CMake) | Nav2 stack with SmacPlanner2D/DWB | Navigation/SLAM launch files |
| **mini_pupper_slam** | Launch configs (CMake) | Cartographer and SLAM Toolbox integration | SLAM configs |
| **mini_pupper_tracking** | Python (ament_python) | YOLO11n person detection with multi-object tracking | 4 nodes, ONNX model |
| **mini_pupper_fleet** | C++ (ament_cmake) | Multi-robot fleet coordination with EKF | 3 C++ nodes |
| **mini_pupper_dance** | Python (ament_python) | Choreographed dance sequences | Service-based commands |
| **mini_pupper_music** | Python (ament_python) | Audio playback for dances | Music service |
| **mini_pupper_recognition** | Python (ament_python) | Line detection for line-following | CV-based line tracking |
| **mini_pupper_simulation** | Launch configs (CMake) | Gazebo simulation with ros2_control | Simulation worlds |

---

## Package Architecture

### Dependency Graph

```
mini_pupper_interfaces (Messages & Services)
         ↑
         |
         ├─── stanford_controller (Gait Control)
         ├─── mini_pupper_driver (Hardware Layer)
         ├─── mini_pupper_tracking (Computer Vision)
         ├─── mini_pupper_fleet (Multi-robot)
         └─── mini_pupper_dance (Choreography)

mini_pupper_bringup
         ↓
    Orchestrates:
         ├─── mini_pupper_driver
         ├─── mini_pupper_description
         ├─── stanford_controller
         └─── EKF Localization

mini_pupper_navigation
         ↓
    Uses: mini_pupper_slam
         ↓
    Uses: mini_pupper_description
```

### External Dependencies

Defined in `.minipupper.repos` (vcstool format):

- **champ**: Quadruped controller framework (alternative to Stanford controller)
- **champ_teleop**: Teleoperation package for champ
- **ldlidar_stl_ros2**: LiDAR driver for LD06 sensor

### Hardware Abstraction Layers

```
High-level Control (Navigation, Tracking, Dance)
                    ↓
      Stanford Controller / CHAMP
                    ↓
        Joint Trajectory Messages
                    ↓
          mini_pupper_driver
                    ↓
    MangDang.mini_pupper.HardwareInterface
                    ↓
         Physical Hardware (Servos, IMU, etc.)
```

---

## Development Workflows

### Git Branching Strategy

Follow **GitHub Flow**:

1. **Main branches**:
   - `ros2`: Primary development branch for ROS 2 Humble
   - `ros1`: Legacy ROS 1 branch

2. **Feature branches**:
   - Create from `ros2`: `git checkout ros2 && git checkout -b your-feature-branch`
   - Submit PRs targeting `ros2` branch

3. **Branch naming**: Use descriptive names (e.g., `feature/fleet-coordination`, `fix/imu-calibration`)

### Contribution Process

**IMPORTANT**: Before making any changes:

1. **Create an issue** or post on [Discord](https://discord.gg/xJdt3dHBVw) describing your planned changes
2. **Fork the repository** (for external contributors)
3. **Write code** following the conventions in this document
4. **Test thoroughly** (see Testing Guidelines section)
5. **Create a pull request** with the provided template
6. **Address feedback** until all discussions are resolved and CI passes

### Pull Request Requirements

A PR will only be merged if:

- All discussions are resolved
- All checklist items are checked off
- CI jobs pass successfully (industrial_ci and lint checks)
- Code follows ROS style guidelines
- Tests are included for new features

---

## Code Conventions

### Language Versions

- **Python**: Python 3.x (3.10+ on Ubuntu 22.04)
- **C++**: C++17 standard
- **ROS**: ROS 2 Humble (LTS)

### Code Style

#### Python Style

Follow [ROS Python Style Guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html):

- **Linter**: Flake8 with configuration in `.flake8`
- **Docstrings**: PEP 257 compliant
- **Line length**: 99 characters maximum
- **Imports**: Google import order style

**Flake8 configuration** (`.flake8`):
```ini
[flake8]
extend-ignore = B902,C816,D100,D101,D102,D103,D104,D105,D106,D107,D203,D212,D404,I202
import-order-style = google
max-line-length = 99
```

**Check Python style**:
```bash
ament_flake8
ament_pep257
```

#### C++ Style

Follow [ROS C++ Style Guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html):

- **Line length**: 100 characters maximum
- **Formatting**: Use ament_clang_format and ament_uncrustify

**Check C++ style**:
```bash
ament_clang_format --reformat
ament_uncrustify --reformat
ament_cpplint
```

### File Headers

**All files must include SPDX license header**:

```python
#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2025 MangDang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
```

### Naming Conventions

#### Packages
- Format: `mini_pupper_<function>` (lowercase with underscores)
- Examples: `mini_pupper_driver`, `mini_pupper_tracking`

#### ROS 2 Nodes
- Format: `<function>_node` (lowercase with underscores)
- Examples: `stanford_controller_node`, `fleet_controller_node`, `tracking_node`

#### Messages and Services
- Format: PascalCase
- Messages: `Command.msg`, `Tracking.msg`, `TrackingArray.msg`
- Services: `DanceCommand.srv`, `PlayMusic.srv`, `StopMusic.srv`

#### Python Classes
- Format: PascalCase
- Examples: `StanfordControllerNode`, `TrackingNode`, `FleetControllerNode`

#### Variables and Functions
- Format: snake_case (Python and C++)
- Examples: `horizontal_velocity`, `calculate_inverse_kinematics()`

### Node Structure Pattern

#### Python Node Template

```python
#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ExampleNode(Node):
    """ROS 2 Node for example functionality."""

    def __init__(self):
        """Initialize the node with parameters and communication interfaces."""
        super().__init__('example_node')

        # Declare parameters
        self.declare_parameter('param_name', default_value)
        self.param = self.get_parameter('param_name').value

        # Create subscriptions
        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.callback,
            10  # QoS depth
        )

        # Create publishers
        self.publisher = self.create_publisher(String, 'output_topic', 10)

        # Create timers for periodic execution
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.get_logger().info('Example node initialized')

    def callback(self, msg):
        """Process incoming messages."""
        self.get_logger().info(f'Received: {msg.data}')

    def timer_callback(self):
        """Execute periodic tasks."""
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**See**: `stanford_controller/stanford_controller/stanford_controller_node.py:23-60`

#### C++ Node Template

```cpp
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ExampleNode : public rclcpp::Node {
public:
    ExampleNode() : Node("example_node") {
        // Declare parameters
        this->declare_parameter("param_name", default_value);

        // Create subscriptions
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "input_topic",
            10,
            std::bind(&ExampleNode::callback, this, std::placeholders::_1)
        );

        // Create publishers
        publisher_ = this->create_publisher<std_msgs::msg::String>("output_topic", 10);

        // Create timers
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ExampleNode::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Example node initialized");
    }

private:
    void callback(const std_msgs::msg::String::ConstSharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
    }

    void timer_callback() {
        // Periodic execution
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExampleNode>());
    rclcpp::shutdown();
    return 0;
}
```

**See**: `mini_pupper_fleet/src/fleet_controller_node.cpp:20-49`

### Launch File Pattern

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Create nodes
    example_node = Node(
        package='package_name',
        executable='node_executable',
        name='node_name',
        parameters=[{
            'use_sim_time': use_sim_time,
            'param_name': param_value
        }],
        output='screen',
        condition=IfCondition(some_condition)  # Optional conditional launching
    )

    # Include other launch files
    included_launch = IncludeLaunchDescription(
        PathToLaunchFile(...),
        launch_arguments={'arg_name': 'arg_value'}.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        example_node,
        included_launch
    ])
```

**See**: `mini_pupper_bringup/launch/bringup.launch.py`

### Configuration Files

#### YAML Configuration Pattern

Configuration files follow this structure:

```yaml
# Node name
node_name:
  ros__parameters:
    # Parameter categories with comments
    category_name:
      parameter_1: value
      parameter_2: value

    # Another category
    another_category:
      parameter_3: value
```

**Example** (`mini_pupper_tracking/config/tracking_params.yaml`):
```yaml
mini_pupper_tracking_node:
  ros__parameters:
    yolo:
      image_size: 320
      confidence_threshold: 0.7
      iou_threshold: 0.35
    flask:
      image_display_size: 1280
      frame_rate: 15
      auto_open_browser: true
```

#### Hardware Configuration

Hardware sensor configuration (`mini_pupper_bringup/config/mini_pupper_2.yaml`):
```yaml
sensors:
  lidar: true
  imu: true
  camera: false
ports:
  lidar: '/dev/ttyAMA1'
```

---

## Testing Guidelines

### Test Framework

The project uses:
- **pytest**: Python unit and integration tests
- **launch_testing**: ROS 2 launch-based integration tests
- **ament_lint**: Code style and documentation checks

### Running Tests

#### All Packages
```bash
# From ROS workspace root
colcon test --packages-select-regex "mini_pupper*"
colcon test-result --verbose
```

#### Single Package
```bash
colcon test --packages-select mini_pupper_driver
colcon test-result --verbose
```

#### Style Checks Only
```bash
# Python
ament_flake8
ament_pep257

# C++
ament_clang_format --reformat
ament_uncrustify --reformat
ament_cpplint
```

### Test File Organization

Each package should include a `test/` directory:

```
package_name/
├── test/
│   ├── test_flake8.py          # Python linting
│   ├── test_pep257.py          # Docstring style
│   ├── test_copyright.py       # License headers
│   └── test_<functionality>.py # Integration tests
```

### Integration Test Template

```python
import pytest
import unittest
import rclpy
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions

@pytest.mark.rostest
def generate_test_description():
    """Generate launch description for testing."""
    node_under_test = Node(
        package='package_name',
        executable='node_executable',
        parameters=[{'param': 'value'}]
    )

    return LaunchDescription([
        node_under_test,
        launch_testing.actions.ReadyToTest(),
    ])

class TestNodeFunctionality(unittest.TestCase):
    """Test suite for node functionality."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS context."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS context."""
        rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures."""
        self.node = rclpy.create_node('test_node')
        self.publisher = self.node.create_publisher(MessageType, 'topic', 10)
        self.received_messages = []
        self.subscription = self.node.create_subscription(
            MessageType,
            'output_topic',
            lambda msg: self.received_messages.append(msg),
            10
        )

    def tearDown(self):
        """Clean up test fixtures."""
        self.node.destroy_node()

    def test_functionality(self):
        """Test specific functionality."""
        # Test implementation
        pass
```

**See**: `stanford_controller/test/test_normal_commands.py`

### CI/CD Pipeline

GitHub Actions runs automatically on push and pull requests:

#### Lint Workflow (`.github/workflows/lint.yml`)
- **flake8**: Python style checking with reviewdog
- **cpplint**: C++ style checking with reviewdog

#### Industrial CI Workflow (`.github/workflows/industrial_ci.yml`)
- Fetches external dependencies from `.minipupper.repos`
- Builds all packages with `colcon build`
- Runs all package tests
- Reports results

**CI must pass before PR merge.**

---

## Common Development Tasks

### Adding a New ROS 2 Node

#### Python Node

1. **Create node file** in `package_name/package_name/new_node.py`
2. **Implement node class** following the template in Code Conventions
3. **Add entry point** in `setup.py`:
   ```python
   entry_points={
       'console_scripts': [
           'new_node = package_name.new_node:main',
       ],
   }
   ```
4. **Create launch file** in `package_name/launch/new_node.launch.py`
5. **Add launch file to install** in `setup.py`:
   ```python
   (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
   ```
6. **Write tests** in `package_name/test/test_new_node.py`

#### C++ Node

1. **Create header** in `package_name/include/package_name/new_node.hpp`
2. **Create source** in `package_name/src/new_node.cpp` and `new_node_main.cpp`
3. **Add to CMakeLists.txt**:
   ```cmake
   add_executable(new_node src/new_node.cpp src/new_node_main.cpp)
   ament_target_dependencies(new_node rclcpp std_msgs)
   install(TARGETS new_node DESTINATION lib/${PROJECT_NAME})
   ```
4. **Create launch file** in `package_name/launch/new_node.launch.py`
5. **Install launch file** in `CMakeLists.txt`:
   ```cmake
   install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
   ```

### Adding a New Package

1. **Create package**:
   ```bash
   # Python package
   ros2 pkg create --build-type ament_python mini_pupper_<name>

   # C++ package
   ros2 pkg create --build-type ament_cmake mini_pupper_<name>
   ```

2. **Update package.xml**:
   - Set correct maintainer, description, and license (Apache-2.0)
   - Add dependencies

3. **Create standard directories**:
   ```
   mini_pupper_<name>/
   ├── launch/
   ├── config/
   ├── test/
   └── (src/ for C++ or package_name/ for Python)
   ```

4. **Add to README.md** if it's a major feature package

### Adding Custom Messages/Services

1. **Create message/service file** in `mini_pupper_interfaces/msg/` or `srv/`:
   ```
   # Example: NewMessage.msg
   float64 field_1
   string field_2
   int32 field_3
   ```

2. **Add to CMakeLists.txt** in `mini_pupper_interfaces`:
   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/NewMessage.msg"
     # ... other messages
     DEPENDENCIES std_msgs
   )
   ```

3. **Add dependency** in consuming packages' `package.xml`:
   ```xml
   <depend>mini_pupper_interfaces</depend>
   ```

4. **Use in code**:
   ```python
   from mini_pupper_interfaces.msg import NewMessage
   ```

### Modifying URDF/Robot Description

1. **Edit Xacro files** in `mini_pupper_description/urdf/mini_pupper_2/`
2. **Test URDF parsing**:
   ```bash
   ros2 launch mini_pupper_description mini_pupper_description.launch.py
   ```
3. **Visualize in RViz**:
   ```bash
   ros2 launch mini_pupper_bringup rviz.launch.py
   ```
4. **Check collision geometry** if adding new links

### Adding New Hardware Sensor

1. **Create driver node** in `mini_pupper_driver/mini_pupper_driver/new_sensor.py`
2. **Add launch file** for sensor: `mini_pupper_driver/launch/new_sensor.launch.py`
3. **Update hardware configuration** in `mini_pupper_bringup/config/mini_pupper_2.yaml`:
   ```yaml
   sensors:
     new_sensor: true
   ports:
     new_sensor: '/dev/ttyXXX'
   ```
4. **Include conditionally** in `mini_pupper_bringup/launch/hardware_interface.launch.py`

### Tuning Navigation Parameters

Navigation configs are in `mini_pupper_navigation/config/`:

- **SLAM**: Modify `slam_toolbox.yaml` or Cartographer configs
- **Planner**: Modify SmacPlanner2D parameters in navigation launch files
- **Controller**: Modify DWB or Regulated Pure Pursuit parameters

**Test in simulation first**:
```bash
ros2 launch mini_pupper_simulation simulation.launch.py
ros2 launch mini_pupper_navigation navigation.launch.py use_sim_time:=true
```

### Working with Multi-Robot Fleet

1. **Launch robots with namespaces**:
   ```bash
   # Robot 1
   ros2 launch mini_pupper_bringup bringup.launch.py namespace:=robot1

   # Robot 2
   ros2 launch mini_pupper_bringup bringup.launch.py namespace:=robot2
   ```

2. **Launch fleet controller**:
   ```bash
   ros2 launch mini_pupper_fleet fleet_controller.launch.py
   ```

3. **Send global commands**:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "..."
   ```

**See**: `mini_pupper_fleet/README.md` for detailed architecture

---

## Key File Locations

### Configuration Files

| File | Path | Purpose |
|------|------|---------|
| Mini Pupper 2 config | `mini_pupper_bringup/config/mini_pupper_2.yaml` | Hardware sensor configuration |
| Original Pupper config | `mini_pupper_bringup/config/mini_pupper.yaml` | Legacy hardware config |
| Tracking params | `mini_pupper_tracking/config/tracking_params.yaml` | YOLO and Flask settings |
| Movement params | `mini_pupper_tracking/config/movement_params.yaml` | PID control parameters |
| SLAM Toolbox config | `mini_pupper_slam/config/real_table.yaml` | SLAM parameters |
| Flake8 config | `.flake8` | Python linting rules |
| External deps | `.minipupper.repos` | vcstool dependency specification |

### Launch Files

| Launch File | Path | Purpose |
|-------------|------|---------|
| Main bringup | `mini_pupper_bringup/launch/bringup.launch.py` | Primary robot launch file |
| Hardware interface | `mini_pupper_bringup/launch/hardware_interface.launch.py` | Conditional sensor loading |
| Navigation | `mini_pupper_navigation/launch/navigation.launch.py` | Nav2 stack launch |
| SLAM | `mini_pupper_slam/launch/slam_toolbox.launch.py` | SLAM Toolbox launch |
| Tracking | `mini_pupper_tracking/launch/tracking.launch.py` | Person tracking system |
| Fleet controller | `mini_pupper_fleet/launch/fleet_controller.launch.py` | Multi-robot coordination |
| Simulation | `mini_pupper_simulation/launch/simulation.launch.py` | Gazebo simulation |

### Documentation

| Document | Path | Content |
|----------|------|---------|
| Main README | `README.md` | Project overview, quick start |
| Setup guide | `docs/detailed-setup-guide.md` | Installation, networking, troubleshooting |
| Contributing | `CONTRIBUTING.md` | Contribution workflow and guidelines |
| Code of conduct | `CODE_OF_CONDUCT.md` | Community standards |
| License | `LICENSE` | Apache 2.0 license text |
| Tracking docs | `mini_pupper_tracking/README.md` | YOLO tracking system details |
| Fleet docs | `mini_pupper_fleet/README.md` | Multi-robot architecture |
| Navigation docs | `mini_pupper_navigation/README.md` | SLAM and Nav2 integration |

### Core Nodes

| Node | Path | Function |
|------|------|----------|
| Stanford controller | `stanford_controller/stanford_controller/stanford_controller_node.py` | Quadruped gait control |
| Servo interface | `mini_pupper_driver/mini_pupper_driver/servo_interface.py` | Servo motor control |
| Tracking node | `mini_pupper_tracking/mini_pupper_tracking/tracking_node.py` | YOLO person detection |
| Fleet controller | `mini_pupper_fleet/src/fleet_controller_node.cpp` | Multi-robot command distribution |
| IMU EKF | `mini_pupper_fleet/src/imu_ekf_node.cpp` | Extended Kalman Filter for pose |

### Models and Data

| File | Path | Purpose |
|------|------|---------|
| YOLO model | `mini_pupper_tracking/models/yolo11n.onnx` | Pre-exported ONNX model |
| Meshes | `mini_pupper_description/meshes/mini_pupper_2/` | STL mesh files |
| URDF | `mini_pupper_description/urdf/mini_pupper_2/mini_pupper_description.urdf.xacro` | Robot model definition |

---

## Troubleshooting

### Common Issues

#### Build Failures

**Problem**: `colcon build` fails with missing dependencies

**Solution**:
1. Ensure external dependencies are fetched:
   ```bash
   vcs import < .minipupper.repos
   ```
2. Install ROS dependencies:
   ```bash
   rosdep install --from-paths . --ignore-src -r -y
   ```

#### Test Failures

**Problem**: Tests fail locally but not on CI

**Solution**:
- Ensure you're sourcing the workspace: `source install/setup.bash`
- Check ROS_DOMAIN_ID is consistent
- Verify test dependencies are installed

#### Multi-Robot Connection Issues

**Problem**: Robots on the same network can't see each other

**Solution**:
1. Check ROS_DOMAIN_ID is the same:
   ```bash
   export | grep ROS_DOMAIN_ID
   ```
2. Set the same ID on all machines:
   ```bash
   export ROS_DOMAIN_ID=42  # Add to ~/.bashrc
   ```
3. Verify network connectivity with `ros2 topic list`

**See**: `docs/detailed-setup-guide.md` (lines 71-100) for detailed networking setup

#### Tracking/Camera Issues

**Problem**: YOLO tracking not detecting people

**Solution**:
1. Verify camera is publishing images: `ros2 topic echo /camera/image_raw`
2. Check tracking parameters in `mini_pupper_tracking/config/tracking_params.yaml`
3. Lower confidence threshold for more detections (trade-off with false positives)
4. Ensure ONNX model exists: `mini_pupper_tracking/models/yolo11n.onnx`

#### Navigation/SLAM Issues

**Problem**: Robot doesn't navigate correctly

**Solution**:
1. Verify LiDAR is publishing: `ros2 topic echo /scan`
2. Check map is being built in RViz
3. Tune controller parameters in navigation config files
4. Ensure proper TF tree: `ros2 run tf2_tools view_frames.py`

### Getting Help

1. **Check documentation**: Start with `README.md` and package-specific READMEs
2. **Review issues**: Check [GitHub Issues](https://github.com/mangdangroboticsclub/mini_pupper_ros/issues)
3. **Ask on Discord**: Join the [Mini Pupper Discord](https://discord.gg/xJdt3dHBVw)
4. **Online docs**: Visit [minipupperdocs.readthedocs.io](https://minipupperdocs.readthedocs.io/en/latest/)

---

## Best Practices for AI Assistants

### When Making Changes

1. **Always read relevant files first**: Don't make assumptions about code structure
2. **Follow existing patterns**: Match the style and structure of surrounding code
3. **Test before committing**: Run `colcon build` and `colcon test`
4. **Update documentation**: If changing functionality, update corresponding README
5. **Use specific file paths**: Reference code with `file:line` format (e.g., `servo_interface.py:45`)

### When Analyzing Code

1. **Start with package.xml**: Understand dependencies first
2. **Check launch files**: Understand how nodes are started and configured
3. **Review message definitions**: In `mini_pupper_interfaces/` before working with topics
4. **Examine YAML configs**: Configuration often drives behavior

### When Debugging

1. **Check ROS topics**: Use `ros2 topic list` and `ros2 topic echo`
2. **Verify TF transforms**: Use `ros2 run tf2_tools view_frames.py`
3. **Monitor node output**: Launch with `output='screen'` in launch files
4. **Use RViz**: Visualize robot state, sensors, and navigation

### Communication Patterns

1. **Be specific**: Reference exact file paths and line numbers
2. **Explain trade-offs**: When suggesting changes, explain implications
3. **Follow ROS conventions**: Use standard ROS terminology (topics, services, actions)
4. **Cite documentation**: Link to relevant sections of this guide or READMEs

---

## Quick Reference

### Build and Test Commands

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select mini_pupper_driver

# Test all packages
colcon test --packages-select-regex "mini_pupper*"
colcon test-result --verbose

# Style checks
ament_flake8      # Python linting
ament_pep257      # Python docstrings
ament_cpplint     # C++ linting
```

### ROS 2 Useful Commands

```bash
# List all topics
ros2 topic list

# Echo topic data
ros2 topic echo /topic_name

# List all nodes
ros2 node list

# Node info
ros2 node info /node_name

# View TF tree
ros2 run tf2_tools view_frames.py

# Launch file
ros2 launch package_name launch_file.launch.py

# Run node directly
ros2 run package_name executable_name
```

### Package Structure Quick Template

```
mini_pupper_new_package/
├── CMakeLists.txt           (or setup.py for Python)
├── package.xml
├── setup.cfg                (Python only)
├── resource/                (Python only)
├── launch/
│   └── example.launch.py
├── config/
│   └── params.yaml
├── mini_pupper_new_package/ (Python) or src/ (C++)
│   └── example_node.py
└── test/
    ├── test_flake8.py
    ├── test_pep257.py
    └── test_example.py
```

---

## Version Information

- **ROS 2 Distro**: Humble (LTS)
- **Ubuntu**: 22.04 LTS (Jammy Jellyfish)
- **Python**: 3.10+
- **C++**: C++17
- **License**: Apache 2.0

---

**Last Updated**: 2025-11-17

For questions or suggestions about this guide, please open an issue on GitHub or discuss on Discord.
