# Vehicle Planner

Vehicle Planner is a ROS 2 motion-planning package that builds collision-aware paths from odometry and sensor data, optimizes B-spline trajectories, and publishes motion commands for an autonomous vehicle.

## Features

- Global and local path planning with A*, topological PRM, and room graphs
- Euclidean signed distance field (ESDF/SDF) and OctoMap-based collision checking
- Non-uniform B-spline trajectory generation and optimization
- Online replanning and trajectory execution
- ROS 2 visualization and custom B-spline messages

## Environment

The recommended development environment is:

- Ubuntu 22.04
- ROS 2 Humble
- C++17-compatible GCC or Clang
- CMake 3.8 or later
- colcon and rosdep
- Boost (`date_time`)
- Eigen3
- OpenCV
- PCL
- yaml-cpp
- NLopt
- OpenMP
- OctoMap and the ROS 2 PCL/OctoMap bridges

The package also uses the following ROS 2 packages: `rclcpp`, `sensor_msgs`,
`nav_msgs`, `geometry_msgs`, `visualization_msgs`, `tf2_ros`, `cv_bridge`,
`pcl_ros`, `pcl_conversions`, `octomap_ros`, `octomap_msgs`, and the ROS
interface generators.

## Project Structure

```text
.
├── LICENSE
├── README.md
└── planner_manager/
    └── planner/
        ├── CMakeLists.txt          # Build configuration and executable targets
        ├── package.xml             # ROS 2 package metadata and dependencies
        ├── config/                 # Planner, map, and optimization parameters
        ├── include/                # Planner headers
        │   ├── plugin/             # Planning and mapping algorithms
        │   └── utils/              # Shared utility headers
        ├── launch/                 # ROS 2 launch files
        ├── map/                    # OctoMap data
        ├── msg/                    # Custom ROS 2 messages
        ├── script/                 # Trajectory-analysis Python scripts
        ├── src/                    # Planner implementation
        │   ├── plugin/             # Algorithm implementations
        │   └── utils/              # Utility implementations
        └── test/                   # Test and example nodes
```

## Installation

1. Install ROS 2 Humble by following the
   [official ROS 2 documentation](https://docs.ros.org/en/humble/Installation.html),
   then install the build tools:

   ```bash
   sudo apt update
   sudo apt install python3-colcon-common-extensions python3-rosdep
   ```

2. Clone the repository:

   ```bash
   git clone <repository-url> vehicle_planner-XJTLU-FYP
   cd vehicle_planner-XJTLU-FYP
   ```

3. Initialize `rosdep` if it has not been initialized on the machine:

   ```bash
   sudo rosdep init
   rosdep update
   ```

4. Source ROS 2 and install the declared package dependencies:

   ```bash
   source /opt/ros/humble/setup.bash
   rosdep install --from-paths planner_manager --ignore-src -r -y
   ```

5. Install the ROS bridge packages and required native development libraries:

   ```bash
   sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions \
     ros-humble-octomap-ros ros-humble-octomap-msgs ros-humble-cv-bridge \
     libboost-date-time-dev libeigen3-dev libopencv-dev libpcl-dev \
     libyaml-cpp-dev libnlopt-cxx-dev liboctomap-dev
   ```

6. Build the workspace from the repository root:

   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

## Launch

Source ROS 2 and the local workspace in every new terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Launch the planner manager and trajectory server:

```bash
ros2 launch planner all.launch.py
```

By default, the planner expects odometry on `/fastlio2/lio_odom`, a point cloud
on `/fastlio2/world_cloud`, IMU data on `/livox/imu`, and a target on
`/target_pos` (`geometry_msgs/msg/PoseStamped`) or `/path4local`
(`nav_msgs/msg/Path`). The trajectory server publishes velocity commands as
`geometry_msgs/msg/Twist` on `/traj`.

The default topic names can be changed with ROS 2 parameters. For example:

```bash
ros2 run planner planner_manager --ros-args \
  -p odom_topic_name:=/odom \
  -p cloud_topic_name:=/points \
  -p target_pos_name:=/goal_pose
```

Available example launch files include:

```bash
ros2 launch planner test_bspline.launch.py
ros2 launch planner test_example_target.launch.py
ros2 launch planner test_path.launch.py
```

Planner behavior is configured through the YAML files in
`planner_manager/planner/config/`. In particular, `initial.yaml` contains the
motion limits, planning modes, replanning thresholds, and preset waypoints.

## License

See [LICENSE](LICENSE) for license information.
