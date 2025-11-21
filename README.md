# Bumperbot ROS2 Workspace

This workspace contains the packages and resources for the "Bumperbot" mobile robot. It includes core bringup and control stacks, localization and mapping tools, message definitions, examples in C++ and Python, and shared utilities. This workspace is influenced by the series-courses **Self Driving and ROS 2 - Learn by Doing**, made by Antonio Brandi.

## Link to Courses

- [Odometry & Control](https://www.udemy.com/course/self-driving-and-ros-2-learn-by-doing-odometry-control)
- [Map & Localization](https://www.udemy.com/course/self-driving-and-ros-2-learn-by-doing-map-localization)
- [Plan & Navigation](https://www.udemy.com/course/self-driving-and-ros-2-learn-by-doing-plan-navigation)

## Quick Start

- Build the workspace:
```bash
colcon build --executor sequential
```
- Source the workspace (in a new shell):
```bash
source install/setup.bash
```
- Run a bringup or controller launch, for example:
```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py
```

## Project Structure

Top-level `src/` contains the following packages:

- `bumperbot_bringup` (ament_cmake)
  - Purpose: Launches and configures the core robot systems and launch files required to bring the robot up.
  - Notes: Contains top-level launch files for simulation and bring-up.

- `bumperbot_controller` (ament_cmake)
  - Purpose: Contains controller implementations, launch files, and integration for running the robot's control stack.
  - Notes: Provides both C++ and Python controllers, configuration files for controllers, and joystick teleop integration.

- `bumperbot_mapping` (ament_cmake)
  - Purpose: Provides mapping nodes and utilities for building and serving environment maps using the robot's sensors.

- `bumperbot_localization` (ament_cmake)
  - Purpose: Provides localization and state-estimation configuration and nodes for accurate robot pose tracking.
  - Notes: Integrates with `robot_localization` and Nav2 lifecycle components where applicable.

- `bumperbot_description` (ament_cmake)
  - Purpose: Contains robot URDF/Xacro, visualization assets, and launch files for publishing the robot description.
  - Notes: Includes meshes, RViz configurations, and Gazebo/ignition launch helpers.

- `bumperbot_msgs` (ament_cmake, rosidl)
  - Purpose: Defines the custom messages and actions used across the bumperbot system.
  - Notes: Message and action definitions are used by other packages in this workspace.

- `bumperbot_cpp_examples` (ament_cmake)
  - Purpose: Example C++ nodes and demos demonstrating rclcpp, actions, components, and TF usage with the robot.
  - Notes: Good reference for porting algorithms to C++ using ROS2 best practices.

- `bumperbot_py_examples` (ament_python)
  - Purpose: Example Python nodes and tutorials demonstrating rclpy usage, TF integration, and custom messages.
  - Notes: Contains small example nodes, tests and setup for Python packaging.

- `bumperbot_utils` (ament_cmake)
  - Purpose: Utility libraries, helper nodes, and tools shared across the bumperbot packages.

## Development Notes

- Package manifests (`package.xml`) include the MIT license entry. If you want a full license file at the workspace root, I can add a `LICENSE` file containing the MIT text.
- Most packages are `ament_cmake` projects and are built with `colcon build`. The Python examples use `ament_python` and will be installed into the workspace during build.

## Running and Testing

- To run a single package's launch file, source the workspace and call `ros2 launch <package> <launchfile>` as shown above.
- To run examples:
  - C++ examples (after building) can be launched or run via `ros2 run bumperbot_cpp_examples <executable>`.
  - Python examples (after building) can be run via `ros2 run bumperbot_py_examples <node>`.

## Contributing

- Open a branch, make small, well-scoped changes, and submit a PR. Include tests where practical and update this README when adding new top-level functionality.

## Contact

- Maintainer: matan <matanvinkler@gmail.com>