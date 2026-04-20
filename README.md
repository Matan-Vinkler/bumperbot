# Bumperbot ROS2 Workspace

This workspace contains the packages and resources for the "Bumperbot" mobile robot. It includes core bringup and control stacks, localization and mapping tools, message definitions, examples in C++ and Python, and shared utilities. This workspace is influenced by the series-courses **Self Driving and ROS 2 - Learn by Doing**, made by Antonio Brandi.

## Link to Courses

- [Odometry & Control](https://www.udemy.com/course/self-driving-and-ros-2-learn-by-doing-odometry-control)
- [Map & Localization](https://www.udemy.com/course/self-driving-and-ros-2-learn-by-doing-map-localization)
- [Plan & Navigation](https://www.udemy.com/course/self-driving-and-ros-2-learn-by-doing-plan-navigation)

## Quick Start

- Clone the workspace and navigate to it:
```bash
git clone https://github.com/Matan-Vinkler/bumperbot.git bumperbot_ws
cd bumperbot_ws
```
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
ros2 launch bumperbot_bringup simulated_robot.launch.py world_name:=small_house
```
- In RViz2, use the "Nav2 Goal" button to point a goal location on the map and watch bumperbot autonomously navigate there. See the [Navigation Demo](#navigation-demo) section below for a video example.

## Navigation Demo

<video src="assets/navigation-video.mp4" type="video/mp4" width="640" height="400" controls></video>

[View demo](assets/navigation-video.mp4)

## Project Structure

Top-level `src/` contains the following packages:

- `bumperbot_bringup`
  - Purpose: Launches and configures the core robot systems and launch files required to bring the robot up.
  - Notes: Contains top-level launch files for simulation and bring-up.

- `bumperbot_controller`
  - Purpose: Contains controller implementations, launch files, and integration for running the robot's control stack.
  - Notes: Provides both C++ and Python controllers, configuration files for controllers, and joystick teleop integration.

- `bumperbot_mapping`
  - Purpose: Provides mapping nodes and utilities for building and serving environment maps using the robot's sensors.

- `bumperbot_localization`
  - Purpose: Provides localization and state-estimation configuration and nodes for accurate robot pose tracking.
  - Notes: Integrates with `robot_localization` and Nav2 lifecycle components where applicable.

- `bumperbot_navigation`
  - Purpose: Implements the robot's high‑level navigation layer by wrapping and configuring a Nav2-based stack.
  - Notes: Provides launch and configuration files that glue custom planner/motion plugins with Nav2 servers for autonomous path planning and obstacle avoidance.
- `bumperbot_description`
  - Purpose: Contains robot URDF/Xacro, visualization assets, and launch files for publishing the robot description.
  - Notes: Includes meshes, RViz configurations, and Gazebo/ignition launch helpers.

- `bumperbot_msgs`
  - Purpose: Defines the custom messages and actions used across the bumperbot system.
  - Notes: Message and action definitions are used by other packages in this workspace.

- `bumperbot_cpp_examples`
  - Purpose: Example C++ nodes and demos demonstrating rclcpp, actions, components, and TF usage with the robot.
  - Notes: Good reference for porting algorithms to C++ using ROS2 best practices.

- `bumperbot_py_examples`
  - Purpose: Example Python nodes and tutorials demonstrating rclpy usage, TF integration, and custom messages.
  - Notes: Contains small example nodes, tests and setup for Python packaging.

- `bumperbot_utils`
  - Purpose: Utility libraries, helper nodes, and tools shared across the bumperbot packages.

- `bumperbot_planning`
  - Purpose: Provides planning and trajectory generation nodes and utilities for high-level path planning and motion execution.
  - Notes: Implements planners and helper interfaces to integrate with controllers and navigation stacks.

- `bumperbot_motion`
  - Purpose: Motion layer utilities, trajectory followers, and controller interfaces for executing planned trajectories.
  - Notes: Bridges high-level planners with low-level controllers and includes helpers for smoothing and tracking trajectories.

## Running and Testing

- To run a single package's launch file, source the workspace and call `ros2 launch <package> <launchfile>` as shown above.
- To run examples:
  - C++ examples (after building) can be launched or run via `ros2 run bumperbot_cpp_examples <executable>`.
  - Python examples (after building) can be run via `ros2 run bumperbot_py_examples <node>`.

## Contributing

- Open a branch, make small, well-scoped changes, and submit a PR. Include tests where practical and update this README when adding new top-level functionality.

## Contact

- Maintainer: matan <matanvinkler@gmail.com>