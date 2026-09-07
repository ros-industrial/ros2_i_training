# ROS 2 Jazzy CLI cheat sheet

Quick-reference commands for ROS 2 Jazzy Jalisco.

## Setup

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Nodes and topics

```bash
ros2 node list
ros2 node info /node

ros2 topic list -t
ros2 topic type /topic
ros2 topic info /topic -v
ros2 topic echo /topic
ros2 topic hz /topic
ros2 topic bw /topic
ros2 topic delay /topic
ros2 topic pub /topic TYPE '{field: value}'
ros2 topic pub --once /topic TYPE '{field: value}'
ros2 topic pub -r 10 /topic TYPE '{field: value}'
```

## Services and actions

```bash
ros2 service list -t
ros2 service type /service
ros2 service find TYPE
ros2 service call /service TYPE '{field: value}'

ros2 action list -t
ros2 action info /action
ros2 action send_goal /action TYPE '{field: value}'
ros2 action send_goal /action TYPE '{field: value}' --feedback
```

## TF2 and TF visualization

Install the TF2 command-line tools:

```bash
sudo apt install ros-jazzy-tf2-ros ros-jazzy-tf2-tools
```

Inspect transforms and generate a TF tree:

```bash
ros2 run tf2_ros tf2_echo SOURCE_FRAME TARGET_FRAME
ros2 run tf2_ros tf2_monitor
ros2 run tf2_tools view_frames
ros2 run tf2_ros static_transform_publisher --help
```

`view_frames` writes the TF tree to a PDF file. RViz visualizes TF frames live.

## Parameters and interfaces

```bash
ros2 param list /node
ros2 param get /node parameter_name
ros2 param set /node parameter_name value
ros2 param describe /node parameter_name
ros2 param dump /node
ros2 param load /node params.yaml

ros2 interface list
ros2 interface show package_name/msg/Type
ros2 interface proto package_name/msg/Type
```

## Packages, executables, and launch files

```bash
ros2 pkg list
ros2 pkg executables package_name
ros2 pkg prefix package_name
ros2 pkg xml package_name
ros2 run package_name executable_name
ros2 launch package_name file.launch.py name:=value
```

## Package creation

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --license Apache-2.0 my_pkg
ros2 pkg create --build-type ament_python --license Apache-2.0 my_pkg
ros2 pkg create --dependencies rclcpp std_msgs my_pkg
```

## Workspaces, colcon, and rosdep

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
colcon build --packages-select my_pkg
colcon build --packages-up-to my_pkg
source install/setup.bash
colcon test --packages-select my_pkg
colcon test-result --verbose
```

### Create a workspace from source

```bash
mkdir workspace_name
cd workspace_name
mkdir src
cd src
git clone <some-ROS-2-package-from-GitHub>
cd ..
vcs import src < <some-ROS-2-package-from-GitHub>/<dependency-file.repos>
rosdep init
rosdep install --from-paths src -iry
```

## Common ROS arguments and rosbag

```bash
--ros-args -r old:=new
--ros-args -r __node:=name
--ros-args -r __ns:=/namespace
--ros-args -p name:=value
--ros-args --params-file file.yaml

ros2 bag record -a
ros2 bag record /topic
ros2 bag info BAG
ros2 bag play BAG
```

## ros2_control

Install ros2_control and the standard controllers:

```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
```

Manage controllers and inspect hardware:

```bash
ros2 control list_controllers
ros2 control list_controller_types
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 control load_controller NAME PARAMS.yaml
ros2 control unload_controller NAME
ros2 control set_controller_state NAME active
ros2 control set_controller_state NAME inactive
ros2 control switch_controllers --activate A --deactivate B
ros2 control cleanup_controller NAME
```

Use `-c /namespace/controller_manager` when targeting a namespaced controller manager.

## GUI and visualization tools

### rqt

```bash
sudo apt install ros-jazzy-rqt
rqt
```

### rqt_graph

```bash
sudo apt install ros-jazzy-rqt-graph
rqt_graph
```

### PlotJuggler

```bash
sudo apt install ros-jazzy-plotjuggler-ros
ros2 run plotjuggler plotjuggler
```

### RViz

```bash
sudo apt install ros-jazzy-rviz2
rviz2
```

## Daemon, diagnostics, lifecycle, and components

```bash
ros2 daemon status
ros2 daemon start
ros2 daemon stop
ros2 doctor --report
ros2 wtf

ros2 lifecycle nodes
ros2 lifecycle get /node
ros2 lifecycle list /node
ros2 lifecycle set /node configure
ros2 lifecycle set /node activate
ros2 lifecycle set /node deactivate
ros2 lifecycle set /node cleanup
ros2 lifecycle set /node shutdown

ros2 component list
ros2 component types
ros2 component load /container package_name Plugin
ros2 component unload /container component_id
```

Use `-h` or `--help` to display all options for a command.

## References

- [ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/)
- [ros2_control Jazzy documentation](https://control.ros.org/jazzy/)
