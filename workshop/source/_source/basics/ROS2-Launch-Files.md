# ROS 2 launch files

## 1. Introduction

The TF2 example from the previous tutorial requires several terminals and
commands. A ROS 2 launch file describes those processes in one place and starts
them together with a single command.

In this exercise, the launch file will start:

- The Turtlesim simulator.
- A dynamic TF broadcaster for `turtle1`.
- A second turtle through the `/spawn` service.
- A dynamic TF broadcaster for `turtle2`.
- The TF listener that makes `turtle2` follow `turtle1`.
- The static broadcaster for the `turtle_cam1` frame.

The keyboard teleoperation node will still run in its own terminal because it
needs direct access to keyboard input.

## 2. Requirements

Complete the [TF2 tutorial](../navigation/ROS2-TF2.md) first. This exercise
assumes that its Python package is named `tf2_workshop` and provides these
executables:

```text
broadcaster = tf2_workshop.broadcaster:main
listener = tf2_workshop.listener:main
```

If you used different package or executable names, adjust the examples below.

The commands assume that the package is in `~/dev_ws/src/tf2_workshop`. Replace
`~/dev_ws` with your workspace path if necessary.

## 3. Create the launch directory

Launch files conventionally live in a package-level `launch` directory, next
to `package.xml` and `setup.py`:

```bash
cd ~/dev_ws/src/tf2_workshop
mkdir -p launch
touch launch/turtle_tf2_demo.launch.py
```

The package should now contain:

```text
tf2_workshop/
├── launch/
│   └── turtle_tf2_demo.launch.py
├── package.xml
├── setup.py
└── tf2_workshop/
    ├── broadcaster.py
    └── listener.py
```

## 4. Write the launch file

Open `launch/turtle_tf2_demo.launch.py` and add:

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen',
    )

    turtle1_broadcaster = Node(
        package='tf2_workshop',
        executable='broadcaster',
        name='turtle1_broadcaster',
        arguments=['turtle1'],
        output='screen',
    )

    spawn_turtle2 = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/spawn',
            'turtlesim/srv/Spawn',
            '{x: 2.0, y: 2.0, theta: 0.2, name: "turtle2"}',
        ],
        output='screen',
    )

    set_turtle2_pen = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/turtle2/set_pen',
            'turtlesim/srv/SetPen',
            '{r: 255, g: 0, b: 0, width: 5, "off": 0}',
        ],
        output='screen',
    )

    turtle2_broadcaster = Node(
        package='tf2_workshop',
        executable='broadcaster',
        name='turtle2_broadcaster',
        arguments=['turtle2'],
        output='screen',
    )

    turtle_follower = Node(
        package='tf2_workshop',
        executable='listener',
        name='turtle_follower',
        arguments=['turtle1', 'turtle2'],
        output='screen',
    )

    static_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='turtle_camera_static_broadcaster',
        arguments=[
            '--x', '0.1',
            '--y', '0.0',
            '--z', '0.0',
            '--yaw', '-1.57',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'turtle1',
            '--child-frame-id', 'turtle_cam1',
        ],
        output='screen',
    )

    start_turtle2_nodes = TimerAction(
        period=2.0,
        actions=[
            spawn_turtle2,
            turtle2_broadcaster,
            turtle_follower,
        ],
    )

    set_turtle2_pen_after_spawn = TimerAction(
        period=3.0,
        actions=[set_turtle2_pen],
    )

    return LaunchDescription([
        turtlesim,
        turtle1_broadcaster,
        static_broadcaster,
        start_turtle2_nodes,
        set_turtle2_pen_after_spawn,
    ])
```

The `Node` actions replace the individual `ros2 run` commands. The
`ExecuteProcess` actions call the `/spawn` and `/turtle2/set_pen` services.
The `TimerAction` delays give Turtlesim time to start and create `turtle2`
before its pen is configured.

The pen configuration is equivalent to:

```bash
ros2 service call /turtle2/set_pen turtlesim/srv/SetPen \
  '{r: 255, g: 0, b: 0, width: 5, "off": 0}'
```

The second turtle draws a thick red trail, making it easy to distinguish from
the turtle controlled with the keyboard.

The two broadcaster processes use explicit launch names. This avoids both
instances appearing in the ROS graph with the broadcaster's default node name.

## 5. Install the launch file

For an `ament_python` package, `setup.py` must install the launch file into the
package share directory.

Add the launch-directory entry to the existing `data_files` list:

```python
data_files=[
    (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name],
    ),
    ('share/' + package_name, ['package.xml']),
    (
        'share/' + package_name + '/launch',
        ['launch/turtle_tf2_demo.launch.py'],
    ),
],
```

Do not create a second `data_files` argument. Extend the list that is already
passed to `setup()`.

## 6. Declare runtime dependencies

Ensure these dependencies are present inside the `<package>` element of
`package.xml`. Do not duplicate entries that are already there:

```xml
<depend>geometry_msgs</depend>
<depend>rclpy</depend>
<depend>tf2_ros</depend>
<depend>turtlesim</depend>

<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
<exec_depend>python3-scipy</exec_depend>
<exec_depend>ros2launch</exec_depend>
```

`launch` and `launch_ros` provide the Python launch actions. The `ros2launch`
dependency ensures that the `ros2 launch` command and launch-file format
support are available at runtime.

## 7. Build and launch

Install any missing dependencies, build the package, and source the workspace:

```bash
cd ~/dev_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select tf2_workshop
source install/setup.bash
```

Start the complete TF2 application:

```bash
ros2 launch tf2_workshop turtle_tf2_demo.launch.py
```

Turtlesim should open with two turtles. The second turtle may start moving as
soon as its listener receives both transforms.

## 8. Drive the first turtle

Open another sourced terminal and start keyboard teleoperation:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/dev_ws/install/setup.bash
ros2 run turtlesim turtle_teleop_key
```

Select this terminal and use the arrow keys. The first turtle should move under
keyboard control, and the second turtle should follow it.

## 9. Verify the launched system

List the nodes from another sourced terminal:

```bash
ros2 node list
```

The output should include nodes similar to:

```text
/turtle1_broadcaster
/turtle2_broadcaster
/turtle_camera_static_broadcaster
/turtle_follower
/turtlesim
```

Inspect the transform between the turtles:

```bash
ros2 run tf2_ros tf2_echo turtle2 turtle1
```

Inspect the static camera transform:

```bash
ros2 run tf2_ros tf2_echo turtle1 turtle_cam1
```

You can also generate a diagram of the complete TF tree:

```bash
ros2 run tf2_tools view_frames
```

Press `Ctrl+C` in the launch terminal to stop all processes managed by the
launch file. Stop keyboard teleoperation separately in its own terminal.

## 10. Further reading

- [ROS 2 Jazzy: Integrating launch files into ROS 2
  packages](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-system.html)
- [ROS 2 launch framework](https://github.com/ros2/launch)
