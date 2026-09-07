# ROS 2 command-line interface

## 1. Introduction

The `ros2` command-line interface (CLI) lets you start nodes and inspect or
interact with a running ROS 2 system. Its subcommands cover nodes, topics,
services, actions, parameters, interfaces, and many other ROS 2 concepts.

The official [ROS 2 Jazzy beginner CLI
tutorials](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html)
provide additional examples.

Source ROS 2 in every new terminal:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

Display the available subcommands:

```bash
ros2 --help
```

Most commands provide more detailed help:

```bash
ros2 topic --help
ros2 topic echo --help
```

## 2. Communication patterns

ROS 2 uses three principal communication patterns:

| Interface | Pattern | Typical use |
| --- | --- | --- |
| Topic | Publisher/subscriber | Continuous streams of data |
| Service | Request/response | Short operations that return one response |
| Action | Goal/feedback/result | Longer operations that can provide feedback and be cancelled |

Messages, services, and actions are described by interface definitions. Inspect
an interface with:

```bash
ros2 interface show geometry_msgs/msg/Twist
```

List the interfaces currently installed:

```bash
ros2 interface list
```

## 3. Run and inspect nodes

The general form for starting an executable is:

```text
ros2 run PACKAGE EXECUTABLE
```

For example:

```bash
ros2 run turtlesim turtlesim_node
```

In another terminal, list running nodes:

```bash
ros2 node list
```

Inspect a particular node:

```bash
ros2 node info /turtlesim
```

Node names should be unique within a ROS graph. ROS 2 can start nodes with the
same name, but doing so makes introspection and communication ambiguous.

## 4. Inspect and publish topics

With Turtlesim running, list topics and their types:

```bash
ros2 topic list -t
```

Inspect a topic:

```bash
ros2 topic info /turtle1/cmd_vel --verbose
```

Display messages:

```bash
ros2 topic echo /turtle1/pose
```

Measure publishing frequency and bandwidth:

```bash
ros2 topic hz /turtle1/pose
ros2 topic bw /turtle1/pose
```

Publish a velocity command at 2 Hz:

```bash
ros2 topic pub --rate 2 /turtle1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0}, angular: {z: 1.0}}"
```

Stop a continuous publisher with `Ctrl+C`. To publish only once, replace
`--rate 2` with `--once`.

## 5. Inspect and call services

List services and their types:

```bash
ros2 service list -t
```

Inspect the type used by a service:

```bash
ros2 service type /spawn
ros2 interface show turtlesim/srv/Spawn
```

Call the service:

```bash
ros2 service call /spawn turtlesim/srv/Spawn \
  "{x: 2.0, y: 2.0, theta: 0.0, name: second_turtle}"
```

The request fields and their types must match the service interface.

## 6. Inspect and invoke actions

List action servers and their types:

```bash
ros2 action list -t
```

Inspect an action:

```bash
ros2 action info /turtle1/rotate_absolute
ros2 interface show turtlesim/action/RotateAbsolute
```

Send a goal and display feedback:

```bash
ros2 action send_goal /turtle1/rotate_absolute \
  turtlesim/action/RotateAbsolute "{theta: 1.57}" --feedback
```

Actions are appropriate for operations that take time and may need feedback or
cancellation.

## 7. Inspect and change parameters

List parameters:

```bash
ros2 param list
```

Read and describe a parameter:

```bash
ros2 param get /turtlesim background_r
ros2 param describe /turtlesim background_r
```

Set a parameter:

```bash
ros2 param set /turtlesim background_r 100
```

Save all parameters from a node to a YAML file:

```bash
ros2 param dump /turtlesim
```

## 8. Remap names

ROS arguments follow `--ros-args`. Rename a node at runtime:

```bash
ros2 run turtlesim turtlesim_node --ros-args \
  --remap __node:=workshop_turtlesim
```

Remap a topic for both Turtlesim nodes:

```bash
ros2 run turtlesim turtlesim_node --ros-args \
  --remap /turtle1/cmd_vel:=/robot/cmd_vel
```

In another terminal:

```bash
ros2 run turtlesim turtle_teleop_key --ros-args \
  --remap /turtle1/cmd_vel:=/robot/cmd_vel
```

The nodes still communicate because the original topic is mapped to the same
new name in both processes.

## 9. Practice

Start Turtlesim and use only the CLI to:

1. Find the names of all running nodes.
2. Determine the type of `/turtle1/pose`.
3. Measure the publication rate of `/turtle1/pose`.
4. Find the request fields of the `/spawn` service.
5. Spawn a second turtle.
6. Find the goal, result, and feedback fields of the rotate action.
7. Change one background-color parameter.

Use `--help`, `ros2 interface show`, and tab completion when you are
uncertain about a command.
