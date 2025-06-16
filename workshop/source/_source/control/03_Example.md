# Example 3: Robots with Multiple Interfaces

This example demonstrates how to implement a multi-interface robot hardware, taking care of the interfaces used.

For *example\_3*, the hardware interface plugin supports multiple command and state interfaces:

* Communication is done using a proprietary API to interact with the robot control box.
* Data for all joints is exchanged simultaneously.
* Examples: KUKA FRI, ABB Yumi, Schunk LWA4p, etc.

Two intentionally misconfigured controllers demonstrate how the hardware interface can reject invalid joint interface claims.

## Tutorial Steps

### 1. Launch RRBot Visualization

```sh
ros2 launch ros2_control_demo_example_3 view_robot.launch.py
```

> **Note**: Warnings like `Invalid frame ID "odom"` are expected while `joint_state_publisher_gui` initializes. It provides a GUI to generate random configurations displayed in RViz.

### 2. Start RRBot with Multi-Interface Support

```sh
ros2 launch ros2_control_demo_example_3 rrbot_system_multi_interface.launch.py
```

Useful options:

* `robot_controller:=forward_position_controller`: spawns a position controller.
* `robot_controller:=forward_acceleration_controller`: spawns an acceleration controller.

This launch file loads the hardware, controllers, and RViz. Console output will show the internal hardware state (for demonstration only).

If RViz shows two orange and one yellow rectangles, the system has started correctly.

### 3. Verify Hardware Interface

```sh
ros2 control list_hardware_interfaces
```

Expected output:

```sh
command interfaces
    joint1/acceleration [available] [unclaimed]
    joint1/position [available] [unclaimed]
    joint1/velocity [available] [claimed]
    joint2/acceleration [available] [unclaimed]
    joint2/position [available] [unclaimed]
    joint2/velocity [available] [claimed]
state interfaces
    joint1/acceleration
    joint1/position
    joint1/velocity
    joint2/acceleration
    joint2/position
    joint2/velocity
```

### 4. Check Active Controllers

```sh
ros2 control list_controllers
```

Example output:

```sh
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
forward_velocity_controller[velocity_controllers/JointGroupVelocityController] active
```

This output varies based on the `robot_controller` argument used.

### 5. Send Commands

#### a. Using CLI

* For `forward_position_controller`:

```sh
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.5
- 0.5"
```

* For `forward_velocity_controller` (default):

```sh
ros2 topic pub /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray "data:
- 5
- 5"
```

* For `forward_acceleration_controller`:

```sh
ros2 topic pub /forward_acceleration_controller/commands std_msgs/msg/Float64MultiArray "data:
- 10
- 10"
```

#### b. Using Demo Node (for position controller):

```sh
ros2 launch ros2_control_demo_example_3 test_forward_position_controller.launch.py
```

Terminal will log updates:

```sh
[INFO] Writing commands:
  command pos: 0.00, vel: 5.00, acc: 0.00 for joint 0
  command pos: 0.00, vel: 5.00, acc: 0.00 for joint 1
[INFO] Reading states:
  pos: 0.67, vel: 5.00, acc: 0.00 for joint 0
  pos: 0.67, vel: 5.00, acc: 0.00 for joint 1
```

### 6. Switch Controllers at Runtime

Load a new controller (if not already active):

```sh
ros2 control load_controller forward_position_controller $(ros2 pkg prefix ros2_control_demo_example_3 --share)/config/rrbot_multi_interface_forward_controllers.yaml
ros2 control set_controller_state forward_position_controller inactive
```

Then switch controllers:

```sh
ros2 control switch_controllers --deactivate forward_velocity_controller --activate forward_position_controller
```

Check updated interface claims:

```sh
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

Now send commands using the new controller.

### 7. Demonstrate Illegal Controllers

Launch with:

* `robot_controller:=forward_illegal1_controller` or
* `robot_controller:=forward_illegal2_controller`

Expected error output:

```sh
[ERROR] [resource_manager]: Component 'RRBotSystemMultiInterface' did not accept new command resource combination:
  Start interfaces:
    joint1/position
  Stop interfaces:
    (none)
[ERROR] [controller_manager]: Could not switch controllers since prepare command mode switch was rejected.
[ERROR] [spawner_forward_illegal1_controller]: Failed to activate controller
```

Run:

```sh
ros2 control list_hardware_interfaces
```

Output:

```sh
command interfaces
  joint1/acceleration [available] [unclaimed]
  joint1/position [available] [unclaimed]
  joint1/velocity [available] [unclaimed]
  joint2/acceleration [available] [unclaimed]
  joint2/position [available] [unclaimed]
  joint2/velocity [available] [unclaimed]
```

```sh
ros2 control list_controllers
```

Output:

```sh
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
forward_illegal1_controller[forward_command_controller/ForwardCommandController] inactive
```

## Files Used in This Demo

* **Launch file**: [rrbot\_system\_multi\_interface.launch.py](https://github.com/ros-controls/ros2_control_demos/tree/master/example_3/bringup/launch/rrbot_system_multi_interface.launch.py)
* **Controllers YAML**: [rrbot\_multi\_interface\_forward\_controllers.yaml](https://github.com/ros-controls/ros2_control_demos/tree/master/example_3/bringup/config/rrbot_multi_interface_forward_controllers.yaml)
* **URDF**:

  * [rrbot\_system\_multi\_interface.urdf.xacro](https://github.com/ros-controls/ros2_control_demos/tree/master/example_3/description/urdf/rrbot_system_multi_interface.urdf.xacro)
  * [rrbot\_description.urdf.xacro](https://github.com/ros-controls/ros2_control_demos/tree/master/ros2_control_demo_description/rrbot/urdf/rrbot_description.urdf.xacro)
  * [rrbot\_system\_multi\_interface.ros2\_control.xacro](https://github.com/ros-controls/ros2_control_demos/tree/master/example_3/description/ros2_control/rrbot_system_multi_interface.ros2_control.xacro)
* **RViz Config**: [rrbot.rviz](https://github.com/ros-controls/ros2_control_demos/tree/master/ros2_control_demo_description/rrbot/rviz/rrbot.rviz)
* **Hardware Interface Plugin**: [rrbot\_system\_multi\_interface.cpp](https://github.com/ros-controls/ros2_control_demos/blob/master/example_3/hardware/rrbot_system_multi_interface.cpp)

## Controllers in This Demo

* **Joint State Broadcaster** ([docs](https://github.com/ros-controls/ros2_controllers/tree/master/joint_state_broadcaster))
* **Forward Command Controller** ([docs](https://github.com/ros-controls/ros2_controllers/tree/master/forward_command_controller))
