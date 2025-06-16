# MoveIt 2 Jazzy: Python Move Group Tutorial

This tutorial walks you through creating a Python-based MoveIt 2 interface package using ROS 2 Jazzy. You'll learn how to:

1. Create a Python ROS 2 package.
2. Write a script that moves a robot to a specific pose.
3. Configure the package for MoveIt.
4. Run the motion planning script.

---

## Step 1: Create a Python Package for Move Group

Open a terminal in your ROS 2 workspace `src` directory:

```sh
cd ~/ws_moveit2/src
ros2 pkg create --build-type ament_python moveit_python_demo --dependencies rclpy moveit_commander geometry_msgs
```

This will generate the basic Python package structure.

### Update `setup.py`

Edit the generated `setup.py` file and update the `entry_points` block:

```python
entry_points={
    'console_scripts': [
        'move_to_pose = moveit_python_demo.move_to_pose:main',
    ],
},
```

---

## Step 2: Create Script to Move Robot to a Pose

Create a file called `move_to_pose.py` inside the `moveit_python_demo/moveit_python_demo/` directory:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from geometry_msgs.msg import PoseStamped

class MoveToPoseDemo(Node):
    def __init__(self):
        super().__init__('move_to_pose_demo')
        rclpy.spin_once(self, timeout_sec=1.0)  # ensure time is updated

        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander('arm')

        self.group.set_pose_reference_frame('world')
        self.group.set_max_velocity_scaling_factor(0.5)
        self.group.set_max_acceleration_scaling_factor(0.5)

        self.move_to_ready_pose()

    def move_to_ready_pose(self):
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'world'
        target_pose.pose.position.x = 0.4
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.4
        target_pose.pose.orientation.w = 1.0

        self.group.set_pose_target(target_pose)
        success = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        if success:
            self.get_logger().info('Motion executed successfully!')
        else:
            self.get_logger().error('Motion planning failed!')


def main():
    rclpy.init()
    node = MoveToPoseDemo()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Make it executable:

```sh
chmod +x moveit_python_demo/moveit_python_demo/move_to_pose.py
```

---

## Step 3: Configure the Package

### Add `install_requires` to `setup.py`

Make sure your `setup.py` includes:

```python
install_requires=['setuptools'],
```

### Add an `__init__.py`

Create an empty `__init__.py` file:

```sh
touch moveit_python_demo/moveit_python_demo/__init__.py
```

### Update `package.xml`

Make sure your `package.xml` includes the correct dependencies:

```xml
<exec_depend>moveit_commander</exec_depend>
<exec_depend>rclpy</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
```

---

## Step 4: Build and Run the Package

Build your workspace:

```sh
cd ~/ws_moveit2
colcon build --packages-select moveit_python_demo
source install/setup.bash
```

Launch the robot simulation (if not already running):

```sh
ros2 launch panda_moveit_config demo.launch.py
```

Run your Python move group script:

```sh
ros2 run moveit_python_demo move_to_pose
```

You should see your robot arm move to the specified pose in RViz.

---

## Summary

You have now:

* Created a Python ROS 2 package.
* Implemented a motion planning script using MoveIt Commander.
* Configured and launched a robot demo.
* Executed a pose target command.

Feel free to expand the script to include waypoints, cartesian motions, or object collision handling!
