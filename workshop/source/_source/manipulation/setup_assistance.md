# MoveIt Setup Assistant

## Prepare

```bash
cd
mkdir -p ws_moveit2/src
cd ws_moveit2/src
git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
cd ..
rosdep install -y --from-paths src --iry
colcon build --symlink-install
source install/setup.bash
```

Now look for `src/Universal_Robots_ROS2_Description/urdf/ur.urdf.xacro`

**Warning**: This setup is using UR robots instead of the Panda robot. Therefore, please replace **panda** with **ur** in the following instructions.

![Setup Assistant Launch](setup_assistant_launch.png)

## Overview

The MoveIt Setup Assistant is a GUI tool for configuring a robot for MoveIt. It generates a Semantic Robot Description Format (SRDF) file and other necessary configuration files. A URDF is required to use the Setup Assistant.

Once you have a URDF, import it into the Setup Assistant to configure kinematics, planning groups, end effectors, and collision checking. 

## Getting Started

### MoveIt and ROS 2

* [Install MoveIt](/doc/tutorials/getting_started/getting_started) if you haven't already.
* Use the `moveit_resources_panda_description` package, included in your workspace after installation.

## Step 1: Start

```sh
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

Click **Create New MoveIt Configuration Package**.

![Create Package](setup_assistant_create_package.png)

Browse to and load:

```sh
~/ws_moveit2/src/moveit_resources/panda_description/urdf/panda.urdf
```

![Load URDF](setup_assistant_load_panda_urdf.png)

## Step 2: Generate Self-Collision Matrix

Set the sampling density and click **Generate Collision Matrix**.

![Collision Matrix](setup_assistant_panda_collision_matrix.png)

View and modify results:

![Collision Matrix Done](setup_assistant_panda_collision_matrix_done.png)

## Step 3: Add Virtual Joints

* Joint Name: `virtual_joint`
* Child Link: `panda_link0`
* Parent Frame: `world`
* Type: `fixed`

![Virtual Joints](setup_assistant_panda_virtual_joints.png)

> For mobile robots, use planar or floating joints.

## Step 4: Add Planning Groups

### Arm Group

* Name: `panda_arm`
* Solver: `kdl_kinematics_plugin/KDLKinematicsPlugin`

![Planning Groups](setup_assistant_panda_planning_groups.png)

Select joints from `virtual_joint` to `panda_joint8`:

![Arm Joints](setup_assistant_panda_arm_group_joints.png)

Save group:

![Arm Group Saved](setup_assistant_panda_arm_group_saved.png)

### End Effector Group

* Name: `hand`
* Solver: `None`
* Links: `panda_hand`, `panda_leftfinger`, `panda_rightfinger`

![Hand Group](setup_assistant_panda_hand_group_links.png)

Final group list:

![Planning Groups Done](setup_assistant_panda_planning_groups_done.png)

> Use **Add Subgroup** for compound groups (e.g., dual-arm systems).

## Step 5: Add Robot Poses

### Ready Pose for Arm

* Group: `panda_arm`
* Pose: `ready`
* Values: `{0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785}`

![Ready Pose](setup_assistant_panda_predefined_arm_pose.png)

### Open/Close Poses for Hand

* Group: `hand`
* `open`: `0.035`
* `close`: `0.0`

![Open Pose](setup_assistant_panda_predefined_hand_open_pose.png)

![Close Pose](setup_assistant_panda_predefined_hand_close_pose.png)

> Only `panda_finger_joint1` is shown as `panda_finger_joint2` mimics it.

Final poses:

![Predefined Poses Done](setup_assistant_panda_predefined_poses_done.png)

## Step 6: Label End Effectors

* Name: `hand`
* Group: `hand`
* Parent Link: `panda_link8`

![Add End Effector](setup_assistant_panda_add_end_effector.png)

## Step 7: Add Passive Joints

> Skip this step for Panda. Use if robot has unactuated joints.

## Step 8: ros2\_control URDF Modification

Define command/state interfaces (default: position command; position + velocity states).

![ros2\_control Tags](setup_assistant_ros2_control_tags.png)

## Step 9: ROS 2 Controllers

### Panda Arm Controller

* Name: `panda_arm_controller`
* Type: `joint_trajectory_controller/JointTrajectoryController`
* Group: `panda_arm`

![Arm Controller Type](setup_assistant_panda_arm_ros2_controller_type.png)

![Arm Controller Group](setup_assistant_panda_arm_ros2_controller_group.png)

### Hand Controller

* Name: `hand_controller`
* Type: `position_controllers/GripperActionController`
* Group: `hand`

![Hand Controller Type](setup_assistant_hand_ros2_controller_type.png)

![Hand Controller Group](setup_assistant_hand_ros2_controller_group.png)

Final list:

![Controllers Done](setup_assistant_ros2_controllers_done.png)

## Step 10: MoveIt Controllers

Configure MoveIt interfaces to match ROS 2 controllers.

### Arm

* Name: `panda_arm_controller`
* Type: `FollowJointTrajectory`
* Group: `panda_arm`

![MoveIt Arm Controller](setup_assistant_panda_arm_moveit_controller_type.png)

### Hand

* Name: `hand_controller`
* Type: `Gripper Command`
* Group: `hand`

![MoveIt Hand Controller](setup_assistant_hand_moveit_controller_type_gripper.png)

Final MoveIt controller list:

![MoveIt Controllers Done](setup_assistant_moveit_controllers_done_gripper.png)

## Step 11: Perception

Use this pane to define 3D sensors. Skip or choose **None** if not needed.

![Perception None](setup_assistant_panda_3d_perception.png)

Example (for robots with cameras):

![Point Cloud](setup_assistant_panda_3d_perception_point_cloud.png)

See [Perception Pipeline tutorial](/doc/examples/perception_pipeline/perception_pipeline_tutorial).

## Step 12: Launch Files

View generated launch files and their purposes.

![Launch Files](setup_assistant_launch_files.png)

## Step 13: Add Author Information

Enter your name and email in the **Author Information** pane.

## Step 14: Generate Configuration Files

Click **Configuration Files**, choose a location (e.g., `~/ws_moveit2/src/panda_moveit_config`), and click **Generate Package**.

![Done](setup_assistant_done.png)

## Build and Run the Demo

```sh
cd ~/ws_moveit2
colcon build --packages-select panda_moveit_config
source install/setup.bash

ros2 launch panda_moveit_config demo.launch.py
```

Watch the [YouTube demo](https://youtu.be/f__udZlnTdM).
