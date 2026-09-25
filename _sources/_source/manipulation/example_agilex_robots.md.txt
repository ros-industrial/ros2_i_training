# Example: AgileX robots

This example creates a MoveIt configuration for an AgileX Piper H manipulator
with its gripper. It applies the generic workflow from the previous tutorial
and plans from a tool center point at the center of the gripper.

## Prerequisites

This example assumes ROS 2 Humble, a workspace at `/home/ros/ros2_ws`, and the
forked [AgileX arm description packages](https://github.com/ipa-may/agx_arm_urdf)
in the workspace source directory.

Build and source the robot description:

```bash
source /opt/ros/humble/setup.bash
cd /home/ros/ros2_ws
colcon build --packages-select agx_arm_urdf
source install/setup.bash
ros2 pkg prefix agx_arm_urdf
```

The last command should print the installed package path.

## 1. Add the Piper H tool center point

Open:

```text
/home/ros/ros2_ws/src/agx_arm_urdf/piper_h/urdf/piper_h_with_gripper_description.xacro
```

Inside the top-level `<robot>` element, add an empty TCP link after the existing
`gripper_base_joint`:

```xml
<link name="gripper_tcp"/>

<joint name="gripper_tcp_joint" type="fixed">
  <origin xyz="0 0 0.138" rpy="0 0 0"/>
  <parent link="gripper_base"/>
  <child link="gripper_tcp"/>
</joint>
```

The fixed joint places the planning target at the nominal grasp center without
adding a degree of freedom. Verify the `0.138` metre offset against the gripper
CAD or physical robot.

Validate the modified description:

```bash
xacro \
  /home/ros/ros2_ws/src/agx_arm_urdf/piper_h/urdf/piper_h_with_gripper_description.xacro \
  > /tmp/piper_h_with_gripper.urdf

check_urdf /tmp/piper_h_with_gripper.urdf
```

The parsed link tree should contain `gripper_tcp` below `gripper_base`.

## 2. Configure the planning groups

Start the assistant and load the combined Piper H description:

```bash
ros2 run moveit_setup_assistant moveit_setup_assistant
```

```text
/home/ros/ros2_ws/src/agx_arm_urdf/piper_h/urdf/piper_h_with_gripper_description.xacro
```

Create the arm planning group with:

- **Group name:** `piper_h_with_gripper`
- **Type:** kinematic chain
- **Base link:** `base_link`
- **Tip link:** `gripper_tcp`
- **Kinematics solver:** `kdl_kinematics_plugin/KDLKinematicsPlugin`

Create the gripper group with:

- **Group name:** `piper_h_gripper`
- **Links:** `gripper_link`, `gripper_link1`, and `gripper_link2`

Create the end effector with:

- **Name:** `piper_h_gripper`
- **End-effector group:** `piper_h_gripper`
- **Parent group:** `piper_h_with_gripper`
- **Parent link:** `gripper_tcp`

The corresponding SRDF structure is:

```xml
<group name="piper_h_with_gripper">
  <chain base_link="base_link" tip_link="gripper_tcp"/>
</group>

<end_effector
  name="piper_h_gripper"
  parent_link="gripper_tcp"
  group="piper_h_gripper"
  parent_group="piper_h_with_gripper"/>
```

## 3. Configure kinematics and controllers

Keep KDL assigned to the arm group:

```yaml
piper_h_with_gripper:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```

Configure a trajectory controller for the arm and a gripper action controller
for `piper_h_gripper`. The MoveIt arm-controller entry must include the action
namespace:

```yaml
piper_h_with_gripper_controller:
  default: true
  action_ns: follow_joint_trajectory
  type: FollowJointTrajectory
```

Generate the package at:

```text
/home/ros/ros2_ws/src/piper_h_with_gripper_moveit_config
```

Check that numeric values in `joint_limits.yaml` are floating point. For
example, use `5.0` and `0.0`, not `5` and `0`.

## 4. Build and run

```bash
cd /home/ros/ros2_ws
colcon build --packages-select \
  agx_arm_urdf \
  piper_h_with_gripper_moveit_config
source install/setup.bash

ros2 launch piper_h_with_gripper_moveit_config demo.launch.py
```

In RViz, select `piper_h_with_gripper` as the planning group. The interactive
marker should appear at `gripper_tcp`. Select `piper_h_gripper` when testing the
gripper group; it does not have a six-dimensional IK marker.

If the marker is missing, verify the planning group in RViz, the TCP link in
the SRDF, and the KDL entry in `kinematics.yaml`. If execution fails, confirm
that `moveit_controllers.yaml` contains both `default: true` and
`action_ns: follow_joint_trajectory`.

