# Build moveit config of any manipulator

This tutorial shows the general workflow for creating a MoveIt configuration
for a manipulator with a gripper. Start with a working URDF or Xacro model in
which the arm and gripper form one valid robot tree.

The generated package is a starting point. Its joint limits, controllers,
kinematics, and launch files must still match the real robot.

## Dependencies

This workshop targets ROS 2 Humble and uses the following packages:

| Package | Why it is needed |
| --- | --- |
| `python3-colcon-common-extensions` | Builds the ROS 2 workspace. |
| `python3-rosdep` | Installs dependencies declared by ROS packages. |
| `python3-vcstool` | Imports source repositories from `.repos` files. |
| `ros-${ROS_DISTRO}-joint-state-publisher-gui` | Publishes test joint values while checking the robot description. |
| `ros-${ROS_DISTRO}-moveit` | Provides planning, collision checking, trajectory execution, and RViz integration. |
| `ros-${ROS_DISTRO}-moveit-setup-assistant` | Generates the robot-specific MoveIt configuration package. |
| `ros-${ROS_DISTRO}-robot-state-publisher` | Publishes the robot TF tree from its description and joint states. |
| `ros-${ROS_DISTRO}-ros2-control` | Provides the controller and hardware-interface framework. |
| `ros-${ROS_DISTRO}-ros2-controllers` | Provides standard arm and gripper controllers. |
| `ros-${ROS_DISTRO}-xacro` | Expands Xacro descriptions into URDF. |

Install the dependencies declared by the packages in your workspace:

```bash
source /opt/ros/humble/setup.bash
cd /home/ros/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Robot-description sources include:

- [Universal Robots ROS 2 Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
- [Yaskawa Motoman ROS 2 support packages](https://github.com/Yaskawa-Global/motoman_ros2_support_packages)
- [AgileX arm descriptions](https://github.com/ipa-may/agx_arm_urdf)
- [Schunk gripper descriptions](https://github.com/ipa-may/schunk_gripper_description)

## 1. Prepare the robot description

Before using MoveIt Setup Assistant, check that:

- the manipulator and gripper are connected in one URDF tree;
- every actuated joint has the correct type, axis, and limits;
- visual and collision geometry use the correct scale;
- mimic joints are defined for mechanically coupled gripper fingers; and
- the base link and intended tool link are known.

Expand and validate the Xacro before continuing:

```bash
xacro /path/to/robot_with_gripper.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

Resolve Xacro and URDF errors here. The Setup Assistant cannot repair a broken
robot description.

## 2. Define a tool center point

MoveIt plans poses for a link. If the required tool center point (TCP) is not
already represented, add an empty link and attach it to a stable tool or
gripper link with a fixed joint:

```xml
<link name="tool_tcp"/>

<joint name="tool_tcp_joint" type="fixed">
  <origin xyz="0 0 0.10" rpy="0 0 0"/>
  <parent link="gripper_base"/>
  <child link="tool_tcp"/>
</joint>
```

Replace the example offset with a value measured from CAD or the physical
tool. Do not attach the TCP to a moving finger.

## 3. Start MoveIt Setup Assistant

```bash
ros2 run moveit_setup_assistant moveit_setup_assistant
```

Select **Create New MoveIt Configuration Package** and load the combined robot
and gripper URDF or Xacro.

## 4. Configure collision checking

Generate the self-collision matrix. It disables checks for adjacent links and
for pairs that never collide in the sampled configurations. Review the result
instead of disabling additional collisions without verification.

Add a virtual joint only when the robot needs one. A fixed industrial arm whose
base is already fixed in the description usually does not require it. Mobile
bases commonly use a planar or floating virtual joint.

## 5. Create planning groups

Create an arm group as a kinematic chain:

- **Base link:** the manipulator base
- **Tip link:** the TCP or tool link
- **Kinematics solver:** for example,
  `kdl_kinematics_plugin/KDLKinematicsPlugin`

Create a separate gripper group containing its actuated joints or links. The
gripper group normally has no kinematics solver.

## 6. Configure the end effector and poses

Add an end effector with:

- the gripper planning group as its **End Effector Group**;
- the arm group as its **Parent Group**; and
- the TCP or gripper mounting link as its **Parent Link**.

Add useful named poses such as `home`, `ready`, `open`, and `closed`. Confirm
that each pose belongs to the correct planning group and respects joint limits.

## 7. Configure ros2_control and MoveIt controllers

For the arm, add the required command and state interfaces and create a
`joint_trajectory_controller/JointTrajectoryController`. Map it to a MoveIt
controller using `FollowJointTrajectory`.

A corresponding entry in `moveit_controllers.yaml` typically contains:

```yaml
arm_controller:
  default: true
  action_ns: follow_joint_trajectory
  type: FollowJointTrajectory
```

Configure the gripper with a controller matching its hardware interface. A
common choice is `position_controllers/GripperActionController`, exposed to
MoveIt as a gripper command controller.

Skip perception when no 3D sensor is available. It can be added later without
regenerating the whole package.

## 8. Generate the configuration package

Enter the author information, select the required launch files, and generate a
package under the workspace source directory, for example:

```text
/home/ros/ros2_ws/src/my_robot_moveit_config
```

Important generated files include:

| File | Purpose |
| --- | --- |
| `config/*.srdf` | Planning groups, end effectors, named poses, and disabled collisions. |
| `config/kinematics.yaml` | IK solver settings for each planning group. |
| `config/joint_limits.yaml` | MoveIt velocity, acceleration, and position-limit overrides. |
| `config/moveit_controllers.yaml` | Maps MoveIt groups to controller actions. |
| `config/ros2_controllers.yaml` | Configures the ros2_control controllers used by the robot. |
| `config/initial_positions.yaml` | Initial joint values used by fake hardware or demos. |

On ROS 2 Humble, use floating-point joint-limit values such as `5.0` and `0.0`
rather than integers such as `5` and `0`.

## 9. Build and test

```bash
cd /home/ros/ros2_ws
colcon build --packages-select my_robot_moveit_config
source install/setup.bash
ros2 launch my_robot_moveit_config demo.launch.py
```

In RViz, select the arm planning group. Confirm that the interactive marker is
located at the TCP, then test **Plan** before using **Plan & Execute**.

If the package cannot be found, source the workspace again. If no interactive
marker appears, check the selected planning group, the chain tip, and
`kinematics.yaml`. If execution fails, verify that the controller name, joints,
action type, and `action_ns` agree with the running ros2_control controller.

For details, see the official [MoveIt Setup Assistant tutorial](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)
and [MoveIt configuration guide](https://moveit.picknik.ai/main/doc/how_to_guides/moveit_configuration/moveit_configuration_tutorial.html).

