# MoveIt Architecture

MoveIt connects a robot model, the current state of the robot, motion-planning
algorithms, collision checking, and robot controllers. The central component is
the `move_group` node. Applications normally send requests to `move_group`
instead of calling planners or controllers directly.

```text
Application, RViz or MoveItPy
              |
              v
          move_group
       /       |       \
Robot model  Planning   Planning pipeline
URDF + SRDF  scene      and planner plugins
       \       |       /
        Trajectory execution
                 |
                 v
        ros2_control controller
                 |
                 v
               Robot
```

## Main components

### Robot model

The **URDF** describes the links, joints, geometry, and physical structure of
the robot. The **SRDF** adds information used for planning, including planning
groups, end effectors, named poses, virtual joints, and allowed collisions.

### Planning Scene Monitor

The Planning Scene Monitor combines the robot's current joint state with its
surroundings. It updates the planning scene from joint states, TF transforms,
attached objects, collision objects, and optional 3D sensors. Motion plans are
checked against this scene.

### Planning pipeline

The planning pipeline sends a request to a planner plugin, such as OMPL or the
Pilz Industrial Motion Planner. Request adapters can validate or modify the
request and process the resulting trajectory.

### Trajectory execution

After planning, MoveIt sends the trajectory to a compatible controller. For a
typical arm, MoveIt uses a `FollowJointTrajectory` action provided by a
`joint_trajectory_controller`. A gripper commonly uses a gripper action
controller.

### The `moveit_config` package

A robot-specific `moveit_config` package connects these components. It usually
contains the SRDF, kinematics settings, joint limits, planner settings,
controller mappings, and launch files. The MoveIt Setup Assistant generates a
starting package, which can then be adjusted for the robot and application.

## Official documentation

- [MoveIt concepts](https://moveit.picknik.ai/main/doc/concepts/concepts.html)
- [`move_group` architecture](https://moveit.picknik.ai/main/doc/concepts/move_group.html)
- [Planning Scene Monitor](https://moveit.picknik.ai/main/doc/concepts/planning_scene_monitor.html)
- [Motion planning](https://moveit.picknik.ai/main/doc/concepts/motion_planning.html)
- [MoveIt configuration](https://moveit.picknik.ai/main/doc/how_to_guides/moveit_configuration/moveit_configuration_tutorial.html)
- [Controller configuration](https://moveit.picknik.ai/main/doc/examples/controller_configuration/controller_configuration_tutorial.html)

