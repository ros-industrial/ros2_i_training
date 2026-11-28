# Python MoveIt Tutorial (MoveItPy)

This tutorial shows how to plan and execute robot motions from Python with **MoveItPy**, on ROS 2
Jazzy. You will:

1. Start a robot with mock hardware and a `move_group`, in Docker.
2. Run the MoveItPy examples against a UR5e, a Franka FR3, a Motoman GP50 or an AgileX Piper.
3. Understand what a MoveItPy node needs (and why it differs from MoveGroupInterface).
4. Write and run your own MoveItPy script.

The examples are in the [`moveit_examples`](https://github.com/ROS-Industrial-Session-Common/moveit_examples)
repository: the same ten examples exist in C++ (`move_group_interface_examples`) and in Python
(`moveit_py_examples`), plus MoveIt Task Constructor examples (`mtc_examples`).

---

## Why not `moveit_commander`?

In ROS 1, Python programs used `moveit_commander`. It does **not** exist in ROS 2:

- `moveit_commander` was not a Python implementation, but a thin `boost::python` wrapper around the
  C++ `MoveGroupInterface`. It started a hidden roscpp node inside the Python process and passed
  messages across the language boundary as serialized byte strings.
- In ROS 2, Python (`rclpy`) and C++ (`rclcpp`) nodes do not share a process-wide node or executor,
  and `boost::python` was replaced by `pybind11`. The wrapper was never ported.
- MoveIt 2 provides **MoveItPy** instead: Python bindings of **MoveItCpp**, the C++ core of MoveIt.

The important difference:

| | MoveGroupInterface (C++) | MoveItPy (Python) |
| --- | --- | --- |
| Where MoveIt runs | in the `move_group` node; your node is a client | **inside your process** |
| Parameters your node needs | none (robot model from topics) | **all** MoveIt parameters of the robot + MoveItPy settings |
| Planning scene | messages to `move_group` | direct read/write access |

---

## Step 1: Start a robot

The setup is split like on a real cell: one container per robot (`ros2_control`, controllers,
`robot_state_publisher`, **no MoveIt**), one generic `move_group` container, and the example
clients. The URDF has a single source: the `/robot_description` topic of the robot bringup.

| Robot | Repository | Bringup service (mock hardware) | Descriptions image |
| --- | --- | --- | --- |
| `ur` | `simple_ur_cell` | `simple_ur_cell_mock` | `simple_ur_cell_descriptions:jazzy` |
| `fr3` | `dual_fr3_cell` | `dual_fr3_cell_mock` | `dual_fr3_cell_descriptions:jazzy` |
| `gp50` | `motoman_gp50` | `gp50_mock` (`--profile mock`) | `motoman_gp50_descriptions:jazzy` |
| `piper` | `agx_piper_h` | `agx_piper_h_mock` | `agx_piper_h_descriptions:jazzy` |

Example with the Piper; use the other rows of the table for the other robots.

```bash
# Terminal 1: the robot (mock hardware)
cd agx_piper_h/docker
docker compose --profile descriptions build agx_piper_h_descriptions   # once
docker compose up agx_piper_h_mock
```

```bash
# Terminal 2: move_group and RViz for this robot
cd move_group_server
xhost +local:docker
ROBOT_DESCRIPTIONS_IMAGE=agx_piper_h_descriptions:jazzy docker compose --profile rviz up
```

Wait for `You can start planning now!` in the log of `move_group`, then check RViz: the robot model
and the MotionPlanning panel appear.

```{note}
The descriptions image is a data-only image with the description packages, the MoveIt config
package and a `robot.env` file. `move_group` mounts it read-only to load the SRDF, kinematics,
joint limits, planning pipelines and controllers of the robot.
```

---

## Step 2: Run the MoveItPy examples

```bash
# Terminal 3: the examples
cd moveit_examples/docker
cp .env.example .env
# edit .env: ROBOT=piper and ROBOT_DESCRIPTIONS_IMAGE=agx_piper_h_descriptions:jazzy
docker compose --profile examples build py_01_named_goal   # once: all example services share this image
docker compose run --rm py_01_named_goal
```

Watch the robot in RViz. Then try the other examples:

| Service | What it shows |
| --- | --- |
| `py_01_named_goal` | SRDF named states (`home`, `UP`, …) |
| `py_02_joint_goal` | joint-space goal: no IK |
| `py_03_pose_goal` | pose of the tip link: the planner samples IK solutions |
| `py_04_planners` | OMPL, Pilz PTP, CHOMP, STOMP on the same goal, several pipelines in parallel, Pilz LIN |
| `py_05_inverse_kinematics` | `RobotState.set_from_ik`, then plan to the joint solution |
| `py_06_path_constraints` | keep the tool pointing in the same direction during the motion |
| `py_07_cartesian_path` | straight lines: Pilz LIN, and the `/compute_cartesian_path` service of `move_group` |
| `py_08_planning_scene` | add a box obstacle, plan around it, remove it |
| `py_09_attach_detach` | attach a part to the tool (and use the gripper), move, detach |
| `py_10_allowed_collision_matrix` | a part touching the tool blocks planning until the ACM allows the contacts |

The C++ versions are the `mgi_*` services: compare both implementations.

```{note}
MoveItPy has its own planning scene, in the example process. Objects added by a MoveItPy example
are therefore **not** shown in the RViz MotionPlanning panel, which displays the scene of
`move_group`. The C++ examples (`mgi_*`) edit the scene of `move_group`: their objects appear in RViz.
```

---

## Step 3: What a MoveItPy node needs

Open `moveit_py_examples/moveit_py_examples/launch_utils.py`. The launch file of every example
builds the parameters of the node:

```python
config = (
    MoveItConfigsBuilder(robot_name, package_name=package)      # e.g. agx_piper_h_moveit_config
    .moveit_cpp(file_path=str(share / "config" / "moveit_py.yaml"))
    .to_moveit_configs()
)
parameters = config.to_dict()
parameters.pop("robot_description", None)  # from the topic of the bringup

Node(
    package="moveit_py_examples",
    executable=executable,
    parameters=[parameters, str(share / "config" / "robots" / f"{robot}.yaml")],
)
```

- `config.to_dict()`: SRDF, kinematics, joint limits, planning pipelines and controllers of the
  robot, read from its MoveIt config package. MoveItPy **is** MoveIt: it needs all of them.
- `robot_description` is removed: MoveItPy reads the URDF from the `/robot_description` topic of
  the robot bringup, like `move_group`.
- `config/moveit_py.yaml`: the MoveItPy settings, robot independent:
  - `planning_scene_monitor_options`: topics of the planning scene monitor
  - `planning_pipelines.pipeline_names`: the pipelines to load (`ompl`, `pilz_industrial_motion_planner`, `chomp`, `stomp`)
  - `plan_request_params`: the default plan request (pipeline, planning time, scaling)
  - named plan requests (`ompl_rrtc`, `pilz_ptp`, `pilz_lin`, …), used with `PlanRequestParameters(robot, "pilz_lin")`
- `config/robots/<robot>.yaml`: the robot profile of the examples (planning group, tip link, named
  states, test poses). The code contains no robot names.

---

## Step 4: Read the code

### Named goal and pose goal

`moveit_py_examples/moveit_py_examples/named_goal.py` and `pose_goal.py`:

```python
import rclpy
from moveit.planning import MoveItPy

rclpy.init()
robot = MoveItPy(node_name="moveit_py")                    # loads the model, starts the scene monitor
arm = robot.get_planning_component(profile.planning_group)  # e.g. "arm" for the Piper

arm.set_start_state_to_current_state()
arm.set_goal_state(configuration_name="home")              # SRDF named state
result = arm.plan()
if result:
    robot.execute(result.trajectory, controllers=[])       # [] : MoveIt chooses the controllers

arm.set_start_state_to_current_state()
arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=profile.tip_link)
result = arm.plan()
```

- `MoveItPy` is the robot: model, planning scene monitor, trajectory execution.
- `PlanningComponent` is one planning group.
- Goals: `configuration_name`, `robot_state`, `pose_stamped_msg` + `pose_link`, or
  `motion_plan_constraints`.

### Choosing a planner

`planners.py`:

```python
from moveit.planning import MultiPipelinePlanRequestParameters, PlanRequestParameters

arm.plan(single_plan_parameters=PlanRequestParameters(robot, "pilz_ptp"))

# several pipelines in parallel: the best solution is returned
multi = MultiPipelinePlanRequestParameters(robot, ["ompl_rrtc", "pilz_ptp", "stomp_planner"])
arm.plan(multi_plan_parameters=multi)
```

### Planning scene

`planning_scene.py`: MoveItPy edits its planning scene directly.

```python
with robot.get_planning_scene_monitor().read_write() as scene:
    scene.apply_collision_object(box)    # moveit_msgs/CollisionObject
    scene.current_state.update()
```

### Ending the program

All examples end with `example_utils.shutdown()`:

```python
def shutdown():
    rclpy.shutdown()
    os._exit(0)
```

On Jazzy, MoveItPy crashes (segmentation fault) when its objects are destroyed at exit
([moveit/moveit2#3721](https://github.com/moveit/moveit2/issues/3721)). Exiting without running the
destructors avoids it.

---

## Step 5: Write your own script

Create a package next to `moveit_py_examples` (in `moveit_examples/`):

```bash
ros2 pkg create --build-type ament_python my_moveit_py \
  --dependencies rclpy moveit_py moveit_py_examples geometry_msgs
```

`my_moveit_py/my_moveit_py/square.py`: move the tool along the four sides of a square with Pilz LIN.

```python
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.logging import get_logger

from moveit_py_examples.example_utils import load_profile, move_to_named_state, plan_and_execute, shutdown


def main():
    rclpy.init()
    logger = get_logger("square")
    profile = load_profile()                  # config/robots/<robot>.yaml
    robot = MoveItPy(node_name="moveit_py")
    arm = robot.get_planning_component(profile.planning_group)

    move_to_named_state(robot, arm, logger, profile.named_states[0])
    with robot.get_planning_scene_monitor().read_only() as scene:
        start = scene.current_state.get_pose(profile.tip_link)    # in the model frame

    lin = PlanRequestParameters(robot, "pilz_lin")
    side = 0.05                                                    # m
    for dx, dy in [(side, 0.0), (side, side), (0.0, side), (0.0, 0.0)]:
        goal = PoseStamped()
        goal.header.frame_id = robot.get_robot_model().model_frame
        goal.pose.position.x = start.position.x + dx
        goal.pose.position.y = start.position.y + dy
        goal.pose.position.z = start.position.z
        goal.pose.orientation = start.orientation
        arm.set_start_state_to_current_state()
        arm.set_goal_state(pose_stamped_msg=goal, pose_link=profile.tip_link)
        plan_and_execute(robot, arm, logger, f"corner ({dx}, {dy})", single_plan_parameters=lin)

    shutdown()


if __name__ == "__main__":
    main()
```

Register the script in `setup.py`:

```python
entry_points={
    "console_scripts": [
        "square = my_moveit_py.square:main",
    ],
},
```

and add a launch file `my_moveit_py/launch/square.launch.py` that reuses the launch description of
the examples (MoveIt parameters of the robot + robot profile):

```python
from moveit_py_examples.launch_utils import example_launch


def generate_launch_description():
    return example_launch("square", package="my_moveit_py")
```

Install the launch directory with the package (`setup.py`, `data_files`):

```python
("share/" + package_name + "/launch", ["launch/square.launch.py"]),
```

Build and run it, natively (with `ros-jazzy-moveit-py` and the robot packages in the workspace):

```bash
colcon build --packages-up-to my_moveit_py
source install/setup.bash
ros2 launch my_moveit_py square.launch.py robot:=piper
```

or in the examples container, with your package mounted next to the examples of the image:

```bash
cd moveit_examples/docker
docker compose run --rm -v "$(pwd)/../my_moveit_py:/ros2_ws/src/my_moveit_py" py_01_named_goal \
  bash -c "colcon build --packages-select my_moveit_py && source install/setup.bash \
           && ros2 launch my_moveit_py square.launch.py robot:=piper"
```

---

## Exercises

1. Run `py_04_planners` on two different robots. Which planner gives the shortest trajectory? Run it
   twice: which planners give the same result each time?
2. In your `square.py`, use the default plan request (OMPL) instead of `pilz_lin`. What changes?
3. Add a box obstacle in the middle of the square (see `planning_scene.py`) and plan the square with
   OMPL, then with Pilz LIN. Why does Pilz fail?
4. Use `MultiPipelinePlanRequestParameters` with `["ompl_rrtc", "pilz_ptp"]` for the moves between
   the corners. Which pipeline wins?
5. Compare `py_07_cartesian_path` with `mgi_07_cartesian_path`: which MoveIt interface does each one use?

---

## Troubleshooting

| Symptom | Cause and fix |
| --- | --- |
| `Could not find parameter robot_description and did not receive robot_description via std_msgs::msg::String subscription` | The robot bringup is not running, or another `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` |
| `Planning plugin name is empty or not defined in namespace 'chomp'` | A named plan request in `moveit_py.yaml` uses the name of a pipeline (`chomp`, `ompl`, …): rename it |
| `CheckStartStateCollision failed, because '… contact(s) detected'` | The robot starts in collision: objects in the scene, or SRDF collision pairs missing for links of the URDF |
| `Joint acceleration limit of … violated` (Pilz) | Pilz checks the joint limits along the line: lower the scaling factors of the plan request |
| `Execution failed! No active controllers configured for group ''` | A trajectory without planning group: set `trajectory.joint_model_group_name` |
| Segmentation fault at the end of the program | MoveItPy teardown bug on Jazzy: end with `os._exit(0)` after `rclpy.shutdown()` (see Step 4) |

---

## Summary

- `moveit_commander` does not exist in ROS 2; MoveItPy runs MoveIt **inside** your Python process.
- A MoveItPy node needs the MoveIt parameters of the robot (`MoveItConfigsBuilder`) and the MoveItPy
  settings (`moveit_py.yaml`); the URDF can come from the topic of the robot bringup.
- `PlanningComponent`: set the start state and the goal, plan, execute.
- Plan request parameters select the pipeline and the planner; several pipelines can plan in parallel.
- The planning scene of MoveItPy is local to your process.
