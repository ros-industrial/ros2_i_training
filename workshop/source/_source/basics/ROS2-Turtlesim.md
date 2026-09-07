# Understanding ROS 2 with Turtlesim

## 1. Introduction

Turtlesim is a teaching application for ROS and ROS 2. It demonstrates the
basic communication concepts with a small simulated turtle.

This workshop encourages you to refer to the cheat sheet for the syntax and type the commands on your own in order to learn by trial and error. Make use of the `--help` option for commands as well. However, solutions are also provided in the end. Feel free to approach this any way as you wish.

### Requirements

Install Turtlesim if it is not already available:

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-turtlesim
```

Make sure ROS 2 is sourced in every terminal you open.

## 2. Starting the Turtle simulator

You can start the main application by simply executing two of its nodes. Refer to the cheat sheet for the syntax to execute a node.

The package name you need in this case is `turtlesim` and the nodes you need to start are `turtlesim_node` and `turtle_teleop_key`. Make sure to source ROS 2 and run these nodes in two separate terminals.

> Start the 2 nodes of the application

Once you successfully start these nodes, a Qt window should open with a blue
background and a turtle in the middle. This turtle will help us understand ROS
2. Treat it as an automated guided vehicle observed from above.

![Turtlesim coordinate system showing x, y, and theta](../../_static/turtlesim_xytheta.png)

Before moving the turtle, you might find it useful to right click on the title bar of the simulator screen and select *Always on top* (undo this when you no longer need it).

> Keep the simulator window on top

Next, make sure your currently active window is the terminal where you started the teleop node by clicking once on it.

> Activate the teleop terminal

Now, as the instructions on the terminal say, you can move the turtle around by pressing the arrow keys on the keyboard, or use the other keys listed to set absolute orientations.

> Move the turtle with the arrow keys on the keyboard

Once you are satisfied with playing with the turtle, you can go ahead to the next step.

## 3. Observing

You can start introspecting at this point and already see many interesting things. Please spend a few minutes trying to list the nodes, parameters, topics, services and actions. Refer to the cheat sheet for the CLI syntax for the list/info/show/echo commands.

> List all available entities

Try to get information about each of them as well as the associated types (msg and srv).

> Get more information on these entities

You can find out which of the topics are publishers, and listen in on what these topics are currently publishing.

> Echo the topics

Perform all of the above introspection activities using RQT wherever applicable. Refer to the cheat sheet to see what plugins are available, or simply explore the RQT `Plugins` menu!

> Use RQT for introspection

**NOTE**

You might notice some extra topics with the word `action` in the name when you use RQT Node Graph, which you do not see when you list the topics in CLI. You can safely ignore these for now, as they will be addressed in a later lecture.

### 3.1 Description of topics
The topic `/turtle1/color_sensor` tells you the RGB values of the color of the trail left behind when the turtle moves.
The topic `/turtle1/cmd_vel` is used to instruct the turtle to move. Echo this topic in a separate terminal and then use the teleop node to move the turtle to observe how velocity commands are given to it.
The topic `/turtle1/pose` publishes the current pose of the turtle. Echo this topic to see how the pose of the turtle changes as you move it.

Inspect the velocity message:

```bash
ros2 topic type /turtle1/cmd_vel
ros2 interface show geometry_msgs/msg/Twist
```

### 3.2 Description of services

The services `/kill` and `/spawn` are used to kill and spawn turtles respectively.
The service `/clear` clears the background of trail lines and `/reset` resets the position of the turtle.
The service `/turtle1/set_pen` sets the color and thickness of the trail line.
The services `/turtle1/teleport_xx` move the turtle instantly.

You can ignore the services with the word `parameter` in the names for now.

## 4. Interacting

In this step you will practice interacting with the topics, services, actions and parameters provided by the turtlesim.

### 4.1 Services
Call the service `/turtle1/set_pen` from the CLI first by referring to the cheat sheet for the syntax. For selecting the right type, use double tab after typing in the name of the service to retrieve a list of options for the type and use the most appropriate. (Hint: Service types DO NOT have -- in their names, and have their names in/hierarchical/format).

In order to find out the right dictionary format to use, inspect the service type from another terminal (Hint: `ros2 interface show` and `ros2 interface proto`). The key values take an integer in the range 0-255. Once you call the service correctly, move the turtle with teleop to observe the change in trail color.

> Set the pen color of the turtle, and visually verify the change

 You can echo `/turtle1/color_sensor` to verify if this is the same RGB value that you set.

> Check the topic to cross verify

You can try to call the same service from RQT as well. Refer to the cheat sheet to select the right plugin.

> Use RQT to perform the same service call

You can similarly call the other services as well and observe what changes they make.

> Invoke all the other services as well using CLI and/or RQT.

Inspect and call the empty `/clear` service:

```bash
ros2 service type /clear
ros2 interface show std_srvs/srv/Empty
ros2 service call /clear std_srvs/srv/Empty "{}"
```

Inspect and call `/spawn`:

```bash
ros2 service type /spawn
ros2 interface show turtlesim/srv/Spawn
ros2 service call /spawn turtlesim/srv/Spawn \
  "{x: 1.0, y: 5.0, theta: 0.0, name: second_turtle}"
```

Remove the new turtle:

```bash
ros2 service call /kill turtlesim/srv/Kill \
  "{name: second_turtle}"
```

### 4.2 Parameters

The parameter commands are fairly straightforward. Try to list the params available and see what values they have. Change the background color of the sim by setting one or more of the relevant params. The allowed range of values for this is 0 - 255.

> Change background color of the same by setting a parameter

```bash
ros2 param list
ros2 param get /turtlesim background_r
ros2 param set /turtlesim background_r 125
```

The parameter services can also be inspected directly:

```bash
ros2 service type /turtlesim/list_parameters
ros2 interface show rcl_interfaces/srv/ListParameters
ros2 service call /turtlesim/list_parameters \
  rcl_interfaces/srv/ListParameters "{prefixes: [], depth: 0}"
```

### 4.3 Actions
Next, you can invoke the action provided by this sim. List the actions available, find out the interface type, and then invoke the action with meaningful values.

In this case, the action server can be invoked by a command line client using the `ros2 action send_goal <goal>` command. The syntax is similar to calling a service. The input parameter is *theta* which is an angle measured in radians, i.e. The valid range is -3.14 < *theta* < 3.14.

Using the `--feedback` option with the command prints the feedback to the console. The result of the action is always displayed.

> Invoke an action call from the terminal

```bash
ros2 action send_goal /turtle1/rotate_absolute \
  turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback
```

### 4.4 Topics
Finally, you can publish a velocity command on `/turtle1/cmd_vel` from CLI and RQT in a similar manner as calling a service, but of course, using the right commands for topics/messages instead. Make sure the teleop node is shut down before attempting this.

   (Hint: The message type is composite in this case, using *Vector3*. Each
   *Vector3* has three fields: `x`, `y`, and `z`. Use nested YAML mappings
   such as `linear: {x: 0.5}`. The solution is provided at the end.)

> Control the turtle from a terminal publisher

```bash
ros2 topic pub --rate 2 /turtle1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.5}}"
```

## 5. Using ros2bags

In a separate terminal, start recording the `/turtle1/cmd_vel` topic:

```bash
mkdir -p ~/dev_ws/bags
cd ~/dev_ws/bags
ros2 bag record /turtle1/cmd_vel -o turtle_commands
```

Back in the keyboard teleoperation terminal, give some velocity commands to
move the turtle. Return to the rosbag terminal and press `Ctrl+C` to stop
recording.

Stop the keyboard teleoperation node before replaying the bag:

```bash
ros2 bag play ~/dev_ws/bags/turtle_commands
```

The turtle should repeat the recorded motion commands.

## 6. Advanced - Remapping and other options
Every ROS 2 command and sub-command has a list of options that you can use to modify its behavior. The list of options can be seen with `-h` and included as desired. You can experiment with these as well and see how the commands you have already executed so far change.

For example, when you publish a velocity command from CLI, it publishes this message continuously and the turtle keeps moving until you kill the publisher. You could instead give a `-r 0.5` option to make it publish at 0.5 Hz. meaning there is a slight pause before every movement.

You could try adding the same rate option command to the spawn service (make sure to remove the name field as this would otherwise cause a name clash error). You will observe that turtles keep spawning until you kill the service caller.

## 7. Solutions

### 7.1 Starting the Turtle simulator
`ros2 run turtlesim turtlesim_node`
`ros2 run turtlesim turtle_teleop_key`


### 7.2 Observing
`ros2 node list`
`ros2 topic list -t`
`ros2 topic info /turtle1/cmd_vel`
`ros2 interface show turtlesim/msg/Pose`
`ros2 service list`
`ros2 interface show turtlesim/srv/Spawn`
`ros2 interface proto turtlesim/srv/Spawn`

### 7.3 Interacting
`ros2 service call /spawn turtlesim/srv/Spawn "{x: 5,y: 5,theta: 0}"`
`ros2 service call /reset std_srvs/srv/Empty`
```
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5"
```
```
ros2 topic pub -r 0.5 /turtle1/cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5"
```
`ros2 service call -r 0.5 /spawn turtlesim/srv/Spawn "{x: 5,y: 5,theta: 0}"`

`ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "r: 100
g: 0
b: 0
width: 0
'off': 0"`

`ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute {'theta: -1.57'} --feedback`

`ros2 param set  /turtlesim background_r 125`

### 7.4 Using ros2bags

`ros2 bag record /turtle1/cmd_vel -o turtle_commands`
