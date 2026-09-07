
# Simple Publisher - Subscriber pair
**Based on the [ROS 2 Tutorials](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html)**

## Introduction

As we understood from the lectures, nodes are the fundamental units in ROS2 which are usually written to perform a specific task. They can be created in a few different ways, some of them are:

1. As simple in-line code in a script,
2. As local functions, and
3. As class objects


We will be using the 3rd method, though it is the more complex, so as to get better used to this concept.

We start by writing two separate simple nodes, one that includes only publisher and another that includes only a subscriber. Finally, we will write a third node that includes both within the same program and are managed through an executor.

The first step is to create a python package to house all our nodes. You can do so using the command
```bash
ros2 pkg create --build-type ament_python <package_name>
```

***Note: Ensure first that ROS2 is sourced in every new terminal***

- Make sure you run this command in the *src* directory of your workspace. You can use any package name you want, but for reference in this document, we call it `wshop_nodes`.


## 1. Publisher Node

The publisher and subscriber nodes used here are in fact the [example code](https://github.com/ros2/examples/blob/master/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py) that ROS 2 provides.

We first present the code completely, and then discuss the interesting parts:

```Python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

```

Inside the python package you created above, there should be another folder with the same name. Create a python file inside that folder and paste this code in. You can name the file anything you want, but for reference in this document we assign the name `minimal_publisher.py` to it.

### 1.1 Explanation

```Python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
```

`rclpy` is the *ROS 2 Client Library* that provides the API for invoking ROS 2 through Python.
`Node` is the main class which will be inherited here to instantiate our own node.
`std_msgs.msg` is the library for standard messages that includes the `String` message type which we use in this node. This has to be declared as a dependency in `package.xml`, which we do next.

```Python
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
```
As explained above, we create a subclass of type `MinimalPublisher` using the base class `Node`.
In the constructor `__init__()`, we pass the name of the node that we wish to assign to the constructer of the parent class using `super()`. The parent class `Node` takes care of actually assigning this string as a name.
`self.publisher_ = self.create_publisher(String, 'topic', 10)` This line actually creates a publisher, using the message type `String` that we imported, with the name `topic` that we choose and having a queue size of `10`. Queue size is the size of the output buffer. The commands used till now are typical when creating a subscriber. What follows next is only logic that is relevant to this node, and you may implement this in any way depending on your requirements.

```Python
timer_period = 0.5  # seconds
self.timer = self.create_timer(timer_period, self.timer_callback)
self.i = 0
```
This creates a timer that ticks every 0.5s (2Hz), and calls the function `timer_callback` at every tick.

```Python
def timer_callback(self):
    msg = String()
    msg.data = 'Hello World: %d' % self.i
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.data)
    self.i += 1
```
In the callback, we create an object `msg` of the type of the message we wish to publish, i.e `String`.
We then populate the message with information we wish to publish. Looking at the description of the msg type using `ros2 interface show std_msgs/msg/String`, we see that it has only one field, which is `string data`. So we add a string into this field. Depending on the type of message used, you can populate it with relevant data.
Once the data msg object is done, we simply publish it using the `publish()` method of the `publisher_` object.
We also display this same message on the console for our verification using the `get_logger().info()` method of our `Node` class object.
Publishing from within this timer callback ensures we have a consistent publishing rate of 2Hz. You could publish this in any way you want, using the proper message type and publish call.

```Python
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
```

In the main method, we first declare that this Python script uses the rclpy library by invoking `init()` and passing any command line arguments provided (in this case none).
We instantiate an object of the class we just created. Since the contructor already spawns the timer which publishes messages, no further action is needed to setup our node.
The `spin()` method ensures that all the items of work, such as callbacks, are continuously executed until a `shutdown()` is called. This is quintessential to ensure that your node actually does its job!
Finally, we destroy the node and manually call shutdown.

### 1.2 Add dependencies

In the base root folder of this package, you will find the `package.xml` which is important for declaring all dependancies of the package. We will now edit this file to ensure our code runs properly.

The `description`, `maintainer` and `license` tage should be appropriately filled out. For license, use any valid open source license like `Apache License 2.0`.

The buildtool we use by default is `ament_python` and you can see that this has already been assigned when we used the `ros2 pkg create` command. Below this, add the following two lines :

```XML
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

We already know from section **1.1** what these dependancies are. We just need to declare that these two libraries need to be included during execution time. (Buildtime dependencies are not required for Python)

### 1.3 Declaring the executable

Now that we have our code written and dependencies setup, we need to tell our build system that the script we created should be treated as an executable. We do this in `setup.py`.

Here, edit the `maintainer`, `maintainer_email`, `description` and `license` fields to assign exactly the same values as you did in `package.xml`.

Next, look for the section that starts with `entry_points={`. We edit this part to declare the executable and its entry point to look like this:

```Python
entry_points={
        'console_scripts': [
                'talker = wshop_nodes.minimal_publisher:main',
        ],
},
```
In this case, `talker` is the name we assign to the executable, `wshop_nodes` is the package, `minimal_publisher` is the name of the python file and `main` is the entry point to this executable (i.e. main function). Replace with the names you chose accordingly.

You can use the same prototype to declare executables in all ROS 2 python packages.


### 1.4 Setup.cfg

The final configuration file is `setup.cfg`, which, fortunately for us, is already configured properly and needs no more changes! These settings indicate to ROS 2 where the executable shall be put for discovery after building the package.

## 2 Subscriber Node

Similar to the publisher, we will now create a subscriber. Next to the publisher file, create another python file and paste in the code below, which will be explained next.

This document refers to this file as `minimal_subscriber.py`.

```Python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
### 2.1 Explanation

The subscriber code has many similarities to the publisher code, and in this section we review what differs.

The part that is immediately relevant is creating the subscriber

```Python
self.subscription = self.create_subscription(
    String,
    'topic',
    self.listener_callback,
    10)
```

The first parameter to pass to the function is the msg type, the second is the name of the topic - this should be the same as declared in the publisher, the third is the callback function for the subscriber and the last is the message buffer size.

The next part to understand is the callback function.

```Python
def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)
```

The parameter that is automatically passed to this function is the incoming message. In this case, it is simply printed to console.

### 2.2 Declaring the executable

We need to declare a new executable for this subscriber node. We add another line to `setup.py` similarly as before and it would look like this:

```Python
entry_points={
        'console_scripts': [
                'talker = wshop_nodes.minimal_publisher:main',
                'listener = wshop_nodes.minimal_subscriber:main'
        ],
},
```
## 3 Build and run

Before building, it is always good to check if all dependencies have been installed. We execute the following from the **base workspace folder** (i.e. just above the src folder of your **workspace**):

```bash
rosdep install --from-paths src --ignore-src -r --rosdistro <distro> -y
```
Substitute <distro> with the current version of ROS2 you are running on. Ex: `jazzy`


From the same location, build the workspace:

colcon build --symlink-install
```
Now we need to source this workspace in order to be able to discover the executable that we just built:

```bash
source install/local_setup.bash
```
Finally, we are ready to run an executable. Recalling from section **1.3**, the name we assigned to the executable with the publisher is `talker`. So we run this:

```bash
ros2 run wshop_nodes talker
```
Open another terminal and similarly source ros2 and this workspace to run the subscriber executable:
```bash
ros2 run wshop_nodes listener
````


## 4 Composed nodes

We will now create a third executable, that demonstrates the node composition feature using executors. It will be a single Python script that implements two nodes - the same publisher and subscriber as above, but composes them with a single executor.

Create yet another python file, which for reference here is named as `composed_nodes.py` and paste the following:

```Python
import rclpy

from wshop_nodes.minimal_publisher import MinimalPublisher
from wshop_nodes.minimal_subscriber import MinimalSubscriber

from rclpy.executors import SingleThreadedExecutor


def main(args=None):
    rclpy.init(args=args)
    try:
        minimal_publisher = MinimalPublisher()
        minimal_subscriber = MinimalSubscriber()

        executor = SingleThreadedExecutor()
        executor.add_node(minimal_publisher)
        executor.add_node(minimal_subscriber)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            minimal_publisher.destroy_node()
            minimal_subscriber.destroy_node()

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4.1 Explanation

```Python
from wshop_nodes.minimal_publisher import MinimalPublisher
from wshop_nodes.minimal_subscriber import MinimalSubscriber

from rclpy.executors import SingleThreadedExecutor
```
We import the previous two node classes that we created into this executable.
We also import the single threaded executor that we will be using to compose the nodes.


```Python
minimal_publisher = MinimalPublisher()
minimal_subscriber = MinimalSubscriber()
```
Similarly as before, we create node objects from the two node classes.

```Python
executor = SingleThreadedExecutor()
executor.add_node(minimal_publisher)
executor.add_node(minimal_subscriber)
```
This is the new part, where we create an executor object, and add our two nodes into it.

```Python
try:
    executor.spin()
finally:
    executor.shutdown()
    minimal_publisher.destroy_node()
    minimal_subscriber.destroy_node()
```
Instead of spinning the main executable directly, we instead spin the executor that contains our two nodes.

In this case, we have added a publisher and a subscriber within the same executor to simply demonstrate how multiple nodes can be added. In a practical scenario, this may not make much sense, and requires separate executors. This, however, is a more advanced topic and out of scope for this workshop.



### 4.2 Declaring the executable

Similarly as the previous two cases, we need to declare a third executor that points to the code we just created, and the result would look like this:

```Python
    entry_points={
        'console_scripts': [
		'talker = wshop_nodes.minimal_publisher:main',
		'listener = wshop_nodes.minimal_subscriber:main',
        'composed = wshop_nodes.composed_nodes:main'
        ],
    },
```

### 4.3 Build and run

Build the workspace and run this new executable, whose name in this case is `composed`.
You will observe that indeed both the publisher and subscriber are running from within the same executable.


## 5 Exercise: Use a different message type

The publisher and subscriber in this tutorial use `std_msgs/msg/String`. Your
task is to write another publisher and subscriber that communicate using a
different ROS 2 message type.

Browse the message packages in
[ros2/common_interfaces](https://github.com/ros2/common_interfaces) and select
a message type other than `std_msgs/msg/String).

Beginner-friendly choices include:

- `geometry_msgs/msg/Point`
- `geometry_msgs/msg/Twist`
- `sensor_msgs/msg/Temperature`

### 5.1 Inspect the interface

Before writing code, inspect the fields of the selected message. For example:

```bash
ros2 interface show geometry_msgs/msg/Point
```

Replace `geometry_msgs/msg/Point` with your selected interface. Identify:

1. the Python package and class that must be imported
2. the fields that the publisher must populate
3. the fields that the subscriber should display

### 5.2 Create the nodes

Inside the `wshop_nodes/wshop_nodes` Python module, create:

```text
custom_publisher.py
custom_subscriber.py
```

Use the publisher and subscriber from this tutorial as a starting point, but
make the following changes:

- import the selected message class
- use that class when creating the publisher and subscription
- publish on a new topic such as `custom_topic`
- populate the message with valid, changing values
- log meaningful fields when the subscriber receives a message

The publisher and subscriber must use exactly the same message type and topic
name.

### 5.3 Add the dependency

Add the package containing your selected message to `package.xml`. For
example, when using `geometry_msgs/msg/Point` or
`geometry_msgs/msg/Twist`:

```xml
<exec_depend>geometry_msgs</exec_depend>
```

When using `sensor_msgs/msg/Temperature`:

```xml
<exec_depend>sensor_msgs</exec_depend>
```

Use the package name from the part before `/msg/` in the interface name.

### 5.4 Register the executables

Add entry points for both new nodes to the `console_scripts` list in
`setup.py`. For example:

```python
'custom_talker = wshop_nodes.custom_publisher:main',
'custom_listener = wshop_nodes.custom_subscriber:main',
```

### 5.5 Build and run

From the root of the workspace:

```bash
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install --packages-select wshop_nodes
source install/setup.bash
```

Run the publisher:

```bash
ros2 run wshop_nodes custom_talker
```

In another terminal, source ROS 2 and the workspace, then run the subscriber:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/dev_ws
source install/setup.bash
ros2 run wshop_nodes custom_listener
```

### 5.6 Verify the result

While both nodes are running, use a third terminal to inspect the topic:

```bash
ros2 topic info /custom_topic --verbose
ros2 topic echo /custom_topic
```

Check that:

- the topic uses the message type you selected
- one publisher and one subscriber are connected
- the published field values change over time
- the subscriber prints the expected values

As an additional test, stop your publisher and publish one message manually
with `ros2 topic pub --once`. Use `ros2 interface show` to determine the
correct YAML fields for your selected message.
