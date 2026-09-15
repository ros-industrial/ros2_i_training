# ROS 2 Logging

## 1. Introduction

When a robot does not behave as expected, logs help explain what its nodes
are doing: starting up, receiving data, waiting for a transform, or reporting
an error. A node's logger identifies the source and importance of each message.

For example, Turtlesim logs information when it starts and spawns a turtle.

ROS logs can reach the terminal, files on disk, and other ROS tools through
the `/rosout` topic. Ordinary `printf`, `print`, or `println!` output does
not automatically become a ROS log message.

In this exercise, run Turtlesim and keyboard teleoperation and observe their
existing output. No source-code changes or new packages are needed.

## 2. Severity levels

| Level | `/rosout` value | Example use |
| --- | --- | --- |
| DEBUG | 10 | Inspect a computed value during debugging. |
| INFO | 20 | Report successful startup or a state change. |
| WARN | 30 | Report an unexpected condition that permits continued operation. |
| ERROR | 40 | Report a failed operation. |
| FATAL | 50 | Report a condition that prevents useful operation. |

The default threshold is `INFO`. \
A threshold of `WARN` permits `WARN`, `ERROR`, and `FATAL`, while suppressing `DEBUG` and `INFO`. \
A `FATAL` log call alone does not shut down a node; application code decides how to handle the failure.

See the [ROS 2 logging concepts](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Logging.html)
for severity filtering and logger configuration.

## 3. Start Turtlesim and teleop

In the first terminal, source ROS 2 and start the simulator:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run turtlesim turtlesim_node
```

The Turtlesim window opens. Look at the startup log messages in this terminal:
they include a severity, timestamp, logger name, and message.

In a second terminal, start keyboard teleoperation:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run turtlesim turtle_teleop_key
```

Keep the second terminal focused and use the arrow keys to move the turtle.
Watch the simulator terminal for log messages. Moving the turtle does not
necessarily produce a message for every key press. The keyboard instructions
printed by teleop are ordinary terminal output, not necessarily ROS logs.

## 4. Change the logging threshold

Stop only the simulator with `Ctrl+C`, then restart it with a warning threshold:

```bash
ros2 run turtlesim turtlesim_node --ros-args --log-level turtlesim:=WARN
```

The startup `INFO` messages are now suppressed. Warnings and errors can still
appear when those conditions occur. Teleop can stay running while you restart
the simulator.

Here, `turtlesim` is the logger name, while `turtlesim_node` is the executable.
Restart without the extra arguments to restore the default `INFO` threshold:

```bash
ros2 run turtlesim turtlesim_node
```

## 5. Observe /rosout

In a third terminal, start an observer, then restart Turtlesim with its default
logging threshold to observe the startup messages:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /rosout
```

The topic carries `rcl_interfaces/msg/Log` messages. Inspect the definition:

```bash
ros2 interface show rcl_interfaces/msg/Log
```

Each message includes `stamp`, `level`, `name`, `msg`, and source-location
fields `file`, `function`, and `line`. Available source-location details
depend on the client library.

Stop the observer with `Ctrl+C` and try each filter separately.
Show only the message text from Turtlesim:

```bash
ros2 topic echo /rosout --filter "m.name == 'turtlesim'" --field msg
```

Show warnings and more severe messages from any logger:

```bash
ros2 topic echo /rosout --filter "m.level >= 30"
```

These filters select messages received by the observer. They cannot recover
debug messages suppressed by the producing node's threshold. Start the
observer before the node to capture startup messages; `/rosout` is not a
complete historical log archive.

## 6. Use rqt Console

Start the graphical log viewer in another sourced terminal:

```bash
ros2 run rqt_console rqt_console
```

Alternatively, run `rqt` and choose **Plugins -> Logging -> Console**.
If the package is missing on Ubuntu with Jazzy, install `ros-jazzy-rqt-console`.

Restart Turtlesim and locate the `turtlesim` logger in the received messages.
Use severity and text filters to narrow the view. Compare an `INFO` run with
a `WARN` run: the viewer only receives messages the node actually emits.

## 7. Find log files and control destinations

With the default logging backend, files normally appear under `~/.ros/log`.
Set `ROS_LOG_DIR` before starting the node to choose an explicit directory:

```bash
export ROS_LOG_DIR="$HOME/turtlesim_logs"
```

Restart Turtlesim from that terminal, stop it with `Ctrl+C`, and inspect
that directory:

```bash
ls -lt "$ROS_LOG_DIR"
```

Filenames and launch-created subdirectories vary. Open the newest relevant
file and find the simulator's startup messages from an `INFO` run.
Restore the default afterward:

```bash
unset ROS_LOG_DIR
```

The following ROS arguments selectively disable destinations:

| Argument after `--ros-args` | Effect |
| --- | --- |
| `--disable-rosout-logs` | Stop publishing logs to `/rosout`. |
| `--disable-external-lib-logs` | Disable the external backend, normally responsible for disk logs. |
| `--disable-stdout-logs` | Disable console logs, despite the default stream being stderr. |

For example, restart the simulator with:

```bash
ros2 run turtlesim turtlesim_node --ros-args --disable-rosout-logs
```

Terminal logs should continue, but new messages from this node should no
longer arrive in the `/rosout` observer.

## 8. Stop the example

Press `Ctrl+C` in the teleop and simulator terminals. Stop the
`/rosout` observer and close rqt Console if you opened them.

## 9. Troubleshooting

| Symptom | Check |
| --- | --- |
| No startup messages | Restart Turtlesim with the default `INFO` threshold. |
| Terminal output but no rosout messages | Use ROS logging calls, keep the node alive, and check that rosout is enabled. |
| No messages from another terminal or computer | Check environment setup, matching `ROS_DOMAIN_ID`, and network discovery. |
| Some startup messages are missing | Start the observer first, then restart the example. |
| No log file in the expected directory | Check `ROS_LOG_DIR`, `ROS_HOME`, directory permissions, and whether disk logging was disabled. |

## 10. How it works internally

Client libraries expose language-specific logging calls. Underneath, the ROS logging infrastructure filters messages and routes enabled output to
the console, the external file backend, and `/rosout` through the middleware.

### How logs are published to /rosout

There is a per-node rosout publisher when rosout logging is enabled.
The client library manages its initialization and cleanup with the node's lifecycle. 
`/rosout` is the network output destination; console and disk logging are separate destinations.

### RCLCPP

Node initialization performs approximately the following steps. This is simplified pseudocode, not code to add to your application:

```cpp
rcl_node_init(node, ...);

if (
    rcl_logging_rosout_enabled() &&
    node_options.enable_rosout
) {
    rcl_logging_rosout_init_publisher_for_node(node);
}
```

The call to
[`rcl_logging_rosout_init_publisher_for_node`](https://github.com/ros2/rclcpp/blob/2209942eb1361fdaf48ec8512b6dccf70235bccb/rclcpp/src/rclcpp/node_interfaces/node_base.cpp#L118)
in `NodeBase` initializes the publisher for `/rosout`. With a DDS-based
middleware, this creates the underlying DDS publisher through the ROS
middleware layer.

During node destruction, `NodeBase` calls
[`rcl_logging_rosout_fini_publisher_for_node`](https://github.com/ros2/rclcpp/blob/2209942eb1361fdaf48ec8512b6dccf70235bccb/rclcpp/src/rclcpp/node_interfaces/node_base.cpp#L130)
to clean up that rosout publisher before finalizing the node.

### RCLPY

RCLPY follows the same explicit lifecycle in its underlying C++ binding:
initialize the RCL node, initialize its rosout publisher when enabled, and finalize that publisher during node destruction. 
Python application code uses `self.get_logger()`.

### Publisher lifecycle and message flow

```text
Node creation (rclcpp / rclpy)
    |
    +-- rcl_node_init()
    |
    +-- if rosout is enabled:
            rcl_logging_rosout_init_publisher_for_node()

Regular logging calls (rclcpp / rclpy / rclrs)
    |
    +-- severity filtering and output routing
            |
            +-- console
            +-- disk via the external logging backend
            +-- /rosout via the node's publisher, when enabled

Node destruction (rclcpp / rclpy)
    |
    +-- clean up rosout publisher:
    |       rcl_logging_rosout_fini_publisher_for_node()
    |
    +-- rcl_node_fini()
```

Creating the publisher and emitting a log message are separate operations.
The publisher is prepared during node initialization and used by subsequent logging calls. 
Application code should use the node's logger rather than manually creating a `/rosout` publisher or calling these internal functions.

## 11. Further reading

- [ROS 2 Jazzy: Logging and logger configuration](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Logging.html)
- [ROS 2 Jazzy: Logging demo](https://docs.ros.org/en/jazzy/Tutorials/Demos/Logging-and-logger-configuration.html)
- [ROS 2 Jazzy: rqt Console](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Using-Rqt-Console/Using-Rqt-Console.html)
- [rclrs 0.7 logging macros and examples](https://docs.rs/rclrs/0.7.0/rclrs/#logging). Warning: rclrs 0.7 does not publish /rosout, however next release should.
