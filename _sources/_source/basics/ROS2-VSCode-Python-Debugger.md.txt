# Debugging ROS 2 Python nodes with VS Code

This guide uses the `tf2_workshop` package in this workspace as an example. It
starts each Python node directly with the VS Code Python debugger, allowing
breakpoints to be placed in the source files under `src/`.


## 1. Install the VS Code extensions

Install these extensions from the VS Code Extensions view:

1. **Python** (`ms-python.python`)
2. **Python Debugger** (`ms-python.debugpy`)

The ROS extension is useful for working with ROS files, but it is not required
for the Python breakpoint configuration in this guide.

More info on Python debugging in VS code [here](https://code.visualstudio.com/docs/python/debugging).

## 2. Build and source the workspace

Open a terminal and run from the ROS 2 workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tf2_workshop --symlink-install
source install/setup.bash
```

Using `--symlink-install` is convenient while developing Python packages.
However, the debug configuration below runs the files under `src/` directly,
so ordinary edits to these Python files do not require a rebuild before every
debug session.

## 3. Start VS Code with the ROS environment

Start VS Code from the same sourced terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
code .
```

This is important because VS Code inherits ROS environment variables such as
`AMENT_PREFIX_PATH` and `PYTHONPATH` from the terminal.

Make sure VS Code opens the workspace folder, not only the `tf2_workshop` package directory.

## 4. Select the Python interpreter

In VS Code:

1. Open the Command Palette with `Ctrl+Shift+P`.
2. Run **Python: Select Interpreter**.
3. Select `/usr/bin/python3` (Python 3.12 on this ROS 2 Jazzy system).

The debug configuration also specifies `/usr/bin/python3` explicitly.

## 5. Create the VS Code debug configuration

### Generate `launch.json`

The Python Debugger extension can generate the initial VS Code debug
configuration:

1. Open **Run and Debug** with `Ctrl+Shift+D`.
2. Select **create a launch.json file**.
3. Select **Python Debugger**.
4. Select **Python File**.

VS Code creates `.vscode/launch.json`. 

A workspace normally has one `launch.json` containing one or more configurations. 
Once the file exists, VS Code no longer offers to create it again; new configurations are added to its `configurations` array.

A generated configuration for the current Python file looks like this:

```json
{
    "name": "Python Debugger: Current File",
    "type": "debugpy",
    "request": "launch",
    "program": "${file}",
    "console": "integratedTerminal"
}
```

Creating the configuration and starting it are separate actions. After the file has been generated, the green start button runs the selected configuration; it does not generate another configuration.

### Understand `${file}`

The value:

```json
"program": "${file}"
```

means "execute the file in the currently active editor." Therefore before pressing the green start button, open the Python file that you intend to debug and click inside its editor tab.

If `launch.json` is the active editor, VS Code attempts to execute the JSON file as Python and reports a `SyntaxError`. If `broadcaster.py` or `listener.py` is active, the Python file starts, but the basic generated configuration does not supply the arguments expected by these example nodes.
They can therefore report:

```text
IndexError: list index out of range
```

### Add a generated configuration that asks for arguments

Use the Command Palette to add a configuration to the existing file:

1. Press `Ctrl+Shift+P`.
2. Run **Debug: Add Configuration...**.
3. Select **Python Debugger**.
4. Select **Python File with Arguments**.

The generated configuration should look similar to this:

```json
{
    "name": "Python Debugger: Current File with Arguments",
    "type": "debugpy",
    "request": "launch",
    "program": "${file}",
    "console": "integratedTerminal",
    "args": "${command:pickArgs}"
}
```

`${command:pickArgs}` tells VS Code to ask for command-line arguments whenever the configuration starts.

To debug the first broadcaster:

1. Open
   `src/ros2_i_training_example_scripts/tf2_workshop/tf2_workshop/broadcaster.py`.
2. Select **Python Debugger: Current File with Arguments**.
3. Press the green start button.
4. Enter this when VS Code asks for arguments:

   ```text
   turtle1
   ```

To debug the listener, open `listener.py` and enter:

```text
turtle1 turtle2
```

The required simulator, turtles, and other ROS nodes must also be running. The following sections show how to start them.

## 6. Set useful breakpoints

Open the following source file:

```text
src/ros2_i_training_example_scripts/tf2_workshop/tf2_workshop/broadcaster.py
```

Set a breakpoint inside `DynamicBroadcaster.handle_pose()`, for example on:

```python
tfs = TransformStamped()
```

This breakpoint is reached whenever a turtle pose message is received.

Next, open:

```text
src/ros2_i_training_example_scripts/tf2_workshop/tf2_workshop/listener.py
```

Set a breakpoint inside `TfListener.timer_callback()`, for example on:

```python
trans = self._tf_buffer.lookup_transform(
```

You can also place a breakpoint on:

```python
self.publisher_.publish(self.cmd_)
```

Click in the gutter to the left of the line number, or put the cursor on a line
and press `F9`.

## 7. Start turtlesim

Do not run the existing `turtle_tf2_demo.launch.py` for this debug session. It
would start additional non-debug copies of the Python nodes. Instead, start
only the simulator manually.

In a new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run turtlesim turtlesim_node
```

Leave this terminal running.

Debugging listener.py without the second turtle spawned: during node construction, the create_publisher throws an exception that is not caught.


## 8. Spawn the second turtle

In another sourced terminal, run:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 service call /spawn turtlesim/srv/Spawn \
  "{x: 2.0, y: 2.0, theta: 0.2, name: 'turtle2'}"
```

The service response should report the name `turtle2`.

If the service is not available yet, wait until the turtlesim window has
opened and run the command again.

## 9. Start the debugger

In VS Code:

1. Open **Run and Debug** with `Ctrl+Shift+D`.
2. Select **tf2: debug all Python nodes** from the configuration menu.
3. Press `F5` or click the green start button.

VS Code starts three debug sessions:

1. A transform broadcaster for `turtle1`
2. A transform broadcaster for `turtle2`
3. The transform listener that makes `turtle2` follow `turtle1`

The broadcaster breakpoints should be reached as pose messages arrive. The
listener breakpoint is reached by its timer callback. When execution pauses,
use the Variables, Watch, and Call Stack sections in the Run and Debug view to
inspect values.

Useful debugger controls are:

- Continue: `F5`
- Step over: `F10`
- Step into: `F11`
- Step out: `Shift+F11`
- Stop: `Shift+F5`

Because pose and timer callbacks run repeatedly, a breakpoint in either
callback can be reached very frequently. Disable the breakpoint after the
first useful pause if necessary.

## 10. Move turtle1 manually

To move `turtle1`, open one more sourced terminal and run:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run turtlesim turtle_teleop_key
```

Keep that terminal focused and use its arrow keys. After the debugger is
continued, `turtle2` should follow `turtle1`.

## Debug only one node

The three individual configurations can also be selected from the Run and
Debug menu. For example, select **tf2: listener (turtle follower)** to debug
only `listener.py`.

When debugging only one node, the other required ROS nodes must already be
running. Do not start a second copy of the same node unless that is intentional.

## Troubleshooting

### A breakpoint is gray or says "Unbound breakpoint"

Check all of the following:

1. The selected debug configuration uses `type: "debugpy"`.
2. The Python Debugger extension is installed and enabled.
3. The breakpoint is in a file under `src/ros2_i_training_example_scripts/`.
4. The `program` path in `launch.json` points to that same source file.
5. The relevant callback is actually being invoked.

Do not put breakpoints in these generated or installed copies:

```text
build/tf2_workshop/...
install/tf2_workshop/...
```

### `ModuleNotFoundError: No module named 'rclpy'`

VS Code was probably started without the ROS environment. Close every VS Code
window for this workspace, then start it again from a sourced terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
code .
```

Also confirm that `/usr/bin/python3` is the selected interpreter.

### `IndexError: list index out of range`

The example nodes read their turtle names from `sys.argv`. Start them with one
of the configurations in this guide so the required `args` are supplied.

### Turtle2 already exists

The `/spawn` service reports an error if `turtle2` has already been created.
Either keep using the existing turtle or restart `turtlesim` before running the
spawn command again.

### The listener repeatedly reports a transform lookup error

Confirm that:

1. Both broadcaster debug sessions are running.
2. Both `turtle1` and `turtle2` exist in turtlesim.
3. Paused broadcaster breakpoints have been continued with `F5`.

While execution is stopped at a broadcaster breakpoint, that node cannot
publish new transforms. Temporary lookup failures from the listener are
therefore normal during step-by-step debugging.

### Source changes do not appear when using `ros2 run`

The configurations in this guide execute source files directly. If you switch
back to `ros2 run`, rebuild and source the workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tf2_workshop --symlink-install
source install/setup.bash
```

With `--symlink-install`, later edits to installed Python source are normally
visible without copying the files again. Changes to package metadata, entry
points, or generated interfaces still require a rebuild.

## Stop the example

Stop the compound debug session with `Shift+F5`. Then stop `turtlesim` and
`turtle_teleop_key` with `Ctrl+C` in their terminals.
