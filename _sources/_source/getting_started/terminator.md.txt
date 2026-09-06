# Terminator

Terminator is a terminal emulator that can split one window into several
terminals. This is convenient for ROS 2 exercises because several nodes often
need to run simultaneously.

## Install Terminator

```bash
sudo apt update
sudo apt install terminator
```

Start it from an existing terminal:

```bash
terminator
```

It can also be opened from the Ubuntu application menu.

## Useful shortcuts

| Shortcut | Action |
| --- | --- |
| `Ctrl+Shift+O` | Split horizontally |
| `Ctrl+Shift+E` | Split vertically |
| `Ctrl+Shift+T` | Open a new tab |
| `Ctrl+Tab` | Move to the next terminal |
| `Ctrl+Shift+N` | Move to the next terminal |
| `Ctrl+Shift+W` | Close the current terminal |

## Suggested workshop layout

A useful layout is:

- one terminal for building the workspace
- one terminal for running a ROS 2 node
- one terminal for inspecting topics, services, or actions

Each new terminal needs the ROS 2 environment and, when applicable, the
workspace environment:

```bash
source /opt/ros/jazzy/setup.bash
source ~/dev_ws/install/setup.bash
```

Use `/opt/ros/humble/setup.bash` instead when working with ROS 2 Humble.

Run `pwd` in each terminal if you are unsure which directory it is using.
