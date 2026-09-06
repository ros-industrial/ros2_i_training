# Prerequisites

Prepare the computer before starting the exercises. The workshop assumes an
Ubuntu installation with a matching ROS 2 distribution.

## Supported environments

| Ubuntu | ROS 2 |
| --- | --- |
| Ubuntu 24.04 | Jazzy |
| Ubuntu 22.04 | Humble |

Use the ROS 2 distribution that matches your Ubuntu version. Do not source two
ROS 2 distributions in the same terminal.

Follow the official ROS 2 installation instructions:

- [Install ROS 2 Jazzy on Ubuntu 24.04](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Install ROS 2 Humble on Ubuntu 22.04](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

Install the command-line tools used during the workshop:

```bash
sudo apt update
sudo apt install git terminator python3-vcstool python3-colcon-common-extensions
```

Docker and Visual Studio Code are useful but optional:

- [Install Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/)
- [Install Visual Studio Code on Linux](https://code.visualstudio.com/docs/setup/linux)

## Check the installation

Check the Ubuntu version:

```bash
lsb_release -ds
```

Source the ROS 2 installation. This example uses Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
```

For Ubuntu 22.04 with ROS 2 Humble, use:

```bash
source /opt/ros/humble/setup.bash
```

Verify the tools:

```bash
printenv ROS_DISTRO
ros2 --help
colcon --help
vcs help
git --version
```

The first command should print the ROS 2 distribution selected above.

## Configure Bash

ROS 2 must be sourced in every new terminal. To source it automatically, add
the appropriate setup command to `~/.bashrc`. Add only one ROS 2 distribution.

For Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```

For Humble, replace `jazzy` with `humble`.

Apply changes made to `~/.bashrc` in the current terminal:

```bash
source ~/.bashrc
```

## Set the ROS domain

`ROS_DOMAIN_ID` separates independent ROS 2 systems on the same network.
During a workshop, use the domain ID assigned by the instructor so that
participants do not interfere with one another.

For example:

```bash
export ROS_DOMAIN_ID=10
```

Add the export to `~/.bashrc` if it should apply to every new terminal. All
machines that need to communicate with each other must use the same domain ID.

Check the current value:

```bash
printenv ROS_DOMAIN_ID
```
