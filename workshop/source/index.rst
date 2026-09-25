Welcome to the ROS 2 workshop!
==============================

`Training slides <slides/index.html>`_

.. toctree::
   :maxdepth: 3
   :hidden:

   1. Getting Started <_source/getting_started/index>
   2. ROS 2 Basics <_source/basics/index>
   3. ROS 2 Navigation <_source/navigation/index>
   4. ROS 2 Manipulation <_source/manipulation/index>
   5. ROS 2 Control <_source/control/index>

1. Getting Started
------------------

- :doc:`Installation and prerequisites <_source/getting_started/prerequisites>` — prepare Ubuntu, ROS 2, and the required packages.
- :doc:`Linux and terminal basics <_source/getting_started/linux_and_terminal>` and :doc:`Terminator <_source/getting_started/terminator>` — work efficiently from the command line.
- :doc:`Networking <_source/getting_started/networking>` — configure and verify communication between ROS 2 systems.
- :doc:`Git and vcstool <_source/getting_started/git_and_vcstool>` and :doc:`Docker <_source/getting_started/docker>` — manage source repositories and containerized environments.

2. ROS 2 Basics
---------------

- Command-line and GUI tools: :doc:`ROS 2 CLI and Turtlesim <_source/basics/ROS2-CLI>` and :doc:`rqt <_source/basics/ROS2-RQt>`.
- Workspaces, packages, and launch files: :doc:`ROS 2 file system <_source/basics/ROS2-Filesystem>` and :doc:`launch files <_source/basics/ROS2-Launch-Files>`.
- Python nodes: :doc:`publishers and subscribers <_source/basics/ROS2-Simple-Publisher-Subscriber>` and :doc:`services <_source/basics/ROS2-Simple-Service>`.
- Data and debugging: :doc:`rosbag and PlotJuggler <_source/basics/ROS2-Bags-PlotJuggler>`, :doc:`logging <_source/basics/ROS2-Logging>`, and the :doc:`VS Code Python debugger <_source/basics/ROS2-VSCode-Python-Debugger>`.
- Robot transforms with :doc:`TF2 <_source/navigation/ROS2-TF2>`.
- Reference material: :doc:`cheat sheet <_source/basics/ROS2-Basics-CheatSheet>`, :doc:`GitHub examples <_source/basics/ROS2-Github_Examples>`, and :doc:`training scripts <_source/basics/ROS2-Training-Scripts>`.

3. ROS 2 Navigation
-------------------

- :doc:`TurtleBot 3 <_source/navigation/ROS2-Turtlebot>` — set up and operate the mobile robot.
- :doc:`Cartographer <_source/navigation/ROS2-Cartographer>` — create maps with SLAM.
- :doc:`Navigation <_source/navigation/ROS2-Navigation>` — localize the robot and navigate autonomously.

4. ROS 2 Manipulation
---------------------

- :doc:`Robot Description <_source/manipulation/robot_description/index>` — learn URDF, build a Cartesian robot with Xacro, and combine robot and gripper models.
- :doc:`MoveIt <_source/manipulation/moveit/index>` — understand the architecture, create configurations for manipulators and grippers, explore Yaskawa and AgileX examples, and control MoveIt from Python.

5. ROS 2 Control
----------------

- Basic control examples: :doc:`RRBot <_source/control/01_Example>` and :doc:`DiffBot <_source/control/02_Example>`.
- Hardware interfaces and sensors: :doc:`multiple interfaces <_source/control/03_Example>` and :doc:`external sensor integration <_source/control/04_Example>`.
- Integrated robots: :doc:`WBot mobile manipulation <_source/control/05_Example>` and :doc:`tricycle robot control <_source/control/06_Example>`.
