## Introduction

This repository contains the source files for training material for **ROS 2 Jazzy**. The topics covered are:

- ROS 2 basics
    - Node composition, publish/subscribe, services, actions, parameters, launch system
    - Managed nodes, Quality of Service (QoS)
    - File system
- Navigation
    - SLAM, navigation
- Manipulation
    - Basics of manipulation
- ROS 2 control
    - Hardware interfaces, controllers, and sensor integration
    - Hands-on exercises with robot arms, mobile robots, and mobile manipulators

## Training material

- [Slides](slides/readme.md): Presentation material that can be built into PDF slide decks.
- [Workshop material](workshop/readme.md): Hands-on training documentation that can be built as an HTML site.

See the linked READMEs for build instructions.

## Development

Enable pre-commit hooks to keep formatting tidy before pushing changes:

```sh
pip install pre-commit
pre-commit install
pre-commit run --all-files
```

***
<!--
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="https://raw.githubusercontent.com/rosin-project/press_kit/master/img/rosin_ack_logo_wide.png" alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="https://raw.githubusercontent.com/rosin-project/press_kit/master/img/rosin_eu_flag.jpg" alt="eu_flag" height="45" align="left" >

This project has received funding from the European Union’s Horizon 2020
research and innovation programme under grant agreement no. 732287.
***
<a rel="license" href="http://creativecommons.org/licenses/by/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by/4.0/80x15.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by/4.0/">Creative Commons Attribution 4.0 International License</a>.
