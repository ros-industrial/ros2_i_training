## Introduction

The workshop documents are written in `Markdown` language and built using [Sphinx](https://docs.readthedocs.io/en/stable/intro/getting-started-with-sphinx.html?) documentation generator.

## Usage
### Install dependencies

````bash
cd ~/ros2_i_training/workshop/
python3 -m pip install -r requirements.txt
````

### To add new content
Place the content source in the following folders in the appropriate session.
-  `_source`: Markup (.md) files 
- `_static`: Images and other resource files  

Add the workshop's heading and filepath relative to `~/ros2_i_training/workshop/source/_source` to `index.rst`. To build the html:
 ````bash
 cd ~/ros2_i_training/workshop/
 make html
 ````
`index.rst` is built into `index.html` in the documentation output directory `~/ros2_i_training/workshop/build/html/index.html`. 

![docs](/workshop/source/_static/demo_rtd.png)


### Setup Docker for the workshops

To install `docker` on your `Ubuntu` system, follow the instructions [here](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-18-04). To use `docker` without `sudo`, make sure to follow the instructions in step 2, "Executing the Docker Command Without Sudo".

The docker image for the training is available on [DockerHub](https://hub.docker.com/r/ipahsd/ros2-training-foxy).

```
docker pull ipahsd/ros2-training-foxy:01
```
To test the docker image, run:
```
docker run -it ipahsd/ros2-training-foxy:01 /bin/bash
---
root@fe7b28bad402:/#
```
The command creates a new container and starts it. This can be confirmed in a new terminal
```
docker container ls
---
CONTAINER ID   IMAGE                          COMMAND                  CREATED          STATUS          PORTS          NAMES
05100b6119b1   ipahsd/ros2-training-foxy:01   "/ros_entrypoint.sh …"   8 seconds ago    Up 7 seconds                   nostalgic_ptolemy
```
The last column `NAMES` shows the name of the container `nostalgic_ptolemy` that is randomly assigned. These names can be manually assgined with `--name` option.
```
docker run -it --name test ipahsd/ros2-training-foxy:01 /bin/bash
```
Now to confirm the name of the container
```
docker container ls
---
CONTAINER ID   IMAGE                          COMMAND                  CREATED          STATUS          PORTS         NAMES
0a52451af871   ipahsd/ros2-training-foxy:01   "/ros_entrypoint.sh …"   7 seconds ago    Up 6 seconds                  test
```
To stop a container without removing it
```
docker container stop test
```
The stopped container can be restarted with
```
docker start test
```
To execute an existing container which has been started
```
docker exec -it test /bin/bash
```
The command line prompt of docker (e.g. `root@fe7b28bad402:/#`) can be used like a typical Ubuntu terminal, but without graphics.
Below is an example of [talker-listener](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html) by creating two containers on separate terminals and communicating between them through `ROS` messages.
```
docker run -it --name talker ipahsd/ros2-training-foxy:01 /bin/bash
---
root@fe7b28bad402:/#
---
root@fe7b28bad402:/# ros2 run demo_nodes_cpp talker
```
and in a new terminal
```
docker run -it --name listener ipahsd/ros2-training-foxy:01 /bin/bash
---
root@5ab4cf0d8dac:/#
---
root@5ab4cf0d8dac:/# ros2 run demo_nodes_cpp listener
```
Now the docker container in second terminal starts receiving messages published by the docker container in the first terminal.
To list these two running containers created in the previous step:
```
docker container ls
---
CONTAINER ID   IMAGE                          COMMAND                  CREATED          STATUS          PORTS            NAMES
ad9a8a8e4f26   ipahsd/ros2-training-foxy:01   "/ros_entrypoint.sh …"   15 minutes ago   Up 15 minutes                    talker
d605c3f3b106   ipahsd/ros2-training-foxy:01   "/ros_entrypoint.sh …"   15 minutes ago   Up 15 minutes                    listener
```

Another option to run `ROS` commands is
```
docker run -it --rm --name talker ipahsd/ros2-training-foxy:01 ros2 run demo_nodes_cpp talker
```
In a new terminal, run:
```
docker run -it --rm --name listener ipahsd/ros2-training-foxy:01 ros2 run demo_nodes_cpp listener
```
Notice `--rm` option in the above commands. With this option, the container is removed when exited.


To exit the container
```
root@fe7b28bad402:/# exit
```

### To use docker with GUI

To be able to use RViz and Gazebo, the docker container needs graphics support. We modified [moveit/gui-docker](https://github.com/ros-planning/moveit/blob/master/.docker/gui-docker). To download the script
```
wget https://raw.githubusercontent.com/ros-industrial/ros2_i_training/main/gui-docker
chmod +x gui-docker
```
To test `Rviz2` with docker
```
./gui-docker -it ipahsd/ros2-training-foxy:01 rviz2
```

#### Toubleshooting
In case of errors while using graphics with docker
```
QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-root'
```
Or
```
[ERROR] [1636105083.324525273] [rviz2]: rviz::RenderSystem: error creating render window: RenderingAPIException: Unable to create a suitable GLXContext in GLXContext::GLXContext at /tmp/binarydeb/ros-foxy-rviz-ogre-vendor-8.2.1/obj-x86_64-linux-gnu/ogre-v1.12.1-prefix/src/ogre-v1.12.1/RenderSystems/GLSupport/src/GLX/OgreGLXContext.cpp (line 60)
[ERROR] [1636105083.328652053] [rviz2]: Failed to create an OpenGL context. BadValue (integer parameter out of range for operation)
[ERROR] [1636105083.328664706] [rviz2]: RenderingAPIException: Unable to create a suitable GLXContext in GLXContext::GLXContext at /tmp/binarydeb/ros-foxy-rviz-ogre-vendor-8.2.1/obj-x86_64-linux-gnu/ogre-v1.12.1-prefix/src/ogre-v1.12.1/RenderSystems/GLSupport/src/GLX/OgreGLXContext.cpp (line 60)
[ERROR] [1636105083.328690617] [rviz2]: rviz::RenderSystem: error creating render window: RenderingAPIException: Unable to create a suitable GLXContext in GLXContext::GLXContext at /tmp/binarydeb/ros-foxy-rviz-ogre-vendor-8.2.1/obj-x86_64-linux-gnu/ogre-v1.12.1-prefix/src/ogre-v1.12.1/RenderSystems/GLSupport/src/GLX/OgreGLXContext.cpp (line 60)
[ERROR] [1636105083.328699397] [rviz2]: Unable to create the rendering window after 100 tries
terminate called after throwing an instance of 'std::runtime_error'
what(): Unable to create the rendering window after 100 tries
```
Or
```
docker: Error response from daemon: invalid volume specification: '/tmp/.X11-unix: tmp/.X11-unix:rw': invalid mount config for type "bind": invalid mount path: ' tmp/.X11-unix' mount path must be absolute.
```
most likely the graphics card used is NVidia.

To check graphic card:
```
lspci | grep -i nvidia
```
or
```
sudo lshw -C display
```
If NVidia graphic card is active, `nvidia-docker` or `nvidia-docker2` needs to be installed. For installation instructions, check ["How to install nvidia-docker2"](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit)

After the installation, try running docker again
```
./gui-docker -it ipahsd/ros2-training-foxy:01 rviz2
```

#### Related links
[ROS: Using Hardware Acceleration with Docker](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration)

[How to install nvidia-docker2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

[NVIDIA CONTAINER TOOLKIT](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/user-guide.html#) If you have an external nvida gpu installed, you might need to deactivate it for graphical docker. In order to do that start nvidia-seetings and search (on the left side) for "prime". Select your internal gpu and reboot.

[nvidia-docker wiki](https://github.com/nvidia/nvidia-docker/wiki)
