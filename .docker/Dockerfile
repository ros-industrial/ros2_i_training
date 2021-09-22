FROM moveit/moveit2:foxy-source

RUN apt-get -qq update && apt-get install -y curl gnupg2 lsb-release python3-pip git wget \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN TZ="Europe/Berlin" \
    && echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections \
    && echo keyboard-configuration keyboard-configuration/layout select 'German' | debconf-set-selections \
    && apt-get -qq update && apt-get -qq dist-upgrade -y \
    && apt-get install -y ros-foxy-desktop \
                          ros-foxy-cartographer \
                          ros-foxy-cartographer-ros \
                          ros-foxy-navigation2 \
                          ros-foxy-nav2-bringup \
                          ros-foxy-turtlebot3 \
                          ros-foxy-turtlebot3-simulations \
    && rm -rf /var/lib/apt/lists/*

COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
