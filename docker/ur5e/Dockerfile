FROM nvidia/opengl:1.2-glvnd-devel-ubuntu18.04

RUN apt-get update && apt-get install -y --no-install-recommends curl lsb-release wget gnupg
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -

RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get install -y --no-install-recommends tzdata
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-desktop-full \
    ros-melodic-gazebo-ros-pkgs \
    ros-melodic-gazebo-ros-control \
    ros-melodic-ros-control \
    ros-melodic-ros-controllers \
    ros-melodic-control-toolbox \
    ros-melodic-moveit \
    ros-melodic-ur-client-library \
    ros-melodic-effort-controllers \
    ros-melodic-forward-command-controller \
    ros-melodic-joint-state-controller \
    ros-melodic-joint-trajectory-controller \
    ros-melodic-cartesian-interface \
    ros-melodic-cartesian-control-msgs \
    ros-melodic-cartesian-trajectory-controller \
    ros-melodic-cartesian-trajectory-interpolation \
    ros-melodic-speed-scaling-interface \
    ros-melodic-twist-controller \
    ros-melodic-industrial-robot-status-interface \
    ros-melodic-pass-through-controllers \
    ros-melodic-ur-msgs \
    ros-melodic-industrial-robot-status-controller \
    ros-melodic-force-torque-sensor-controller \
    ros-melodic-speed-scaling-state-controller \
    ros-melodic-joint-state-publisher-gui \
    ros-melodic-scaled-joint-trajectory-controller \
    ros-melodic-position-controllers \
    ros-melodic-velocity-controllers \
    ros-melodic-tf \
    ros-melodic-tf2-tools \
    ros-melodic-robot-state-publisher \
    ros-melodic-genpy \
    ros-melodic-industrial-core \
    ros-melodic-roslint \
    socat \
    libcanberra-gtk* \
    python-catkin-tools \
    build-essential \
    python-rosdep \
    byobu \
    libudev-dev \
    python-pip \
    python-setuptools \
    git \
    vim \
    net-tools \
    dbus

RUN mkdir -p /tmp/
RUN mkdir -p /catkin_ws/src && \
    cd /catkin_ws/src && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_init_workspace" && \
    cd /catkin_ws && rosdep init && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin build" && \
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc && \
    echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/catkin_ws" >> ~/.bashrc && \
    echo "export ROS_WORKSPACE=/catkin_ws" >> ~/.bashrc

# Universal Robots
WORKDIR /catkin_ws/src/
RUN git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git --recursive \
    && cd Universal_Robots_ROS_Driver \
    && git reset --hard 189543f18e11d0f1e418e48266c3a30a23e88091
RUN git clone https://github.com/takuya-ki/universal_robot.git -b calibration_devel --recursive \
    && cd universal_robot \
    && git reset --hard 69752c5e9bb61081e5cc783fcc93e768d8f0e4e1

COPY ./catkin_ws/src/ur5e_tutorials /catkin_ws/src/ur5e_tutorials
RUN cp -f /catkin_ws/src/ur5e_tutorials/universal_robot/ur5e_moveit_config/config/joint_limits.yaml /catkin_ws/src/universal_robot/ur5e_moveit_config/config/  

# Generates bash commands
# real robot
RUN echo "roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.11" >> ~/.bash_history
RUN echo "roslaunch ur5e_moveit_config ur5e_moveit_planning_execution.launch sim:=false" >> ~/.bash_history
RUN echo "roslaunch ur5e_moveit_config moveit_rviz.launch rviz_config:=/catkin_ws/src/universal_robot/ur5e_moveit_config/launch/moveit.rviz" >> ~/.bash_history
RUN echo "roslaunch ur5e_tutorials wiggle.launch fake_execution=false" >> ~/.bash_history
RUN echo "roslaunch ur5e_tutorials pick_and_place.launch use_gripper:=true fake_execution=false" >> ~/.bash_history
RUN echo "roslaunch ur5e_tutorials pick_and_toss.launch use_gripper:=true fake_execution=false" >> ~/.bash_history
# simulation
RUN echo "roslaunch ur_description view_ur5e.launch" >> ~/.bash_history
RUN echo "roslaunch ur5e_moveit_config demo.launch" >> ~/.bash_history
RUN echo "roslaunch ur5e_tutorials wiggle.launch fake_execution=true" >> ~/.bash_history
RUN echo "roslaunch ur5e_tutorials pick_and_place.launch fake_execution=true" >> ~/.bash_history
RUN echo "roslaunch ur5e_tutorials pick_and_toss.launch fake_execution=true" >> ~/.bash_history

WORKDIR /catkin_ws/src/
