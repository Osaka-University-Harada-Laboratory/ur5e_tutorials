# ur5e_tutorials

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
[![license: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

- ROS package for Universal Robots UR5e tutorial.
  - [ur5e_tutorials](/catkin_ws/src/ur5e_tutorials): A tutorial package to execute simple demonstrations.
- Docker for simulation and control environments for Universal Robots UR5e.

## Dependencies

### Docker build environment

- Ubuntu 18.04 (arch=amd64)
  - NVIDIA GeForce RTX2080Ti
  	- NVIDIA Driver 455.23.05
    - CUDA 11.1
  - Docker 20.10.12
  - Docker Compose 1.29.2
  - NVIDIA Docker 2.10.0

## UR5e with a robotiq gripper

- Ubuntu 18.04
  - [ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)
  - [UniversalRobots/Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
  - [fmauch/universal_robot](https://github.com/fmauch/universal_robot.git)
- UniversalRobots UR5e 
- Robotiq 2F-140

## Installation

### Driver modules
1. Install URCap on a e-series robot by following [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_e_series.md).
    - Don't forget the last step of starting the External Control program on the teach pendant ([known issue](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/55)).
    - After [bringup](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/usage_example.md), start the External Control program to establish the connection.  

2. (option to use a gripper) Install [tool communication](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/setup_tool_communication.md) on universal robot pendant.  

### Docker environment
```bash
git clone git@github.com:Osaka-University-Harada-Laboratory/sda5f_tutorials.git --depth 1  
cd ur5e_tutorials
COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker compose build --no-cache --parallel  
docker compose up
```

## Usage with docker

1. bringup the robot  
```bash
xhost + && docker exec -it ur5e_container bash -it -c "roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=XX.XX.XX.XX"  # e.g., 172.32.1.148
```
2. execute the external control script on the pendant  
3. execute a tutorial script from below options

- Executing a wiggle demonstration in real-world
```bash
./utils/ur5e_wiggle.sh
```

- Executing a pick-and-place demonstration in real-world
```bash
./utils/ur5e_pp.sh
```

- Executing a pick-and-toss demonstration in real-world
```bash
./utils/ur5e_pt.sh
```

## Author / Contributor

[Takuya Kiyokawa](https://takuya-ki.github.io/)

## License

This software is released under the MIT License, see [LICENSE](./LICENSE).
