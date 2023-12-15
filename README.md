# ur5e_tutorials

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
[![license: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![repo size](https://img.shields.io/github/repo-size/Osaka-University-Harada-Laboratory/ur5e_tutorials)

- ROS package for Universal Robots UR5e tutorial.
  - [ur5e_tutorials](/catkin_ws/src/ur5e_tutorials): A tutorial package to execute simple demonstrations.
- Docker for simulation and control environments for Universal Robots UR5e.

## Dependencies

### Docker build environment

- [Ubuntu 18.04 PC](https://ubuntu.com/certified/laptops?q=&limit=20&vendor=Dell&vendor=Lenovo&vendor=HP&release=18.04+LTS)
  - NVIDIA GeForce RTX2080Ti
  	- NVIDIA Driver 455.23.05
    - CUDA 11.1
  - Docker 20.10.12
  - Docker Compose 1.29.2
  - NVIDIA Docker 2.10.0

## UR5e with a robotiq gripper

- [Ubuntu 18.04 PC](https://ubuntu.com/certified/laptops?q=&limit=20&vendor=Dell&vendor=Lenovo&vendor=HP&release=18.04+LTS)
  - [ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)
  - [UniversalRobots/Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
  - [fmauch/universal_robot](https://github.com/fmauch/universal_robot.git)
  - [Byobu](https://www.byobu.org/)
- [UniversalRobots UR5e](https://www.universal-robots.com/products/ur5-robot/) 
- [Robotiq 2F-140](https://robotiq.com/products/2f85-140-adaptive-robot-gripper)

## Installation

### Driver modules
1. Install URCap on a e-series robot by following [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_e_series.md).
    - Don't forget the last step of starting the External Control program on the teach pendant ([known issue](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/55)).
    - After [bringup](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/usage_example.md), start the External Control program to establish the connection.  

2. (option to use a gripper) Install [tool communication](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/setup_tool_communication.md) on universal robot pendant.  

### Docker environment
```bash
git clone git@github.com:Osaka-University-Harada-Laboratory/ur5e_tutorials.git --depth 1  
cd ur5e_tutorials
sudo apt install byobu
COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker compose build --no-cache --parallel  
docker compose up
```

## Usage with docker

1. bringup the robot  
```bash
xhost + && docker exec -it ur5e_container bash -it -c "roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=XX.XX.XX.XX"  # e.g., 172.32.1.148
```
2. execute the external control script on the pendant  
3. execute a tutorial script from the below options

- Executing the moveit
```bash
./utils/ur5e_moveit.sh
```

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
